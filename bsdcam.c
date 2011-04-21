
/*
 * BSDCam,
 * a webcam capture/streaming app for *BSD systems.
 * 
 * (C)opyleft 2003-2011 by Jedi/Sector One <j@pureftpd.org>
 * 
 * Based upon the Linux kernel 2.6.0-test5 ov511 driver.
 * 
 * Original credits :
 * 
 * Copyright (c) 1999-2003 Mark W. McClelland
 * Original decompression code Copyright 1998-2000 OmniVision Technologies
 * Many improvements by Bret Wallach <bwallac1@san.rr.com>
 * Color fixes by by Orion Sky Lawlor <olawlor@acm.org> (2/26/2000)
 * Snapshot code by Kevin Moore
 * OV7620 fixes by Charl P. Botha <cpbotha@ieee.org>
 * Changes by Claudio Matsuoka <claudio@conectiva.com>
 * Original SAA7111A code by Dave Perks <dperks@ibm.net>
 * URB error messages from pwc driver by Nemosoft
 * generic_ioctl() code from videodev.c by Gerd Knorr and Alan Cox
 * Memory management (rvmalloc) code from bttv driver, by Gerd Knorr and others
 *
 * Based on the Linux CPiA driver written by Peter Pregler,
 * Scott J. Bertin and Johannes Erdfelt.
 * 
 * Some functions were also based upon the Vid utility for Peter S. Housel.
 * 
 */

#define IN_BSDCAM_H 1

#include <stdio.h>
#include <stdlib.h>
#include <errno.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <fcntl.h>
#include <sys/types.h>
#include <sys/ioctl.h>
#include <dev/usb/usb.h>
#include <jpeglib.h>
#include "bsdcam.h"

#define VERSION "0.4"

#define OV518_PACKET_NUMBERING 1

#define MODERN_USB_CODE 1
#ifndef __FreeBSD__
# define MODERN_USB_DEVICES 1
#endif

/**********************************************************************
 * Symbolic Names
 **********************************************************************/

/* Known OV511-based cameras */
static struct symbolic_list camlist[] = {
	{   0, "Generic Camera (no ID)" },
	{   1, "Mustek WCam 3X" },
	{   3, "D-Link DSB-C300" },
	{   4, "Generic OV511/OV7610" },
	{   5, "Puretek PT-6007" },
	{   6, "Lifeview USB Life TV (NTSC)" },
	{  21, "Creative Labs WebCam 3" },
	{  22, "Lifeview USB Life TV (PAL D/K+B/G)" },
	{  36, "Koala-Cam" },
	{  38, "Lifeview USB Life TV (PAL)" },
	{  41, "Samsung Anycam MPC-M10" },
	{  43, "Mtekvision Zeca MV402" },
	{  46, "Suma eON" },
	{  70, "Lifeview USB Life TV (PAL/SECAM)" },
	{ 100, "Lifeview RoboCam" },
	{ 102, "AverMedia InterCam Elite" },
	{ 112, "MediaForte MV300" },	/* or OV7110 evaluation kit */
	{ 134, "Ezonics EZCam II" },
	{ 192, "Webeye 2000B" },
	{ 253, "Alpha Vision Tech. AlphaCam SE" },
	{  -1, NULL }
};

static struct symbolic_list brglist[] = {
	{ BRG_OV511,		"OV511" },
	{ BRG_OV511PLUS,	"OV511+" },
	{ BRG_OV518,		"OV518" },
	{ BRG_OV518PLUS,	"OV518+" },
	{ -1, NULL }
};

#define OV511_I2C_RETRIES 5
#define ENABLE_Y_QUANTABLE 1
#define ENABLE_UV_QUANTABLE 1

#define DEFAULT_JPEG_QUALITY 80

#define OV511_MAX_UNIT_VIDEO 16

static unsigned char yQuanTable511[] = OV511_YQUANTABLE;
static unsigned char uvQuanTable511[] = OV511_UVQUANTABLE;
static unsigned char yQuanTable518[] = OV518_YQUANTABLE;
static unsigned char uvQuanTable518[] = OV518_UVQUANTABLE;

#define FATAL_ERROR(rc) ((rc) < 0 && (rc) != -EPERM)

/* These variables (and all static globals) default to zero */
static int autobright		= 1;
static int autogain		= 1;
static int autoexp		= 1;
static int debug;
static int snapshot;
static int cams			= 1;
static int compress;
static int testpat;
static int dumppix;
static int led 			= 1;
static int dump_bridge;
static int dump_sensor;
static int printph;
static int phy			= 0x1f;
static int phuv			= 0x05;
static int pvy			= 0x06;
static int pvuv			= 0x06;
static int qhy			= 0x14;
static int qhuv			= 0x03;
static int qvy			= 0x04;
static int qvuv			= 0x04;
static int lightfreq;
static int bandingfilter;
static int clockdiv		= -1;
static int packetsize		= -1;
static int framedrop		= -1;
static int force_palette;
static int backlight;
static int unit_video[OV511_MAX_UNIT_VIDEO];
static int remove_zeros;
static int mirror;
int ov518_color = 1;

static int ignore_desync;
static char *output_filename;
static int usbgen_id;
static unsigned int wait_between_frames;
static int jpeg_quality = DEFAULT_JPEG_QUALITY;
static uid_t switchto_uid;

int reg_r(const struct usb_ov511 * const ov, const int reg) {
	unsigned char data[1024];	
	struct usb_ctl_request ur;
	
#ifdef MODERN_USB_CODE
	ur.ucr_request.bmRequestType = UT_READ_VENDOR_INTERFACE;
	ur.ucr_request.bRequest = ov->bclass == BCL_OV518 ? 1 : 3;
	USETW(ur.ucr_request.wValue, 0);	/* unused */
	USETW(ur.ucr_request.wIndex, reg);	/* index */
	USETW(ur.ucr_request.wLength, 1);	/* payload len in bytes */
	ur.ucr_data = data;
	ur.ucr_flags = 0;
	ur.ucr_actlen = 0;	
#else
	ur.request.bmRequestType = UT_READ_VENDOR_INTERFACE;
	ur.request.bRequest = ov->bclass == BCL_OV518 ? 1 : 3;	
	USETW(ur.request.wValue, 0); /* unused */
	USETW(ur.request.wIndex, reg); /* index */
	USETW(ur.request.wLength, 1);        /* payload len in bytes */
	ur.data = data;
	ur.flags = 0;
	ur.actlen = 0;	
#endif
	
	if (ioctl(ov->fd, USB_DO_REQUEST, &ur) < 0) {
		return -1;
	}
	
	return data[0] & 0xFF;
}

int reg_w(const struct usb_ov511 * const ov, const int reg, const int val) {
	unsigned char data[1024];	
	struct usb_ctl_request ur;
	
	data[0] = val;
	
	ur.ucr_request.bmRequestType = UT_WRITE_VENDOR_INTERFACE;
	ur.ucr_request.bRequest = ov->bclass== BCL_OV518 ? 1 : 2;
	
	USETW(ur.ucr_request.wValue, 0);	/* unused */
	USETW(ur.ucr_request.wIndex, reg);	/* index */
	USETW(ur.ucr_request.wLength, 1);	/* payload len in bytes */
	ur.ucr_data = data;
	ur.ucr_flags = 0;
	ur.ucr_actlen = 0;
  
	if (ioctl(ov->fd, USB_DO_REQUEST, &ur) < 0) {
		return -1;
	}
	
	return 0;
}

int reg_w_mask(const struct usb_ov511 * const ov,
	       const unsigned char reg, unsigned char value,
	       const unsigned char mask)
{
	int ret;
	unsigned char oldval, newval;

	ret = reg_r(ov, reg);
	if (ret < 0) {
		return ret;
	}
	oldval = (unsigned char) ret;
	oldval &= (~mask);		/* Clear the masked bits */
	value &= mask;			/* Enforce mask on value */
	newval = oldval | value;	/* Set the desired bits */

	return reg_w(ov, reg, newval);
}

/* 
 * Writes multiple (n) byte value to a single register. Only valid with certain
 * registers (0x30 and 0xc4 - 0xce).
 */
int ov518_reg_w32(const struct usb_ov511 * const ov,
		  const unsigned char reg, const u_int32_t val,
		  const int n)
{
	u_int32_t data = htole32(val);
	struct usb_ctl_request ur;
	
	ur.ucr_request.bmRequestType = UT_WRITE_VENDOR_INTERFACE;
	ur.ucr_request.bRequest = 1;
	
	USETW(ur.ucr_request.wValue, 0);	/* unused */
	USETW(ur.ucr_request.wIndex, reg);	/* index */
	USETW(ur.ucr_request.wLength, n);	/* payload len in bytes */
	ur.ucr_data = &data;
	ur.ucr_flags = 0;
	ur.ucr_actlen = 0;
  
	if (ioctl(ov->fd, USB_DO_REQUEST, &ur) < 0) {
		return -1;
	}
	
	return 0;
}

int i2c_r(const struct usb_ov511 * const ov, const int reg) 
{
	int status = 0;
	int val = 0;
	int retries = OV7610_I2C_RETRIES;
	
	while (--retries >= 0) {
		/* wait until bus idle */
		do {
			if ((status = reg_r(ov, R511_I2C_CTL)) < 0) {
				return -1;
			}
		} while ((status & 0x01) == 0);
		
		/* perform a dummy write cycle to set the register */
		if (reg_w(ov, R51x_I2C_SADDR_2, reg) < 0) {
			return -1;
		}
		
		/* initiate the dummy write */
		if (reg_w(ov, R511_I2C_CTL, 0x03) < 0) {
			return -1;
		}
		
		/* wait until bus idle */
		do {
			if ((status = reg_r(ov, R511_I2C_CTL)) < 0) {
				return -1;
			}
		} while ((status & 0x01) == 0);

		if ((status & 0x2) == 0) {
			break;
		}
	}
	
	if (retries < 0) {
		return -1;
	}
	
	retries = OV7610_I2C_RETRIES;
	while (--retries >= 0) {
		/* initiate read */
		if (reg_w(ov, R511_I2C_CTL, 0x05) < 0) {
			return -1;
		}
		
		/* wait until bus idle */
		do {
			if ((status = reg_r(ov, R511_I2C_CTL)) < 0) {
				return -1;
			}
		} while ((status & 0x01) == 0);
		
		if ((status & 0x2) == 0) {
			break;
		}
		
		/* abort I2C bus before retrying */
		if (reg_w(ov, R511_I2C_CTL, 0x10) < 0) {
			return -1;
		}
	}
	if (retries < 0) {
		return -1;
	}
	
	/* retrieve data */
	val = reg_r(ov, R51x_I2C_DATA);
	
	/* issue another read for some weird reason */
	if (reg_w(ov, R511_I2C_CTL, 0x05) < 0) {
		return -1;
	}
	
	return val;
}

int i2c_w(const struct usb_ov511 * const ov, const int reg, const int val) 
{
	int status = 0;
	int retries = OV7610_I2C_RETRIES;
	
	while (--retries >= 0) {
		if (reg_w(ov, R51x_I2C_SADDR_3, reg) < 0) {
			return -1;
		}
		
		if (reg_w(ov, R51x_I2C_DATA, val) < 0) {
			return -1;
		}
		
		if (reg_w(ov, R511_I2C_CTL, 0x1) < 0) {
			return -1;
		}
		/* wait until bus idle */
		do {
			if ((status = reg_r(ov, R511_I2C_CTL)) < 0) {
				return -1;
			}
		} while ((status & 0x01) == 0);

		/* OK if ACK */
		if ((status & 0x02) == 0) {
			return 0;
		}
	}
	
	return -1;
}

/* NOTE: Do not call this function directly!
 * The OV518 I2C I/O procedure is different, hence, this function.
 * This is normally only called from i2c_w(). Note that this function
 * always succeeds regardless of whether the sensor is present and working.
 */
int ov518_i2c_write_internal(struct usb_ov511 * const ov,
			     unsigned char reg,
			     unsigned char value)
{
	int rc;

	/* Select camera register */
	rc = reg_w(ov, R51x_I2C_SADDR_3, reg);
	if (rc < 0) {
		return rc;
	}

	/* Write "value" to I2C data port of OV511 */
	rc = reg_w(ov, R51x_I2C_DATA, value);
	if (rc < 0) {
		return rc;
	}

	/* Initiate 3-byte write cycle */
	rc = reg_w(ov, R518_I2C_CTL, 0x01);
	if (rc < 0) {
		return rc;
	}

	return 0;
}

/* NOTE: Do not call this function directly! */
int ov511_i2c_write_internal(struct usb_ov511 * const ov,
			     unsigned char reg,
			     unsigned char value)
{
	int rc, retries;

	/* Three byte write cycle */
	for (retries = OV511_I2C_RETRIES; ; ) {
		/* Select camera register */
		rc = reg_w(ov, R51x_I2C_SADDR_3, reg);
		if (rc < 0) {
			break;
		}
		/* Write "value" to I2C data port of OV511 */
		rc = reg_w(ov, R51x_I2C_DATA, value);
		if (rc < 0) {
			break;
		}

		/* Initiate 3-byte write cycle */
		rc = reg_w(ov, R511_I2C_CTL, 0x01);
		if (rc < 0) {
			break;
		}

		/* Retry until idle */
		do {
			rc = reg_r(ov, R511_I2C_CTL);
		} while (rc > 0 && ((rc&1) == 0));
		
		if (rc < 0) {
			break;
		}

		/* Ack? */
		if ((rc & 2) == 0) {
			rc = 0;
			break;
		}
#if 0
		/* I2C abort */
		reg_w(ov, R511_I2C_CTL, 0x10);
#endif
		if (--retries < 0) {
			fprintf(stderr, "i2c write retries exhausted\n");
			rc = -1;
			break;
		}
	}

	return rc;
}

/* NOTE: Do not call this function directly!
 * The OV518 I2C I/O procedure is different, hence, this function.
 * This is normally only called from i2c_r(). Note that this function
 * always succeeds regardless of whether the sensor is present and working.
 */

int ov518_i2c_read_internal(struct usb_ov511 * const ov, unsigned char reg)
{
	int rc, value;

	/* Select camera register */
	rc = reg_w(ov, R51x_I2C_SADDR_2, reg);
	if (rc < 0) {
		return rc;
	}

	/* Initiate 2-byte write cycle */
	rc = reg_w(ov, R518_I2C_CTL, 0x03);
	if (rc < 0) {
		return rc;
	}

	/* Initiate 2-byte read cycle */
	rc = reg_w(ov, R518_I2C_CTL, 0x05);
	if (rc < 0) {
		return rc;
	}

	return reg_r(ov, R51x_I2C_DATA);
}

/* NOTE: Do not call this function directly!
 * returns: negative is error, pos or zero is data */
int ov511_i2c_read_internal(struct usb_ov511 * const ov, unsigned char reg)
{
	int rc, value, retries;

	/* Two byte write cycle */
	for (retries = OV511_I2C_RETRIES; ; ) {
		/* Select camera register */
		rc = reg_w(ov, R51x_I2C_SADDR_2, reg);
		if (rc < 0) {
			return rc;
		}

		/* Initiate 2-byte write cycle */
		rc = reg_w(ov, R511_I2C_CTL, 0x03);
		if (rc < 0) {
			return rc;
		}

		/* Retry until idle */
		do {
			 rc = reg_r(ov, R511_I2C_CTL);
		} while (rc > 0 && ((rc&1) == 0));
		
		if (rc < 0) {
			return rc;
		}
		if ((rc & 2) == 0) {/* Ack? */
			break;
		}

		/* I2C abort */
		reg_w(ov, R511_I2C_CTL, 0x10);

		if (--retries < 0) {
			fprintf(stderr, "i2c write retries exhausted\n");
			return -1;
		}
	}

	/* Two byte read cycle */
	for (retries = OV511_I2C_RETRIES; ; ) {
		/* Initiate 2-byte read cycle */
		rc = reg_w(ov, R511_I2C_CTL, 0x05);
		if (rc < 0) {
			return rc;
		}

		/* Retry until idle */
		do {
			rc = reg_r(ov, R511_I2C_CTL);
		} while (rc > 0 && ((rc&1) == 0));
		if (rc < 0) {
			return rc;
		}

		if ((rc&2) == 0) { /* Ack? */
			break;
		}

		/* I2C abort */
		rc = reg_w(ov, R511_I2C_CTL, 0x10);
		if (rc < 0) {
			return rc;
		}
			
		if (--retries < 0) {
			fprintf(stderr, "i2c read retries exhausted\n");
			return -1;
		}
	}

	value = reg_r(ov, R51x_I2C_DATA);

	/* This is needed to make i2c_w() work */
	rc = reg_w(ov, R511_I2C_CTL, 0x05);
	if (rc < 0) {
		return rc;
	}

	return value;
}

/* Do not call this function directly! */
int ov51x_i2c_write_mask_internal(struct usb_ov511 * const ov,
				  unsigned char reg,
				  unsigned char value,
				  unsigned char mask)
{
	int rc;
	unsigned char oldval, newval;

	if (mask == 0xff) {
		newval = value;
	} else {
		if (ov->bclass == BCL_OV518){
			rc = ov518_i2c_read_internal(ov, reg);
		} else {
			rc = ov511_i2c_read_internal(ov, reg);
		}
		if (rc < 0) {
			return rc;
		}

		oldval = (unsigned char) rc;
		oldval &= (~mask);		/* Clear the masked bits */
		value &= mask;			/* Enforce mask on value */
		newval = oldval | value;	/* Set the desired bits */
	}

	if (ov->bclass == BCL_OV518) {
		return ov518_i2c_write_internal(ov, reg, newval);
	} else {
		return ov511_i2c_write_internal(ov, reg, newval);
	}
}

int i2c_w_mask(struct usb_ov511 * const ov,
	       unsigned char reg, unsigned char value, unsigned char mask)
{
	return ov51x_i2c_write_mask_internal(ov, reg, value, mask);
}

int ov51x_reset(struct usb_ov511 * const ov, unsigned char reset_type)
{
	int rc;

	/* Setting bit 0 not allowed on 518/518Plus */
	if (ov->bclass == BCL_OV518) {
		reset_type &= 0xfe;
	}

	rc  = reg_w(ov, R51x_SYS_RESET, reset_type);
	rc |= reg_w(ov, R51x_SYS_RESET, 0);

	return rc;
}

int init_ov_sensor(struct usb_ov511 * const ov)
{
	int i, success;
	
	/* Reset the sensor */
	if (i2c_w(ov, 0x12, 0x80) < 0) {
		return -EIO;
	}
	usleep(150U);
	
	for (i = 0, success = 0; i < I2C_DETECT_TRIES && !success; i++) {
		if ((i2c_r(ov, OV7610_REG_ID_HIGH) == 0x7F) &&
		    (i2c_r(ov, OV7610_REG_ID_LOW) == 0xA2)) {
			success = 1;
			continue;
		}
		/* Reset the sensor */
		if (i2c_w(ov, 0x12, 0x80) < 0) {
			return -EIO;
		}
		/* Wait for it to initialize */
		usleep(150U);
		/* Dummy read to sync I2C */
		if (i2c_r(ov, 0x00) < 0) {
			return -EIO;
		}
	}

	if (!success) {
		return -EIO;
	}

	fprintf(stderr, "I2C synced in %d attempt(s)\n", i);

	return 0;
}

int i2c_set_slave_internal(struct usb_ov511 * const ov, unsigned char slave)
{
	int rc;
	
	rc = reg_w(ov, R51x_I2C_W_SID, slave);
	if (rc < 0) {
		return rc;
	}
	rc = reg_w(ov, R51x_I2C_R_SID, slave + 1);
	if (rc < 0) {
		return rc;
	}
	
	return 0;
}

int ov51x_set_slave_ids(struct usb_ov511 *ov, unsigned char sid)
{	
	int rc;
	
	rc = i2c_set_slave_internal(ov, sid);
	(void) ov51x_reset(ov, OV511_RESET_NOREGS);
	
	return rc;
}

int write_regvals(struct usb_ov511 * const ov, 
		  const struct ov511_regvals *pRegvals)
{
	int rc;

	while (pRegvals->bus != OV511_DONE_BUS) {
		if (pRegvals->bus == OV511_REG_BUS) {
			if ((rc = reg_w(ov, pRegvals->reg, pRegvals->val)) < 0) {
				return rc;
			}
		} else if (pRegvals->bus == OV511_I2C_BUS) {
			if ((rc = i2c_w(ov, pRegvals->reg, pRegvals->val)) < 0) {
				return rc;
			}
		} else {
			fprintf(stderr, "Bad regval array");
			return -1;
		}
		pRegvals++;
	}
	return 0;
}

void ov51x_led_control(const struct usb_ov511 * const ov, const int enable)
{
	if (ov->bridge == BRG_OV511PLUS) {
		reg_w(ov, R511_SYS_LED_CTL, enable ? 1 : 0);
	} else if (ov->bclass == BCL_OV518) {
		reg_w_mask(ov, R518_GPIO_OUT, enable ? 0x02 : 0x00, 0x02);
	}
	fprintf(stderr, "Led: %d\n", enable);
}

int ov511_upload_quant_tables(const struct usb_ov511 * const ov)
{
	unsigned char *pYTable = yQuanTable511;
	unsigned char *pUVTable = uvQuanTable511;
	unsigned char val0, val1;
	int i, rc, reg = R511_COMP_LUT_BEGIN;

	fprintf(stderr, "Uploading quantization tables\n");

	for (i = 0; i < OV511_QUANTABLESIZE / 2; i++) {
		if (ENABLE_Y_QUANTABLE)	{
			val0 = *pYTable++;
			val1 = *pYTable++;
			val0 &= 0x0f;
			val1 &= 0x0f;
			val0 |= val1 << 4;
			rc = reg_w(ov, reg, val0);
			if (rc < 0)
				return rc;
		}

		if (ENABLE_UV_QUANTABLE) {
			val0 = *pUVTable++;
			val1 = *pUVTable++;
			val0 &= 0x0f;
			val1 &= 0x0f;
			val0 |= val1 << 4;
			rc = reg_w(ov, reg + OV511_QUANTABLESIZE/2, val0);
			if (rc < 0)
				return rc;
		}

		reg++;
	}

	return 0;
}

int ov511_init_compression(struct usb_ov511 * const ov)
{
	int rc = 0;

	if (!ov->compress_inited) {
		reg_w(ov, 0x70, phy);
		reg_w(ov, 0x71, phuv);
		reg_w(ov, 0x72, pvy);
		reg_w(ov, 0x73, pvuv);
		reg_w(ov, 0x74, qhy);
		reg_w(ov, 0x75, qhuv);
		reg_w(ov, 0x76, qvy);
		reg_w(ov, 0x77, qvuv);

		if (ov511_upload_quant_tables(ov) < 0) {
			fprintf(stderr,
				"Error uploading quantization tables\n");
			rc = -EIO;
			goto out;
		}
	}

	ov->compress_inited = 1;
out:
	return rc;
}

/* Temporarily stops OV511 from functioning. Must do this before changing
 * registers while the camera is streaming */
static inline int ov51x_stop(struct usb_ov511 * const ov)
{
	fprintf(stderr, "stopping\n");
	ov->stopped = 1;
	if (ov->bclass == BCL_OV518) {
		return (reg_w_mask(ov, R51x_SYS_RESET, 0x3a, 0x3a));
	}
	return (reg_w(ov, R51x_SYS_RESET, 0x3d));
}

/* Restarts OV511 after ov511_stop() is called. Has no effect if it is not
 * actually stopped (for performance). */
static inline int ov51x_restart(struct usb_ov511 * const ov)
{
	if (ov->stopped) {
		fprintf(stderr, "restarting\n");
		ov->stopped = 0;

		/* Reinitialize the stream */
		if (ov->bclass == BCL_OV518) {
			reg_w(ov, 0x2f, 0x80);
		}
		return reg_w(ov, R51x_SYS_RESET, 0x00);
	}

	return 0;
}

/* Resets the hardware snapshot button */
void ov51x_clear_snapshot(const struct usb_ov511 * const ov)
{
	if (ov->bclass == BCL_OV511) {
		reg_w(ov, R51x_SYS_SNAP, 0x00);
		reg_w(ov, R51x_SYS_SNAP, 0x02);
		reg_w(ov, R51x_SYS_SNAP, 0x00);
	} else if (ov->bclass == BCL_OV518) {
		fprintf(stderr, "snapshot reset not supported yet on OV518(+)\n");
	} else {
		fprintf(stderr, "clear snap: invalid bridge type\n");
	}
}

int ov_set_interface(const struct usb_ov511 * const ov, const int altv)
{
	struct usb_alt_interface alt;	/* interface/alternate selection */

#ifdef MODERN_USB_CODE
	alt.uai_interface_index = 0;
	alt.uai_alt_no = altv;
#else
	alt.interface_index = 0;
	alt.alt_no = altv;
#endif
	fprintf(stderr, "setting altinterface with alt (%d)\n", altv);
	if(ioctl(ov->fd, USB_SET_ALTINTERFACE, &alt) < 0) {
		fprintf(stderr, "error while setting altinterface\n");
		return -1;
	}
	return 0;
}

int ov511_set_packet_size(struct usb_ov511 * const ov, const int size)
{
	int alt, mult;

	if (ov51x_stop(ov) < 0) {
		return -EIO;
	}

	mult = size >> 5;

	if (ov->bridge == BRG_OV511) {
		if (size == 0)
			alt = OV511_ALT_SIZE_0;
		else if (size == 257)
			alt = OV511_ALT_SIZE_257;
		else if (size == 513)
			alt = OV511_ALT_SIZE_513;
		else if (size == 769)
			alt = OV511_ALT_SIZE_769;
		else if (size == 993)
			alt = OV511_ALT_SIZE_993;
		else {
			fprintf(stderr, 
				"Set packet size: invalid size (%d)\n", size);
			return -EINVAL;
		}
	} else if (ov->bridge == BRG_OV511PLUS) {
		if (size == 0)
			alt = OV511PLUS_ALT_SIZE_0;
		else if (size == 33)
			alt = OV511PLUS_ALT_SIZE_33;
		else if (size == 129)
			alt = OV511PLUS_ALT_SIZE_129;
		else if (size == 257)
			alt = OV511PLUS_ALT_SIZE_257;
		else if (size == 385)
			alt = OV511PLUS_ALT_SIZE_385;
		else if (size == 513)
			alt = OV511PLUS_ALT_SIZE_513;
		else if (size == 769)
			alt = OV511PLUS_ALT_SIZE_769;
		else if (size == 961)
			alt = OV511PLUS_ALT_SIZE_961;
		else {
			fprintf(stderr, 
				"Set packet size: invalid size (%d)\n", size);
			return -EINVAL;
		}
	} else {
		fprintf(stderr, "Set packet size: Invalid bridge type\n");
		return -EINVAL;
	}

	fprintf(stderr, "%d, mult=%d, alt=%d\n", size, mult, alt);

	if (reg_w(ov, R51x_FIFO_PSIZE, mult) < 0) {
		return -EIO;
	}

	if (ov_set_interface(ov, alt) < 0) {
		fprintf(stderr, "Set packet size: set interface error\n");
		return -EBUSY;
	}

	if (ov51x_reset(ov, OV511_RESET_NOREGS) < 0) {
		return -EIO;
	}

	ov->packet_size = size;

	if (ov51x_restart(ov) < 0) {
		return -EIO;
	}

	return 0;	
}

/* Note: Unlike the OV511/OV511+, the size argument does NOT include the
 * optional packet number byte. The actual size *is* stored in ov->packet_size,
 * though. */
int ov518_set_packet_size(struct usb_ov511 * const ov, int size)
{
	int alt;

	if (ov51x_stop(ov) < 0) {
		return -EIO;
	}

	if (ov->bclass == BCL_OV518) {
		if (size == 0) {
			alt = OV518_ALT_SIZE_0;
		} else if (size == 128) {
			alt = OV518_ALT_SIZE_128;
		} else if (size == 256) {
			alt = OV518_ALT_SIZE_256;
		} else if (size == 384) {
			alt = OV518_ALT_SIZE_384;
		} else if (size == 512) {
			alt = OV518_ALT_SIZE_512;
		} else if (size == 640) {
			alt = OV518_ALT_SIZE_640;
		} else if (size == 768) {
			alt = OV518_ALT_SIZE_768;
		} else if (size == 896) {
			alt = OV518_ALT_SIZE_896;
		} else {
			fprintf(stderr, "Set packet size: invalid size (%d)", 
				size);
			return -EINVAL;
		}
	} else {
		fprintf(stderr, "Set packet size: Invalid bridge type\n");
		return -EINVAL;
	}

	ov->packet_size = size;
	if (size > 0) {
		/* Program ISO FIFO size reg (packet number isn't included) */
		ov518_reg_w32(ov, 0x30, size, 2);

		if (ov->packet_numbering) {
			++ov->packet_size;
		}
	}

	if (ov_set_interface(ov, alt) < 0) {
		fprintf(stderr, "Set packet size: set interface error\n");
		return -EBUSY;
	}

	/* Initialize the stream */
	if (reg_w(ov, 0x2f, 0x80) < 0) {
		return -EIO;
	}

	if (ov51x_restart(ov) < 0) {
		return -EIO;
	}

	if (ov51x_reset(ov, OV511_RESET_NOREGS) < 0) {
		return -EIO;
	}

	return 0;
}

int ks0127_configure(struct usb_ov511 * const ov)
{
	fprintf(stderr, "ks0127 sensor not supported yet\n");
	abort();
}

int saa7111a_configure(struct usb_ov511 * const ov)
{
	int rc;

	/* Since there is no register reset command, all registers must be
	 * written, otherwise gives erratic results */
	static struct ov511_regvals aRegvalsNormSAA7111A[] = {
		{ OV511_I2C_BUS, 0x06, 0xce },
		{ OV511_I2C_BUS, 0x07, 0x00 },
		{ OV511_I2C_BUS, 0x10, 0x44 }, /* YUV422, 240/286 lines */
		{ OV511_I2C_BUS, 0x0e, 0x01 }, /* NTSC M or PAL BGHI */
		{ OV511_I2C_BUS, 0x00, 0x00 },
		{ OV511_I2C_BUS, 0x01, 0x00 },
		{ OV511_I2C_BUS, 0x03, 0x23 },
		{ OV511_I2C_BUS, 0x04, 0x00 },
		{ OV511_I2C_BUS, 0x05, 0x00 },
		{ OV511_I2C_BUS, 0x08, 0xc8 }, /* Auto field freq */
		{ OV511_I2C_BUS, 0x09, 0x01 }, /* Chrom. trap off, APER=0.25 */
		{ OV511_I2C_BUS, 0x0a, 0x80 }, /* BRIG=128 */
		{ OV511_I2C_BUS, 0x0b, 0x40 }, /* CONT=1.0 */
		{ OV511_I2C_BUS, 0x0c, 0x40 }, /* SATN=1.0 */
		{ OV511_I2C_BUS, 0x0d, 0x00 }, /* HUE=0 */
		{ OV511_I2C_BUS, 0x0f, 0x00 },
		{ OV511_I2C_BUS, 0x11, 0x0c },
		{ OV511_I2C_BUS, 0x12, 0x00 },
		{ OV511_I2C_BUS, 0x13, 0x00 },
		{ OV511_I2C_BUS, 0x14, 0x00 },
		{ OV511_I2C_BUS, 0x15, 0x00 },
		{ OV511_I2C_BUS, 0x16, 0x00 },
		{ OV511_I2C_BUS, 0x17, 0x00 },
		{ OV511_I2C_BUS, 0x02, 0xc0 },	/* Composite input 0 */
		{ OV511_DONE_BUS, 0x0, 0x00 },
	};

// FIXME: I don't know how to sync or reset it yet
#if 0
	if (ov51x_init_saa_sensor(ov) < 0) {
		fprintf(stderr, "Failed to initialize the SAA7111A\n");
		return -1;
	} else {
		fprintf(stderr, "SAA7111A sensor detected\n");
	}
#endif

	/* 640x480 not supported with PAL */
	if (ov->pal) {
		ov->maxwidth = 320;
		ov->maxheight = 240;		/* Even field only */
	} else {
		ov->maxwidth = 640;
		ov->maxheight = 480;		/* Even/Odd fields */
	}

	ov->minwidth = 320;
	ov->minheight = 240;		/* Even field only */

	ov->has_decoder = 1;
	ov->num_inputs = 8;
	ov->norm = 0;
	ov->stop_during_set = 0;	/* Decoder guarantees stable image */

	/* Decoder doesn't change these values, so we use these instead of
	 * acutally reading the registers (which doesn't work) */
	ov->brightness = 0x80 << 8;
	ov->contrast = 0x40 << 9;
	ov->colour = 0x40 << 9;
	ov->hue = 32768;

	fprintf(stderr, "Writing SAA7111A registers\n");
	if (write_regvals(ov, aRegvalsNormSAA7111A)) {
		return -1;
	}

	/* Detect version of decoder. This must be done after writing the
         * initial regs or the decoder will lock up. */
	rc = i2c_r(ov, 0x00);

	if (rc < 0) {
		fprintf(stderr, "Error detecting sensor version\n");
		return -1;
	} else {
		fprintf(stderr, "Sensor is an SAA7111A (version 0x%x)\n", rc);
		ov->sensor = SEN_SAA7111A;
	}

	// FIXME: Fix this for OV518(+)
	/* Latch to negative edge of clock. Otherwise, we get incorrect
	 * colors and jitter in the digital signal. */
	if (ov->bclass == BCL_OV511) {
		reg_w(ov, 0x11, 0x00);
	} else {
		fprintf(stderr, 
			"SAA7111A not yet supported with OV518/OV518+\n");
	}

	return 0;
}

/* This initializes the OV6620, OV6630, OV6630AE, or OV6630AF sensor. */
int ov6xx0_configure(struct usb_ov511 * const ov)
{
	int rc;

	static struct ov511_regvals aRegvalsNorm6x20[] = {
		{ OV511_I2C_BUS, 0x12, 0x80 }, /* reset */
		{ OV511_I2C_BUS, 0x11, 0x01 },
		{ OV511_I2C_BUS, 0x03, 0x60 },
		{ OV511_I2C_BUS, 0x05, 0x7f }, /* For when autoadjust is off */
		{ OV511_I2C_BUS, 0x07, 0xa8 },
		/* The ratio of 0x0c and 0x0d  controls the white point */
		{ OV511_I2C_BUS, 0x0c, 0x24 },
		{ OV511_I2C_BUS, 0x0d, 0x24 },
		{ OV511_I2C_BUS, 0x0f, 0x15 }, /* COMS */
		{ OV511_I2C_BUS, 0x10, 0x75 }, /* AEC Exposure time */
		{ OV511_I2C_BUS, 0x12, 0x24 }, /* Enable AGC */
		{ OV511_I2C_BUS, 0x14, 0x04 },
		/* 0x16: 0x06 helps frame stability with moving objects */
		{ OV511_I2C_BUS, 0x16, 0x06 },
//		{ OV511_I2C_BUS, 0x20, 0x30 }, /* Aperture correction enable */
		{ OV511_I2C_BUS, 0x26, 0xb2 }, /* BLC enable */
		/* 0x28: 0x05 Selects RGB format if RGB on */
		{ OV511_I2C_BUS, 0x28, 0x05 },
		{ OV511_I2C_BUS, 0x2a, 0x04 }, /* Disable framerate adjust */
//		{ OV511_I2C_BUS, 0x2b, 0xac }, /* Framerate; Set 2a[7] first */
		{ OV511_I2C_BUS, 0x2d, 0x99 },
		{ OV511_I2C_BUS, 0x33, 0xa0 }, /* Color Procesing Parameter */
		{ OV511_I2C_BUS, 0x34, 0xd2 }, /* Max A/D range */
		{ OV511_I2C_BUS, 0x38, 0x8b },
		{ OV511_I2C_BUS, 0x39, 0x40 },

		{ OV511_I2C_BUS, 0x3c, 0x39 }, /* Enable AEC mode changing */
		{ OV511_I2C_BUS, 0x3c, 0x3c }, /* Change AEC mode */
		{ OV511_I2C_BUS, 0x3c, 0x24 }, /* Disable AEC mode changing */

		{ OV511_I2C_BUS, 0x3d, 0x80 },
		/* These next two registers (0x4a, 0x4b) are undocumented. They
		 * control the color balance */
		{ OV511_I2C_BUS, 0x4a, 0x80 },
		{ OV511_I2C_BUS, 0x4b, 0x80 },
		{ OV511_I2C_BUS, 0x4d, 0xd2 }, /* This reduces noise a bit */
		{ OV511_I2C_BUS, 0x4e, 0xc1 },
		{ OV511_I2C_BUS, 0x4f, 0x04 },
// Do 50-53 have any effect?
// Toggle 0x12[2] off and on here?
		{ OV511_DONE_BUS, 0x0, 0x00 },	/* END MARKER */
	};
	
	static struct ov511_regvals aRegvalsNorm6x30[] = {
	/*OK*/	{ OV511_I2C_BUS, 0x12, 0x80 }, /* reset */
		{ OV511_I2C_BUS, 0x11, 0x00 },
	/*OK*/	{ OV511_I2C_BUS, 0x03, 0x60 },
	/*0A?*/	{ OV511_I2C_BUS, 0x05, 0x7f }, /* For when autoadjust is off */
		{ OV511_I2C_BUS, 0x07, 0xa8 },
		/* The ratio of 0x0c and 0x0d  controls the white point */
	/*OK*/	{ OV511_I2C_BUS, 0x0c, 0x24 },
	/*OK*/	{ OV511_I2C_BUS, 0x0d, 0x24 },
	/*A*/	{ OV511_I2C_BUS, 0x0e, 0x20 },
//	/*04?*/	{ OV511_I2C_BUS, 0x14, 0x80 },
		{ OV511_I2C_BUS, 0x16, 0x03 },
//	/*OK*/	{ OV511_I2C_BUS, 0x20, 0x30 }, /* Aperture correction enable */
		// 21 & 22? The suggested values look wrong. Go with default
	/*A*/	{ OV511_I2C_BUS, 0x23, 0xc0 },
	/*A*/	{ OV511_I2C_BUS, 0x25, 0x9a }, // Check this against default
//	/*OK*/	{ OV511_I2C_BUS, 0x26, 0xb2 }, /* BLC enable */

		/* 0x28: 0x05 Selects RGB format if RGB on */
//	/*04?*/	{ OV511_I2C_BUS, 0x28, 0x05 },
//	/*04?*/	{ OV511_I2C_BUS, 0x28, 0x45 }, // DEBUG: Tristate UV bus

	/*OK*/	{ OV511_I2C_BUS, 0x2a, 0x04 }, /* Disable framerate adjust */
//	/*OK*/	{ OV511_I2C_BUS, 0x2b, 0xac }, /* Framerate; Set 2a[7] first */
		{ OV511_I2C_BUS, 0x2d, 0x99 },
//	/*A*/	{ OV511_I2C_BUS, 0x33, 0x26 }, // Reserved bits on 6620
//	/*d2?*/	{ OV511_I2C_BUS, 0x34, 0x03 }, /* Max A/D range */
//	/*8b?*/	{ OV511_I2C_BUS, 0x38, 0x83 },
//	/*40?*/	{ OV511_I2C_BUS, 0x39, 0xc0 }, // 6630 adds bit 7
//		{ OV511_I2C_BUS, 0x3c, 0x39 }, /* Enable AEC mode changing */
//		{ OV511_I2C_BUS, 0x3c, 0x3c }, /* Change AEC mode */
//		{ OV511_I2C_BUS, 0x3c, 0x24 }, /* Disable AEC mode changing */
		{ OV511_I2C_BUS, 0x3d, 0x80 },
//	/*A*/	{ OV511_I2C_BUS, 0x3f, 0x0e },

		/* These next two registers (0x4a, 0x4b) are undocumented. They
		 * control the color balance */
//	/*OK?*/	{ OV511_I2C_BUS, 0x4a, 0x80 }, // Check these
//	/*OK?*/	{ OV511_I2C_BUS, 0x4b, 0x80 },
		{ OV511_I2C_BUS, 0x4d, 0x10 }, /* U = 0.563u, V = 0.714v */
	/*c1?*/	{ OV511_I2C_BUS, 0x4e, 0x40 },

		/* UV average mode, color killer: strongest */
		{ OV511_I2C_BUS, 0x4f, 0x07 },

		{ OV511_I2C_BUS, 0x54, 0x23 }, /* Max AGC gain: 18dB */
		{ OV511_I2C_BUS, 0x57, 0x81 }, /* (default) */
		{ OV511_I2C_BUS, 0x59, 0x01 }, /* AGC dark current comp: +1 */
		{ OV511_I2C_BUS, 0x5a, 0x2c }, /* (undocumented) */
		{ OV511_I2C_BUS, 0x5b, 0x0f }, /* AWB chrominance levels */
//		{ OV511_I2C_BUS, 0x5c, 0x10 },
		{ OV511_DONE_BUS, 0x0, 0x00 },	/* END MARKER */
	};

	fprintf(stderr, "starting sensor configuration\n");

	if (init_ov_sensor(ov) < 0) {
		fprintf(stderr,
			"Failed to read sensor ID.\n"
			"You might not have an OV6xx0.\n");
		return -1;
	} else {
		fprintf(stderr, "OV6xx0 sensor detected\n");
	}

	/* Detect sensor (sub)type */
	rc = i2c_r(ov, OV7610_REG_COM_I);

	if (rc < 0) {
		fprintf(stderr, "Error detecting sensor type\n");
		return -1;
	}

	if ((rc & 3) == 0) {
		ov->sensor = SEN_OV6630;
		fprintf(stderr, "Sensor is an OV6630\n");
	} else if ((rc & 3) == 1) {
		ov->sensor = SEN_OV6620;
		fprintf(stderr, "Sensor is an OV6620\n");
	} else if ((rc & 3) == 2) {
		ov->sensor = SEN_OV6630;
		fprintf(stderr, "Sensor is an OV6630AE\n");
	} else if ((rc & 3) == 3) {
		ov->sensor = SEN_OV6630;
		fprintf(stderr, "Sensor is an OV6630AF\n");
	}

	/* Set sensor-specific vars */
	ov->maxwidth = 352;
	ov->maxheight = 288;
	ov->minwidth = 64;
	ov->minheight = 48;

	// FIXME: These do not match the actual settings yet
	ov->brightness = 0x80 << 8;
	ov->contrast = 0x80 << 8;
	ov->colour = 0x80 << 8;
	ov->hue = 0x80 << 8;

	if (ov->sensor == SEN_OV6620) {
		fprintf(stderr, "Writing 6x20 registers\n");
		if (write_regvals(ov, aRegvalsNorm6x20)) {
			return -1;
		}
	} else {
		fprintf(stderr, "Writing 6x30 registers\n");
		if (write_regvals(ov, aRegvalsNorm6x30)) {
			return -1;
		}
	}

	return 0;
}

int ov7xx0_configure(struct usb_ov511 * const ov)
{
	int i, success;
	int rc;

	/* Lawrence Glaister <lg@jfm.bc.ca> reports:
	 *
	 * Register 0x0f in the 7610 has the following effects:
	 *
	 * 0x85 (AEC method 1): Best overall, good contrast range
	 * 0x45 (AEC method 2): Very overexposed
	 * 0xa5 (spec sheet default): Ok, but the black level is
	 *	shifted resulting in loss of contrast
	 * 0x05 (old driver setting): very overexposed, too much
	 *	contrast
	 */
	static struct ov511_regvals aRegvalsNorm7610[] = {
		{ OV511_I2C_BUS, 0x10, 0xff },
		{ OV511_I2C_BUS, 0x16, 0x06 },
		{ OV511_I2C_BUS, 0x28, 0x24 },
		{ OV511_I2C_BUS, 0x2b, 0xac },
		{ OV511_I2C_BUS, 0x12, 0x00 },
		{ OV511_I2C_BUS, 0x38, 0x81 },
		{ OV511_I2C_BUS, 0x28, 0x24 },	/* 0c */
		{ OV511_I2C_BUS, 0x0f, 0x85 },	/* lg's setting */
		{ OV511_I2C_BUS, 0x15, 0x01 },
		{ OV511_I2C_BUS, 0x20, 0x1c },
		{ OV511_I2C_BUS, 0x23, 0x2a },
		{ OV511_I2C_BUS, 0x24, 0x10 },
		{ OV511_I2C_BUS, 0x25, 0x8a },
		{ OV511_I2C_BUS, 0x26, 0xa2 },
		{ OV511_I2C_BUS, 0x27, 0xc2 },
		{ OV511_I2C_BUS, 0x2a, 0x04 },
		{ OV511_I2C_BUS, 0x2c, 0xfe },
		{ OV511_I2C_BUS, 0x2d, 0x93 },
		{ OV511_I2C_BUS, 0x30, 0x71 },
		{ OV511_I2C_BUS, 0x31, 0x60 },
		{ OV511_I2C_BUS, 0x32, 0x26 },
		{ OV511_I2C_BUS, 0x33, 0x20 },
		{ OV511_I2C_BUS, 0x34, 0x48 },
		{ OV511_I2C_BUS, 0x12, 0x24 },
		{ OV511_I2C_BUS, 0x11, 0x01 },
		{ OV511_I2C_BUS, 0x0c, 0x24 },
		{ OV511_I2C_BUS, 0x0d, 0x24 },
		{ OV511_DONE_BUS, 0x0, 0x00 },
	};

	static struct ov511_regvals aRegvalsNorm7620[] = {
		{ OV511_I2C_BUS, 0x00, 0x00 },
		{ OV511_I2C_BUS, 0x01, 0x80 },
		{ OV511_I2C_BUS, 0x02, 0x80 },
		{ OV511_I2C_BUS, 0x03, 0xc0 },
		{ OV511_I2C_BUS, 0x06, 0x60 },
		{ OV511_I2C_BUS, 0x07, 0x00 },
		{ OV511_I2C_BUS, 0x0c, 0x24 },
		{ OV511_I2C_BUS, 0x0c, 0x24 },
		{ OV511_I2C_BUS, 0x0d, 0x24 },
		{ OV511_I2C_BUS, 0x11, 0x01 },
		{ OV511_I2C_BUS, 0x12, 0x24 },
		{ OV511_I2C_BUS, 0x13, 0x01 },
		{ OV511_I2C_BUS, 0x14, 0x84 },
		{ OV511_I2C_BUS, 0x15, 0x01 },
		{ OV511_I2C_BUS, 0x16, 0x03 },
		{ OV511_I2C_BUS, 0x17, 0x2f },
		{ OV511_I2C_BUS, 0x18, 0xcf },
		{ OV511_I2C_BUS, 0x19, 0x06 },
		{ OV511_I2C_BUS, 0x1a, 0xf5 },
		{ OV511_I2C_BUS, 0x1b, 0x00 },
		{ OV511_I2C_BUS, 0x20, 0x18 },
		{ OV511_I2C_BUS, 0x21, 0x80 },
		{ OV511_I2C_BUS, 0x22, 0x80 },
		{ OV511_I2C_BUS, 0x23, 0x00 },
		{ OV511_I2C_BUS, 0x26, 0xa2 },
		{ OV511_I2C_BUS, 0x27, 0xea },
		{ OV511_I2C_BUS, 0x28, 0x20 },
		{ OV511_I2C_BUS, 0x29, 0x00 },
		{ OV511_I2C_BUS, 0x2a, 0x10 },
		{ OV511_I2C_BUS, 0x2b, 0x00 },
		{ OV511_I2C_BUS, 0x2c, 0x88 },
		{ OV511_I2C_BUS, 0x2d, 0x91 },
		{ OV511_I2C_BUS, 0x2e, 0x80 },
		{ OV511_I2C_BUS, 0x2f, 0x44 },
		{ OV511_I2C_BUS, 0x60, 0x27 },
		{ OV511_I2C_BUS, 0x61, 0x02 },
		{ OV511_I2C_BUS, 0x62, 0x5f },
		{ OV511_I2C_BUS, 0x63, 0xd5 },
		{ OV511_I2C_BUS, 0x64, 0x57 },
		{ OV511_I2C_BUS, 0x65, 0x83 },
		{ OV511_I2C_BUS, 0x66, 0x55 },
		{ OV511_I2C_BUS, 0x67, 0x92 },
		{ OV511_I2C_BUS, 0x68, 0xcf },
		{ OV511_I2C_BUS, 0x69, 0x76 },
		{ OV511_I2C_BUS, 0x6a, 0x22 },
		{ OV511_I2C_BUS, 0x6b, 0x00 },
		{ OV511_I2C_BUS, 0x6c, 0x02 },
		{ OV511_I2C_BUS, 0x6d, 0x44 },
		{ OV511_I2C_BUS, 0x6e, 0x80 },
		{ OV511_I2C_BUS, 0x6f, 0x1d },
		{ OV511_I2C_BUS, 0x70, 0x8b },
		{ OV511_I2C_BUS, 0x71, 0x00 },
		{ OV511_I2C_BUS, 0x72, 0x14 },
		{ OV511_I2C_BUS, 0x73, 0x54 },
		{ OV511_I2C_BUS, 0x74, 0x00 },
		{ OV511_I2C_BUS, 0x75, 0x8e },
		{ OV511_I2C_BUS, 0x76, 0x00 },
		{ OV511_I2C_BUS, 0x77, 0xff },
		{ OV511_I2C_BUS, 0x78, 0x80 },
		{ OV511_I2C_BUS, 0x79, 0x80 },
		{ OV511_I2C_BUS, 0x7a, 0x80 },
		{ OV511_I2C_BUS, 0x7b, 0xe2 },
		{ OV511_I2C_BUS, 0x7c, 0x00 },
		{ OV511_DONE_BUS, 0x0, 0x00 },
	};

	fprintf(stderr, "starting configuration\n");
	/* This looks redundant, but is necessary for WebCam 3 */
	ov->primary_i2c_slave = OV7xx0_SID;
	if (ov51x_set_slave_ids(ov, OV7xx0_SID) < 0) {
		return -1;
	}
	if (init_ov_sensor(ov) >= 0) {
		fprintf(stderr, "OV7xx0 sensor initalized (method 1)\n");
	} else {
	/* Reset the 76xx */
		if (i2c_w(ov, 0x12, 0x80) < 0) {
			return -1;
		}

		/* Wait for it to initialize */
		usleep(150U);

		i = 0;
		success = 0;
		while (i <= I2C_DETECT_TRIES) {
			if ((i2c_r(ov, OV7610_REG_ID_HIGH) == 0x7F) &&
			    (i2c_r(ov, OV7610_REG_ID_LOW) == 0xA2)) {
				success = 1;
				break;
			} else {
				i++;
			}
		}

// Was (i == I2C_DETECT_TRIES) previously. This obviously used to always report
// success. Whether anyone actually depended on that bug is unknown
		if ((i >= I2C_DETECT_TRIES) && (success == 0)) {
			fprintf(stderr, "Failed to read sensor ID\n");
// Only issue a warning for now
//			return -1;
		} else {
			fprintf(stderr,
				"OV7xx0 initialized (method 2, %dx)\n", i + 1);
		}		
	}

	/* Detect sensor (sub)type */
	rc = i2c_r(ov, OV7610_REG_COM_I);
	if (rc < 0) {
		fprintf(stderr, "Error detecting sensor type\n");
		return -1;
	} else if ((rc & 3) == 3) {
		fprintf(stderr, "Sensor is an OV7610\n");
		ov->sensor = SEN_OV7610;
	} else if ((rc & 3) == 1) {
		/* I don't know what's different about the 76BE yet. */
		if (i2c_r(ov, 0x15) & 1) {
			fprintf(stderr, "Sensor is an OV7620AE\n");
		} else {
			fprintf(stderr, "Sensor is an OV76BE\n");
		}

		/* OV511+ will return all zero isoc data unless we
		 * configure the sensor as a 7620. Someone needs to
		 * find the exact reg. setting that causes this. */
		if (ov->bridge == BRG_OV511PLUS) {
			fprintf(stderr, "Enabling 511+/7620AE workaround\n");
			ov->sensor = SEN_OV7620;
		} else {
			ov->sensor = SEN_OV76BE;
		}
	} else if ((rc & 3) == 0) {
		fprintf(stderr, "Sensor is an OV7620\n");
		ov->sensor = SEN_OV7620;
	} else {
		fprintf(stderr, "Unknown image sensor version: %d\n", rc & 3);
		return -1;
	}
	if (ov->sensor == SEN_OV7620) {
		fprintf(stderr, "Writing 7620 registers\n");
		if (write_regvals(ov, aRegvalsNorm7620)) {
			return -1;
		}
	} else {
		fprintf(stderr, "Writing 7610 registers\n");
		if (write_regvals(ov, aRegvalsNorm7610)) {
			return -1;
		}
	}
	/* Set sensor-specific vars */
	ov->maxwidth = 640;
	ov->maxheight = 480;
	ov->minwidth = 64;
	ov->minheight = 48;

	// FIXME: These do not match the actual settings yet
	ov->brightness = 0x80 << 8;
	ov->contrast = 0x80 << 8;
	ov->colour = 0x80 << 8;
	ov->hue = 0x80 << 8;

	return 0;
}

/* OV518 quantization tables are 8x4 (instead of 8x8) */
int ov518_upload_quan_tables(struct usb_ov511 * const ov)
{
	unsigned char *pYTable = yQuanTable518;
	unsigned char *pUVTable = uvQuanTable518;
	unsigned char val0, val1;
	int i, rc, reg = R511_COMP_LUT_BEGIN;

	fprintf(stderr, "Uploading quantization tables\n");

	for (i = 0; i < OV518_QUANTABLESIZE / 2; i++) {
		if (ENABLE_Y_QUANTABLE) {
			val0 = *pYTable++;
			val1 = *pYTable++;
			val0 &= 0x0f;
			val1 &= 0x0f;
			val0 |= val1 << 4;
			rc = reg_w(ov, reg, val0);
			if (rc < 0) {
				return rc;
			}
		}

		if (ENABLE_UV_QUANTABLE) {
			val0 = *pUVTable++;
			val1 = *pUVTable++;
			val0 &= 0x0f;
			val1 &= 0x0f;
			val0 |= val1 << 4;
			rc = reg_w(ov, reg + OV518_QUANTABLESIZE/2, val0);
			if (rc < 0) {
				return rc;
			}
		}

		reg++;
	}

	return 0;
}

/* Upload compression params and quantization tables. Returns 0 for success. */
int ov518_init_compression(struct usb_ov511 * const ov)
{
	int rc = 0;

	if (!ov->compress_inited) {
		if (ov518_upload_quan_tables(ov) < 0) {
			fprintf(stderr,
				"Error uploading quantization tables\n");
			rc = -EIO;
			goto out;
		}
	}

	ov->compress_inited = 1;
out:
	return rc;
}

/* This initializes the OV518/OV518+ and the sensor */
int ov518_configure(struct usb_ov511 * const ov)
{
	/* For 518 and 518+ */
	static struct ov511_regvals aRegvalsInit518[] = {
		{ OV511_REG_BUS, R51x_SYS_RESET,	0x40 },
	 	{ OV511_REG_BUS, R51x_SYS_INIT,		0xe1 },
	 	{ OV511_REG_BUS, R51x_SYS_RESET,	0x3e },
		{ OV511_REG_BUS, R51x_SYS_INIT,		0xe1 },
		{ OV511_REG_BUS, R51x_SYS_RESET,	0x00 },
		{ OV511_REG_BUS, R51x_SYS_INIT,		0xe1 },
		{ OV511_REG_BUS, 0x46,			0x00 }, 
		{ OV511_REG_BUS, 0x5d,			0x03 },
		{ OV511_DONE_BUS, 0x0, 0x00},
	};

	static struct ov511_regvals aRegvalsNorm518[] = {
		{ OV511_REG_BUS, R51x_SYS_SNAP,		0x02 }, /* Reset */
		{ OV511_REG_BUS, R51x_SYS_SNAP,		0x01 }, /* Enable */
		{ OV511_REG_BUS, 0x31, 			0x0f },
		{ OV511_REG_BUS, 0x5d,			0x03 },
		{ OV511_REG_BUS, 0x24,			0x9f },
		{ OV511_REG_BUS, 0x25,			0x90 },
		{ OV511_REG_BUS, 0x20,			0x00 },
		{ OV511_REG_BUS, 0x51,			0x04 },
		{ OV511_REG_BUS, 0x71,			0x19 },
		{ OV511_DONE_BUS, 0x0, 0x00 },
	};

	static struct ov511_regvals aRegvalsNorm518Plus[] = {
		{ OV511_REG_BUS, R51x_SYS_SNAP,		0x02 }, /* Reset */
		{ OV511_REG_BUS, R51x_SYS_SNAP,		0x01 }, /* Enable */
		{ OV511_REG_BUS, 0x31, 			0x0f },
		{ OV511_REG_BUS, 0x5d,			0x03 },
		{ OV511_REG_BUS, 0x24,			0x9f },
		{ OV511_REG_BUS, 0x25,			0x90 },
		{ OV511_REG_BUS, 0x20,			0x60 },
		{ OV511_REG_BUS, 0x51,			0x02 },
		{ OV511_REG_BUS, 0x71,			0x19 },
		{ OV511_REG_BUS, 0x40,			0xff },
		{ OV511_REG_BUS, 0x41,			0x42 },
		{ OV511_REG_BUS, 0x46,			0x00 },
		{ OV511_REG_BUS, 0x33,			0x04 },
		{ OV511_REG_BUS, 0x21,			0x19 },
		{ OV511_REG_BUS, 0x3f,			0x10 },
		{ OV511_DONE_BUS, 0x0, 0x00 },
	};

	/* First 5 bits of custom ID reg are a revision ID on OV518 */
	fprintf(stderr, "Device revision %d\n",
		0x1F & reg_r(ov, R511_SYS_CUST_ID));

	/* Give it the default description */
	ov->desc = symbolic(camlist, 0);

	if (write_regvals(ov, aRegvalsInit518)) {
		goto error;
	}

	/* Set LED GPIO pin to output mode */
	if (reg_w_mask(ov, 0x57, 0x00, 0x02) < 0) {
		goto error;
	}

	/* LED is off by default with OV518; have to explicitly turn it on */
	if (ov->led_policy == LED_OFF || ov->led_policy == LED_AUTO) {
		ov51x_led_control(ov, 0);
	} else {
		ov51x_led_control(ov, 1);
	}

	/* Don't require compression if dumppix is enabled; otherwise it's
	 * required. OV518 has no uncompressed mode, to save RAM. */
	if (!dumppix && !ov->compress) {
		ov->compress = 1;
		fprintf(stderr, 
			"Compression required with OV518...enabling\n");
	}

	if (ov->bridge == BRG_OV518) {
		if (write_regvals(ov, aRegvalsNorm518)) {
			goto error;
		}
	} else if (ov->bridge == BRG_OV518PLUS) {
		if (write_regvals(ov, aRegvalsNorm518Plus)) {
			goto error;
		}
	} else {
		fprintf(stderr, "Invalid bridge\n");
	}

	if (reg_w(ov, 0x2f, 0x80) < 0) {
		goto error;
	}

	if (ov518_init_compression(ov)) {
		goto error;
	}

	if (ov->bridge == BRG_OV518) { /* XXX - TODO */		
#if 0
		struct usb_interface *ifp = ov->dev->config[0].interface[0];
		u_int16_t mxps = 
		ifp->altsetting[7].endpoint[0].desc.wMaxPacketSize;

		/* Some OV518s have packet numbering by default, some don't */
		if (mxps == 897) {
			ov->packet_numbering = 1;
		} else {
			ov->packet_numbering = 0;
		}
#else
		ov->packet_numbering = OV518_PACKET_NUMBERING;
#endif
	} else {
		/* OV518+ has packet numbering turned on by default */
		ov->packet_numbering = 1;
	}

	ov518_set_packet_size(ov, 0);

	ov->snap_enabled = snapshot;

	/* Test for 76xx */
	ov->primary_i2c_slave = OV7xx0_SID;
	if (ov51x_set_slave_ids(ov, OV7xx0_SID) < 0) {
		goto error;
	}

	/* The OV518 must be more aggressive about sensor detection since
	 * I2C write will never fail if the sensor is not present. We have
	 * to try to initialize the sensor to detect its presence */

	if (init_ov_sensor(ov) < 0) {
		/* Test for 6xx0 */
		ov->primary_i2c_slave = OV6xx0_SID;
		if (ov51x_set_slave_ids(ov, OV6xx0_SID) < 0) {
			goto error;
		}

		if (init_ov_sensor(ov) < 0) {
			/* Test for 8xx0 */
			ov->primary_i2c_slave = OV8xx0_SID;
			if (ov51x_set_slave_ids(ov, OV8xx0_SID) < 0) {
				goto error;
			}

			if (init_ov_sensor(ov) < 0) {
				fprintf(stderr,
					"Can't determine sensor slave IDs\n");
 				goto error;
			} else {
				fprintf(stderr,
					"Detected unsupported OV8xx0 sensor\n");
				goto error;
			}
		} else {
			if (ov6xx0_configure(ov) < 0) {
				fprintf(stderr, 
					"Failed to configure OV6xx0\n");
 				goto error;
			}
		}
	} else {
		if (ov7xx0_configure(ov) < 0) {
			fprintf(stderr, "Failed to configure OV7xx0\n");
	 		goto error;
		}
	}

	ov->maxwidth = 352;
	ov->maxheight = 288;

	// The OV518 cannot go as low as the sensor can
	ov->minwidth = 160;
	ov->minheight = 120;

	return 0;

error:
	fprintf(stderr, "OV518 Config failed\n");

	return -EBUSY;
}

int ov511_configure(struct usb_ov511 * const ov)
{
	int i, success;
	int rc;
	
	static struct ov511_regvals aRegvalsInit511[] = {
		{ OV511_REG_BUS, R51x_SYS_RESET,	0x7f },
	 	{ OV511_REG_BUS, R51x_SYS_INIT,		0x01 },
	 	{ OV511_REG_BUS, R51x_SYS_RESET,	0x7f },
		{ OV511_REG_BUS, R51x_SYS_INIT,		0x01 },
		{ OV511_REG_BUS, R51x_SYS_RESET,	0x3f },
		{ OV511_REG_BUS, R51x_SYS_INIT,		0x01 },
		{ OV511_REG_BUS, R51x_SYS_RESET,	0x3d },
		{ OV511_DONE_BUS, 0x0, 0x00},
	};

	static struct ov511_regvals aRegvalsNorm511[] = {
		{ OV511_REG_BUS, R511_DRAM_FLOW_CTL, 	0x01 },
		{ OV511_REG_BUS, R51x_SYS_SNAP,		0x00 },
		{ OV511_REG_BUS, R51x_SYS_SNAP,		0x02 },
		{ OV511_REG_BUS, R51x_SYS_SNAP,		0x00 },
		{ OV511_REG_BUS, R511_FIFO_OPTS,	0x1f },
		{ OV511_REG_BUS, R511_COMP_EN,		0x00 },
		{ OV511_REG_BUS, R511_COMP_LUT_EN,	0x03 },
		{ OV511_DONE_BUS, 0x0, 0x00 },
	};

	static struct ov511_regvals aRegvalsNorm511Plus[] = {
		{ OV511_REG_BUS, R511_DRAM_FLOW_CTL,	0xff },
		{ OV511_REG_BUS, R51x_SYS_SNAP,		0x00 },
		{ OV511_REG_BUS, R51x_SYS_SNAP,		0x02 },
		{ OV511_REG_BUS, R51x_SYS_SNAP,		0x00 },
		{ OV511_REG_BUS, R511_FIFO_OPTS,	0xff },
		{ OV511_REG_BUS, R511_COMP_EN,		0x00 },
		{ OV511_REG_BUS, R511_COMP_LUT_EN,	0x03 },
		{ OV511_DONE_BUS, 0x0, 0x00 },
	};

	ov->customid = reg_r(ov, R511_SYS_CUST_ID);
	if (ov->customid < 0) {
		fprintf(stderr, "Unable to read camera bridge registers\n");
		goto error;
	}	
	fprintf(stderr, "CustomID = %d\n", ov->customid);
	ov->desc = symbolic(camlist, ov->customid);
	fprintf(stderr, "model: %s\n", ov->desc);
	if (strcmp(ov->desc, NOT_DEFINED_STR) == 0) {
		fprintf(stderr, "Camera type (%d) not recognized\n",
			ov->customid);
	}
	if (ov->customid == 70) {		/* USB Life TV (PAL/SECAM) */
		ov->pal = 1;
	}
	if (write_regvals(ov, aRegvalsInit511)) {
		goto error;
	}
	if (ov->led_policy == LED_OFF || ov->led_policy == LED_AUTO) {
		ov51x_led_control(ov, 0);
	}
	/* The OV511+ has undocumented bits in the flow control register.
	 * Setting it to 0xff fixes the corruption with moving objects. */
	if (ov->bridge == BRG_OV511) {
		if (write_regvals(ov, aRegvalsNorm511)) {
			goto error;
		}
	} else if (ov->bridge == BRG_OV511PLUS) {
		if (write_regvals(ov, aRegvalsNorm511Plus)) {
			goto error;
		}
	} else {
		fprintf(stderr, "Invalid bridge\n");
	}
	if (ov511_init_compression(ov)) {
		goto error;
	}

	ov->packet_numbering = 1;
	ov511_set_packet_size(ov, 0);

	ov->snap_enabled = snapshot;	
	
	fprintf(stderr, "Testing for 0V7xx0\n");
	ov->primary_i2c_slave = OV7xx0_SID;
	if (ov51x_set_slave_ids(ov, OV7xx0_SID) < 0) {
		goto error;
	}
	if (i2c_w(ov, 0x12, 0x80) < 0) {
		/* Test for 6xx0 */
		fprintf(stderr, "Testing for 0V6xx0\n");
		ov->primary_i2c_slave = OV6xx0_SID;
		if (ov51x_set_slave_ids(ov, OV6xx0_SID) < 0)
			goto error;

		if (i2c_w(ov, 0x12, 0x80) < 0) {
			/* Test for 8xx0 */
			fprintf(stderr, "Testing for 0V8xx0\n");
			ov->primary_i2c_slave = OV8xx0_SID;
			if (ov51x_set_slave_ids(ov, OV8xx0_SID) < 0)
				goto error;

			if (i2c_w(ov, 0x12, 0x80) < 0) {
				/* Test for SAA7111A */
				fprintf(stderr, "Testing for SAA7111A\n");
				ov->primary_i2c_slave = SAA7111A_SID;
				if (ov51x_set_slave_ids(ov, SAA7111A_SID) < 0)
					goto error;

				if (i2c_w(ov, 0x0d, 0x00) < 0) {
					/* Test for KS0127 */
					fprintf(stderr, "Testing for KS0127\n");
					ov->primary_i2c_slave = KS0127_SID;
					if (ov51x_set_slave_ids(ov, KS0127_SID) < 0)
						goto error;

					if (i2c_w(ov, 0x10, 0x00) < 0) {
						fprintf(stderr, "Can't determine sensor slave IDs\n");
		 				goto error;
					} else {
						if (ks0127_configure(ov) < 0) {
							fprintf(stderr, "Failed to configure KS0127\n");
	 						goto error;
						}
					}
				} else {
					if (saa7111a_configure(ov) < 0) {
						fprintf(stderr, "Failed to configure SAA7111A\n");
	 					goto error;
					}
				}
			} else {
				fprintf(stderr, "Detected unsupported OV8xx0 sensor\n");
				goto error;
			}
		} else {
			if (ov6xx0_configure(ov) < 0) {
				fprintf(stderr, "Failed to configure OV6xx0\n");
 				goto error;
			}
		}
	} else {
		if (ov7xx0_configure(ov) < 0) {
			fprintf(stderr, "Failed to configure OV7xx0\n");
	 		goto error;
		}
	}
	
	return 0;
	
	error:
	fprintf(stderr, "OV511 Config failed\n");
	
	return -EBUSY;
}

int ov511_probe(struct usb_ov511 * const ov)
{
	struct usb_device_info udi;	/* device info */
	const size_t sizeof_dev = 42;
	
	if ((ov->dev = malloc(sizeof_dev)) == NULL) {
		fprintf(stderr, "Out of memory\n");
		return -1;
	}
#ifdef MODERN_USB_DEVICES
	snprintf(ov->dev, sizeof_dev, "/dev/ugen%d.00", usbgen_id);
#else
	snprintf(ov->dev, sizeof_dev, "/dev/ugen%d", usbgen_id);	
#endif
	if ((ov->fd = open(ov->dev, O_RDWR)) == -1) {
		perror(ov->dev);
		return -1;
	}
	if (ioctl(ov->fd, USB_GET_DEVICEINFO, &udi) != 0) {
	        perror("Unable to get device info");
		return -1;
	}
	ov->led_policy = led;
	ov->compress = compress;
	ov->lightfreq = lightfreq; /* 50 or 60 Hz */
	ov->num_inputs = cams;
	ov->stop_during_set = 0;
	ov->backlight = backlight;
	ov->mirror = mirror;
	ov->auto_brt = autobright;
	ov->auto_gain = autogain;
	ov->auto_exp = autoexp;
	ov->decomp_ops = NULL;
	
#ifdef MODERN_USB_CODE
	switch (udi.udi_productNo)
#else
	switch (udi.productNo)
#endif	
	{
	case PROD_OV511:
		ov->bridge = BRG_OV511;
		ov->bclass = BCL_OV511;
		break;
	case PROD_OV511PLUS:
		ov->bridge = BRG_OV511PLUS;
		ov->bclass = BCL_OV511;
		break;
	case PROD_OV518:
		ov->bridge = BRG_OV518;
		ov->bclass = BCL_OV518;
		break;
	case PROD_OV518PLUS:
		ov->bridge = BRG_OV518PLUS;
		ov->bclass = BCL_OV518;
		break;
	case PROD_ME2CAM:
#ifdef MODERN_USB_CODE
		if (udi.udi_vendorNo != VEND_MATTEL) {
			goto error;
		}
#else
		if (udi.vendorNo != VEND_MATTEL) {
			goto error;
		}		
#endif
		ov->bridge = BRG_OV511PLUS;
		ov->bclass = BCL_OV511;
		break;
	default:
#ifdef MODERN_USB_CODE
		fprintf(stderr, "Unknown product ID 0x%04x\n",
			udi.udi_productNo);
#else
		fprintf(stderr, "Unknown product ID 0x%04x\n",
			udi.productNo);		
#endif
		goto error;
	}
	fprintf(stderr, "USB %s video device found\n",
		symbolic(brglist, ov->bridge));
	if (ov->bclass == BCL_OV518) {
		if (ov518_configure(ov) < 0) {
			goto error;
		}
	} else {
		if (ov511_configure(ov) < 0) {
			goto error;
		}
	}
	
	return 0;
	
	error:
	fprintf(stderr, "No supported camera detected\n");
	
	return -EIO;
}

int ov511_mode_init_regs(struct usb_ov511 * const ov,
			 int width, int height, int mode, int sub_flag)
{
	int hsegs, vsegs;

	if (sub_flag) {
		width = ov->subw;
		height = ov->subh;
	}

	fprintf(stderr, "width:%d, height:%d, mode:%d, sub:%d\n",
		width, height, mode, sub_flag);

	// FIXME: This should be moved to a 7111a-specific function once
	// subcapture is dealt with properly
	if (ov->sensor == SEN_SAA7111A) {
		if (width == 320 && height == 240) {
			/* No need to do anything special */
		} else if (width == 640 && height == 480) {
			/* Set the OV511 up as 320x480, but keep the
			 * V4L resolution as 640x480 */
			width = 320;
		} else {
			fprintf(stderr,
				"SAA7111A only allows 320x240 or 640x480\n");
			return -EINVAL;
		}
	}

	/* Make sure width and height are a multiple of 8 */
	if (width % 8 || height % 8) {
		fprintf(stderr, "Invalid size (%d, %d) (mode = %d)", 
			width, height, mode);
		return -EINVAL;
	}

	if (width < ov->minwidth || height < ov->minheight) {
		fprintf(stderr, "Requested dimensions are too small\n");
		return -EINVAL;
	}

	if (ov51x_stop(ov) < 0) {
		return -EIO;
	}

	reg_w(ov, R511_CAM_UV_EN, 0x01);
		reg_w(ov, R511_SNAP_UV_EN, 0x01);
	reg_w(ov, R511_SNAP_OPTS, 0x03);

	/* Here I'm assuming that snapshot size == image size.
	 * I hope that's always true. --claudio
	 */
	hsegs = (width >> 3) - 1;
	vsegs = (height >> 3) - 1;

	reg_w(ov, R511_CAM_PXCNT, hsegs);
	reg_w(ov, R511_CAM_LNCNT, vsegs);
	reg_w(ov, R511_CAM_PXDIV, 0x00);
	reg_w(ov, R511_CAM_LNDIV, 0x00);

	/* YUV420, low pass filter on */
	reg_w(ov, R511_CAM_OPTS, 0x03);

	/* Snapshot additions */
	reg_w(ov, R511_SNAP_PXCNT, hsegs);
	reg_w(ov, R511_SNAP_LNCNT, vsegs);
	reg_w(ov, R511_SNAP_PXDIV, 0x00);
	reg_w(ov, R511_SNAP_LNDIV, 0x00);

	if (ov->compress) {
		/* Enable Y and UV quantization and compression */
		reg_w(ov, R511_COMP_EN, 0x07);
		reg_w(ov, R511_COMP_LUT_EN, 0x03);
		ov51x_reset(ov, OV511_RESET_OMNICE);
	}

	if (ov51x_restart(ov) < 0) {
		return -EIO;
	}

	return 0;
}

int ov518_mode_init_regs(struct usb_ov511 * const ov,
			 int width, int height, int mode, int sub_flag)
{
	int hsegs, vsegs, hi_res;

	if (sub_flag) {
		width = ov->subw;
		height = ov->subh;
	}

	if (width % 16 || height % 8) {
		fprintf(stderr, "Invalid size (%d, %d)\n", width, height);
		return -EINVAL;
	}

	if (width < ov->minwidth || height < ov->minheight) {
		fprintf(stderr, "Requested dimensions are too small\n");
		return -EINVAL;
	}

	if (width >= 320 && height >= 240) {
		hi_res = 1;
	} else if (width >= 320 || height >= 240) {
		fprintf(stderr, 
			"Invalid width/height combination (%d, %d)\n", 
			width, height);
		return -EINVAL;
	} else {
		hi_res = 0;
	}

	if (ov51x_stop(ov) < 0) {
		return -EIO;
	}

	/******** Set the mode ********/

	reg_w(ov, 0x2b, 0);
	reg_w(ov, 0x2c, 0);
	reg_w(ov, 0x2d, 0);
	reg_w(ov, 0x2e, 0);
	reg_w(ov, 0x3b, 0);
	reg_w(ov, 0x3c, 0);
	reg_w(ov, 0x3d, 0);
	reg_w(ov, 0x3e, 0);

	if (ov->bridge == BRG_OV518 && ov518_color) {
		/* OV518 needs U and V swapped */
		i2c_w_mask(ov, 0x15, 0x00, 0x01);
		
		/* Set 8-bit (YVYU) input format */
		reg_w_mask(ov, 0x20, 0x08, 0x08);
		
		/* Set 12-bit (4:2:0) output format */
		reg_w_mask(ov, 0x28, 0x80, 0xf0);
		reg_w_mask(ov, 0x38, 0x80, 0xf0);
	} else {
		reg_w(ov, 0x28, 0x80);
		reg_w(ov, 0x38, 0x80);
	}

	hsegs = width / 16;
	vsegs = height / 4;

	reg_w(ov, 0x29, hsegs);
	reg_w(ov, 0x2a, vsegs);

	reg_w(ov, 0x39, hsegs);
	reg_w(ov, 0x3a, vsegs);

	/* Windows driver does this here; who knows why */
	reg_w(ov, 0x2f, 0x80);

	/******** Set the framerate (to 15 FPS) ********/

	/* Mode independent, but framerate dependent, regs */
	reg_w(ov, 0x51, 0x02);	/* Clock divider; lower==faster */
	reg_w(ov, 0x22, 0x18);
	reg_w(ov, 0x23, 0xff);

	if (ov->bridge == BRG_OV518PLUS) {
		reg_w(ov, 0x21, 0x19);
	} else {
		reg_w(ov, 0x71, 0x19);	/* Compression-related? */
	}

	// FIXME: Sensor-specific
	/* Bit 5 is what matters here. Of course, it is "reserved" */
	i2c_w(ov, 0x54, 0x23);

	reg_w(ov, 0x2f, 0x80);

	if (ov->bridge == BRG_OV518PLUS) {
		reg_w(ov, 0x24, 0x94);
		reg_w(ov, 0x25, 0x90);
		ov518_reg_w32(ov, 0xc4,    400, 2);	/* 190h   */
		ov518_reg_w32(ov, 0xc6,    540, 2);	/* 21ch   */
		ov518_reg_w32(ov, 0xc7,    540, 2);	/* 21ch   */
		ov518_reg_w32(ov, 0xc8,    108, 2);	/* 6ch    */
		ov518_reg_w32(ov, 0xca, 131098, 3);	/* 2001ah */
		ov518_reg_w32(ov, 0xcb,    532, 2);	/* 214h   */
		ov518_reg_w32(ov, 0xcc,   2400, 2);	/* 960h   */
		ov518_reg_w32(ov, 0xcd,     32, 2);	/* 20h    */
		ov518_reg_w32(ov, 0xce,    608, 2);	/* 260h   */
	} else {
		reg_w(ov, 0x24, 0x9f);
		reg_w(ov, 0x25, 0x90);
		ov518_reg_w32(ov, 0xc4,    400, 2);	/* 190h   */
		ov518_reg_w32(ov, 0xc6,    500, 2);	/* 1f4h   */
		ov518_reg_w32(ov, 0xc7,    500, 2);	/* 1f4h   */
		ov518_reg_w32(ov, 0xc8,    142, 2);	/* 8eh    */
		ov518_reg_w32(ov, 0xca, 131098, 3);	/* 2001ah */
		ov518_reg_w32(ov, 0xcb,    532, 2);	/* 214h   */
		ov518_reg_w32(ov, 0xcc,   2000, 2);	/* 7d0h   */
		ov518_reg_w32(ov, 0xcd,     32, 2);	/* 20h    */
		ov518_reg_w32(ov, 0xce,    608, 2);	/* 260h   */
	}

	reg_w(ov, 0x2f, 0x80);

	if (ov51x_restart(ov) < 0) {
		return -EIO;
	}

	/* Reset it just for good measure */
	if (ov51x_reset(ov, OV511_RESET_NOREGS) < 0) {
		return -EIO;
	}

	return 0;
}

static int
mode_init_ov_sensor_regs(struct usb_ov511 *ov, int width, int height,
			 int mode, int sub_flag, int qvga)
{
	int clock;

	/******** Mode (VGA/QVGA) and sensor specific regs ********/

	switch (ov->sensor) {
	case SEN_OV7610:
		i2c_w(ov, 0x14, qvga?0x24:0x04);
// FIXME: Does this improve the image quality or frame rate?
#if 0
		i2c_w_mask(ov, 0x28, qvga?0x00:0x20, 0x20);
		i2c_w(ov, 0x24, 0x10);
		i2c_w(ov, 0x25, qvga?0x40:0x8a);
		i2c_w(ov, 0x2f, qvga?0x30:0xb0);
		i2c_w(ov, 0x35, qvga?0x1c:0x9c);
#endif
		break;
	case SEN_OV7620:
//		i2c_w(ov, 0x2b, 0x00);
		i2c_w(ov, 0x14, qvga?0xa4:0x84);
		i2c_w_mask(ov, 0x28, qvga?0x00:0x20, 0x20);
		i2c_w(ov, 0x24, qvga?0x20:0x3a);
		i2c_w(ov, 0x25, qvga?0x30:0x60);
		i2c_w_mask(ov, 0x2d, qvga?0x40:0x00, 0x40);
		i2c_w_mask(ov, 0x67, qvga?0xf0:0x90, 0xf0);
		i2c_w_mask(ov, 0x74, qvga?0x20:0x00, 0x20);
		break;
	case SEN_OV76BE:
//		i2c_w(ov, 0x2b, 0x00);
		i2c_w(ov, 0x14, qvga?0xa4:0x84);
// FIXME: Enable this once 7620AE uses 7620 initial settings
#if 0
		i2c_w_mask(ov, 0x28, qvga?0x00:0x20, 0x20);
		i2c_w(ov, 0x24, qvga?0x20:0x3a);
		i2c_w(ov, 0x25, qvga?0x30:0x60);
		i2c_w_mask(ov, 0x2d, qvga?0x40:0x00, 0x40);
		i2c_w_mask(ov, 0x67, qvga?0xb0:0x90, 0xf0);
		i2c_w_mask(ov, 0x74, qvga?0x20:0x00, 0x20);
#endif
		break;
	case SEN_OV6620:
		i2c_w(ov, 0x14, qvga?0x24:0x04);
		break;
	case SEN_OV6630:
		i2c_w(ov, 0x14, qvga?0xa0:0x80);
		break;
	default:
		fprintf(stderr, "Invalid sensor\n");
		return -EINVAL;
	}

	/******** Palette-specific regs ********/

	
	if (ov->sensor == SEN_OV7610 || ov->sensor == SEN_OV76BE) {
		/* not valid on the OV6620/OV7620/6630? */
		i2c_w_mask(ov, 0x0e, 0x00, 0x40);
	}
	
	/* The OV518 needs special treatment. Although both the OV518
	 * and the OV6630 support a 16-bit video bus, only the 8 bit Y
	 * bus is actually used. The UV bus is tied to ground.
	 * Therefore, the OV6630 needs to be in 8-bit multiplexed
	 * output mode */
	
	if (ov->sensor == SEN_OV6630 && ov->bridge == BRG_OV518
	    && ov518_color) {
		i2c_w_mask(ov, 0x12, 0x10, 0x10);
		i2c_w_mask(ov, 0x13, 0x20, 0x20);
	} else {
		i2c_w_mask(ov, 0x13, 0x00, 0x20);
	}
	
	/******** Clock programming ********/

	/* The OV6620 needs special handling. This prevents the 
	 * severe banding that normally occurs */
	if (ov->sensor == SEN_OV6620 || ov->sensor == SEN_OV6630)
	{
		/* Clock down */

		i2c_w(ov, 0x2a, 0x04);

		if (ov->compress) {
//			clock = 0;    /* This ensures the highest frame rate */
			clock = 3;
		} else if (clockdiv == -1) {   /* If user didn't override it */
			clock = 3;    /* Gives better exposure time */
		} else {
			clock = clockdiv;
		}

		fprintf(stderr, "Setting clock divisor to %d\n", clock);

		i2c_w(ov, 0x11, clock);

		i2c_w(ov, 0x2a, 0x84);
		/* This next setting is critical. It seems to improve
		 * the gain or the contrast. The "reserved" bits seem
		 * to have some effect in this case. */
		i2c_w(ov, 0x2d, 0x85);
	} else {
		if (ov->compress) {
			clock = 1;    /* This ensures the highest frame rate */
		} else if (clockdiv == -1) {   /* If user didn't override it */
			/* Calculate and set the clock divisor */
			clock = ((sub_flag ? ov->subw * ov->subh
				  : width * height)
				 * 3 / 2)
				 / 66000;
		} else {
			clock = clockdiv;
		}

		fprintf(stderr, "Setting clock divisor to %d\n", clock);

		i2c_w(ov, 0x11, clock);
	}

	/******** Special Features ********/

	if (framedrop >= 0) {
		i2c_w(ov, 0x16, framedrop);
	}

	/* Test Pattern */
	i2c_w_mask(ov, 0x12, (testpat?0x02:0x00), 0x02);

	/* Enable auto white balance */
	i2c_w_mask(ov, 0x12, 0x04, 0x04);

	// This will go away as soon as ov51x_mode_init_sensor_regs()
	// is fully tested.
	/* 7620/6620/6630? don't have register 0x35, so play it safe */
	if (ov->sensor == SEN_OV7610 || ov->sensor == SEN_OV76BE) {
		if (width == 640 && height == 480) {
			i2c_w(ov, 0x35, 0x9e);
		} else {
			i2c_w(ov, 0x35, 0x1e);
		}
	}

	return 0;
}

static inline long int get_frame_length(const struct ov511_frame * const frame)
{
	if (!frame) {
		return 0;
	} else {
		return (frame->width * frame->height * (12)) >> 3;
	}
}


int set_ov_sensor_window(struct usb_ov511 * const ov,
			 int width, int height, int mode, int sub_flag)
{
	int ret;
	int hwsbase, hwebase, vwsbase, vwebase, hwsize, vwsize; 
	int hoffset, voffset, hwscale = 0, vwscale = 0;

	/* The different sensor ICs handle setting up of window differently.
	 * IF YOU SET IT WRONG, YOU WILL GET ALL ZERO ISOC DATA FROM OV51x!!! */
	switch (ov->sensor) {
	case SEN_OV7610:
	case SEN_OV76BE:
		hwsbase = 0x38;
		hwebase = 0x3a;
		vwsbase = vwebase = 0x05;
		break;
	case SEN_OV6620:
	case SEN_OV6630:
		hwsbase = 0x38;
		hwebase = 0x3a;
		vwsbase = 0x05;
		vwebase = 0x06;
		break;
	case SEN_OV7620:
		hwsbase = 0x2f;		/* From 7620.SET (spec is wrong) */
		hwebase = 0x2f;
		vwsbase = vwebase = 0x05;
		break;
	default:
		fprintf(stderr, "Invalid sensor\n");
		return -EINVAL;
	}

	if (ov->sensor == SEN_OV6620 || ov->sensor == SEN_OV6630) {
		/* Note: OV518(+) does downsample on its own) */
		if ((width > 176 && height > 144)
		    || ov->bclass == BCL_OV518) {  /* CIF */
			ret = mode_init_ov_sensor_regs(ov, width, height,
				mode, sub_flag, 0);
			if (ret < 0) {
				return ret;
			}
			hwscale = 1;
			vwscale = 1;  /* The datasheet says 0; it's wrong */
			hwsize = 352;
			vwsize = 288;
		} else if (width > 176 || height > 144) {
			fprintf(stderr, "Illegal dimensions\n");
			return -EINVAL;
		} else {			    /* QCIF */
			ret = mode_init_ov_sensor_regs(ov, width, height,
				mode, sub_flag, 1);
			if (ret < 0) {
				return ret;
			}
			hwsize = 176;
			vwsize = 144;
		}
	} else {
		if (width > 320 && height > 240) {  /* VGA */
			ret = mode_init_ov_sensor_regs(ov, width, height,
				mode, sub_flag, 0);
			if (ret < 0)
				return ret;
			hwscale = 2;
			vwscale = 1;
			hwsize = 640;
			vwsize = 480;
		} else if (width > 320 || height > 240) {
			fprintf(stderr, "Illegal dimensions\n");
			return -EINVAL;
		} else {			    /* QVGA */
			ret = mode_init_ov_sensor_regs(ov, width, height,
				mode, sub_flag, 1);
			if (ret < 0) {
				return ret;
			}
			hwscale = 1;
			hwsize = 320;
			vwsize = 240;
		}
	}

	/* Center the window */
	hoffset = ((hwsize - width) / 2) >> hwscale;
	voffset = ((vwsize - height) / 2) >> vwscale;

	/* FIXME! - This needs to be changed to support 160x120 and 6620!!! */
	if (sub_flag) {
		i2c_w(ov, 0x17, hwsbase+(ov->subx>>hwscale));
		i2c_w(ov, 0x18,	hwebase+((ov->subx+ov->subw)>>hwscale));
		i2c_w(ov, 0x19, vwsbase+(ov->suby>>vwscale));
		i2c_w(ov, 0x1a, vwebase+((ov->suby+ov->subh)>>vwscale));
	} else {
		i2c_w(ov, 0x17, hwsbase + hoffset);
		i2c_w(ov, 0x18, hwebase + hoffset + (hwsize>>hwscale));
		i2c_w(ov, 0x19, vwsbase + voffset);
		i2c_w(ov, 0x1a, vwebase + voffset + (vwsize>>vwscale));
	}

	return 0;
}

/* If enable is true, turn on the sensor's auto brightness control, otherwise
 * turn it off.
 *
 * Unsupported: KS0127, KS0127B, SAA7111A
 * Returns: 0 for success
 */

int sensor_set_auto_brightness(struct usb_ov511 * const ov, const int enable)
{
	int rc;

	fprintf(stderr, "auto_brightness (%s)\n",
		enable ? "turn on" : "turn off");

	if (ov->sensor == SEN_KS0127 || ov->sensor == SEN_KS0127B
		|| ov->sensor == SEN_SAA7111A) {
		fprintf(stderr, "Unsupported with this sensor\n");
		return -EPERM;
	}

	rc = i2c_w_mask(ov, 0x2d, enable ? 0x10 : 0x00, 0x10);
	if (rc < 0) {
		return rc;
	}
	ov->auto_brt = enable;

	return 0;
}

/* If enable is true, turn on the sensor's auto exposure control, otherwise
 * turn it off.
 *
 * Unsupported: KS0127, KS0127B, SAA7111A
 * Returns: 0 for success
 */

int sensor_set_auto_exposure(struct usb_ov511 * const ov, const int enable)
{
	fprintf(stderr, "set_auto_exposure (%s)\n",
		enable ? "turn on" : "turn off");

	switch (ov->sensor) {
	case SEN_OV7610:
		i2c_w_mask(ov, 0x29, enable?0x00:0x80, 0x80);
		break;
	case SEN_OV6620:
	case SEN_OV7620:
	case SEN_OV76BE:
	case SEN_OV8600:
		i2c_w_mask(ov, 0x13, enable?0x01:0x00, 0x01);
		break;
	case SEN_OV6630:
		i2c_w_mask(ov, 0x28, enable?0x00:0x10, 0x10);
		break;
	case SEN_KS0127:
	case SEN_KS0127B:
	case SEN_SAA7111A:
		fprintf(stderr, "Unsupported with this sensor\n");
		return -EPERM;
	default:
		fprintf(stderr, "Sensor not supported for set_auto_exposure\n");
		return -EINVAL;
	}

	ov->auto_exp = enable;

	return 0;
}

/* Modifies the sensor's exposure algorithm to allow proper exposure of objects
 * that are illuminated from behind.
 *
 * Tested with: OV6620, OV7620
 * Unsupported: OV7610, OV76BE, KS0127, KS0127B, SAA7111A
 * Returns: 0 for success
 */
int sensor_set_backlight(struct usb_ov511 * const ov, const int enable)
{
	fprintf(stderr, "set_backlight: (%s)\n",
		enable ? "turn on" : "turn off");

	switch (ov->sensor) {
	case SEN_OV7620:
	case SEN_OV8600:
		i2c_w_mask(ov, 0x68, enable?0xe0:0xc0, 0xe0);
		i2c_w_mask(ov, 0x29, enable?0x08:0x00, 0x08);
		i2c_w_mask(ov, 0x28, enable?0x02:0x00, 0x02);
		break;
	case SEN_OV6620:
		i2c_w_mask(ov, 0x4e, enable?0xe0:0xc0, 0xe0);
		i2c_w_mask(ov, 0x29, enable?0x08:0x00, 0x08);
		i2c_w_mask(ov, 0x0e, enable?0x80:0x00, 0x80);
		break;
	case SEN_OV6630:
		i2c_w_mask(ov, 0x4e, enable?0x80:0x60, 0xe0);
		i2c_w_mask(ov, 0x29, enable?0x08:0x00, 0x08);
		i2c_w_mask(ov, 0x28, enable?0x02:0x00, 0x02);
		break;
	case SEN_OV7610:
	case SEN_OV76BE:
	case SEN_KS0127:
	case SEN_KS0127B:
	case SEN_SAA7111A:
		fprintf(stderr, "Unsupported with this sensor\n");
		return -EPERM;
	default:
		fprintf(stderr, "Sensor not supported for set_backlight\n");
		return -EINVAL;
	}

	ov->backlight = enable;

	return 0;
}

int sensor_set_mirror(struct usb_ov511 *ov, int enable)
{
	fprintf(stderr, "set_mirror (%s)\n", enable ? "turn on" : "turn off");

	switch (ov->sensor) {
	case SEN_OV6620:
	case SEN_OV6630:
	case SEN_OV7610:
	case SEN_OV7620:
	case SEN_OV76BE:
	case SEN_OV8600:
		i2c_w_mask(ov, 0x12, enable?0x40:0x00, 0x40);
		break;
	case SEN_KS0127:
	case SEN_KS0127B:
	case SEN_SAA7111A:
		fprintf(stderr, "Unsupported with this sensor\n");
		return -EPERM;
	default:
		fprintf(stderr, "Sensor not supported for set_mirror\n");
		return -EINVAL;
	}

	ov->mirror = enable;

	return 0;
}

/* If enable is true, turn on the sensor's banding filter, otherwise turn it
 * off. This filter tries to reduce the pattern of horizontal light/dark bands
 * caused by some (usually fluorescent) lighting. The light frequency must be
 * set either before or after enabling it with ov51x_set_light_freq().
 *
 * Tested with: OV7610, OV7620, OV76BE, OV6620.
 * Unsupported: KS0127, KS0127B, SAA7111A
 * Returns: 0 for success
 */
int sensor_set_banding_filter(struct usb_ov511 * const ov, const int enable)
{
	int rc;

	fprintf(stderr, "banding_filter (%s)\n",
		enable ? "turn on" : "turn off");

	if (ov->sensor == SEN_KS0127 || ov->sensor == SEN_KS0127B
		|| ov->sensor == SEN_SAA7111A) {
		fprintf(stderr, "Unsupported with this sensor\n");
		return -EPERM;
	}

	rc = i2c_w_mask(ov, 0x2d, enable?0x04:0x00, 0x04);
	if (rc < 0) {
		return rc;
	}

	ov->bandfilt = enable;

	return 0;
}

/* Matches the sensor's internal frame rate to the lighting frequency.
 * Valid frequencies are:
 *	50 - 50Hz, for European and Asian lighting
 *	60 - 60Hz, for American lighting
 *
 * Tested with: OV7610, OV7620, OV76BE, OV6620
 * Unsupported: KS0127, KS0127B, SAA7111A
 * Returns: 0 for success
 */
int sensor_set_light_freq(struct usb_ov511 * const ov, const int freq)
{
	int sixty;

	fprintf(stderr, "light_freq %d Hz\n", freq);

	if (freq == 60) {
		sixty = 1;
	} else if (freq == 50) {
		sixty = 0;
	} else {
		fprintf(stderr, "Invalid light freq (%d Hz)\n", freq);
		return -EINVAL;
	}

	switch (ov->sensor) {
	case SEN_OV7610:
		i2c_w_mask(ov, 0x2a, sixty?0x00:0x80, 0x80);
		i2c_w(ov, 0x2b, sixty?0x00:0xac);
		i2c_w_mask(ov, 0x13, 0x10, 0x10);
		i2c_w_mask(ov, 0x13, 0x00, 0x10);
		break;
	case SEN_OV7620:
	case SEN_OV76BE:
	case SEN_OV8600:
		i2c_w_mask(ov, 0x2a, sixty?0x00:0x80, 0x80);
		i2c_w(ov, 0x2b, sixty?0x00:0xac);
		i2c_w_mask(ov, 0x76, 0x01, 0x01);
		break;
	case SEN_OV6620:
	case SEN_OV6630:
		i2c_w(ov, 0x2b, sixty?0xa8:0x28);
		i2c_w(ov, 0x2a, sixty?0x84:0xa4);
		break;
	case SEN_KS0127:
	case SEN_KS0127B:
	case SEN_SAA7111A:
		fprintf(stderr, "Unsupported with this sensor\n");
		return -EPERM;
	default:
		fprintf(stderr, "Sensor not supported for set_light_freq\n");
		return -EINVAL;
	}

	ov->lightfreq = freq;

	return 0;
}

/* This is a wrapper around the OV511, OV518, and sensor specific functions */
int mode_init_regs(struct usb_ov511 * const ov,
		   int width, int height, int mode, int sub_flag)
{
	int rc = 0;

	if (!ov || !ov->dev) {
		return -EFAULT;
	}

	if (ov->bclass == BCL_OV518) {
		rc = ov518_mode_init_regs(ov, width, height, mode, sub_flag);
	} else {
		rc = ov511_mode_init_regs(ov, width, height, mode, sub_flag);
	}

	if (FATAL_ERROR(rc)) {
		return rc;
	}

	switch (ov->sensor) {
	case SEN_OV7610:
	case SEN_OV7620:
	case SEN_OV76BE:
	case SEN_OV8600:
	case SEN_OV6620:
	case SEN_OV6630:
		rc = set_ov_sensor_window(ov, width, height, mode, sub_flag);
		break;
	case SEN_KS0127:
	case SEN_KS0127B:
		fprintf(stderr, "KS0127-series decoders not supported yet\n");
		rc = -EINVAL;
		break;
	case SEN_SAA7111A:
//		rc = mode_init_saa_sensor_regs(ov, width, height, mode,
//					       sub_flag);

		fprintf(stderr, "SAA status = 0x%02X\n", i2c_r(ov, 0x1f));
		break;
	default:
		fprintf(stderr, "Unknown sensor\n");
		rc = -EINVAL;
	}

	if (FATAL_ERROR(rc)) {
		return rc;
	}

	/* Sensor-independent settings */
	rc = sensor_set_auto_brightness(ov, ov->auto_brt);
	if (FATAL_ERROR(rc)) {
		return rc;
	}

	rc = sensor_set_auto_exposure(ov, ov->auto_exp);
	if (FATAL_ERROR(rc)) {
		return rc;
	}

	rc = sensor_set_banding_filter(ov, bandingfilter);
	if (FATAL_ERROR(rc)) {
		return rc;
	}

	if (ov->lightfreq) {
		rc = sensor_set_light_freq(ov, lightfreq);
		if (FATAL_ERROR(rc)) {
			return rc;
		}
	}

	rc = sensor_set_backlight(ov, ov->backlight);
	if (FATAL_ERROR(rc)) {
		return rc;
	}

	rc = sensor_set_mirror(ov, ov->mirror);
	if (FATAL_ERROR(rc)) {
		return rc;
	}

	return 0;
}

int ov51x_set_default_params(struct usb_ov511 * const ov)
{
	int i;

	/* Set default sizes in case IOCTL (VIDIOCMCAPTURE) is not used
	 * (using read() instead). */
	for (i = 0; i < OV511_NUMFRAMES; i++) {
		ov->frame[i].width = ov->maxwidth;
		ov->frame[i].height = ov->maxheight;
		ov->frame[i].bytes_read = 0;
		ov->frame[i].depth = 12;
	}

	fprintf(stderr, "resolution: %dx%d\n", ov->maxwidth, ov->maxheight);

	/* Initialize to max width/height, YUV420 or RGB24 (if supported) */
	if (mode_init_regs(ov, ov->maxwidth, ov->maxheight,
			   ov->frame[0].format, 0) < 0){
		return -EINVAL;
	}

	return 0;	
}

int isoc_open(struct usb_ov511 * const ov)
{
	char *isoc_dev;
	const size_t sizeof_isoc_dev = 42;
	
	if ((isoc_dev = malloc(sizeof_isoc_dev)) == NULL) {
		fprintf(stderr, "Out of memory\n");
		return -1;
	}
#ifdef MODERN_USB_DEVICES
	snprintf(isoc_dev, sizeof_isoc_dev, "/dev/ugen%d.01", usbgen_id);
#else
	snprintf(isoc_dev, sizeof_isoc_dev, "/dev/ugen%d.1", usbgen_id);
#endif
	if ((ov->isoc_fd = open(isoc_dev, O_RDONLY)) == -1) {
		perror("Unable to open isochronous device");
		return -1;
	}
	return 0;
}

/**********************************************************************
 *
 * Decompression
 *
 **********************************************************************/

/* Chooses a decompression module, locks it, and sets ov->decomp_ops
 * accordingly. Returns -ENXIO if decompressor is not available, otherwise
 * returns 0 if no other error.
 */
int request_decompressor(struct usb_ov511 * const ov)
{
	if (!ov) {
		return -ENODEV;
	}

	if (ov->decomp_ops) {
		fprintf(stderr, "ERROR: Decompressor already requested!\n");
		return -EINVAL;
	}
	if (ov->bclass == BCL_OV511) {
		fprintf(stderr, "Using OV511 decompressor\n");
		ov->decomp_ops = &ov511_decomp_ops;
	} else if (ov->bclass == BCL_OV518) {
		fprintf(stderr, "Using OV518 decompressor\n");
		ov->decomp_ops = &ov518_decomp_ops;
	} else {
		fprintf(stderr, "Unknown bridge\n");
	}

	if (!ov->decomp_ops) {
		goto nosys;
	}
	
	return 0;

 nosys:
	return -ENOSYS;	
}

/* Unlocks decompression module and nulls ov->decomp_ops. Safe to call even
 * if ov->decomp_ops is NULL.
 */
void release_decompressor(struct usb_ov511 * const ov)
{
	int released = 0;	/* Did we actually do anything? */

	if (!ov) {
		return;
	}

	if (ov->decomp_ops) {
		released = 1;
	}

	ov->decomp_ops = NULL;

	if (released) {
		fprintf(stderr, "Decompressor released\n");
	}
}

void decompress(struct usb_ov511 *ov, struct ov511_frame *frame,
		unsigned char *pIn0, unsigned char *pOut0)
{
	fprintf(stderr, "Decompression requested\n");
	
	if (ov->decomp_ops == NULL) {
		if (request_decompressor(ov)) {
			return;
		}
	}

	fprintf(stderr, "Decompressing %d bytes\n", frame->bytes_recvd);
	
	if (ov->decomp_ops->decomp_420) {		
		int ret = ov->decomp_ops->decomp_420
		(pIn0,
		 pOut0,
		 frame->compbuf,
		 frame->rawwidth,
		 frame->rawheight,
		 frame->bytes_recvd);
		fprintf(stderr, "DEBUG: decomp_420 returned %d\n", ret);
	} else {
		fprintf(stderr, "Decompressor does not support this format\n");
	}	
}

/**********************************************************************
 *
 * Format conversion
 *
 **********************************************************************/

/* Fuses even and odd fields together, and doubles width.
 * INPUT: an odd field followed by an even field at pIn0, in YUV planar format
 * OUTPUT: a normal YUV planar image, with correct aspect ratio
 */
void deinterlace(struct ov511_frame *frame, int rawformat,
		 unsigned char *pIn0, unsigned char *pOut0)
{
	const int fieldheight = frame->rawheight / 2;
	const int fieldpix = fieldheight * frame->rawwidth;
	const int w = frame->width;
	int x, y;
	unsigned char *pInEven, *pInOdd, *pOut;

	if (frame->rawheight != frame->height) {
		fprintf(stderr, "invalid height\n");
		return;
	}

	if ((frame->rawwidth * 2) != frame->width) {
		fprintf(stderr, "invalid width\n");
		return;
	}

	/* Y */
	pInOdd = pIn0;
	pInEven = pInOdd + fieldpix;
	pOut = pOut0;
	for (y = 0; y < fieldheight; y++) {
		for (x = 0; x < frame->rawwidth; x++) {
			*pOut = *pInEven;
			*(pOut+1) = *pInEven++;
			*(pOut+w) = *pInOdd;
			*(pOut+w+1) = *pInOdd++;
			pOut += 2;
		}
		pOut += w;
	}

	if (rawformat == RAWFMT_YUV420) {
	/* U */
		pInOdd = pIn0 + fieldpix * 2;
		pInEven = pInOdd + fieldpix / 4;
		for (y = 0; y < fieldheight / 2; y++) {
			for (x = 0; x < frame->rawwidth / 2; x++) {
				*pOut = *pInEven;
				*(pOut+1) = *pInEven++;
				*(pOut+w/2) = *pInOdd;
				*(pOut+w/2+1) = *pInOdd++;
				pOut += 2;
			}
			pOut += w/2;
		}
	/* V */
		pInOdd = pIn0 + fieldpix * 2 + fieldpix / 2;
		pInEven = pInOdd + fieldpix / 4;
		for (y = 0; y < fieldheight / 2; y++) {
			for (x = 0; x < frame->rawwidth / 2; x++) {
				*pOut = *pInEven;
				*(pOut+1) = *pInEven++;
				*(pOut+w/2) = *pInOdd;
				*(pOut+w/2+1) = *pInOdd++;
				pOut += 2;
			}
			pOut += w/2;
		}
	}
}

/**********************************************************************
 *
 * Raw data parsing
 *
 **********************************************************************/

/* Copies a 64-byte segment at pIn to an 8x8 block at pOut. The width of the
 * image at pOut is specified by w.
 */
static inline void make_8x8(unsigned char *pIn, 
			    unsigned char *pOut, int w)
{
	unsigned char *pOut1 = pOut;
	int x, y;

	for (y = 0; y < 8; y++) {
		pOut1 = pOut;
		for (x = 0; x < 8; x++) {
			*pOut1++ = *pIn++;
		}
		pOut += w;
	}
}


/* LIMIT: convert a 16.16 fixed-point value to a byte, with clipping. */
#define LIMIT(x) ((x)>0xffffff?0xff: ((x)<=0xffff?0:((x)>>16)))

/*
 */
static inline void
v4l_copy_420_block (int yTL, int yTR, int yBL, int yBR, int u, int v, 
	int rowPixels, unsigned char * rgb, int bits)
{
	const int rvScale = 91881;
	const int guScale = -22553;
	const int gvScale = -46801;
	const int buScale = 116129;
	const int yScale  = 65536;
	int r, g, b;

	g = guScale * u + gvScale * v;
	r = rvScale * v;
	b = buScale * u;

	yTL *= yScale; yTR *= yScale;
	yBL *= yScale; yBR *= yScale;

	if (bits == 24) {
		/* Write out top two pixels */
		rgb[0] = LIMIT(b+yTL); rgb[1] = LIMIT(g+yTL); rgb[2] = LIMIT(r+yTL);
		rgb[3] = LIMIT(b+yTR); rgb[4] = LIMIT(g+yTR); rgb[5] = LIMIT(r+yTR);

		/* Skip down to next line to write out bottom two pixels */
		rgb += 3 * rowPixels;
		rgb[0] = LIMIT(b+yBL); rgb[1] = LIMIT(g+yBL); rgb[2] = LIMIT(r+yBL);
		rgb[3] = LIMIT(b+yBR); rgb[4] = LIMIT(g+yBR); rgb[5] = LIMIT(r+yBR);
	} else if (bits == 16) {
		/* Write out top two pixels */
		rgb[0] = ((LIMIT(b+yTL) >> 3) & 0x1F) | ((LIMIT(g+yTL) << 3) & 0xE0);
		rgb[1] = ((LIMIT(g+yTL) >> 5) & 0x07) | (LIMIT(r+yTL) & 0xF8);

		rgb[2] = ((LIMIT(b+yTR) >> 3) & 0x1F) | ((LIMIT(g+yTR) << 3) & 0xE0);
		rgb[3] = ((LIMIT(g+yTR) >> 5) & 0x07) | (LIMIT(r+yTR) & 0xF8);

		/* Skip down to next line to write out bottom two pixels */
		rgb += 2 * rowPixels;

		rgb[0] = ((LIMIT(b+yBL) >> 3) & 0x1F) | ((LIMIT(g+yBL) << 3) & 0xE0);
		rgb[1] = ((LIMIT(g+yBL) >> 5) & 0x07) | (LIMIT(r+yBL) & 0xF8);

		rgb[2] = ((LIMIT(b+yBR) >> 3) & 0x1F) | ((LIMIT(g+yBR) << 3) & 0xE0);
		rgb[3] = ((LIMIT(g+yBR) >> 5) & 0x07) | (LIMIT(r+yBR) & 0xF8);
	}
}


/*
 * For YUV 4:2:0 images, the data show up in 384 byte segments.
 * The first 64 bytes of each segment are U, the next 64 are V.  The U and
 * V are arranged as follows:
 *
 *      0  1 ...  7
 *      8  9 ... 15
 *           ...   
 *     56 57 ... 63
 *
 * U and V are shipped at half resolution (1 U,V sample -> one 2x2 block).
 *
 * The next 256 bytes are full resolution Y data and represent 4 squares
 * of 8x8 pixels as follows:
 *
 *      0  1 ...  7    64  65 ...  71   ...  192 193 ... 199
 *      8  9 ... 15    72  73 ...  79        200 201 ... 207
 *           ...              ...                    ...
 *     56 57 ... 63   120 121 ... 127   ...  248 249 ... 255
 *
 * Note that the U and V data in one segment represent a 16 x 16 pixel
 * area, but the Y data represent a 32 x 8 pixel area. If the width is not an
 * even multiple of 32, the extra 8x8 blocks within a 32x8 block belong to the
 * next horizontal stripe.
 *
 * If dumppix module param is set, _parse_data just dumps the incoming segments,
 * verbatim, in order, into the frame. When used with vidcat -f ppm -s 640x480
 * this puts the data on the standard output and can be analyzed with the
 * parseppm.c utility I wrote.  That's a much faster way for figuring out how
 * these data are scrambled.
 */

/* Converts from raw, uncompressed segments at pIn0 to a YUV420P frame at pOut0.
 *
 * FIXME: Currently only handles width and height that are multiples of 16
 */
void yuv420raw_to_yuv420p(struct ov511_frame *frame,
			  unsigned char *pIn0, unsigned char *pOut0)
{
	int k, x, y;
	unsigned char *pIn, *pOut, *pOutLine;
	const unsigned int a = frame->rawwidth * frame->rawheight;
	const unsigned int w = frame->rawwidth / 2;

	/* Copy U and V */
	pIn = pIn0;
	pOutLine = pOut0 + a;
	for (y = 0; y < frame->rawheight - 1; y += 16) {
		pOut = pOutLine;
		for (x = 0; x < frame->rawwidth - 1; x += 16) {
			make_8x8(pIn, pOut, w);
			make_8x8(pIn + 64, pOut + a/4, w);
			pIn += 384;
			pOut += 8;
		}
		pOutLine += 8 * w;
	}

	/* Copy Y */
	pIn = pIn0 + 128;
	pOutLine = pOut0;
	k = 0;
	for (y = 0; y < frame->rawheight - 1; y += 8) {
		pOut = pOutLine;
		for (x = 0; x < frame->rawwidth - 1; x += 8) {
			make_8x8(pIn, pOut, frame->rawwidth);
			pIn += 64;
			pOut += 8;
			if ((++k) > 3) {
				k = 0;
				pIn += 128;
			}
		}
		pOutLine += 8 * frame->rawwidth;
	}
}

/*
 * convert a YUV420P to a rgb image
 */
int v4l_yuv420p2rgb (unsigned char *rgb_out, unsigned char *yuv_in,
		     int width, int height, int bits)
{
	const int numpix = width * height;
	const unsigned int bytes = bits >> 3;
	int h, w, y00, y01, y10, y11, u, v;
	unsigned char *pY = yuv_in;
	unsigned char *pU = pY + numpix;
	unsigned char *pV = pU + numpix / 4;
	unsigned char *pOut = rgb_out;
	
	if (!rgb_out || !yuv_in) {
		return -1;
	}
	
	for (h = 0; h <= height - 2; h += 2) {
		for (w = 0; w <= width - 2; w += 2) {
			y00 = *(pY);
			y01 = *(pY + 1);
			y10 = *(pY + width);
			y11 = *(pY + width + 1);
			u = (*pU++) - 128;
			v = (*pV++) - 128;

			v4l_copy_420_block (y00, y01, y10, y11, u, v, width, pOut, bits);
	
			pY += 2;
			pOut += bytes << 1;

		}
		pY += width;
		pOut += width * bytes;
	}
	return 0;
}

void put_image_jpeg (FILE *out, char *image, int width,
		     int height, int quality, int palette)
{
	int y, x, line_width;
	JSAMPROW row_ptr[1];
	struct jpeg_compress_struct cjpeg;
	struct jpeg_error_mgr jerr;
	char *line;

	line = malloc(width * 3);
	if (!line)
		return;
	cjpeg.err = jpeg_std_error(&jerr);
	jpeg_create_compress (&cjpeg);
	cjpeg.image_width = width;
	cjpeg.image_height= height;
	cjpeg.input_components = 3;
	cjpeg.in_color_space = JCS_RGB;
	jpeg_set_defaults (&cjpeg);
	jpeg_simple_progression(&cjpeg);	
	jpeg_set_quality (&cjpeg, quality, TRUE);
	cjpeg.dct_method = JDCT_IFAST;
	jpeg_stdio_dest (&cjpeg, out);	
	jpeg_start_compress (&cjpeg, TRUE);
	row_ptr[0] = line;
	line_width = width * 3;
	for ( y = 0; y < height; y++) {
		for (x = 0; x < line_width; x+=3) {
			line[x]   = image[x+2];
			line[x+1] = image[x+1];
			line[x+2] = image[x];
		}
		jpeg_write_scanlines (&cjpeg, row_ptr, 1);
		image += line_width;
	}
	jpeg_finish_compress (&cjpeg);
	jpeg_destroy_compress (&cjpeg);
	free (line);
}

int output_jpeg(struct usb_ov511 * const ov,
		struct ov511_frame *frame,
		const char * const filename)
{
	FILE *fp;	
	char *rgb;
	char *tmpfilename;
	size_t sizeof_rgb;
	size_t sizeof_tmpfilename;
	
	yuv420raw_to_yuv420p(frame, frame->rawdata,
			     frame->data);
	sizeof_rgb = frame->rawwidth * frame->rawheight * 3;	
	if ((rgb = malloc(sizeof_rgb)) == NULL) {
		fprintf(stderr, "Out of memory\n");
		return -1;
	}
	if (v4l_yuv420p2rgb(rgb, frame->data,
			    frame->rawwidth,
			    frame->rawheight,
			    24) != 0) {
		fprintf(stderr, "yuv420p->rgb error\n");
		return -1;
	}
	sizeof_tmpfilename = strlen(filename) + (size_t) 2U;
	if ((tmpfilename = malloc(sizeof_tmpfilename)) == NULL) {
		fprintf(stderr, "out of memory\n");
		return -1;
	}
	snprintf(tmpfilename, sizeof_tmpfilename, "%s~", filename);
	if (filename[0] == '-' && filename[1] == 0) {
		fp = stdout;
	} else if ((fp = fopen(tmpfilename, "w")) == NULL) {
			perror(tmpfilename);
			free(rgb);
			return -1;
	}
	put_image_jpeg(fp, rgb, frame->rawwidth,
		       frame->rawheight, jpeg_quality, 0);
	free(rgb);
	fclose(fp);
	if (rename(tmpfilename, filename) != 0) {
		perror(filename);
		return -1;
	}
 	
	return 0;
}

int ov51x_postprocess_yuv420(struct usb_ov511 * const ov, 
			     struct ov511_frame * const frame)
{
	/* Deinterlace frame, if necessary */
	if (ov->sensor == SEN_SAA7111A && frame->rawheight >= 480) {
		if (frame->compressed) {
			decompress(ov, frame, frame->rawdata, frame->tempdata);
		} else {
			yuv420raw_to_yuv420p(frame, frame->rawdata,
					     frame->tempdata);
		}
		deinterlace(frame, RAWFMT_YUV420, frame->tempdata,
		            frame->data);
	} else {
		if (frame->compressed) {
			decompress(ov, frame, frame->rawdata, frame->data);
		} else {
			yuv420raw_to_yuv420p(frame, frame->rawdata,
					     frame->tempdata);
		}
	}
	output_jpeg(ov, frame, output_filename);
	
	return 0;
}

int ov511_isoc_read(struct usb_ov511 * const ov)
{
	unsigned char in[4096];
	struct ov511_frame frame;	
	ssize_t s;
	size_t num, offset;
	size_t n = 0;
	size_t sizeof_rawdata;
	size_t sizeof_tempdata;
	size_t sizeof_data;
	size_t sizeof_compbuf;
	int foundframe = -1;

	frame.width = ov->maxwidth;
	frame.height = ov->maxheight;
	sizeof_rawdata = get_frame_length(&frame) + ov->packet_size;
	sizeof_tempdata = sizeof_data = sizeof_rawdata;
	sizeof_compbuf = (size_t) 4096U;
	if ((frame.rawdata = malloc(sizeof_rawdata)) == NULL ||
	    (frame.tempdata = malloc(sizeof_tempdata)) == NULL ||
	    (frame.data = malloc(sizeof_data)) == NULL ||
	    (frame.compbuf = malloc(sizeof_compbuf)) == NULL) {
		fprintf(stderr, "Out of memory\n");
		return -1;
	}
	frame.scanstate = frame.grabstate = frame.snapshot = 0;
	frame.bytes_recvd = (size_t) 0U;
	fprintf(stderr, "Waiting for isochronous data\n");
	if (ov->packet_size > (int) sizeof in) {
		fprintf(stderr, "packet_size > %lu\n", (unsigned long)
			sizeof in);
		return -1;
	}
	while ((s = read(ov->isoc_fd, in, ov->packet_size)) >= (ssize_t) 0) {
		n = (size_t) s;
		/* Check for SOF/EOF packet */		
		if ((in[0] | in[1] | in[2] | in[3] | in[4] | in[5] | in[6] |
		     in[7]) || (~in[8] & 0x08)) {
			goto check_middle;
		}
		
		/* Frame end */
		if (in[8] & 0x80) {
			frame.rawwidth = ((int)(in[9]) + 1) * 8;
			frame.rawheight = ((int)(in[10]) + 1) * 8;
			
			/* Validate the header data */
			RESTRICT_TO_RANGE(frame.rawwidth, ov->minwidth,
					  ov->maxwidth);
			RESTRICT_TO_RANGE(frame.rawheight, ov->minheight,
					  ov->maxheight);

			/* Don't allow byte count to exceed buffer size */
			RESTRICT_TO_RANGE(frame.bytes_recvd, 8, 
					  (int) sizeof_rawdata);
						
			if (frame.scanstate == STATE_LINES) {
				frame.grabstate = FRAME_DONE;
				if (frame.snapshot != 0) {
					fprintf(stderr, "snapshot detected\n");
					ov51x_clear_snapshot(ov);
					frame.snapshot = 0;
				}
				fprintf(stderr, 
					"Frame completed, res = %d x %d\n",
					frame.rawwidth, frame.rawheight);
				ov51x_postprocess_yuv420(ov, &frame);
				if (wait_between_frames > 0U) {
					fprintf(stderr,
						"waiting %u sec\n",
						wait_between_frames);
					(void) sleep(wait_between_frames);
				}
			}
			continue;
		} else {
			/* Frame start */
			fprintf(stderr, "New frame\n");
			if (in[8] & 0x02) {
				frame.snapshot = 1;
			}
			frame.scanstate = STATE_LINES;
			frame.bytes_recvd = 0;
			ov->curframe = 0;
			if ((frame.compressed = in[8] & 0x40)) {
				fprintf(stderr, "compressed frame detected\n");				
			}
		}		
		check_middle:
		/* Are we in a frame? */		
		if (frame.scanstate != STATE_LINES) {
			continue;
		}
		/* If frame start, skip header */
		if (frame.bytes_recvd == 0) {
			offset = 9;
		} else {
			offset = 0;
		}		
		num = n - offset - 1;
		
		foundframe = in[n - 1];
		if (foundframe != ov->curframe && ignore_desync == 0) {
			fprintf(stderr, "desync (%d) != (%d)\n",
				foundframe, ov->curframe);
			frame.scanstate = 0;
		}
		if (ov->curframe == 255) {
			ov->curframe = 1;
		} else {
			ov->curframe++;
		}

		if (!frame.compressed) {
			frame.bytes_recvd += num;
			if (frame.bytes_recvd <= (int) sizeof_rawdata) {
				memcpy(frame.rawdata + frame.bytes_recvd - num,
				       in + offset, num);
			} else {
				fprintf(stderr,
					"Raw data buffer overrun! (%lu)\n",
					(unsigned long) 
					(frame.bytes_recvd - sizeof_rawdata));
			}
		} else {  /* Remove all-zero FIFO lines (aligned 32-byte blocks) */
			size_t read = 0U, copied = 0U;
			int b, allzero;
			if (offset) {
				frame.bytes_recvd += 32 - offset;	// Bytes out
				memcpy(frame.rawdata, in + offset, 
				       32 - offset);
				read += 32;
			}

			while (read < n - 1) {
				allzero = 1;
				for (b = 0; b < 32; b++) {
					if (in[read + b]) {
						allzero = 0;
						break;
					}
				}				
				if (allzero) {
					/* Don't copy it */
				} else {
					if (frame.bytes_recvd + copied + 32 
					    <= sizeof_rawdata) {
						memcpy(frame.rawdata
						       + frame.bytes_recvd +
						       copied, in + read, 32);
						copied += 32;
					} else {
						fprintf
						(stderr, 
						 "Raw data buffer overrun!!\n");
					}
				}
				read += 32;
			}			
			frame.bytes_recvd += copied;			
		}
	}
	
	return 0;
}

int isoc_read(struct usb_ov511 * const ov)
{
	return ov511_isoc_read(ov);
}

/****************************************************************************
 *
 * Stream initialization and termination
 *
 ***************************************************************************/

int ov51x_init_isoc(struct usb_ov511 * const ov)
{
	struct urb *urb;
	int fx, err, n, size;

	fprintf(stderr, "*** Initializing capture ***\n");

	ov->curframe = -1;

	if (ov->bridge == BRG_OV511) {
		if (cams == 1) {
			size = 993;
		} else if (cams == 2) {
			size = 513;
		} else if (cams == 3 || cams == 4) {
			size = 257;
		} else {
			fprintf(stderr, "\"cams\" parameter too high!\n");
			return -1;
		}
	} else if (ov->bridge == BRG_OV511PLUS) {
		if (cams == 1) {
			size = 961;
		} else if (cams == 2) {
			size = 513;
		} else if (cams == 3 || cams == 4) {
			size = 257;
		} else if (cams >= 5 && cams <= 8) {
			size = 129;
		} else if (cams >= 9 && cams <= 31) {
			size = 33;
		} else {
			fprintf(stderr, "\"cams\" parameter too high!\n");
			return -1;
		}
	} else if (ov->bclass == BCL_OV518) {
		if (cams == 1) {
			size = 896;
		} else if (cams == 2) {
			size = 512;
		} else if (cams == 3 || cams == 4) {
			size = 256;
		} else if (cams >= 5 && cams <= 8) {
			size = 128;
		} else {
			fprintf(stderr, "\"cams\" parameter too high!\n");
			return -1;
		}
	} else {
		fprintf(stderr, "invalid bridge type\n");
		return -1;
	}

	// FIXME: OV518 is hardcoded to 15 FPS (alternate 5) for now
	if (ov->bclass == BCL_OV518) {
		if (packetsize == -1) {
			ov518_set_packet_size(ov, 640);
		} else {
			fprintf(stderr, "Forcing packet size to %d\n",
				packetsize);
			ov518_set_packet_size(ov, packetsize);
		}
	} else {
		if (packetsize == -1) {
			ov511_set_packet_size(ov, size);
		} else {
			fprintf(stderr, "Forcing packet size to %d\n",
				packetsize);
			ov511_set_packet_size(ov, packetsize);
		}
	}
	ov->streaming = 1;

	return 0;
}

void ov51x_stop_isoc(struct usb_ov511 *ov)
{
	if (!ov->streaming || !ov->dev) {
		return;
	}

	fprintf(stderr, "*** Stopping capture ***\n");

	if (ov->bclass == BCL_OV518) {
		ov518_set_packet_size(ov, 0);
	} else {
		ov511_set_packet_size(ov, 0);
	}

	ov->streaming = 0;
}

void usage(void)
{
	puts("BSDCam " VERSION " by Jedi/Sector One <j@pureftpd.org>\n"
	     "\n"
	     "Usage: bsdcam -f <output jpeg file name>\n"
	     "             [-b]\n"
	     "             [-d <usbgen device id>]\n"
	     "             [-H 50|60]\n"
	     "             [-i]\n"
	     "             [-l]\n"
	     "             [-m]\n"
	     "             [-q <jpeg quality>]\n"
	     "             [-u <uid>]\n"
	     "             [-w <seconds>]\n"
	     "\n"
	     "-b : Enable banding filter\n"
	     "-d : USB device is /dev/ugen<arg>\n"
	     "-H : Change light frequency (50 or 60 Hz)\n"
	     "-i : Ignore desynchronized frames\n"
	     "-l : Enable backlight feature\n"
	     "-m : Enable mirror mode\n"
	     "-q : JPEG Quality (0-100, default 80)\n"	     
	     "-u : Drop privileges to the given uid\n"
	     "-w : Wait after every frame\n"	     
	     );
}

int parse_opts(int argc, char *argv[])
{
	int ch;

	while ((ch = getopt(argc, argv, "f:bd:H:hilmq:u:w:")) != -1) {
		switch (ch) {
		case 'f':
			if ((output_filename = strdup(optarg)) == NULL) {
				fprintf(stderr, "Out of memory\n");
				return -1;
			}
			break;
		case 'b':
			bandingfilter = 1;
			break;			
		case 'd':
			usbgen_id = atoi(optarg);
			break;
		case 'H':
			lightfreq = atoi(optarg);
			break;
		case 'h':
		case 'v':
			usage();
			return -1;
		case 'i':
			ignore_desync = 1;
			break;			
		case 'l':
			backlight = 1;
			break;
		case 'm':
			mirror = 1;
			break;
		case 'q':
			jpeg_quality = atoi(optarg);
			break;
		case 'u':
			switchto_uid = (uid_t) strtoul(optarg, NULL, 10);
			break;
		case 'w':
			wait_between_frames = strtoul(optarg, NULL, 10);
			break;			
		}
	}
	if (output_filename == NULL) {
		usage();
		return -1;
	}
	
	return 0;
}

int change_uid(void)
{
	if (switchto_uid <= (uid_t) 0 ||
	    geteuid() != (uid_t) 0) {
		return 0;
	}
	fprintf(stderr, "Dropping privileges to uid (%lu)\n",
		(unsigned long) switchto_uid);	
	if (seteuid(switchto_uid) != 0 || setuid(switchto_uid) != 0) {
		perror("Unable to drop privileges");
		return -1;
	}
	return 0;
}

int main(int argc, char *argv[])
{
	struct usb_ov511 ov;

	if (parse_opts(argc, argv) != 0) {
		return 254;
	}	
	if (ov511_probe(&ov) != 0) {
		(void) sleep(2);
		return 1;
	}
	if (ov51x_set_default_params(&ov) != 0) {
		return 2;
	}
	if (ov51x_init_isoc(&ov) != 0) {
		return 3;
	}
	if (isoc_open(&ov) != 0) {
		return 4;
	}
	if (change_uid() != 0) {
		return 5;
	}
	if (isoc_read(&ov) != 0) {
		return 6;
	}
	
	return 0;
}
