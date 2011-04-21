all: bsdcam

clean:
	rm -f *.o bsdcam *~

bsdcam: bsdcam.c bsdcam.h ov511_decomp.o ov518_decomp.o
	gcc $(CFLAGS) -Wall -W -Wno-unused \
	-I/usr/local/include -I/usr/pkg/include \
	-L/usr/local/lib -L/usr/pkg/lib \
	-o bsdcam bsdcam.c ov511_decomp.o ov518_decomp.o -ljpeg
	
