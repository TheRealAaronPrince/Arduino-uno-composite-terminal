// UnoVideoSerial.h ceptimus 2022-09-08

#ifndef UNOVIDEOSERIAL_H_
#define UNOVIDEOSERIAL_H_

#define BYTES_PER_RASTER 45
#define PIXELS_PER_CHARACTER 8
#define CHARACTER_ROWS 30

// time into scanline when pixel pumper awakes (but it takes time to get going because of the interrupt handler etc.
// Units: us * 16
#define LEFT_EDGE 93
// scan line at which pixel pumper is enabled. 30 minimum to be on-screen on 7-inch monitor
#define TOP_EDGE 49
// and when it's stopped. 303 maximum to be on-screen on 7-inch monitor
#define BOTTOM_EDGE (TOP_EDGE + CHARACTER_ROWS * PIXELS_PER_CHARACTER)

#endif
