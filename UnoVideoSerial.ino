// UnoVideoSerial.cpp 2022-09-09 ceptimus

#include "UnoVideoSerial.h"
#include "font.h"

// demonstrating composite 'serial terminal' video output.  Serial receive at 2400 baud, on a standard Arduino UNO/Nano
// easy to use any other pin for serial receive - just change the (PIND & mask) in the macro below
#define RX_RECEIVE_BIT (PIND & 0x01)

// works at 2400 baud, no parity, 8 data bits, 1 stop bit.
// to do this, the receive pin is sampled 9 scanlines after the leading edge of the start bit is detected, and then with alternating 
// intervals of 7 and 6 scanlines.
// faster, but non-standard, baud rates are possible: 5208 baud works well with an initial nextSample of 4 and subsequent nextSamples of 3

// note if you're using the Arduino IDE serial monitor, you'll see gibberish in the serial monitor display window.
// This is because the Arduino's transmit data pin is actually sending out video pixels, at 8 megabaud.
// Other serial monitors, such as PuTTY (available on Windows and Linux) are better suited anyway.
// PuTTY, by default, doesn't line-buffer its output, and is better at ignoring received gibberish.
// If you want serial output from the Arduino UNO or Nano, while using this method of driving composite video out,
// you'll have to do it by bit banging software, on some other pin.

// for the composite video output use a 1K resistor from pin 1, a 1K2 resistor from pin 9, and a 47 ohm resistor from pin 5.
// other wire of all 3 resistors linked together and driving the composite video signal (centre wire in a coax cable).
// The outer of the coax cable (assuming coax is used) is connected to GND.

// note that the microcontroller spends most of its time just pumping out pixels.  If you want to use it to do some
// complex calculations, best switch off the video while you're doing them. The same method that the Sinclair ZX81 and ZX81 used!

#define PIN_SUPPRESS 5
#define PIN_SYNC 9
int temp = 0x20;
bool protect = false;
bool cursorToggle = true;
int frameTimer = 0;
int timerExp =5;

extern "C" { void pixelsEtc(uint8_t *pScreenRam, const uint8_t *fontSlice, uint16_t tcnt, uint16_t minTCNT); }

#define SCREEN_RAM_SIZE BYTES_PER_RASTER * CHARACTER_ROWS 
uint8_t screenRam[SCREEN_RAM_SIZE + 1]; // the + 1 is for the terminating \0 of the string, but isn't used

volatile uint16_t scanline = 0; // counts 0 - 311 (312 scan lines per frame)

// receive buffer: characters arrive serialy (at 2400 baud, so up to 240 per second)
// main loop probably can't handle them that fast when doing time-consuming things, such as scrolling
#define SERIAL_BUFFER_SIZE 64
uint8_t serialRxBuffer[SERIAL_BUFFER_SIZE];
uint8_t serialRxBufferRead = 0;
volatile uint8_t serialRxBufferWrite = 0;

// this interrupt occurs every 64 microseconds, so performance would be improved if this were to be hand-coded in optimized assembler.
// but the compiler does a pretty good job of optimizing (it's clever enough to convert the RX_RECEIVE_BIT macro into sbic and sbis instructions!)
// improving on the compiler's efforts is left as a task for the interested reader. :)  I included its output, assemblyDump.txt, to look at.
ISR(TIMER1_OVF_vect) { // TIMER1_OVF vector occurs at the start of each scan line's sync pulse
  static uint8_t bitCounter = 0;
  static uint8_t nextSample = 0;
  static uint8_t rxByte = 0;
	if (++scanline == 312) {
    frameTimer+=1;
    frameTimer%=(1<<timerExp);
		OCR1A = 948; // scan lines 0 - 7 have wide 59.3us sync pulses
		scanline = 0;
		} else if (scanline == 8) {
		OCR1A = 74; // swap to short 4.7us sync pulses for scan lines 8 - 311
		// enabling the interrupt generates an immediate 'stored up' interrupt
		// so enable it one scan line early, test and return within interrupt handler to ignore 1st one
		}	else if (scanline == TOP_EDGE) { // scan line 51 is first 'text safe' scan line - will already have been incremented to 52 here
		TIMSK1 |= _BV(OCIE1B);
	}

// timing of the serial port receive routine is not super-critical, so we can allow the other (pixel pumper) interrupt to take precedence during this part.
// this keeps the display rock solid. :)
  sei(); 
  if (!bitCounter) { // awaiting start bit
    if (!RX_RECEIVE_BIT) { // start bit received
      bitCounter = 1;
//  leading edge of start bit, so expected centre of start bit is roughly 3 samples away, and the (best estimate) centres of following bits are spaced every 6.5 samples      
      nextSample = 9; 
      rxByte = 0x00;
    }
  } else if (!--nextSample) { // time to sample next bit
    if (bitCounter == 9) { // check stop bit
      if (RX_RECEIVE_BIT) { // stop bit received
        serialRxBuffer[serialRxBufferWrite++] = rxByte;
        if (serialRxBufferWrite == SERIAL_BUFFER_SIZE) {
          serialRxBufferWrite = 0;
        }
      }
      rxByte = bitCounter = 0; // await next start bit
    } else { // shift in received bit 
      rxByte = RX_RECEIVE_BIT ? (rxByte >> 1) | 0x80 : rxByte >> 1;
      nextSample = bitCounter++ & 0x01 ? 7 : 6; // take sample every 6.5 interrupts (2403.846 baud with 64us interrupts - so 2400 baud + 0.16%)
    }
  }
}

volatile uint16_t minTCNT = 0xFFFF;
volatile uint16_t maxTCNT = 0;

ISR(TIMER1_COMPB_vect) { // occurs at start of 'text safe' area of scan lines 51 - 280
	static uint8_t *pScreenRam;
	static const uint8_t *fontSlice;
	static uint8_t slice;
	
	uint16_t tcnt = TCNT1; // capture timer to allow jitter correction
	
	if (scanline == TOP_EDGE) { // on stored-up 'false trigger' scanline, initialize the pointers
		slice = 0;
		pScreenRam = screenRam; // point to first character (top left) in screenRam
		fontSlice = font; // point to slice before first (top) slice of font pixels (top pixel of each 10 is just RVS cap)
		} else {
		pixelsEtc(pScreenRam, fontSlice, tcnt, minTCNT);
		if (tcnt > maxTCNT) maxTCNT = tcnt;
		if (tcnt < minTCNT) minTCNT = tcnt;

		if (scanline == BOTTOM_EDGE) {
			TIMSK1 &= ~_BV(OCIE1B); // we don't want any more COMPB interrupts this frame
			} else if (++slice == PIXELS_PER_CHARACTER) {
			slice = 0;
			fontSlice = font;
			pScreenRam += BYTES_PER_RASTER;
			} else {
			fontSlice += 128;
		}
	}
}


int serialAvailable() { // returns number of characters waiting in buffer 
  int n = (int)serialRxBufferRead - (int)serialRxBufferWrite;
  return n < 0 ? n + SERIAL_BUFFER_SIZE : n;
}

int serialRead(void) {
  int c = -1;
  
  if (serialAvailable()) {
    c = serialRxBuffer[serialRxBufferRead++];
    if (serialRxBufferRead >= SERIAL_BUFFER_SIZE) {
      serialRxBufferRead = 0;
    }
  }
  return c;
}

uint8_t cursorRow = 0, cursorColumn = 0;
bool cursorShown = false;

uint8_t *hideCursor(void) {
   uint8_t *screen = screenRam + cursorRow * BYTES_PER_RASTER + cursorColumn;
   if(protect)
   {
    *screen = temp;
    protect = false;
   }
   cursorShown = false;
   return screen;
}
void showCursor(void) {
   uint8_t *screen = screenRam + cursorRow * BYTES_PER_RASTER + cursorColumn;
   if(!protect)
   {
    temp = *screen;
    protect = true;
   }
  if(frameTimer>>timerExp-1)
  {
    *screen = 0x5F;
  }
  else
  {
    *screen = 0x20;
  }
   cursorShown = true;
}

void positionCursor(uint8_t row, uint8_t column) {
  uint8_t *oldCursorPosn = 0;
  
  row = min(row, CHARACTER_ROWS - 1);
  column = min(column, BYTES_PER_RASTER - 1);
  cursorRow = row;
  cursorColumn = column;
}

void scrollUp(void) {
  uint8_t *from = screenRam + BYTES_PER_RASTER;
  uint8_t *to = screenRam;
  
  for (uint16_t i = BYTES_PER_RASTER * (CHARACTER_ROWS - 1); i; i--) {
    *to++ = *from++;
  }

  for (uint8_t i = BYTES_PER_RASTER; i; i--) {
    *to++ = ' ';
  }
}

void emit(const char c) { // handles cursor movement and scrolling.
  uint8_t *s = 0, *oldCursorPosn = 0;
  
  if (cursorShown) {
    s = oldCursorPosn = hideCursor();  
  } else {
    s = screenRam + cursorRow * BYTES_PER_RASTER + cursorColumn;
  }
  
  // Always works in insert mode - scrolling any characters at, or to the right of, the cursor to the right
  // you may wish to remove, or toggle, this part, if you want overwrite mode
  uint8_t *to = screenRam + (cursorRow + 1) * BYTES_PER_RASTER - 1;
  uint8_t *from = to - 1;
  for (uint8_t i = BYTES_PER_RASTER - cursorColumn - 1; i; i--) {
    *to-- = *from--;
  }
  // end of the insert mode part
  
  *s++ = c;
  if (++cursorColumn == BYTES_PER_RASTER) {
    cursorColumn = 0;
    if (++cursorRow == CHARACTER_ROWS) {
      scrollUp();
      --cursorRow;
    }
  }
}

void consumeCharAtCursor(void) {
  uint8_t *s = 0, *oldCursorPosn = 0;
  
  if (cursorShown) {
    s = oldCursorPosn = hideCursor();  
  } else {
    s = screenRam + cursorRow * BYTES_PER_RASTER + cursorColumn;
  }
  char *s1 = s + 1;
  for (uint8_t i = BYTES_PER_RASTER - cursorColumn - 1; i; i--) {
    *s++ = *s1++;
  }
  *s = ' ';
}

int displayStringAt(uint8_t row, uint8_t column, const __FlashStringHelper* s) { // displays const strings stored in PROGMEM (saves RAM)
  uint8_t *screen = screenRam + row * BYTES_PER_RASTER + column;
  uint8_t *p = (uint8_t PROGMEM *)s;
  int charCount = 0;

  while (uint8_t c = pgm_read_byte_near(p++)) {
    *screen++ = c;
    charCount++;
  }
  return charCount;
}

int displayStringAt(uint8_t row, uint8_t column, const char* s) { // displays standard (RAM hungry) strings
  uint8_t *screen = screenRam + row * BYTES_PER_RASTER + column;
  int charCount = 0;

  while (*s) {
    *screen++ = *s++;
    charCount++;
  }
  return charCount;
}

void setup() {
      for (uint16_t i = 0; i < SCREEN_RAM_SIZE; i++) { // clear screen
        screenRam[i] = ' '; 
      }

      // displayStringAt(cursorRow, cursorColumn, F("  Arduino UNO composite video demonstration"));
      // displayStringAt(1, 3, F("ceptimus. 2022-09-08  Receiving data on"));
      // displayStringAt(2, 3, F("pin 0 at 2400 baud 8n1."));

      // cursorRow = 3; cursorColumn = 0;
  
	pinMode(PIN_SUPPRESS, INPUT);
	digitalWrite(PIN_SUPPRESS, LOW); // prime pixel suppressor - when Pin is switched to output, it will force BLACK

	// configure USART as master SPI mode 0, MSB first, 8MHz
	UCSR0A = _BV(U2X0); // double speed
	UCSR0B = _BV(TXEN0);
	UCSR0C = _BV(UMSEL01) | _BV(UMSEL00);
	UBRR0L = UBRR0H = 0x00; // fastest possible baud

	// output pin for sync pulses - low 4.7 us pulses at start of visible scan lines;  longer low pulses for vertical blank
	digitalWrite(PIN_SYNC, HIGH);
	pinMode(PIN_SYNC, OUTPUT);

	// configure timer/counter 1 to output scanline sync pulses on Pin9 (OC1A)
	// use mode 7 (fast PWM 10-bit count to TOP=1023) at 16MHz fclk - one cycle per 64us scanline
	cli(); // not necessary
	TCCR1A =  _BV(COM1A1) | _BV(COM1A0) | _BV(WGM11) | _BV(WGM10); // set OC1A output on compare match, (mode 3 so far)
	TCCR1B = _BV(WGM12) | _BV(CS10); // now mode 7 at clk/1 (16MHz)
	OCR1A = 948; // 59.3us wide sync pulse for first 8 scan lines
	OCR1B = LEFT_EDGE;
	TIMSK1 = _BV(TOIE1); // _BV(OCIE1A);
	TCNT1 = 0x0000;
	sei(); // necessary
  TIMSK0 &= ~_BV(TOIE0); // disable timer0 - stops millis() working but necessary to stop timer 0 interrupts spoiling display timing
  pinMode(13, OUTPUT); // builtin LED for diagnostics
}

void loop() {
  static uint8_t extendedCharState = 0;
  uint8_t newCursorColumn, newCursorRow;
  int check = frameTimer&((1<<timerExp-1)-1);
        if(check == 0 && cursorToggle)
        {
          showCursor();
        }
  int c = serialRead();
  while (c >= 0) {
    hideCursor();
    // this seems over-complicated, but is just handling a few of the common 'escape sequences' for cursor movement keys:
    // carriage return, line feed, up, down, left, right, home, end, delete, and  backspace
    // you can strip it all the below out, and just use { emit(c); c= serialRead(); } instead, if you want to keep it simple
    switch (extendedCharState) {
      case 1:
        if (c == '[') {
          extendedCharState = 2;
        } else {
          emit(0x1B);
          emit(c);
          extendedCharState = 0;
        }
        break;
      case 2:
        switch (c) {
          case '1': // possible Home
            extendedCharState = 3;
            break;
          case '2': // possible Ins
            extendedCharState = 4;
            break;
          case '3': // possible Del
            extendedCharState = 5;
            break;
          case '4': // possible End
            extendedCharState = 6;
            break;
          case '5': // possible PgUp
            extendedCharState = 7;
            break;
          case '6': // possible PgDn
            extendedCharState = 8;
            break;
          case 'A': // cursor up
            if (cursorRow) {
              positionCursor(cursorRow - 1, cursorColumn);
            }
            extendedCharState = 0;
            break;
          case 'B': // cursor down
            if (cursorRow < CHARACTER_ROWS - 1) {
              positionCursor(cursorRow + 1, cursorColumn);
            }
            extendedCharState = 0;
            break;
          case 'C': // cursor right
            if (cursorColumn < BYTES_PER_RASTER - 1) {
              newCursorColumn = cursorColumn + 1;
              newCursorRow = cursorRow;
            } else {
              newCursorColumn = 0;
              newCursorRow = cursorRow < CHARACTER_ROWS - 1 ? cursorRow + 1 : cursorRow;
            }
            positionCursor(newCursorRow, newCursorColumn);
            extendedCharState = 0;
            break;
          case 'D': // cursor left
            if (cursorColumn) {
              newCursorColumn = cursorColumn - 1;
              newCursorRow = cursorRow;
            } else {
              newCursorColumn = BYTES_PER_RASTER - 1;
              newCursorRow = cursorRow ? cursorRow - 1 : cursorRow;
            }
            positionCursor(newCursorRow, newCursorColumn);
            extendedCharState = 0;
            break;
          case 'F': // End
            positionCursor(cursorRow, BYTES_PER_RASTER - 1);
            extendedCharState = 0;
            break;
          case 'H': // Home
            positionCursor(cursorRow, 0);
            extendedCharState = 0;
            break;
          default:
            emit(0x1B);
            emit('[');
            emit(c);
            extendedCharState = 0;
            break;
        }
        break;
      case 3: // posible Home
        if (c == '~') { // Home
          positionCursor(cursorRow, 0);
        } else {
          emit(0x1B);
          emit('[');
          emit('1');
          emit(c);
        }
        extendedCharState = 0;
        break;
      case 4: // posible Ins
        if (c == '~') { // Ins
          cursorToggle = !cursorToggle;
        } else {
          emit(0x1B);
          emit('[');
          emit('3');
          emit(c);
        }
        extendedCharState = 0;
        break;
      case 5: // posible Del
        if (c == '~') { // Del
          consumeCharAtCursor();
        } else {
          emit(0x1B);
          emit('[');
          emit('3');
          emit(c);
        }
        extendedCharState = 0;
        break;
      case 6: // posible End
        if (c == '~') { // End
          positionCursor(cursorRow, BYTES_PER_RASTER - 1);
        } else {
          emit(0x1B);
          emit('[');
          emit('4');
          emit(c);
        }
        extendedCharState = 0;
        break;
      case 7: // posible PgUp
        if (c == '~') { // PgUp
          positionCursor(0, 0);
        } else {
          emit(0x1B);
          emit('[');
          emit('5');
          emit(c);
        }
        extendedCharState = 0;
        break;
      case 8: // posible PgDn
        if (c == '~') { // PgDn
          positionCursor(CHARACTER_ROWS, 0);
        } else {
          emit(0x1B);
          emit('[');
          emit('6');
          emit(c);
        }
        extendedCharState = 0;
        break;
      default:
        switch (c) {
          case '\b': // backspace (Ctrl-H)
          case 0x7F: // Backspace (Ctrl-?)
            if (cursorColumn) { // not bothering to delete rows and scroll up bottom part of screen (for now)
              cursorColumn--;
              consumeCharAtCursor();
            }
            break;
          case '\r': // carriage return
            positionCursor(cursorRow, 0);
            /**** Deliberately no break; here - allowing carriage return to fall through and insert an implicit line feed. ****/
            /**** This suits PuTTY. Remove // for other terminal emulators, or if you want to use Ctrl-J for line feed in PuTTY ****/
            // break;
          case '\n': // line feed
            if (cursorRow == CHARACTER_ROWS - 1) {
                scrollUp();
            } else {
              positionCursor(cursorRow + 1, cursorColumn);
            }
            break;
          case 0x1B: // Esc
            extendedCharState = 1;
            break;
          default:
            emit(c);
            break;
        }
        break;
    }
    c = serialRead();
  }
}
