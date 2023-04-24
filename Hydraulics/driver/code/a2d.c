// a2d.c - A2D functions
//
// Version 14.3
// ADC prescaler changed to 64, to give 125kHz AD clock freq.
// With continious sampling, this gives 4,580 samples per second per channel.

#include "a2d.h"
#include <avr/io.h>

// Static variables holding the latest values from the ADC

static uint16_t ADValue[8];

// When the IMU is connected as per MCU_Schematic for 3.3v operation, the A2D
// unit is setup for 0->3.3 volt operation.  When only sampling the Steering
// pot, vRef is setup for 0->5 volt operation. Define AD_VREF as either 5 or 3

#define AD_VREF 5
#define START_CHANNEL 4
#define END_CHANNEL 7
// ------------------------------------------------------------------------
// Initialize Analog to Digital converters.
// Use single ended conversion - one channel at a time,
// Setup to sample channels only when triggered. (not free running)
// For accurate 10 bit conversion, AD clock must be between 50kHz and 200kHz

void A2D_Init(void) {
  uint8_t n;
  for (n = 0; n < 8; n++)
    ADValue[n] = 0;

  ADCSRA = (1 << ADPS1) | (1 << ADPS2) |
           (1 << ADPS0); // Prescale 128 .. @16Mhz ADC clock = 125kHz

  // ADCSRA = (1<<ADPS1) | (1<<ADPS2)  // Prescale 64 .. @16Mhz ADC clock =
  // 250kHz

  ADCSRB = (1 << MUX5); // use channels ADC8 and up

#if AD_VREF == 5
  ADMUX = (1 << REFS0); // AVCC voltage ref, ch 0.
#else
  ADMUX = 0;                  // AVREF voltage ref, ch 0.
#endif

  DIDR2 |= 0xF0; /* disable ADC12-ADC15 digital in */

  ADCSRA |= (1 << ADEN); // Enable the ADC
  ADCSRA |= (1 << ADSC); // Start conversions
}

// -------------------------------------------------------------------------
// poll a2d conversion complete bit - read ADC, start next conversion.
// This avoids waiting or having to turn off interupts to read/write values.
// Sample 8 channels in sequence [0..7] then start over at 0.
// Filter noise using low pass averaging.
// This routine will be called 168,000 times per second. (at 8Mhz)
// Conversions take 13 cycles to complete.
// It works out to over 9,000 samples per second.

void A2D_poll_adc(void) {
  uint8_t chanNum;

  if (bit_is_clear(ADCSRA, ADSC)) { // if clear, conversion is done.
    chanNum = (ADMUX & 0x07);       /* roll over after 3 measurements */
    //    chanNum = (ADMUX & 0x02);	/* roll over after two measurements */

    //	ADValue[chanNum++] = ADC;	// Save measured value - no filtering

    ADValue[chanNum] = ADC;
    chanNum++;

    /* only care about channels 6 and 7 */
    if (chanNum > END_CHANNEL)
      chanNum = START_CHANNEL;

#if AD_VREF == 5
    ADMUX = (1 << REFS0) |
            (chanNum & 0x07); // Use AVCC voltage ref, channel num 0..1
#else
    ADMUX = (chanNum & 0x07); // Use AVREF voltage ref, channel num 0..7
#endif
    ADCSRA |= _BV(ADSC); // Start next conversion
  }
}

// ----------------------------------------------------------------------------------------------------------------------------------------------------------------
// Return most recent value read from A2D converter for requested channel.
// This avoids waiting or having to turn off interupts to read/write values.
static uint8_t map[8] = {2, 1, 0, 1,
                         1, 1, 1}; // remapping. The outside of the box has
                                   // changed, as has the ordering of the cables

uint16_t A2D_read_channel(uint8_t chanNum) {
  if (chanNum < 8)
    return ADValue[map[chanNum]];
  else
    return 0;
}

// ----------------------------------------------------------------------------------------------------------------------------------------------------------------
// End of File
