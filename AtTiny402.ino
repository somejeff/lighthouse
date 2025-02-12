#include <EEPROM.h>            // Used to read/write the ambient light threshold
#define LED_PIN 1              // PA1 = Physical Pin 4
#define BUTTON_PIN 2           // PA2 = Physical Pin 5
#define PHOTO_PIN 3            // PA3 = Physical Pin 7
#define DEFAULT_THRESHOLD 500  // Default ambient light threshold

// LED brightness sequence (0=off to 1024=full brightness)
// the dimmer the LED, the more power that is saved.
const uint16_t brightness[] = {
  0, 1, 1, 1, 2, 2, 2, 3, 3, 4, 6, 8, 12, 18, 27, 40, 58,
  82, 115, 160, 220, 300, 1024, 220, 160, 115, 82, 58, 40, 27, 18, 12, 8, 6, 4, 3, 2, 2, 1, 1, 0
};


volatile uint8_t brightnessIndex = 0;  // index for iterating the brightless array
uint16_t lightThreshold;               // ambient light threshold


// Runs on startup
void setup() {
  PORTA.DIRSET = (1 << LED_PIN);      // Set LED pin as output
  PORTA.DIRCLR = (1 << BUTTON_PIN);   // Set button as input
  PORTA.PIN2CTRL = PORT_PULLUPEN_bm;  // Enable pull-up resistor for the button




  lightThreshold = readStoredThreshold();  // Read the ambient light threshold from stored memory

  checkAndStartLighthouse();  // start the light house program
}



/*
Process to decide if the LED should flsh or not
*/
void checkAndStartLighthouse() {

  uint16_t lightLevel = readLightLevel();  // measure the ambient light

  /* if the button is pressed, store the ambient light setting
   this sets the threshold.  if it gets darker than this then flash the LED.  
   - Holding the button midday pretty would make the LED flash all day.
   - Holding it at night would make it flash only when it's fully dark
   */
  if (!(PORTA.IN & (1 << BUTTON_PIN))) {
    storeThreshold(lightLevel);
  }

  // if it's too light out, power down asap
  if (lightLevel >= lightThreshold) {
    powerDown();
  } else {
    startLighthouse();
  }
}

/*
Reads the voltage from the Photo PIN and gives a number from 0=dark to 1024=full light
*/
uint16_t readLightLevel() {
  /* Setup for the Photo resistor
  The resistor is setup as a voltage divider using the Analog to Digital converter (ADC)
  As it gets darker, voltage drops  

  +3V--/\/\/\---7---\/\/\/---GND
        Photo       10kΩ

  */

  VREF.CTRLA = VREF_ADC0REFSEL_2V5_gc;               // Set the Voltage Reference for the ADC to a 2.5V
  ADC0.CTRLA = ADC_ENABLE_bm | ADC_RESSEL_10BIT_gc;  // Enable ADC, 10-bit resolution
  ADC0.MUXPOS = PHOTO_PIN;                           // Select PA3 as ADC input
  ADC0.COMMAND = ADC_STCONV_bm;                      // start converstion
  // samples are taken here
  while (!(ADC0.INTFLAGS & ADC_RESRDY_bm))
    ;                             // keep sample until the ADC has a value to give
  ADC0.INTFLAGS = ADC_RESRDY_bm;  // reset the flag
  return ADC0.RES;                // return the result
}

/*
Function for the LED pulse

This function overrides the internal clock and leverages TCA0 as a pulse width modulator (PWM)
*/
void startLighthouse() {
  // The TCA is used to determin the brightness of the LED
  TCA0.SINGLE.CTRLB = TCA_SINGLE_CMP1EN_bm | TCA_SINGLE_WGMODE_DSBOTTOM_gc;  // set the TCA0 as a PWM
  TCA0.SINGLE.PER = 1023;                                                    // Set the number of clock cycles as the period
  TCA0.SINGLE.CMP1 = brightness[brightnessIndex];                            // set the initial brightness
  TCA0.SINGLE.CTRLA = TCA_SINGLE_ENABLE_bm | TCA_SINGLE_CLKSEL_DIV64_gc;      // enable the PWM and only count every 1024 clock cycles

  // The RTC is how quick we should iterate thru the brightness array
  RTC.CLKSEL = RTC_CLKSEL_OSC32K_gc;  // enable the lowest powererd clock as possible
  RTC.PITINTCTRL = RTC_PI_bm; // 
  RTC.PITCTRLA = RTC_PERIOD_CYC1024_gc | RTC_PITEN_bm;  // call the interrupt at a given period 

  sei(); // Enable global interrupts  (calls the ISR function when the RTC pulses)
}


/*
Shuts down the chip, leaving only the Watchdog timer (WDT) running to restart it

After the ambient light check, and potential LED sequence, there's no need to have the chip running for another 8 or so seconds
For more or less time between sequences, change the WDT period
*/
void powerDown() {
  TCA0.SINGLE.CTRLA = 0;          // turn off the PWM
  RTC.PITCTRLA = 0;               // turn off the realtime clock

  // enable the watchdog timer to count down about 8 seconds and then reboot the chip
  _PROTECTED_WRITE(WDT.CTRLA, WDT_PERIOD_8KCLK_gc | WDT_WINDOW_OFF_gc);

  // set "Powering Down" (PDOWM) as the lowest possible state and enables sleep capabilities
  _PROTECTED_WRITE(SLPCTRL.CTRLA, SLPCTRL_SMODE_PDOWN_gc | SLPCTRL_SEN_bm);
  asm("sleep"); // power down
}


/* 
This is the interupt for the Real Time Clock
The faster the clock, the faster the brightness Index moves down the array
This creates the lighthouse style flash
[ 1-> 3 -> 50 -> 100 -> 50 -> 3 -> 1 -> 0]

*/
ISR(RTC_PIT_vect) {
  RTC.PITINTFLAGS = RTC_PI_bm; 
  brightnessIndex++; // next down the line in the array sequence
  if (brightnessIndex >= sizeof(brightness) / sizeof(brightness[0])) {
    // done the sequence? power down
    powerDown();
  } else {
    // chance the brightness of the LED
    TCA0.SINGLE.CMP1 = brightness[brightnessIndex];
  }
}


/*
This function is to update the EEPROM with a given ambient light brightness
*/
void storeThreshold(uint16_t lightLevel) {
  // no need to store such a fine grain number, bring it down to a single byte
  uint8_t compressedThreshold = lightLevel / 4;  // Scale to 8-bit (0-255)
  // Store the value 
  EEPROM.write(0, compressedThreshold);

  // This part is for feedback
  // Turn on the LED solid, to indicate that writing has happened
  PORTA.OUTSET = (1 << LED_PIN);  // long blink
  delay(3000);
  PORTA.OUTCLR = (1 << LED_PIN);
  delay(250);
  // output the requested value to be stored.
  blinkNumber(lightLevel);
  delay(3000);
  PORTA.OUTSET = (1 << LED_PIN);  // long blink
  delay(3000);
  PORTA.OUTCLR = (1 << LED_PIN);
  delay(250);
  // output the stored value (to make sure it was prettu much the same)
  blinkNumber(readStoredThreshold());
}

/*
Reads the ambient light threshold from the EEPROM
*/
uint16_t readStoredThreshold() {
  uint8_t storedThreshold = EEPROM.read(0);  // Read address 0
  // scale up to 2 bytes
  return storedThreshold * 4;
}


void blinkNumber(uint16_t value) {
  uint8_t hundreds = value / 100;
  uint8_t tens = (value / 10) % 10;
  uint8_t ones = value % 10;

  blinkDigit(hundreds);
  delay(1000);  // 1-second pause

  blinkDigit(tens);
  delay(1000);

  blinkDigit(ones);
  delay(1000);
}

// Function to blink LED N times for a given digit
void blinkDigit(uint8_t digit) {
  for (uint8_t i = 0; i < digit; i++) {
    PORTA.OUTSET = (1 << LED_PIN);  // Turn LED ON
    delay(250);
    PORTA.OUTCLR = (1 << LED_PIN);  // Turn LED OFF
    delay(250);
  }
}


void loop() {
  //no op
}
