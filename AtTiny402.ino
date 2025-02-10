#define LED_PIN 1            // PA1 = Physical Pin 4
#define PHOTO_PIN 3          // PA3 = Physical Pin 7
#define LIGHT_THRESHOLD 500  // Lower = brighter; Adjust based on testing

const uint16_t brightness[] = {
  0, 1, 1, 1, 2, 2, 2, 3, 3, 4, 6, 8, 12, 18, 27, 40, 58,
  82, 115, 160, 220, 300, 1024, 220, 160, 115, 82, 58, 40, 27, 18, 12, 8, 6, 4, 3, 2, 2, 1, 1, 0
};

volatile uint8_t brightnessIndex = 0;

void setup() {
  PORTA.DIRSET = (1 << LED_PIN);  // Set LED pin as output

  // Set ADC reference to VDD (3.3V)
  VREF.CTRLA = VREF_ADC0REFSEL_2V5_gc;
  ADC0.CTRLA = ADC_ENABLE_bm | ADC_RESSEL_10BIT_gc;  // Enable ADC, 10-bit resolution
  ADC0.MUXPOS = PHOTO_PIN;                           // Select PA3 as ADC input

  // Recheck light every WDT wakeup
  checkAndStartLighthouse();
}

// Reads the current light level from ADC
uint16_t readLightLevel() {
  ADC0.COMMAND = ADC_STCONV_bm;  // Start ADC conversion
  while (!(ADC0.INTFLAGS & ADC_RESRDY_bm))
    ;                             // Wait for conversion
  ADC0.INTFLAGS = ADC_RESRDY_bm;  // Clear flag
  return ADC0.RES;                // Return ADC result
}

// Checks if light is dark enough and starts lighthouse
void checkAndStartLighthouse() {
  uint16_t lightLevel = readLightLevel();

  if (lightLevel >= LIGHT_THRESHOLD) {  // Lower ADC value = brighter
    powerDown();                        // Too bright, sleep again
  } else {
    startLighthouse();
  }
}

// Starts the lighthouse effect
void startLighthouse() {
  TCA0.SINGLE.CTRLB = TCA_SINGLE_CMP1EN_bm | TCA_SINGLE_WGMODE_DSBOTTOM_gc;
  TCA0.SINGLE.PER = 1023;
  TCA0.SINGLE.CMP1 = brightness[brightnessIndex];
  TCA0.SINGLE.CTRLA = TCA_SINGLE_ENABLE_bm | TCA_SINGLE_CLKSEL_DIV8_gc;

  // Configure RTC with 32.768 kHz oscillator
  RTC.CLKSEL = RTC_CLKSEL_OSC32K_gc;
  RTC.PITINTCTRL = RTC_PI_bm;
  RTC.PITCTRLA = RTC_PERIOD_CYC1024_gc | RTC_PITEN_bm;
  // Enable Watchdog Timer (WDT) for 8-second reset
  _PROTECTED_WRITE(WDT.CTRLA, WDT_PERIOD_8KCLK_gc | WDT_WINDOW_OFF_gc);

  sei();  // Enable global interrupts
}

// Stops the lighthouse effect and powers down
void powerDown() {
  TCA0.SINGLE.CTRLA = 0;          // Disable PWM
  RTC.PITCTRLA = 0;               // Disable RTC timer
  PORTA.OUTCLR = (1 << LED_PIN);  // Ensure LED is fully OFF

  // Enable Watchdog Timer (WDT) for 8-second reset
  _PROTECTED_WRITE(WDT.CTRLA, WDT_PERIOD_8KCLK_gc | WDT_WINDOW_OFF_gc);

  // Enter deep sleep mode (WDT wakes up after 8 sec)
  _PROTECTED_WRITE(SLPCTRL.CTRLA, SLPCTRL_SMODE_PDOWN_gc | SLPCTRL_SEN_bm);
  asm("sleep");
}

// RTC Interrupt to cycle brightness
ISR(RTC_PIT_vect) {
  RTC.PITINTFLAGS = RTC_PI_bm;  // Clear interrupt flag

  brightnessIndex++;

  if (brightnessIndex >= sizeof(brightness) / sizeof(brightness[0])) {
    brightnessIndex = 0;
    powerDown();  // Shut down after one full cycle
  } else {
    TCA0.SINGLE.CMP1 = brightness[brightnessIndex];  // Apply brightness level
  }
}

void loop() {
  // Nothing here! Everything runs via interrupts
}
