//
// SharkDrv 0.1
//
// License: Released to the Public Domain.
//
// For use in 105d type drivers, no support for stars.
//

//###############################################################
//  Start user tweaking, don't comment out, just edit values
//###############################################################

// Mode output levels
#define MODE001 3
#define MODE002 5
#define MODE003 8
#define MODE004 10
#define MODE005 13
#define MODE007 18
#define MODE010 26
#define MODE015 38
#define MODE020 51
#define MODE025 64
#define MODE030 77
#define MODE035 89
#define MODE040 102
#define MODE045 115
#define MODE050 128
#define MODE060 153
#define MODE070 179
#define MODE080 204
#define MODE100 255
// Identifiers for special modes
#define MODE_PROGRAM 254 // Just an id dummy number, must not be used for other modes!
#define MODE_STROBE 253 // Just an id dummy number, must not be used for other modes!

// Mode groups
#define GROUP00 {MODE100}
#define GROUP01 {MODE050, MODE100}
#define GROUP02 {MODE010, MODE050, MODE100}
#define GROUP03 {MODE010, MODE050, MODE100, MODE_STROBE}
#define GROUP04 {MODE100, MODE050}
#define GROUP05 {MODE100, MODE050, MODE010}
// Mode group settings
#define GROUP_COUNT 11 // Number of available groups
#define MODE_MEMORY {0, 0, 0, 0, 0, 0, 1, 1, 1, 1, 1} // Set mode memory for groups, 0 off and 1 on

// Strobe settings
#define STROBE_ON 40  // Strobe on time (ms)
#define STROBE_OFF 40 // Strobe off time (ms)
#define STROBE_ON_OUT 255 // Strobe output level (0-255)
#define STROBE_OFF_OUT 0 // Strobe output level (0-255)
// Battery monitoring
#define BATT_MON 1 // Enable battery monitoring, 0 off and 1 on
#define BATT_TIMEOUT 30 // Number of seconds between checks (10-200)
#define ADC_LOW 130 // When do we start ramping
#define ADC_CRIT 120 // When do we shut the light off
#define ADC_LOW_OUT 20 // Output level in low battery mode (0-255)
// Misc settings
#define MODE_TIMEOUT 2 // Number of seconds before mode is saved (1-9)
#define FAST_PWM_START 8 // Above what output level should we switch from phase correct to fast-PWM?
#define PROGRAM_PAUSE 1500 // Pause between blinks (ms)
#define PROGRAM_OUT 100 // Output level (1-255)
#define PROGRAM_BLINKS 8 // Number of blinks when entering program mode
#define BLINK_DELAY 200 // Pause between program blinks (ms)
#define MODE100_LOW MODE050 // Output level (1-255)

//################################
//  End user tweaking
//################################

#define F_CPU 4800000UL // CPU frequency
#define PWM_PIN PB1 // Default PWM pin
#define PWM_LVL OCR0B // OCR0B is the output compare register for PB1
#define PWM_FAST 0x23 // Fast-PWM
#define PWM_PHASE 0x21 // Phase corrected PWM
#define ADC_CHANNEL 0x01 // MUX 01 corresponds with PB2
#define ADC_DIDR ADC1D // Digital input disable bit corresponding with PB2
#define ADC_PRSCL 0x06 // clk/64

#include <avr/io.h>
#include <util/delay.h>
#include <avr/interrupt.h>
#include <avr/wdt.h>
#include <avr/eeprom.h>
#include <avr/sleep.h>

//### Globals start ###
uint8_t eepos; // Mode byte position in eeprom
uint8_t mode_idx = 0; // Mode position in modes array
uint8_t mode_memory; // Mode memory
uint8_t mypwm = 100; // Mode identifier/output level
uint8_t spress_cnt = 0; // Short press counter
uint8_t ftimer; // Full mode timer
//### Globals end ###

void get_mypwm(const uint8_t modes[], uint8_t mode_cnt) {
  if (spress_cnt == 7) { // Enter program mode
    spress_cnt = 0;
    mypwm = 254;
  } else {
    if (mode_idx >= mode_cnt) {
      mode_idx = 0; // Wrap around
    }
    mypwm = modes[mode_idx]; // Get mode identifier/output level
  }
}

inline void get_mode() { // Get mode and store with short press indicator
  uint8_t modesarr;
  uint8_t oldpos;
  modesarr = eeprom_read_byte((const uint8_t *)(uint16_t)0); // Number of group array
  ftimer = eeprom_read_byte((const uint8_t *)(uint16_t)1); // Number of timer seconds
  for (oldpos = 2; oldpos < 64; oldpos++) {
    mode_idx = eeprom_read_byte((const uint8_t *)(uint16_t)oldpos);
    if (mode_idx != 0xff) {
      break;
    }
  }
  eepos = oldpos + 1; // Wear leveling, use next cell
  uint8_t spos = eepos + 1;
  if (eepos > 63) {
    eepos = 2;
    spos = 3;
  } else if (eepos == 63) {
    spos = 2;
  }
  if (mode_idx & 0x10) { // Indicates we did a short press last time
    mode_idx &= 0x0f; // Remove short press indicator
    mode_idx++; // Go to the next mode
    spress_cnt = eeprom_read_byte((const uint8_t *)(uint16_t)eepos);
    if (spress_cnt != 0xff) {
      spress_cnt++;
    }
  }
  const uint8_t memarray[] = MODE_MEMORY;
  mode_memory = memarray[modesarr];
  if (modesarr == 1 || modesarr == 6) {
      const uint8_t modes[] = GROUP01;
      get_mypwm(modes, sizeof(modes));
  } else if (modesarr == 2 || modesarr == 7) {
      const uint8_t modes[] = GROUP02;
      get_mypwm(modes, sizeof(modes));
  } else if (modesarr == 3 || modesarr == 8) {
      const uint8_t modes[] = GROUP03;
      get_mypwm(modes, sizeof(modes));
  } else if (modesarr == 4 || modesarr == 9) {
      const uint8_t modes[] = GROUP04;
      get_mypwm(modes, sizeof(modes));
  } else if (modesarr == 5 || modesarr == 10) {
      const uint8_t modes[] = GROUP05;
      get_mypwm(modes, sizeof(modes));
  } else {
      const uint8_t modes[] = GROUP00;
      get_mypwm(modes, sizeof(modes));
  }
  eeprom_write_byte((uint8_t *)(uint16_t)(eepos), (mode_idx | 0x10)); // Store current mode
  eeprom_write_byte((uint8_t *)(uint16_t)(oldpos), 0xff); // Erase old mode
  eeprom_write_byte((uint8_t *)(uint16_t)(spos), spress_cnt); // Store short press counter
}

inline void WDT_on() { // Setup watchdog timer to only interrupt, not reset
  cli(); // Disable interrupts
  wdt_reset(); // Reset the WDT
  WDTCR |= (1<<WDCE) | (1<<WDE); // Start timed sequence
  WDTCR = (1<<WDTIE) | (1<<WDP2) | (1<<WDP1); // Enable interrupt every second
  sei(); // Enable interrupts
}

inline void WDT_off() { // Stop watchdog timer
  cli(); // Disable interrupts
  wdt_reset(); // Reset the WDT
  MCUSR &= ~(1<<WDRF); // Clear Watchdog reset flag
  WDTCR |= (1<<WDCE) | (1<<WDE); // Start timed sequence
  WDTCR = 0x00; // Disable WDT
  sei(); // Enable interrupts
}

inline void ADC_ctrl(void) { // Battery monitoring
  if (BATT_MON) {
    ADMUX  = (1 << REFS0) | (1 << ADLAR) | ADC_CHANNEL; // 1.1v reference, left-adjust, ADC1/PB2
    DIDR0 |= (1 << ADC_DIDR); // disable digital input on ADC pin to reduce power consumption
    ADCSRA = (1 << ADEN ) | (1 << ADSC ) | ADC_PRSCL; // enable, start, prescale
  } else {
    ADCSRA &= ~(1<<7); // ADC off
  }
}

void set_output(uint8_t pwm_lvl, uint8_t pwm_mode) {
  if (pwm_mode) {
    if (pwm_lvl > FAST_PWM_START && pwm_lvl != 255) {
      TCCR0A = PWM_FAST; // Fast-PWM
    } else {
      TCCR0A = PWM_PHASE; // Phase corrected PWM
    }
  }
  PWM_LVL = pwm_lvl;
}

static inline void mode_strobe(void) {
  set_output(0, 1);
  while(1){
    set_output(STROBE_ON_OUT, 0);
    _delay_ms(STROBE_ON);
    set_output(STROBE_OFF_OUT, 0);
    _delay_ms(STROBE_OFF);
  }
}

static inline void mode_program(void) {
  uint8_t i;
  uint8_t j;
  uint8_t k = GROUP_COUNT;
  for (j = 0; j < 2; j++) {
    for (i = 0; i < PROGRAM_BLINKS; i++) {
      set_output(PROGRAM_OUT, 1);
      _delay_ms(BLINK_DELAY);
      set_output(0, 1);
      _delay_ms(BLINK_DELAY);
    }
    for (i = 0; i < k; i++) {
      set_output(0, 1);
      _delay_ms(PROGRAM_PAUSE);
      set_output(PROGRAM_OUT, 1);
      eeprom_write_byte((uint8_t *)(uint16_t)(j), i);
      _delay_ms(PROGRAM_PAUSE);
    }
    k = 255;
  }
}

ISR(WDT_vect) { // WatchDogTimer interrupt
  static uint8_t lowbatt_cnt = 0;
  static uint8_t ticks = 0;
  if (ticks < 254) ticks++;
  if (ticks == MODE_TIMEOUT) { // Lock mode
    if (mode_memory) { // Store current mode
      eeprom_write_byte((uint8_t *)(uint16_t)(eepos), mode_idx);
    } else { // No mode memory
      eeprom_write_byte((uint8_t *)(uint16_t)(eepos), 0);
    }
  }
  if (mypwm == 255 && ticks == ftimer) { // MODE100 timeout
    mypwm = MODE100_LOW;
    set_output(mypwm, 1);
  }
  if (BATT_MON && ticks == 254) {
    ticks = 254 - BATT_TIMEOUT; // Battery monitoring interval
    ADCSRA |= (1 << ADSC); // Start conversion
    while (ADCSRA & (1 << ADSC)); // Wait for completion
    if (ADCH < ADC_CRIT && (++lowbatt_cnt > 10)) { // See if voltage is lower than what we were looking for
      WDT_off(); // Disable WDT so it doesn't wake us up
      ADCSRA &= ~(1<<7); // ADC off
      DDRB = (0 << PWM_PIN); // Set PWM pin to input
      set_sleep_mode(SLEEP_MODE_PWR_DOWN); // Power down as many components as possible
      sleep_mode();
    }
  }
}

int main(void) {
  DDRB = (1 << PWM_PIN); // Set PWM pin to output
  ACSR |= (1<<7); // AC (Analog Comparator) off
  TCCR0B = 0x01; // pre-scaler for timer (1 => 1, 2 => 8, 3 => 64...)
  set_sleep_mode(SLEEP_MODE_IDLE); // Will allow us to go idle between WDT interrupts
  ADC_ctrl(); // Battery monitoring
  WDT_on(); // Start watchdogtimer
  get_mode(); // Get mode identifier and store with short press indicator
  if (mypwm == MODE_PROGRAM) {
    mode_program();
  } else if (mypwm == MODE_STROBE) {
    mode_strobe();
  } else {
    set_output(mypwm, 1);
  }
  while(1) {
    sleep_mode(); // Enter sleep mode
  }
  return 0;
}

