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
#define GROUP00 {MODE010, MODE050, MODE100}
#define GROUP01 {MODE010, MODE050, MODE100, MODE_STROBE}
#define GROUP02 {MODE_RAMPING, MODE010, MODE050, MODE100}
#define GROUP03 {MODE_RAMPING, MODE007, MODE020, MODE060, MODE100}
#define GROUP04 {MODE100, MODE050, MODE010}
#define GROUP05 {MODE100, MODE050, MODE010, MODE_STROBE}
#define GROUP06 {MODE100}

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
#define PROGRAM_SPRESS 9 // Number of short presses to enter program mode, from 0
#define PROGRAM_MODES 5 // Number of program modes
#define PROGRAM_PAUSE 150 // Pause between blinks (5ms)
#define PROGRAM_OUT MODE020 // Output level (1-255)
#define PROGRAM_BLINKS 20 // Number of strobe blinks when entering program mode
#define PROGRAM_DELAY 45 // Delay between blinks when entering program mode (5ms)
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
uint8_t strobe_delay; // Strobe frequency delay
//### Globals end ###

static void delay_5ms(uint8_t n) { // Use own delay function
  while(n-- > 0) {
    _delay_ms(5);
  }
}

void get_mypwm(const uint8_t modes[], uint8_t mode_cnt) {
  if (spress_cnt >= PROGRAM_SPRESS && spress_cnt < PROGRAM_SPRESS + PROGRAM_MODES) {
    mypwm = 254; // Enter program mode
  } else {
    if (mode_idx >= mode_cnt) {
      mode_idx = 0; // Wrap around
    }
    mypwm = modes[mode_idx]; // Get mode identifier/output level
  }
}

inline void get_mode() { // Get mode and store with short press indicator
  uint8_t groupint;
  uint8_t MODE_RAMPING;
  uint8_t oldpos;
  groupint = eeprom_read_byte((const uint8_t *)(uint16_t)0); // Number of group array
  mode_memory = eeprom_read_byte((const uint8_t *)(uint16_t)1) - 1; // Mode memory
  ftimer = eeprom_read_byte((const uint8_t *)(uint16_t)2) - 1; // Number of timer seconds
  strobe_delay = eeprom_read_byte((const uint8_t *)(uint16_t)3); // Strobe delay
  MODE_RAMPING = eeprom_read_byte((const uint8_t *)(uint16_t)4); // Ramping output level
  if (MODE_RAMPING > 250) {
    MODE_RAMPING = 255;
  }
  for (oldpos = 5; oldpos < 63; oldpos++) {
    mode_idx = eeprom_read_byte((const uint8_t *)(uint16_t)oldpos);
    if (mode_idx != 0xff) {
      break;
    }
  }
  eepos = oldpos + 1; // Wear leveling, use next cell
  uint8_t oldspos = eepos;
  if (eepos > 62) {
    eepos = 5;
  }
  if (mode_idx & 0x10) { // Indicates we did a short press last time
    mode_idx &= 0x0f; // Remove short press indicator
    mode_idx++; // Go to the next mode
    spress_cnt = eeprom_read_byte((const uint8_t *)(uint16_t)oldspos);
    if (spress_cnt != 0xff) {
      spress_cnt++;
    }
  }
  //const uint8_t memarray[] = MODE_MEMORY;
  //mode_memory = memarray[modesarr];
  //if (modesarr > 7) {
  //  mode_memory = 1;
  //}
  if (groupint == 2) {
      const uint8_t modes[] = GROUP01;
      get_mypwm(modes, sizeof(modes));
  } else if (groupint == 3) {
      const uint8_t modes[] = GROUP02;
      get_mypwm(modes, sizeof(modes));
  } else if (groupint == 4) {
      const uint8_t modes[] = GROUP03;
      get_mypwm(modes, sizeof(modes));
  } else if (groupint == 5) {
      const uint8_t modes[] = GROUP04;
      get_mypwm(modes, sizeof(modes));
  } else if (groupint == 6) {
      const uint8_t modes[] = GROUP05;
      get_mypwm(modes, sizeof(modes));
  } else if (groupint == 7) {
      const uint8_t modes[] = GROUP06;
      get_mypwm(modes, sizeof(modes));
  } else {
      const uint8_t modes[] = GROUP00;
      get_mypwm(modes, sizeof(modes));
  }
  eeprom_write_byte((uint8_t *)(uint16_t)(eepos), (mode_idx | 0x10)); // Store current mode
  eeprom_write_byte((uint8_t *)(uint16_t)(oldpos), 0xff); // Erase old mode
  eeprom_write_byte((uint8_t *)(uint16_t)(eepos + 1), spress_cnt); // Store short press counter
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
  while(1) {
    set_output(STROBE_ON_OUT, 0);
    delay_5ms(strobe_delay);
    set_output(STROBE_OFF_OUT, 0);
    delay_5ms(strobe_delay);
  }
}

static inline void mode_program(void) {
  uint8_t i;
  uint8_t j = spress_cnt - PROGRAM_SPRESS;
  for (i = 0; i < PROGRAM_BLINKS; i++) {
    set_output(PROGRAM_OUT, 0);
    delay_5ms(PROGRAM_DELAY);
    set_output(0, 0);
    delay_5ms(PROGRAM_DELAY);
  }
  for (i = 0; i < 255;) {
    i++;
    set_output(0, 0);
    delay_5ms(PROGRAM_PAUSE);
    set_output(PROGRAM_OUT, 0);
    eeprom_write_byte((uint8_t *)(uint16_t)(j), i);
    delay_5ms(PROGRAM_PAUSE);
  }
}

ISR(WDT_vect) { // WatchDogTimer interrupt
  static uint8_t lowbatt_cnt = 0;
  static uint8_t ticks = 0;
  if (ticks < 255) ticks++;
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
  if (BATT_MON && ticks == 255) {
    ticks = 255 - BATT_TIMEOUT; // Battery monitoring interval
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
  set_output(0, 1); // Set phase pwm as default
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

