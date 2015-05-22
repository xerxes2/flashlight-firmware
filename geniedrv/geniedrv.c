//
// GenieDrv 0.1
//
// License: Released to the Public Domain.
//
// For use in 105d type drivers, no support for stars.
//

//###############################################################
//  Start user tweaking, don't comment out, just edit values
//###############################################################

// Identifiers for special modes
#define MODE_PROGRAM 254 // Just an id dummy number, must not be used for other modes!
#define MODE_BATT 253 // Just an id dummy number, must not be used for other modes!
#define MODE_STROBE 1 // Just an id dummy number, must not be used for other modes!
#define MODE_SOS 2 // Just an id dummy number, must not be used for other modes!

// Strobe settings
#define STROBE_ON 40  // Strobe on time (ms)
#define STROBE_OFF 40 // Strobe off time (ms)
#define STROBE_ON_OUT 255 // Strobe output level (0-255)
#define STROBE_OFF_OUT 0 // Strobe output level (0-255)
// SOS settings
#define SOS_PAUSE 255 // Pause between SOS groups (5ms)
#define SOS_DOT 45 // Morse code dot duration (5ms)
#define SOS_OUT 255 // SOS output level (1-255)
// Battery monitoring
#define BATT_MON 1 // Enable battery monitoring, 0 off and 1 on
#define BATT_TIMEOUT 30 // Number of seconds between checks (10-200)
#define ADC_LOW 130 // When do we start ramping
#define ADC_CRIT 120 // When do we shut the light off
#define ADC_LOW_OUT 20 // Output level in low battery mode (0-255)
// Battery check
#define BATT_CHECK 29 // Number of short presses, from 0
#define ADC_100 174 // ADC value for 100% full (4.2V resting)
#define ADC_75 166 // ADC value for 75% full (4.0V resting)
#define ADC_50 158 // ADC value for 50% full (3.8V resting)
#define ADC_25 145 // ADC value for 25% full (3.5V resting)
#define ADC_0 124 // ADC value for 0% full (3.0V resting)
// Misc settings
#define MODE_TIMEOUT 2 // Number of seconds before mode is saved (1-9)
#define FAST_PWM_START 8 // Above what output level should we switch from phase correct to fast-PWM?
#define PROGRAM_SPRESS 9 // Number of short presses to enter program mode, from 0
#define PROGRAM_MODES 11 // Number of program modes
#define PROGRAM_PAUSE 150 // Pause between blinks (5ms)
#define PROGRAM_OUT 50 // Output level (1-255)
#define PROGRAM_BLINKS 20 // Number of strobe blinks when entering program mode
#define PROGRAM_DELAY 40 // Delay between blinks when entering program mode (5ms)
#define MODE_FULL_LOW 127 // Output level (1-255)

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
uint8_t ftimer; // Full mode timer
uint8_t spress_cnt = 0; // Short press counter
uint8_t strobe_delay; // Strobe frequency delay
//### Globals end ###

static void delay_5ms(uint8_t n) { // Use own delay function
  while(n-- > 0) {
    _delay_ms(5);
  }
}

inline void get_mode() { // Get mode and store with short press indicator
  uint8_t mode_cnt; // Number of modes
  uint8_t oldpos;
  mode_memory = eeprom_read_byte((const uint8_t *)(uint16_t)0) - 1; // Mode memory
  ftimer = eeprom_read_byte((const uint8_t *)(uint16_t)1) - 1; // Number of timer seconds
  strobe_delay = eeprom_read_byte((const uint8_t *)(uint16_t)2); // Strobe delay
  mode_cnt = eeprom_read_byte((const uint8_t *)(uint16_t)3); // Number of modes
  if (mode_cnt > 7) {
    mode_cnt = 1;
  }
  for (oldpos = 11; oldpos < 63; oldpos++) {
    mode_idx = eeprom_read_byte((const uint8_t *)(uint16_t)oldpos);
    if (mode_idx != 0xff) {
      break;
    }
  }
  eepos = oldpos + 1; // Wear leveling, use next cell
  uint8_t oldspos = eepos;
  if (eepos > 62) {
    eepos = 11;
  }
  if (mode_idx & 0x10) { // Indicates we did a short press last time
    mode_idx &= 0x0f; // Remove short press indicator
    mode_idx++; // Go to the next mode
    spress_cnt = eeprom_read_byte((const uint8_t *)(uint16_t)(oldspos));
    if (spress_cnt != 0xff) {
      spress_cnt++;
    }
  }
  if (spress_cnt >= PROGRAM_SPRESS && spress_cnt < PROGRAM_SPRESS + PROGRAM_MODES) {
    mypwm = 254; // Enter program mode
  } else if (spress_cnt == BATT_CHECK) {
    mypwm = 253;
  } else {
    if (mode_idx >= mode_cnt) {
      mode_idx = 0; // Wrap around
    }
    mypwm = eeprom_read_byte((const uint8_t *)(uint16_t)(mode_idx + 4)); // Get mode identifier/output level
    if (mypwm > 250) {
      mypwm = 255;
    }
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

void morse_blink(uint8_t dot, uint8_t pcs, uint8_t lvl) { // Morse code
  uint8_t i;
  for (i = 0; i < pcs; i++) {
    set_output(lvl, 0);
    if (dot) {
      delay_5ms(SOS_DOT);
    } else {
      delay_5ms(3 * SOS_DOT);
    }
    set_output(0, 0);
    delay_5ms(SOS_DOT);
  }
}

static inline void mode_sos(void) {
  uint8_t i;
  while(1){
    for (i = 0; i < 3; i++) { // SOS group
      if (i != 1) {
        morse_blink(1, 3, SOS_OUT); // Three short
      } else {
        morse_blink(0, 3, SOS_OUT); // Three long
      }
     delay_5ms(3 * SOS_DOT); // Pause between chars
    }
    delay_5ms(SOS_PAUSE); // Pause between groups
  }
}

static inline void mode_batt(void) {
  morse_blink(1, PROGRAM_BLINKS, PROGRAM_OUT);
  delay_5ms(PROGRAM_PAUSE);
  ADCSRA |= (1 << ADSC); // Start conversion
  while (ADCSRA & (1 << ADSC)); // Wait for completion
  uint8_t i = 0;
  if (ADCH > ADC_100) {
    i = 4;
  } else if (ADCH > ADC_75) {
    i = 3;
  } else if (ADCH > ADC_50) {
    i = 2;
  } else if (ADCH > ADC_25) {
    i = 1;
  }
  morse_blink(0, i, PROGRAM_OUT);
  set_output(PROGRAM_OUT, 0);
}

static inline void mode_program(void) {
  uint8_t i;
  uint8_t j = spress_cnt - PROGRAM_SPRESS;
  morse_blink(1, PROGRAM_BLINKS, PROGRAM_OUT);
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
  if (mypwm == 255 && ticks == ftimer) { // Full mode timer
    mypwm = MODE_FULL_LOW;
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
  } else if (mypwm == MODE_SOS) {
    mode_sos();
  } else if (mypwm == MODE_BATT) {
    mode_batt();
  } else {
    set_output(mypwm, 1);
  }
  while(1) {
    sleep_mode(); // Enter sleep mode
  }
  return 0;
}

