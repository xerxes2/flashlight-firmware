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
#define MODE010 26
#define MODE015 28
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
#define MODE_BEACON 253 // Just an id dummy number, must not be used for other modes!

// Mode arrays
#define MODES00 {MODE100}
#define MODES01 {MODE050, MODE100}
#define MODES02 {MODE010, MODE050, MODE100}
#define MODES03 {MODE100, MODE050}
#define MODES04 {MODE100, MODE050, MODE010}
// Mode arrays settings
#define MODES_COUNT 9 // Number of available groups
#define MODE_MEMORY {0, 0, 0, 0, 0, 1, 1, 1, 1} // Set mode memory for groups, 0 off and 1 on

// Beacon settings
#define BEACON_ON 1000  // Beacon on time (ms)
#define BEACON_OFF 3000 // Beacon off time (ms)
#define BEACON_ON_OUT 255 // Beacon output level (0-255)
#define BEACON_OFF_OUT 0 // Beacon output level (0-255)
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
#define MODE100_TIMEOUT 0 // Number of seconds before lower output, 0 off (0-255)
#define MODE100_LOW 150 // Output level (1-255)
#define BLINK_DELAY 200 // Pause between SOS groups (ms)

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
  modesarr = eeprom_read_byte((const uint8_t *)(uint16_t)0); // Number of modes array
  for (oldpos = 1; oldpos < 64; oldpos++) {
    mode_idx = eeprom_read_byte((const uint8_t *)(uint16_t)oldpos);
    if (mode_idx != 0xff) {
      break;
    }
  }
  eepos = oldpos + 1; // Wear leveling, use next cell
  uint8_t spos = eepos + 1;
  if (eepos > 63) {
    eepos = 1;
    spos = 2;
  } else if (eepos == 63) {
    spos = 1;
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
  if (modesarr == 1 || modesarr == 5) {
      const uint8_t modes[] = MODES01;
      get_mypwm(modes, sizeof(modes));
  } else if (modesarr == 2 || modesarr == 6) {
      const uint8_t modes[] = MODES02;
      get_mypwm(modes, sizeof(modes));
  } else if (modesarr == 3 || modesarr == 7) {
      const uint8_t modes[] = MODES03;
      get_mypwm(modes, sizeof(modes));
  } else if (modesarr == 4 || modesarr == 8) {
      const uint8_t modes[] = MODES04;
      get_mypwm(modes, sizeof(modes));
  } else {
      const uint8_t modes[] = MODES00;
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

static inline void mode_beacon(void) {
  while(1){
    set_output(BEACON_ON_OUT, 1);
    _delay_ms(BEACON_ON);
    set_output(BEACON_OFF_OUT, 1);
    _delay_ms(BEACON_OFF);
  }
}

void group_blink(uint8_t pcs, uint8_t lvl) {
  uint8_t i;
  for (i = 0; i < pcs; i++) {
    set_output(lvl, 1);
    _delay_ms(BLINK_DELAY);
    set_output(0, 1);
    _delay_ms(BLINK_DELAY);
  }
}

static inline void mode_program(void) {
  uint8_t i;
  group_blink(PROGRAM_BLINKS, PROGRAM_OUT);
  for (i = 0; i < MODES_COUNT; i++) {
    set_output(0, 1);
    _delay_ms(PROGRAM_PAUSE);
    set_output(PROGRAM_OUT, 1);
    eeprom_write_byte((uint8_t *)(uint16_t)(0), i); // Store modes array number
    _delay_ms(PROGRAM_PAUSE);
  }
}

uint8_t low_voltage(uint8_t voltage_val) {
  static uint8_t lowbatt_cnt = 0;
  ADCSRA |= (1 << ADSC); // Start conversion
  while (ADCSRA & (1 << ADSC)); // Wait for completion
  if (ADCH < voltage_val) { // See if voltage is lower than what we were looking for
    if (++lowbatt_cnt > 10) { // See if it's been low for a while
      lowbatt_cnt = 0;
      return 1;
    }
  } else {
    lowbatt_cnt = 0;
  }
  return 0;
}

ISR(WDT_vect) { // WatchDogTimer interrupt
  static uint8_t lowbatt_mode = 0;
  static uint8_t ticks = 0;
  if (ticks < 255) ticks++;
  if (ticks == MODE_TIMEOUT) { // Lock mode
    if (mode_memory) { // Store current mode
      eeprom_write_byte((uint8_t *)(uint16_t)(eepos), mode_idx);
    } else { // No mode memory
      eeprom_write_byte((uint8_t *)(uint16_t)(eepos), 0);
    }
  }
  if (mypwm == 255 && ticks == MODE100_TIMEOUT) { // MODE100 timeout
    mypwm = MODE100_LOW;
    set_output(mypwm, 1);
  }
  if (BATT_MON && ticks == 255) {
    ticks = 255 - BATT_TIMEOUT; // Battery monitoring interval
    if (!lowbatt_mode) {
      if (low_voltage(ADC_LOW)) {
        lowbatt_mode = 1;
        if (mypwm > ADC_LOW_OUT) {
            mypwm = ADC_LOW_OUT; // Lower output if not in special mode
        }
        set_output(mypwm, 1);
      }
    } else {
      if (low_voltage(ADC_CRIT)) {
        WDT_off(); // Disable WDT so it doesn't wake us up
        ADCSRA &= ~(1<<7); // ADC off
        DDRB = (0 << PWM_PIN); // Set PWM pin to input
        set_sleep_mode(SLEEP_MODE_PWR_DOWN); // Power down as many components as possible
        sleep_mode();
      }
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
  } else {
    set_output(mypwm, 1);
  }
  while(1) {
    sleep_mode(); // Enter sleep mode
  }
  return 0;
}

