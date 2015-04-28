//
// TacDrv 0.1
//
// License: Released to the Public Domain.
//
// For use in 105d type drivers, no support for stars.
//

//###############################################################
//  Start user tweaking, don't comment out, just edit values
//###############################################################

// Mode output levels
#define MODE_MOON 10 // Output level (1-255)
#define MODE_LOW 20 // Output level (1-255)
#define MODE_MID 120 // Output level (1-255)
#define MODE_HIGH 200 // Output level (1-255)
#define MODE_FULL 255 // Full output, no PWM
// Identifiers for special modes
#define MODE_STROBE 254 // Just an id dummy number, must not be used for other modes!
#define MODE_BEACON 253 // Just an id dummy number, must not be used for other modes!
#define MODE_SOS 252 // Just an id dummy number, must not be used for other modes!

// Modes, inside the brackets, as many as you want
#define MODES {MODE_LOW, MODE_MID, MODE_FULL, MODE_STROBE}

// Strobe settings
#define STROBE_ON 33  // Strobe frequency, tactical should be 10-20hz,
#define STROBE_OFF 33 // default 15hz (1000ms/15hz, 33ms on/off)
#define STROBE_ON_OUT 255 // Output level (0-255)
#define STROBE_OFF_OUT 0 // Output level (0-255)
#define STROBE_GROUP 100 // Strobe group cycle (on/off) counter (10-255)
#define STROBE_PAUSE 0 // Pause between strobe groups (ms)
#define STROBE_PAUSE_OUT 255 // Output level (0-255)
// Beacon settings
#define BEACON_ON 1000  // Beacon on time (ms)
#define BEACON_OFF 3000 // Beacon off time (ms)
#define BEACON_ON_OUT 255 // Beacon output level (0-255)
#define BEACON_OFF_OUT 0 // Beacon output level (0-255)
// SOS settings
#define SOS_PAUSE 5000 // Pause between SOS groups (ms)
#define SOS_DOT 200 // Morse code dot duration (ms)
#define SOS_OUT 255 // SOS output level (0-255)
// Battery monitoring
#define BATT_MON 1 // Enable battery monitoring, 0 off and 1 on
#define BATT_TIMEOUT 20 // Number of WTD ticks between checks, each tick is 1s (10-200)
#define ADC_LOW 130 // When do we start ramping
#define ADC_CRIT 120 // When do we shut the light off
#define ADC_LOW_OUT 20 // Output level in low battery mode (0-255)
// Misc settings
#define MODE_MEMORY 0 // Mode memory, 0 off and 1 on
#define MODE_TIMEOUT 2 // Number of WTD ticks before mode is saved, each tick is 1s (1-255)
#define FAST_PWM_START 8 // Above what output level should we switch from phase correct to fast-PWM?
#define MORSE_CODE 0 // Morse code output level, existing normal mode (i.e 255), 0 off
#define MODE_FULL_TIMEOUT 0 // Number of WTD ticks before lower output, each tick is 1s, 0 off (0-255)
#define MODE_FULL_LOW 150 // Output level (1-255)

//################################
//  End user tweaking
//################################

#define F_CPU 4800000UL // CPU frequency
#define PWM_PIN PB1 // Default PWM pin
#define PWM_LVL OCR0B // OCR0B is the output compare register for PB1
#define PWM_FAST 0b00100011 // fast-PWM
#define PWM_PHASE 0b00100001 // phase corrected PWM
#define ADC_CHANNEL 0x01 // MUX 01 corresponds with PB2
#define ADC_DIDR ADC1D // Digital input disable bit corresponding with PB2
#define ADC_PRSCL 0x06 // clk/64

#include <avr/io.h>
#include <util/delay.h>
#include <avr/interrupt.h>
#include <avr/wdt.h>
#include <avr/eeprom.h>
#include <avr/pgmspace.h>
#include <avr/sleep.h>

//### Globals start ###
uint8_t eepos = 0;
uint8_t mode_idx = 0;
PROGMEM const uint8_t modes[] = MODES;
const uint8_t mode_cnt = sizeof(modes);
uint8_t mypwm = 100; // Output level
uint8_t smode = 1; // Special mode boolean
uint8_t spress = 0; // Short press boolean
//### Globals end ###

void store_mode_idx(uint8_t lvl) { // central method for writing (with wear leveling)
  uint8_t oldpos = eepos;
  eepos=(eepos+1) & 63; // wear leveling, use next cell
  // Write the current mode
  EEARL=eepos; EEDR=lvl; EECR=32+4; EECR=32+4+2; // WRITE  32:write only (no erase)  4:enable  2:go
  while(EECR & 2); // wait for completion
  // Erase the last mode
  EEARL=oldpos; EECR=16+4; EECR=16+4+2; // ERASE  16:erase only (no write)  4:enable  2:go
}

inline void get_mode() { // Get mode and store with short press indicator
  uint8_t eep;
  for(eepos=0; eepos<64; eepos++) {
    eep = eeprom_read_byte((const uint8_t *)(uint16_t)eepos);
    if (eep != 0xff) {
      mode_idx = eep; // Mode position in modes array
      break;
    }
  }
  if (mode_idx & 0x10) { // Indicates we did a short press last time
    mode_idx &= 0x0f; // Remove short press indicator
    mode_idx++; // Go to the next mode
    spress = 1; // Short press boolean
  }
  if (mode_idx >= mode_cnt) {
      mode_idx = 0; // Wrap around
  }
  mypwm = pgm_read_byte(&modes[mode_idx]); // Get mode identifier/output level
  if (spress || (mypwm != MORSE_CODE)) {
    store_mode_idx(mode_idx | 0x10); // Store mode with short press indicator
  }
}

inline void WDT_on() {
  // Setup watchdog timer to only interrupt, not reset
  cli(); // Disable interrupts
  wdt_reset(); // Reset the WDT
  WDTCR |= (1<<WDCE) | (1<<WDE); // Start timed sequence
  WDTCR = (1<<WDTIE) | (1<<WDP2) | (1<<WDP1); // Enable interrupt every second
  sei(); // Enable interrupts
}

inline void WDT_off() {
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
      TCCR0A = PWM_FAST; // fast-PWM
    } else {
      TCCR0A = PWM_PHASE; // phase corrected PWM
    }
  }
  PWM_LVL = pwm_lvl;
}

static inline void mode_strobe(void) {
  uint8_t i;
  while(1){
    set_output(0, 1);
    for(i = 0; i < STROBE_GROUP; i++){ // strobe group
      set_output(STROBE_ON_OUT, 0);
      _delay_ms(STROBE_ON);
      set_output(STROBE_OFF_OUT, 0);
      _delay_ms(STROBE_OFF);
    }
    set_output(STROBE_PAUSE_OUT, 1);
    _delay_ms(STROBE_PAUSE); // pause between groups
  }
}

static inline void mode_beacon(void) {
  while(1){
    set_output(BEACON_ON_OUT, 1);
    _delay_ms(BEACON_ON);
    set_output(BEACON_OFF_OUT, 1);
    _delay_ms(BEACON_OFF);
  }
}

void morse_blink(uint8_t dot, uint8_t pcs, uint8_t lvl) { // Morse code
  uint8_t i;
  for (i = 0; i < pcs; i++) {
    set_output(lvl, 1);
    if (dot) {
      _delay_ms(SOS_DOT);
    } else {
      _delay_ms(3 * SOS_DOT);
    }
    set_output(0, 1);
    _delay_ms(SOS_DOT);
  }
}

static inline void mode_sos(void) {
  uint8_t i;
  while(1){
    for (i = 0; i < 3; i++) { // sos group
      if (i == 0 || i == 2) {
        morse_blink(1, 3, SOS_OUT);
      } else {
        morse_blink(0, 3, SOS_OUT);
      }
      _delay_ms(2*SOS_DOT); // pause between chars
    }
    _delay_ms(SOS_PAUSE); // pause between groups
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
    if (!spress && mypwm == MORSE_CODE) { // Unlock Morse code
      store_mode_idx(++mode_idx);
    } else if (MODE_MEMORY || (mypwm == MORSE_CODE)) { // Save Mode
      store_mode_idx(mode_idx);
    } else { // No mode memory
      store_mode_idx(0);
    }
  }
  if (mypwm == 255 && ticks == MODE_FULL_TIMEOUT) { // MODE_FULL timeout
    mypwm = MODE_FULL_LOW;
    set_output(mypwm, 1);
  }
  if (BATT_MON && ticks == 255) {
    ticks = 255 - BATT_TIMEOUT; // Battery monitoring interval
    if (!lowbatt_mode) {
      if (low_voltage(ADC_LOW)) {
        lowbatt_mode = 1;
        if (!smode) {
          if (mypwm > ADC_LOW_OUT) {
            mypwm = ADC_LOW_OUT; // Lower output if not in special mode
          }
          morse_blink(1, 5, mypwm); // Flash a few times
          set_output(mypwm, 1);
        }
      }
    } else {
      if (low_voltage(ADC_CRIT)) {
        WDT_off(); // Disable WDT so it doesn't wake us up
        ADCSRA &= ~(1<<7); // ADC off
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
  if (mypwm == MODE_STROBE) {
    mode_strobe();
  } else if (mypwm == MODE_BEACON) {
    mode_beacon();
  } else if (mypwm == MODE_SOS){
    mode_sos();
  } else { // All normal modes
    smode = 0; // Special mode boolean
    set_output(mypwm, 1);
  }
  while(1) {
    sleep_mode(); // Enter sleep mode
  }
  return 0;
}

