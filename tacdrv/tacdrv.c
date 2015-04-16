//
// TacDrv 0.1
//
// License: Released to the Public Domain.
//
// This code is inspired by the work of others and if you
// think that it violates any copyright I will happily
// try to modify the code to make you happy.
//
// For use in 105d type drivers, no support for stars.
//

//###############################################################
//  Start user tweaking, don't comment out, just edit values
//###############################################################

// Mode output levels
#define MODE_MOON 10 // Output level (0-255)
#define MODE_LOW 20 // Output level (0-255)
#define MODE_MID 120 // Output level (0-255)
#define MODE_HIGH 200 // Output level (0-255)
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
#define STROBE_GROUP 100 // Strobe group cycle (on/off) counter (0-255)
#define STROBE_PAUSE 0 // Pause between strobe groups (ms)
#define STROBE_PAUSE_OUT 255 // Output level during strobe pause (0-255)
// Beacon settings
#define BEACON_ON 2000  // Beacon on time (ms)
#define BEACON_OFF 4000 // Beacon off time (ms)
#define BEACON_OUT 255 // Beacon output level (0-255)
// SOS settings
#define SOS_PAUSE 5000 // Pause between SOS groups (ms)
#define SOS_DOT 200 // Morse code dot duration (ms)
// Battery monitoring
#define VOLTAGE_MON 1 // Enable battery monitoring, 0 off and 1 on
#define ADC_LOW 130 // When do we start ramping
#define ADC_CRIT 120 // When do we shut the light off
#define ADC_LOW_OUT 20 // Output level in low battery mode (0-255)
// Various settings
#define MEMORY 0 // Mode memory, 0 off and 1 on
#define FAST_PWM_START 8 // Above what output level should we switch from phase correct to fast-PWM?
#define MODE_TIMEOUT 2 // Number of WTD ticks before mode is saved, each tick is 500ms

//################################
//  End user tweaking
//################################

#define F_CPU 4800000UL // CPU frequency

#include <avr/io.h>
#include <util/delay.h>
#include <avr/interrupt.h>
#include <avr/wdt.h>
#include <avr/eeprom.h>
#include <avr/pgmspace.h>
#include <avr/sleep.h>

#define PWM_PIN     PB1 // Default PWM pin
//#define VOLTAGE_PIN PB2
#define ADC_CHANNEL 0x01 // MUX 01 corresponds with PB2
#define ADC_DIDR 	ADC1D // Digital input disable bit corresponding with PB2
#define ADC_PRSCL   0x06 // clk/64
#define PWM_LVL OCR0B // OCR0B is the output compare register for PB1

// ### Globals start ###
uint8_t eepos = 0;
uint8_t eep[32];
uint8_t mode_idx = 0;
PROGMEM const uint8_t modes[]=MODES;
const uint8_t mode_cnt = sizeof(modes);
uint8_t mypwm=0; // Output level
uint8_t pmode=50; // Mode identifier
uint8_t smode=1; // Special mode boolean
// ### Globals end ###

void store_mode_idx(uint8_t lvl) {  // central method for writing (with wear leveling)
  uint8_t oldpos=eepos;
  eepos=(eepos+1)&31;  // wear leveling, use next cell
  // Write the current mode
  EEARL=eepos; EEDR=lvl; EECR=32+4; EECR=32+4+2; // WRITE  32:write only (no erase)  4:enable  2:go
  while(EECR & 2); // wait for completion
  // Erase the last mode
  EEARL=oldpos; EECR=16+4; EECR=16+4+2; // ERASE  16:erase only (no write)  4:enable  2:go
}

inline void read_mode_idx() {
  eeprom_read_block(&eep, 0, 32);
  while((eep[eepos] == 0xff) && (eepos < 32)) eepos++;
  if (eepos < 32) mode_idx = eep[eepos]; // &0x10; What the?
  else eepos=0;
}

inline void get_mode() {
  read_mode_idx(); // Read the last mode that was saved
  if (mode_idx&0x10) {
    // Indicates we did a short press last time, go to the next mode
    // Remove short press indicator first
    mode_idx &= 0x0f;
    mode_idx++;
  }
  if (mode_idx > (mode_cnt - 1)) {
    mode_idx = 0; // Wrap around
  }
  store_mode_idx(mode_idx|0x10); // Store mode with short press indicator
}

inline void WDT_on() {
  // Setup watchdog timer to only interrupt, not reset
  cli(); // Disable interrupts
  wdt_reset(); // Reset the WDT
  WDTCR |= (1<<WDCE) | (1<<WDE); // Start timed sequence
  WDTCR = (1<<WDTIE) | (1<<WDP2) | (1<<WDP0); // Enable interrupt every 500ms
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

inline void ADC_on() {
  ADMUX  = (1 << REFS0) | (1 << ADLAR) | ADC_CHANNEL; // 1.1v reference, left-adjust, ADC1/PB2
  DIDR0 |= (1 << ADC_DIDR); // disable digital input on ADC pin to reduce power consumption
  ADCSRA = (1 << ADEN ) | (1 << ADSC ) | ADC_PRSCL; // enable, start, prescale
}

inline void ADC_off() {
  ADCSRA &= ~(1<<7); // ADC off
}

void set_output(uint8_t pwm_lvl) {
  if (pwm_lvl > FAST_PWM_START) {
    TCCR0A = 0b00100011; // fast-PWM
  } else {
    TCCR0A = 0b00100001; // phase corrected PWM
  }
  PWM_LVL = pwm_lvl;
}

void mode_strobe(void) {
  uint8_t i;
  while(1){
    set_output(0);
    for(i = 0; i < STROBE_GROUP; ++i){ // strobe group
      PWM_LVL = 255;
      _delay_ms(STROBE_ON);
      PWM_LVL = 0;
      _delay_ms(STROBE_OFF);
    }
    set_output(STROBE_PAUSE_OUT);
    _delay_ms(STROBE_PAUSE); // pause between groups
  }
}

void mode_beacon(void) {
  while(1){
    set_output(BEACON_OUT);
    _delay_ms(BEACON_ON);
    set_output(0);
    _delay_ms(BEACON_OFF);
  }
}

void morse_blink(uint8_t blink) { // Morse code, 0 for short, 1 for long
  PWM_LVL = 255;
  if (!blink) {
    _delay_ms(SOS_DOT);
  } else {
    _delay_ms(3*SOS_DOT);
  }
  PWM_LVL = 0;
  _delay_ms(SOS_DOT);
}

void mode_sos(void) {
  set_output(0);
  uint8_t i;
  uint8_t j;

  while(1){
    j = 0;
    // sos group
    while(j < 3) {
      if (j == 0 || j == 2) {
        for(i = 0; i < 3; ++i){
          morse_blink(0);
        }
      } else {
        for(i = 0; i < 3; ++i){
          morse_blink(1);
        }
      }
      _delay_ms(2*SOS_DOT); // pause between chars
      j++;
    }
    _delay_ms(SOS_PAUSE); // pause between groups
  }
}

uint8_t low_voltage(uint8_t voltage_val) {
  static uint8_t lowbatt_cnt = 0;
  ADCSRA |= (1 << ADSC); // Start conversion
  while (ADCSRA & (1 << ADSC)); // Wait for completion
  if (ADCH < voltage_val) { // See if voltage is lower than what we were looking for
    if (++lowbatt_cnt > 30) { // See if it's been low for a while
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
  if (ticks == MODE_TIMEOUT) {
    if (!MEMORY) {
      store_mode_idx(0);
    } else {
      store_mode_idx(mode_idx);
    }
  }

  if (VOLTAGE_MON) {
    if (lowbatt_mode == 0) {
      if (low_voltage(ADC_LOW)) {
          lowbatt_mode = 1;
          if (!smode && mypwm > ADC_LOW_OUT) {
            set_output(ADC_LOW_OUT); // Lower output if not in special mode
          }
      }
    } else {
      if (low_voltage(ADC_CRIT)) {
        WDT_off(); // Disable WDT so it doesn't wake us up
        set_output(0); // Turn off the light
        set_sleep_mode(SLEEP_MODE_PWR_DOWN); // Power down as many components as possible
        sleep_mode();
      }
    }
  }
}

int main(void) {
  DDRB = (1 << PWM_PIN); // Set PWM pin to output
  if (VOLTAGE_MON) {
    ADC_on(); // Enable battery monitoring
  } else {
    ADC_off(); // Disable battery monitoring
  }
  ACSR   |=  (1<<7); // AC off
  TCCR0B = 0x01; // pre-scaler for timer (1 => 1, 2 => 8, 3 => 64...)
  // Enable sleep mode set to Idle that will be triggered by the sleep_mode() command.
  // Will allow us to go idle between WDT interrupts
  set_sleep_mode(SLEEP_MODE_IDLE);
  WDT_on(); // Start watchdogtimer
  get_mode(); // Get mode and store with short press indicator
  pmode=pgm_read_byte(&modes[mode_idx]); // Get mode identifier/output level

  if (pmode == MODE_STROBE) {
    mode_strobe();
  } else if (pmode == MODE_BEACON) {
    mode_beacon();
  } else if (pmode == MODE_SOS){
    mode_sos();
  } else { // All normal modes
    smode=0;
    mypwm=pmode;
    set_output(mypwm);
  }
  while(1) {
    sleep_mode();
  }
  return 0;
}
