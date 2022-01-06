/*
 * NewFirmware.c
 *
 * Created: 12/25/2021 10:30:43 AM
 * Author : naoki
 */ 

#include <avr/io.h>

#include "instruments.h"
#include "ports.h"
#include "hi_hat_wav.h"

#define SET_BIT(port, bit) (port) |= _BV(bit)
#define CLEAR_BIT(port, bit) (port) &= ~_BV(bit)

// Instruments
rim_shot_t g_rim_shot;
hi_hat_t g_hi_hat;

void InitializeInstruments() {
  // rim shot
  g_rim_shot.status = 0;
  
  // hi-hat
  g_hi_hat.status = 0;
  g_hi_hat.pcm_address = 0xffff;  // PCM runs when address < limit
  g_hi_hat.pcm_address_limit = 0;
  g_hi_hat.pcm_value = 0;
}

/**
 * Incremented for each Timer0 cycle to divide its timing signals.
 * The Timer0 rotation happens with:
 *   clock=16e6 / timer_top=256 / prescale=8 = 7812.5 Hz
 * Then the divider bits give following frequencies (LSB -> MSB)
 *  0 : 3906.3
 *  1 : 1953.1
 *  2 :  976.3
 *  3 :  488.3
 *  4 :  244.1
 *  5 :  122.1
 *  6 :   61.0
 *  7 :   30.5
 *  8 :   15.3
 *  9 :    7.6
 * 10 :    3.8
 * 11 :    1.9
 * 12 :    0.95
 * 13 :    0.48
 * 14 :    0.23
 * 15 :    0.12
 */
uint16_t g_divider;

void SetUpTimer() {
  // 8-bit timer, 1/8 prescale = 128 micro second interval
  // Fast non-inverting PWM (WGM01 +_ WGM00 + COM01 + COM00)
  TCCR0 |= _BV(WGM01) | _BV(WGM00) | _BV(COM01) | _BV(COM00) | _BV(CS01);
  OCR0 = 127;
  g_divider = 0;
}

void SetUpIo() {
  // PORTA -- TBD
  DDRA = 0xff;
  PORTA = 0;
 
  // PORT B and E
  DDRB = 0xff;
  DDRE = 0xff;
  PORTB = 0;
  // Set velocities to maximum temporarily
  PORTE = _BV(BIT_PWM_VELOCITY_RIM_SHOT) | _BV(BIT_PWM_VELOCITY_HI_HAT);
  
  // PORT C -- all switches
  DDRC = 0;
  PORTC = 0xff;  // pull up
  
  // PORT D
  DDRD = ~_BV(BIT_MIDI_IN); // only MIDI-in is input
  PORTD = _BV(BIT_MIDI_IN);
  
  // PORT F -- TBD
  DDRF = 0;
  PORTF = 0;
  
  // PORT G -- TBD
  DDRG = 0;
  PORTG = 0;
}

void SetUp() {
  InitializeInstruments();
  SetUpTimer();
  SetUpIo();
}

#define TRIGGER_SHUTDOWN_AT (255 - 16)  // 2.048 ms

void TriggerRimShort(int8_t velocity) {
  SET_BIT(PORT_TRIG_RIM_SHOT, BIT_TRIG_RIM_SHOT);
  SET_BIT(PORT_LED_RIM_SHOT, BIT_LED_RIM_SHOT);
  g_rim_shot.status = 255;
}

void TriggerOpenHiHat(int8_t velocity) {
  SET_BIT(PORT_TRIG_HI_HAT, BIT_TRIG_HI_HAT);
  SET_BIT(PORT_LED_OPEN_HI_HAT, BIT_LED_OPEN_HI_HAT);
  CLEAR_BIT(PORT_LED_CLOSED_HI_HAT, BIT_LED_CLOSED_HI_HAT);
  CLEAR_BIT(PORT_SELECT_HI_HAT, BIT_SELECT_HI_HAT);
  g_hi_hat.status = 255;
}

void TriggerClosedHiHat(int8_t velocity) {
  SET_BIT(PORT_TRIG_HI_HAT, BIT_TRIG_HI_HAT);
  SET_BIT(PORT_LED_CLOSED_HI_HAT, BIT_LED_CLOSED_HI_HAT);
  CLEAR_BIT(PORT_LED_OPEN_HI_HAT, BIT_LED_OPEN_HI_HAT);
  SET_BIT(PORT_SELECT_HI_HAT, BIT_SELECT_HI_HAT);
  g_hi_hat.status = 255;
}

/**
 * Places PCM value for hi-hat. The value is reflected to the output
 * port immediately but it is not propagated to the external DAC until
 * The latch bit is turned on.
 * The two LSBs of the value must be zeros. The function does not check
 * it for execution speed, so the caller must take care of clearing the
 * two LSBs.
 */
inline void PlaceHiHatPcmValue(uint8_t value) {
  PORT_HI_HAT_PCM_VALUE = value;
}

/**
 * Turns on the "PCM latch" bit to propagate the PCM value to the
 * external ADC where the clock port is connected to the latch bit.
 * The function turns on the latch bit but does not turn off by itself.
 * Another function DoneCommitHiHatPcmValue() must be called explicitly
 * afterwards.
 */
inline void CommitHiHatPcmValue() {
  PORT_HI_HAT_PCM_VALUE |= _BV(BIT_HI_HAT_PCM_LATCH);
}

/**
 * Turns off the "PCM latch" bit. When CommitHiHatPcmValue() is called,
 * this function must be called but no earlier than 3.5 microseconds.
 */
inline void DoneCommitHiHatPcmValue() {
  PORT_HI_HAT_PCM_VALUE &= ~_BV(BIT_HI_HAT_PCM_LATCH);
}

void CheckInstruments() {
  if (g_rim_shot.status) {
    if (--g_rim_shot.status == TRIGGER_SHUTDOWN_AT) {
      PORT_TRIG_RIM_SHOT &= ~_BV(BIT_TRIG_RIM_SHOT);
    } else if (g_rim_shot.status == 0) {
      PORT_LED_RIM_SHOT &= ~_BV(BIT_LED_RIM_SHOT);
    }
  }
  
  if (g_hi_hat.status) {
    if (--g_hi_hat.status == TRIGGER_SHUTDOWN_AT) {
      PORT_TRIG_HI_HAT &= ~_BV(BIT_TRIG_HI_HAT);
    } else if (g_hi_hat.status == 0) {      
      PORT_LED_CLOSED_HI_HAT &= ~_BV(BIT_LED_CLOSED_HI_HAT);
      PORT_LED_OPEN_HI_HAT &= ~_BV(BIT_LED_OPEN_HI_HAT);
    }
  }

  // test hi-hat PCM
  /*
  if ((g_cycle_count & 0x1) == 1) {
    DoneCommitHiHatPcmValue();
    PlaceHiHatPcmValue(g_hi_hat.pcm_value++);
  } else {
    CommitHiHatPcmValue();    
  }
  */
}

void CheckSwitches(uint8_t prev_switches, uint8_t new_switches) {
  if ((prev_switches ^ new_switches) == 0) {
    return;
  }
  if (IS_RIM_SHOT_ON(new_switches)) {
    TriggerRimShort(127);
  }
}

int main(void) {
  SetUp();
  /* Replace with your application code */
  // uint32_t count = 0;
  volatile uint8_t prev_timer_value = 0;
  volatile uint8_t prev_switches = PORT_SWITCHES;
  while (1) {
    uint8_t current_timer_value = TCNT0;
    if (current_timer_value < prev_timer_value) {
      ++g_divider;
      CheckInstruments();
      uint8_t current_switches = PORT_SWITCHES;
      if ((g_divider & 0x3f) == 0) { // every 64 cycles = 8ms
        CheckSwitches(prev_switches, current_switches);
        prev_switches = current_switches;
      }
      if ((g_divider & 0xFFF) == 0) { // every 0.5 seconds
        PORT_LED_DIN_MUTE ^= _BV(BIT_LED_DIN_MUTE);
      }
    }
    prev_timer_value = current_timer_value;
  }
}

