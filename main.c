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

// Instruments
rim_shot_t g_rim_shot;
hi_hat_t g_hi_hat;

void InitializeInstruments() {
  g_rim_shot.status = 0;
  g_hi_hat.pcm_value = 0;
}

uint8_t g_cycle_count;
void SetUpTimer() {
  // 8-bit timer, 1/64 prescale = 1.024 ms interval
  TCCR0 |= _BV(CS02);
  g_cycle_count = 0;
}

void SetUpIo() {
  // PORTA -- TBD
  DDRA = 0xff;
  PORTA = 0;
 
  // PORT B and E
  DDRB = 0xff;
  DDRE = 0xff;
  PORTB = 0;
  PORTE = _BV(BIT_PWM_VELOCITY_RIM_SHOT);
  
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

#define TRIGGER_LED_TURNED_ON_CYCLES 50
void TriggerRimShort(int8_t velocity) {
  PORT_TRIG_RIM_SHOT |= _BV(BIT_TRIG_RIM_SHOT);
  PORT_LED_RIM_SHOT |= _BV(BIT_LED_RIM_SHOT);
  g_rim_shot.status = TRIGGER_LED_TURNED_ON_CYCLES;
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
    if (g_rim_shot.status != TRIGGER_LED_TURNED_ON_CYCLES) {  // turn off after 2ms
      PORT_TRIG_RIM_SHOT &= ~_BV(BIT_TRIG_RIM_SHOT);
    }      
    if (--g_rim_shot.status == 0) {
      PORT_LED_RIM_SHOT &= ~_BV(BIT_LED_RIM_SHOT);
    }
  }
  
  // test hi-hat PCM
  if ((g_cycle_count & 0x1) == 1) {
    DoneCommitHiHatPcmValue();
    PlaceHiHatPcmValue(g_hi_hat.pcm_value++);
  } else {
    CommitHiHatPcmValue();    
  }
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
      ++g_cycle_count;
      CheckInstruments();
      uint8_t current_switches = PORT_SWITCHES;
      if ((g_cycle_count & 0xf) == 0) { // every 16 cycles
        CheckSwitches(prev_switches, current_switches);
        prev_switches = current_switches;
      }
    }
    prev_timer_value = current_timer_value;
  }
}

