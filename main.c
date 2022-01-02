/*
 * NewFirmware.c
 *
 * Created: 12/25/2021 10:30:43 AM
 * Author : naoki
 */ 

#include <avr/io.h>

#include "instruments.h"
#include "ports.h"

// Instruments
rim_shot_t g_rim_shot;

void InitializeInstruments() {
  g_rim_shot.status = 0;
}

void SetUpTimer() {
  // 16-bit timer, 1/64 prescale
  TCCR0 |= _BV(CS02) | _BV(CS01) | _BV(CS00);
}

void SetUpIo() {
  // PORTA -- TBD
  DDRA = 0;
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

void TriggerRimShort(int8_t velocity) {
  PORT_TRIG_RIM_SHOT |= _BV(BIT_TRIG_RIM_SHOT);
  PORT_LED_RIM_SHOT |= _BV(BIT_LED_RIM_SHOT);
  g_rim_shot.status = 5;
}

void CheckInstruments() {
  if (g_rim_shot.status) {
    PORT_TRIG_RIM_SHOT &= ~_BV(BIT_TRIG_RIM_SHOT);
    if (--g_rim_shot.status == 0) {
      PORT_LED_RIM_SHOT &= ~_BV(BIT_LED_RIM_SHOT);
    }
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
      CheckInstruments();
      uint8_t current_switches = PORT_SWITCHES;
      CheckSwitches(prev_switches, current_switches);
      prev_switches = current_switches;
    }
    prev_timer_value = current_timer_value;
  }
}

