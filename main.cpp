/*
 * tr909_clone_firmware.cpp
 *
 * Created: 1/7/2022 8:00:11 PM
 * Author : naoki
 */ 

#include <avr/interrupt.h>
#include <avr/io.h>

#include "instruments.h"
#include "ports.h"
#include "hi_hat_wav.h"

inline static void SetBit(volatile uint8_t& port, const uint8_t bit) {
  port |= _BV(bit);
}

inline static void ClearBit(volatile uint8_t& port, const uint8_t bit) {
  port &= ~_BV(bit);
}

inline static void ToggleBit(volatile uint8_t& port, const uint8_t bit) {
  port ^= _BV(bit);
}

/**
 * Places PCM value for hi-hat. The value is reflected to the output
 * port immediately but it is not propagated to the external DAC until
 * The latch bit is turned on.
 * The two LSBs of the value must be zeros. The function does not check
 * it for execution speed, so the caller must take care of clearing the
 * two LSBs.
 */
inline static void PlaceHiHatPcmValue(uint8_t value) {
  PORT_HI_HAT_PCM_VALUE = value;
}

/**
 * Turns on the "PCM latch" bit to propagate the PCM value to the
 * external ADC where the clock port is connected to the latch bit.
 * The function turns on the latch bit but does not turn off by itself.
 * Another function DoneCommitHiHatPcmValue() must be called explicitly
 * afterwards.
 */
inline static void LatchHiHatPcmValue() {
  PORT_HI_HAT_PCM_VALUE |= _BV(BIT_HI_HAT_PCM_LATCH);
}

/**
 * Turns off the "PCM latch" bit. When CommitHiHatPcmValue() is called,
 * this function must be called but no earlier than 3.5 microseconds.
 */
inline void UnlatchHiHatPcmValue() {
  PORT_HI_HAT_PCM_VALUE &= ~_BV(BIT_HI_HAT_PCM_LATCH);
}

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
  g_hi_hat.pcm_phase = 0;
  g_hi_hat.pcm_update_ready = 0;
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
  /*
   * Timer0
   */
  // 8-bit timer, 1/8 prescale = 128 micro second interval (CS01)
  // Fast non-inverting PWM (WGM01 +_ WGM00 + COM01 + COM00)
  TCCR0 = _BV(WGM01) | _BV(WGM00) | _BV(COM01) | _BV(COM00) | _BV(CS01);
  OCR0 = 127;
  g_divider = 0;
  
  /*
   * Timer2
   */
  // Timer2 is stopped in the initial state. See StartPcmClock() and StopPcmClock()
  TCCR2 = 0;
  
  /*
   * Timer interrupts
   */
  // Timer2 overflow interrupt enabled (TOIE2)
  TIMSK = _BV(TOIE2);
}

inline void StartPcmClock() {
  // normal mode (WGM21, WGM20 = 00), no prescale (CS20)
  TCCR2 |= _BV(CS20);
}

inline void StopPcmClock() {
  TCCR2 &= ~_BV(CS20);
}

ISR(TIMER2_OVF_vect) {
  if (g_hi_hat.pcm_phase == 0) {
    // turn off the PCM latch bit and notify the main program to update the PCM value
    // TODO: Clearing the bit may not be necessary; Try removing this line.
    // The latch bit is actually the bit1 of the PCM PORT and setting the port
    // value from the wave table always clears the bit.
    UnlatchHiHatPcmValue();
    g_hi_hat.pcm_update_ready = 1;
  } else {
    // turn on the PCM latch bit
   LatchHiHatPcmValue();
  }
  // switch the PCM phase
  g_hi_hat.pcm_phase ^= 1;
}

void SetUpIo() {
  // PORTA -- Hi-Hat PCM data and latch
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
  
  // PORT G -- Noise and test pins
  DDRG = 0xff;
  PORTG = 0;
}

void SetUp() {
  InitializeInstruments();
  SetUpTimer();
  SetUpIo();
  sei();
}

static constexpr uint16_t TRIGGER_SHUTDOWN_AT = (255 - 16);  // 2.048 ms

// Triggering Rim Shot
void TriggerRimShot(int8_t velocity) {
  SetBit(PORT_TRIG_RIM_SHOT, BIT_TRIG_RIM_SHOT);
  SetBit(PORT_LED_RIM_SHOT, BIT_LED_RIM_SHOT);
  g_rim_shot.status = 255;
}

// Triggering Hi-Hats
template <void (*OpenHiHatLedFunc)(volatile uint8_t&, const uint8_t),
          void (*ClosedHiHatLedFunc)(volatile uint8_t&, const uint8_t),
          void (*HiHatSelectFunc)(volatile uint8_t&, const uint8_t),
          uint16_t pcm_start, uint16_t pcm_end>
void TriggerHiHat(int8_t velocity) {
  SetBit(PORT_TRIG_HI_HAT, BIT_TRIG_HI_HAT);
  OpenHiHatLedFunc(PORT_LED_OPEN_HI_HAT, BIT_LED_OPEN_HI_HAT);
  ClosedHiHatLedFunc(PORT_LED_CLOSED_HI_HAT, BIT_LED_CLOSED_HI_HAT);
  HiHatSelectFunc(PORT_SELECT_HI_HAT, BIT_SELECT_HI_HAT);
  g_hi_hat.status = 255;
  g_hi_hat.pcm_address = pcm_start;
  g_hi_hat.pcm_address_limit = pcm_end;
  g_hi_hat.pcm_phase = 0;
  g_hi_hat.pcm_update_ready = 0;
  StartPcmClock();
}

inline void TriggerOpenHiHat(int8_t velocity) {
  TriggerHiHat<SetBit, ClearBit, SetBit, 0, ADDRESS_CLOSED_HI_HAT_START>(velocity);
}

inline void TriggerClosedHiHat(int8_t velocity) {
  TriggerHiHat<ClearBit, SetBit, ClearBit, ADDRESS_CLOSED_HI_HAT_START, ADDRESS_END>(velocity);
}

void CheckInstruments() {
  if (g_rim_shot.status) {
    if (--g_rim_shot.status == TRIGGER_SHUTDOWN_AT) {
      ClearBit(PORT_TRIG_RIM_SHOT, BIT_TRIG_RIM_SHOT);
    } else if (g_rim_shot.status == 0) {
      ClearBit(PORT_LED_RIM_SHOT, BIT_LED_RIM_SHOT);
    }
  }
  
  if (g_hi_hat.status) {
    if (--g_hi_hat.status == TRIGGER_SHUTDOWN_AT) {
      ClearBit(PORT_TRIG_HI_HAT, BIT_TRIG_HI_HAT);
    } else if (g_hi_hat.status == 0) {      
      ClearBit(PORT_LED_CLOSED_HI_HAT, BIT_LED_CLOSED_HI_HAT);
      ClearBit(PORT_LED_OPEN_HI_HAT, BIT_LED_OPEN_HI_HAT);
    }
  }
}

inline bool IsSwitchOn(uint8_t switch_bit) {
  return (PORT_SWITCHES & _BV(switch_bit)) == 0;
}

template <void (*TriggerFunc)(int8_t)>
void CheckSwitch(uint8_t switch_bit, int8_t velocity) {
  if (IsSwitchOn(switch_bit)) {
    TriggerFunc(velocity);
  }
}

void CheckSwitches(uint8_t prev_switches, uint8_t new_switches) {
  if ((prev_switches ^ new_switches) == 0) {
    return;
  }
  CheckSwitch<TriggerRimShot>(BIT_SW_RIM_SHOT, 127);
  CheckSwitch<TriggerOpenHiHat>(BIT_SW_OPEN_HI_HAT, 127);
  CheckSwitch<TriggerClosedHiHat>(BIT_SW_CLOSED_HI_HAT, 127);
}

int main(void) {
  SetUp();
  /* Replace with your application code */
  // uint32_t count = 0;
  volatile uint8_t prev_timer_value = 0;
  volatile uint8_t prev_switches = PORT_SWITCHES;
  volatile uint8_t noise_clock = 0;
  volatile uint32_t noise_register = ~0;
  while (1) {
    // Check the master app clock
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
        ToggleBit(PORT_LED_DIN_MUTE, BIT_LED_DIN_MUTE);
      }
    }
    prev_timer_value = current_timer_value;

    /*
     * Noise generator
     *
     * Noise generator is updated every 8 Timer0 increments - about 250 kHz
     */
    current_timer_value >>= 3;
    if (current_timer_value != noise_clock) {
      uint8_t temp = (noise_register >> 12) & 1;
      temp ^= (noise_register >> 30) & 1;
      if (temp) {
        SetBit(PORT_NOISE, BIT_NOISE);
      } else {
        ClearBit(PORT_NOISE, BIT_NOISE);
      }
      noise_register <<= 1;
      noise_register += temp;
    }
    noise_clock = current_timer_value & 0x1f;
    
    // Check the hi-hat PCM status
    if (g_hi_hat.pcm_update_ready) {
      if (g_hi_hat.pcm_address < g_hi_hat.pcm_address_limit) {
        PlaceHiHatPcmValue(pgm_read_byte(&hi_hat_wav[g_hi_hat.pcm_address]));
        ++g_hi_hat.pcm_address;
      } else {
        StopPcmClock();
      }        
      g_hi_hat.pcm_update_ready = 0;
    }
  }
}