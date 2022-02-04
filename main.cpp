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

// Instruments /////////////////////////////////////
bass_drum_t g_bass_drum;
snare_drum_t g_snare_drum;
rim_shot_t g_rim_shot;
hand_clap_t g_hand_clap;
hi_hat_t g_hi_hat;

// Utilities ///////////////////////////////////////

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
 * Sets Timer2 management configuration for a hi-hat tune value.
 * The range of tune value is 1024 / 16 = 64.
 */
inline static void SetHiHatTune(uint8_t tune) {
  g_hi_hat.tcnt2_on_overflow = tune + 168;
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

// Initializers //////////////////////////////////////////

void InitializeInstruments() {
  // bass drum
  g_bass_drum.status = 0;

  // snare drum
  g_snare_drum.status = 0;

  // rim shot
  g_rim_shot.status = 0;
  
  // hand clap
  g_hand_clap.status = 0;
  
  // hi-hat
  g_hi_hat.status = 0;
  g_hi_hat.pcm_address = 0xffff;  // PCM runs when address < limit
  g_hi_hat.pcm_address_limit = 0;
  g_hi_hat.pcm_phase = 0;
  g_hi_hat.pcm_update_ready = 0;
  SetHiHatTune(128);  // middle
}

/**
 * Incremented for each Timer0 cycle to divide its timing signals.
 * The Timer0 rotation happens with:
 *   clock=16e6 / timer_top=256 / prescale=8 = 7812.5 Hz
 * Then the divider bits give following frequencies (LSB -> MSB)
 * bits  freq (Hz) interval
 *  0  : 3906.3      256 us
 *  1  : 1953.1      512 us
 *  2  :  976.3     1.02 ms
 *  3  :  488.3     2.05 ms
 *  4  :  244.1     4.10 ms
 *  5  :  122.1     8.19 ms
 *  6  :   61.0     16.4 ms
 *  7  :   30.5     32.8 ms
 *  8  :   15.3     65.5 ms
 *  9  :    7.6      131 ms
 * 10  :    3.8      262 ms
 * 11  :    1.9      524 ms
 * 12  :    0.95    1.05 s
 * 13  :    0.48    2.10 s
 * 14  :    0.23    4.19 s
 * 15  :    0.12    8.39 s
 */
uint16_t g_divider;

void SetUpTimer() {
  /*
   * Timer0
   */
  // 8-bit timer, 1/8 prescale = 128 micro second interval (CS01)
  // Fast non-inverting PWM (WGM01 +_ WGM00 + COM01)
  TCCR0 = _BV(WGM01) | _BV(WGM00) | _BV(COM01) | _BV(CS01);
  OCR0 = 127;
  g_divider = 0;
  REGISTER_VELOCITY_SNARE_DRUM = 255;
  
  /*
   * Timer1
   */
  // 1/8 prescale (CS11)
  // 8-bit Fast non-inverting PWM A, B, & C (WGM12 + WGM10 + COM1A1 + COM1B1 + COM1C1)
  TCCR1A = _BV(WGM10) | _BV(COM1A1) | _BV(COM1B1) | _BV(COM1C1);
  TCCR1B =  _BV(WGM12) | _BV(CS11);
  // clear high values of output compare registers
  OCR1AH = 0;
  OCR1BH = 0;
  OCR1CH = 0;
  // put middle values to tune registers
  REGISTER_TUNE_SNARE_DRUM = 64;
  REGISTER_TUNE_BASS_DRUM = 128;
  // put maximum value to the velocity register
  REGISTER_VELOCITY_BASS_DRUM = 255;
  
  /*
   * Timer2
   */
  // Timer2 is stopped in the initial state. See StartPcmClock() and StopPcmClock()
  TCCR2 = 0;
  
  /*
   * Timer3
   */
  // 1/8 prescale (CS31)
  // 8-bit Fast non-inverting PWM A, B, & C (WGM32 + WGM30 + COM3A1 + COM3B1 + CO3MC1)
  TCCR3A = _BV(WGM30) | _BV(COM3A1) | _BV(COM3B1) | _BV(COM3C1);
  TCCR3B =  _BV(WGM32) | _BV(CS31);
  // clear high values of output compare registers
  OCR3AH = 0;
  OCR3BH = 0;
  OCR3CH = 0;
  // put maximum value to the velocity registers
  REGISTER_VELOCITY_HI_HAT = 255;
  REGISTER_VELOCITY_RIM_SHOT = 255;
  REGISTER_VELOCITY_HAND_CLAP = 64;
  
  /*
   * Timer interrupts
   */
  // Timer2 overflow interrupt enabled (TOIE2)
  TIMSK = _BV(TOIE2);
}

inline void StartPcmClock() {
  // normal mode (WGM21, WGM20 = 00), 1/8 prescale (CS21)
  TCCR2 = _BV(CS21);
}

inline void StopPcmClock() {
  TCCR2 = 0;
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
  
  // Set the Timer2 counter the start value
  TCNT2 = g_hi_hat.tcnt2_on_overflow;
}

void SetUpIo() {
  // PORTA -- Hi-Hat PCM data and latch
  DDRA = 0xff;
  PORTA = 0;
 
  // PORT B and E
  DDRB = 0xff;
  DDRE = 0xff;
  PORTB = 0;
  
  // PORT C -- all switches
  DDRC = 0;
  PORTC = 0xff;  // pull up
  
  // PORT D
  DDRD = ~_BV(BIT_MIDI_IN); // only MIDI-in is input
  PORTD = _BV(BIT_MIDI_IN);
  
  // PORT F -- All input (ADC)
  DDRF = 0;
  PORTF = 0;
  
  // PORT G -- Noise and test pins
  DDRG = 0xff;
  PORTG = 0;
}

// Instrument control functions /////////////////////////////////////////////////

uint8_t g_adc_current_channel = 0;
bool g_adc_ready_to_read = false;

void SetUpAdc() {
  g_adc_current_channel = ADMUX_HI_HAT;
  /*
   * ADC Multiplexer Selection Register
   */
  // voltage reference: AREF (REFS = 00)
  // ADC Left Adjust Result: off (ADLAR = 0)
  // Channel is set f| or hi-hat (ADC0)
  ADMUX = g_adc_current_channel;

  /*
   * ADC Control and Status Register A
   */
  ADCSRA = _BV(ADEN)  // ADC ENable
    // | _BV(ADSC)    // ADC Start Conversion
    // | _BV(ADATE)   // ADC Auto Trigger Enable
    // | _BV(ADIF)    // ADC Interrupt Flag
    // | _BV(ADIE)    // ADC Interrupt Enable
    | _BV(ADPS2)      // ADC Prescaler Select Bits. '111' is 128 that gives 125 kHz for 16MHz clock
    | _BV(ADPS1)
    | _BV(ADPS0);

    g_adc_ready_to_read = false;
}

void SetUp() {
  InitializeInstruments();
  SetUpTimer();
  SetUpIo();
  SetUpAdc();
  sei();
}

static constexpr uint16_t TRIGGER_SHUTDOWN_AT = (255 - 16);  // 2.048 ms

void TriggerBassDrum(int8_t velocity) {
  SetBit(PORT_TRIG_BASS_DRUM, BIT_TRIG_BASS_DRUM);
  SetBit(PORT_LED_BASS_DRUM, BIT_LED_BASS_DRUM);
  g_bass_drum.status = 255;
}

void TriggerSnareDrum(int8_t velocity) {
  SetBit(PORT_TRIG_SNARE_DRUM, BIT_TRIG_SNARE_DRUM);
  SetBit(PORT_LED_SNARE_DRUM, BIT_LED_SNARE_DRUM);
  g_snare_drum.status = 255;
}

void TriggerRimShot(int8_t velocity) {
  SetBit(PORT_TRIG_RIM_SHOT, BIT_TRIG_RIM_SHOT);
  SetBit(PORT_LED_RIM_SHOT, BIT_LED_RIM_SHOT);
  g_rim_shot.status = 255;
}

void TriggerHandClap(int8_t velocity) {
  SetBit(PORT_TRIG_HAND_CLAP, BIT_TRIG_HAND_CLAP);
  SetBit(PORT_LED_HAND_CLAP, BIT_LED_HAND_CLAP);
  g_hand_clap.status = 255;
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

template <void (*TriggerFunc)(int8_t)>
void CheckSwitch(uint8_t prev_switches, uint8_t new_switches, uint8_t switch_bit,
                 int8_t velocity) {
  if ((prev_switches & _BV(switch_bit)) && !(new_switches & _BV(switch_bit))) {
    TriggerFunc(velocity);
  }
}

void CheckSwitches(uint8_t prev_switches, uint8_t new_switches) {
  if ((prev_switches ^ new_switches) == 0) {
    return;
  }
  CheckSwitch<TriggerBassDrum>(prev_switches, new_switches, BIT_SW_BASS_DRUM, 127);
  CheckSwitch<TriggerSnareDrum>(prev_switches, new_switches, BIT_SW_SNARE_DRUM, 127);
  CheckSwitch<TriggerRimShot>(prev_switches, new_switches, BIT_SW_RIM_SHOT, 127);
  CheckSwitch<TriggerHandClap>(prev_switches, new_switches, BIT_SW_HAND_CLAP, 127);
  CheckSwitch<TriggerOpenHiHat>(prev_switches, new_switches, BIT_SW_OPEN_HI_HAT, 127);
  CheckSwitch<TriggerClosedHiHat>(prev_switches, new_switches, BIT_SW_CLOSED_HI_HAT, 127);
}

template<typename InstrumentT>
inline void CheckInstrument(InstrumentT* instrument,
                            volatile uint8_t& trig_port, const uint8_t trig_bit,
                            volatile uint8_t& led_port, const uint8_t led_bit) {
  if (instrument->status) {
    if (--instrument->status == TRIGGER_SHUTDOWN_AT) {
      ClearBit(trig_port, trig_bit);
    } else if (instrument->status == 0) {
      ClearBit(led_port, led_bit);
    }
  } 
}

void CheckInstruments() {
  CheckInstrument(&g_bass_drum, PORT_TRIG_BASS_DRUM, BIT_TRIG_BASS_DRUM,
                  PORT_LED_BASS_DRUM, BIT_LED_BASS_DRUM);

  CheckInstrument(&g_snare_drum, PORT_TRIG_SNARE_DRUM, BIT_TRIG_SNARE_DRUM,
                  PORT_LED_SNARE_DRUM, BIT_LED_SNARE_DRUM);

  CheckInstrument(&g_rim_shot, PORT_TRIG_RIM_SHOT, BIT_TRIG_RIM_SHOT,
                  PORT_LED_RIM_SHOT, BIT_LED_RIM_SHOT);

  CheckInstrument(&g_hand_clap, PORT_TRIG_HAND_CLAP, BIT_TRIG_HAND_CLAP,
                  PORT_LED_HAND_CLAP, BIT_LED_HAND_CLAP);

  if (g_hi_hat.status) {
    if (--g_hi_hat.status == TRIGGER_SHUTDOWN_AT) {
      ClearBit(PORT_TRIG_HI_HAT, BIT_TRIG_HI_HAT);
      } else if (g_hi_hat.status == 0) {
      ClearBit(PORT_LED_CLOSED_HI_HAT, BIT_LED_CLOSED_HI_HAT);
      ClearBit(PORT_LED_OPEN_HI_HAT, BIT_LED_OPEN_HI_HAT);
    }
  }
}

void HandleAdc() {
  if (!g_adc_ready_to_read) {
    // set the next ADC channel
    ADMUX &= 0xF0;
    ADMUX |= g_adc_current_channel;
    // start ADC
    ADCSRA |= _BV(ADSC);
    g_adc_ready_to_read = true;  // the value would be read in the next cycle
  } else if ((ADCSRA & _BV(ADSC)) == 0) {
    switch (g_adc_current_channel) {
     case ADMUX_HI_HAT:
      SetHiHatTune(ADC >> 4);
      break;
     case ADMUX_SNARE_DRUM:
      REGISTER_TUNE_SNARE_DRUM = (ADC >> 4) + 38;  // 38 to 102, CV range about 0.7V to 2V
      break;
     case ADMUX_BASS_DRUM:
      REGISTER_TUNE_BASS_DRUM = (ADC >> 2); // CV rail to rail, range 0V to 5V
      break;
    }
    g_adc_ready_to_read = false;
    g_adc_current_channel = (g_adc_current_channel + 1) % ADMUX_NUM_CHANNELS;
  }
}

int main(void) {
  SetUp();
  /* Replace with your application code */
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

        if ((g_divider & 0x7f) == 0) { // every 128 cycles = 16ms
          HandleAdc();
        }
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
    noise_clock = current_timer_value;
    
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