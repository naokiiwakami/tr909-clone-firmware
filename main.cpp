/*
 * tr909_clone_firmware.cpp
 */

#include <avr/interrupt.h>
#include <avr/io.h>

#include "adc.hpp"
#include "eeprom.hpp"
#include "hi_hat_wav.hpp"
#include "instruments.hpp"
#include "midi.hpp"
#include "ports.hpp"
#include "sequencer.hpp"
#include "system.hpp"
#include "utils.hpp"

// Instruments /////////////////////////////////////
bass_drum_t g_bass_drum;
snare_drum_t g_snare_drum;
rim_shot_t g_rim_shot;
hand_clap_t g_hand_clap;
hi_hat_t g_hi_hat;

// Initializers //////////////////////////////////////////

void InitializeEEPROM() {
  if (eeprom_read_byte(E_MAGIC) != 0xa2) {
    eeprom_write_byte(E_MIDI_CH, 0);
    eeprom_write_byte(E_MAGIC, 0xa2);
    eeprom_write_word(reinterpret_cast<uint16_t*>(E_TEMPO), 150);
  }
}

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
 *
 *       divider
 * bits     mask  freq (Hz) interval
 * ==================================
 *  1  :     0x1  3906.3      256 us
 *  2  :     0x3  1953.1      512 us
 *  3  :     0x7   976.3     1.02 ms
 *  4  :     0xf   488.3     2.05 ms
 *  5  :    0x1f   244.1     4.10 ms
 *  6  :    0x3f   122.1     8.19 ms
 *  7  :    0x7f    61.0     16.4 ms
 *  8  :    0xff    30.5     32.8 ms
 *  9  :   0x1ff    15.3     65.5 ms
 * 10  :   0x3ff     7.6      131 ms
 * 11  :   0x7ff     3.8      262 ms
 * 12  :   0xfff     1.9      524 ms
 * 13  :  0x1fff     0.95    1.05 s
 * 14  :  0x3fff     0.48    2.10 s
 * 15  :  0x7fff     0.23    4.19 s
 * 16  :  0xffff     0.12    8.39 s
 */

void SetUpTimer() {
  /*
   * Timer0
   */
  // 8-bit timer, 1/8 prescale = 128 micro second interval (CS01)
  // Fast non-inverting PWM (WGM01 +_ WGM00 + COM01)
  TCCR0 = _BV(WGM01) | _BV(WGM00) | _BV(COM01) | _BV(CS01);
  OCR0 = 127;
  REGISTER_VELOCITY_SNARE_DRUM = 255;

  /*
   * Timer1
   */
  // 1/8 prescale (CS11)
  // 8-bit Fast non-inverting PWM A, B, & C (WGM12 + WGM10 + COM1A1 + COM1B1 + COM1C1)
  TCCR1A = _BV(WGM10) | _BV(COM1A1) | _BV(COM1B1) | _BV(COM1C1);
  TCCR1B = _BV(WGM12) | _BV(CS11);
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
  // Timer2 is stopped in the initial state. See StartPcmClock() and StopPcmClock() in
  // instruments.hpp
  TCCR2 = 0;

  /*
   * Timer3
   */
  // 1/8 prescale (CS31)
  // 8-bit Fast non-inverting PWM A, B, & C (WGM32 + WGM30 + COM3A1 + COM3B1 + CO3MC1)
  TCCR3A = _BV(WGM30) | _BV(COM3A1) | _BV(COM3B1) | _BV(COM3C1);
  TCCR3B = _BV(WGM32) | _BV(CS31);
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
  PORTB = _BV(BIT_DIN_START) | _BV(BIT_DIN_CLOCK);

  // PORT C -- all switches
  DDRC = 0;
  PORTC = 0xff;  // pull up

  // PORT D
  DDRD = ~_BV(BIT_MIDI_IN);  // only MIDI-in is input
  PORTD = _BV(BIT_MIDI_IN);  // pull up

  // PORT F -- All input (ADC)
  DDRF = 0;
  PORTF = 0;

  // PORT G -- Noise and test pins
  DDRG = 0xff;
  PORTG = 0;
}

Adc g_adc{};

MidiReceiver g_midi_receiver{};

/**
 * Turn on/off LEDs as reflections of bits in the value.
 */
inline void MapToLed(uint8_t value) {
  SetBit(PORT_LED_DIN_MUTE, BIT_LED_DIN_MUTE, (value & 0x40) != 0);
  SetBit(PORT_LED_BASS_DRUM, BIT_LED_BASS_DRUM, (value & 0x20) != 0);
  SetBit(PORT_LED_SNARE_DRUM, BIT_LED_SNARE_DRUM, (value & 0x10) != 0);
  SetBit(PORT_LED_RIM_SHOT, BIT_LED_RIM_SHOT, (value & 0x8) != 0);
  SetBit(PORT_LED_HAND_CLAP, BIT_LED_HAND_CLAP, (value & 0x4) != 0);
  SetBit(PORT_LED_CLOSED_HI_HAT, BIT_LED_CLOSED_HI_HAT, (value & 0x2) != 0);
  SetBit(PORT_LED_OPEN_HI_HAT, BIT_LED_OPEN_HI_HAT, (value & 0x1) != 0);
}

bool CheckSwitchesMidi(uint8_t prev_switches, uint8_t new_switches, uint8_t* midi_indicator) {
  if ((prev_switches & _BV(BIT_SW_SNARE_DRUM)) && !(new_switches & _BV(BIT_SW_SNARE_DRUM))) {
    if (*midi_indicator < 16) {
      ++*midi_indicator;
    }
  }
  if ((prev_switches & _BV(BIT_SW_RIM_SHOT)) && !(new_switches & _BV(BIT_SW_RIM_SHOT))) {
    if (*midi_indicator > 1) {
      --*midi_indicator;
    }
  }
  if ((prev_switches & _BV(BIT_SW_DIN_MUTE)) && !(new_switches & _BV(BIT_SW_DIN_MUTE))) {
    return true;
  }
  return false;
}

/**
 * A subroutine to be used for changing the MIDI channel.
 *
 * The mode starts with the current MIDI channel. The channel number is indicated
 * as a binary number using the LEDs. Pressing the "snare drum" button increments the channel,
 * while the "rim shot" button decrements it. Pressing the "DIN mute" button exits the mode.
 * The MIDI channel is stored into the eeprom before leaving.
 */
void SetUpMidi() {
  uint8_t midi_indicator = g_midi_receiver.GetChannel() + 0x1;
  volatile uint8_t prev_timer_value = 0;
  volatile uint16_t divider = 0;
  uint8_t led_value = 0;
  uint8_t prev_switches = PORT_SWITCHES;
  uint8_t countdown = 0;
  while (true) {
    uint8_t current_timer_value = TCNT0;
    if (current_timer_value < prev_timer_value) {
      ++divider;
      if ((divider & 0xff) == 0) {  // every 256 cycles = 32ms
        if (countdown == 0) {
          uint8_t current_switches = PORT_SWITCHES;
          if (CheckSwitchesMidi(prev_switches, current_switches, &midi_indicator)) {
            MapToLed(midi_indicator);
            uint8_t midi_channel = midi_indicator - 1;
            g_midi_receiver.SetChannel(midi_channel);
            eeprom_write_byte(E_MIDI_CH, midi_channel);
            countdown = 32;
          } else {
            MapToLed(led_value);
            if (led_value == 0) {
              led_value = midi_indicator;
            } else {
              led_value = 0;
            }
          }
          prev_switches = current_switches;
        } else if (--countdown == 0) {
          break;
        }
      }
    }
    prev_timer_value = current_timer_value;
  }
  MapToLed(0);
}

void StartupSequence() {
  volatile uint8_t prev_timer_value = 0;
  uint8_t blink_left = 11;
  uint8_t midi_indicator = g_midi_receiver.GetChannel() + 0x1;
  uint8_t led_value = 0x80;
  uint16_t divider = 0;
  while (blink_left > 0) {
    uint8_t current_timer_value = TCNT0;
    if (current_timer_value < prev_timer_value) {
      ++divider;
      if (blink_left == 11) {
        if ((divider & 0x7f) == 0) {  // every 128 cycles = 16ms
          MapToLed(led_value);
          led_value >>= 1;
          if (led_value == 0) {
            led_value = midi_indicator;
            --blink_left;
          }
        }
      } else {
        if ((divider & 0x7ff) == 0) {
          MapToLed(led_value);
          if (led_value >= 0x40) {
            led_value = midi_indicator;
          } else {
            led_value = 0x40 + midi_indicator;
          }
          --blink_left;
        }
      }
    }
    prev_timer_value = current_timer_value;
  }
  MapToLed(0);
}

void SetUp() {
  InitializeEEPROM();
  InitializeInstruments();
  SetUpTimer();
  SetUpIo();
  g_adc.Initialize();
  g_midi_receiver.Initialize();
  g_sequencer.Initialize();

  sei();
}

// Instrument control functions /////////////////////////////////////////////////

static constexpr uint16_t TRIGGER_SHUTDOWN_AT = (255 - 16);  // 2.048 ms
static constexpr uint8_t kTapHistory = 2;

Sequencer g_sequencer{};

inline void Tap() { g_sequencer.Tap(); }

inline void SequencerStandByRecording() { g_sequencer.StandByRecording(); }

inline void ToggleSequencer() { g_sequencer.Toggle(); }

inline void OnShift() {
  if (g_sequencer.GetState() & (Sequencer::kStandByRecording | Sequencer::kRecording)) {
    g_sequencer.HardStop();
    ClearBit(PORT_LED_DIN_MUTE, BIT_LED_DIN_MUTE);
  }
}

template <void (*Func)()>
void CheckSwitch(uint8_t prev_switches, uint8_t new_switches, uint8_t switch_bit) {
  if ((prev_switches & _BV(switch_bit)) && !(new_switches & _BV(switch_bit))) {
    Func();
  }
}

template <Drum drum>
void CheckDrumSwitch(uint8_t prev_switches, uint8_t new_switches, uint8_t switch_bit) {
  if ((prev_switches & _BV(switch_bit)) && !(new_switches & _BV(switch_bit))) {
    g_sequencer.Trigger<drum>(127);
  }
}

void CheckSwitches(uint8_t prev_switches, uint8_t new_switches) {
  if ((prev_switches ^ new_switches) == 0) {
    return;
  }
  if ((new_switches & _BV(BIT_SW_SHIFT)) == 0) {
    CheckSwitch<OnShift>(prev_switches, new_switches, BIT_SW_SHIFT);
    CheckSwitch<Tap>(prev_switches, new_switches, BIT_SW_BASS_DRUM);
    CheckSwitch<SequencerStandByRecording>(prev_switches, new_switches, BIT_SW_DIN_MUTE);
    return;
  }
  if (g_sequencer.GetTapCount() > 0) {
    g_sequencer.ResetTapCount();
    g_sequencer.StartWritingTempo();
  }
  CheckDrumSwitch<Drum::kBassDrum>(prev_switches, new_switches, BIT_SW_BASS_DRUM);
  CheckDrumSwitch<Drum::kSnareDrum>(prev_switches, new_switches, BIT_SW_SNARE_DRUM);
  CheckDrumSwitch<Drum::kRimShot>(prev_switches, new_switches, BIT_SW_RIM_SHOT);
  CheckDrumSwitch<Drum::kHandClap>(prev_switches, new_switches, BIT_SW_HAND_CLAP);
  CheckDrumSwitch<Drum::kOpenHiHat>(prev_switches, new_switches, BIT_SW_OPEN_HI_HAT);
  CheckDrumSwitch<Drum::kClosedHiHat>(prev_switches, new_switches, BIT_SW_CLOSED_HI_HAT);
  CheckSwitch<ToggleSequencer>(prev_switches, new_switches, BIT_SW_DIN_MUTE);
}

template <typename InstrumentT>
inline void CheckInstrument(InstrumentT* instrument, volatile uint8_t& trig_port,
                            const uint8_t trig_bit, volatile uint8_t& led_port,
                            const uint8_t led_bit) {
  if (instrument->status) {
    if (--instrument->status == TRIGGER_SHUTDOWN_AT) {
      ClearBit(trig_port, trig_bit);
    } else if (instrument->status == 0) {
      ClearBit(led_port, led_bit);
    }
  }
}

void CheckInstruments() {
  CheckInstrument(&g_bass_drum, PORT_TRIG_BASS_DRUM, BIT_TRIG_BASS_DRUM, PORT_LED_BASS_DRUM,
                  BIT_LED_BASS_DRUM);

  CheckInstrument(&g_snare_drum, PORT_TRIG_SNARE_DRUM, BIT_TRIG_SNARE_DRUM, PORT_LED_SNARE_DRUM,
                  BIT_LED_SNARE_DRUM);

  CheckInstrument(&g_rim_shot, PORT_TRIG_RIM_SHOT, BIT_TRIG_RIM_SHOT, PORT_LED_RIM_SHOT,
                  BIT_LED_RIM_SHOT);

  CheckInstrument(&g_hand_clap, PORT_TRIG_HAND_CLAP, BIT_TRIG_HAND_CLAP, PORT_LED_HAND_CLAP,
                  BIT_LED_HAND_CLAP);

  if (g_hi_hat.status) {
    if (--g_hi_hat.status == TRIGGER_SHUTDOWN_AT) {
      ClearBit(PORT_TRIG_HI_HAT, BIT_TRIG_HI_HAT);
    } else if (g_hi_hat.status == 0) {
      ClearBit(PORT_LED_CLOSED_HI_HAT, BIT_LED_CLOSED_HI_HAT);
      ClearBit(PORT_LED_OPEN_HI_HAT, BIT_LED_OPEN_HI_HAT);
    }
  }
}

// The main program /////////////////////////////////////////

int main(void) {
  SetUp();

  if ((PORT_SW_BASS_DRUM & _BV(BIT_SW_BASS_DRUM)) == 0) {
    SetUpMidi();
  } else {
    StartupSequence();
  }

  uint8_t prev_timer_value = 0;
  uint8_t prev_switches = PORT_SWITCHES;
  uint8_t noise_clock = 0;
  uint32_t noise_register = ~0;
  uint16_t divider = 0;
  while (1) {
    // Check MIDI input
    while (UCSR1A & _BV(RXC1)) {
      g_midi_receiver.ParseMidiInput(UDR1);
    }

    // Check the master app clock
    uint8_t current_timer_value = TCNT0;
    if (current_timer_value < prev_timer_value) {
      ++divider;
      CheckInstruments();
      g_sequencer.IncrementClock();
      if ((divider & 0x3f) == 0) {  // every 64 cycles = 8ms
        uint8_t current_switches = PORT_SWITCHES;
        CheckSwitches(prev_switches, current_switches);
        prev_switches = current_switches;
        g_sequencer.Poll();

        if ((divider & 0x7f) == 0) {  // every 128 cycles = 16ms
          g_adc.Update();
        }
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