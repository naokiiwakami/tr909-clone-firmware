/*
 * instruments.h
 */
#ifndef INSTRUMENTS_H_
#define INSTRUMENTS_H_

#include "hi_hat_wav.hpp"
#include "ports.hpp"
#include "system.hpp"
#include "utils.hpp"

/**
 * Enum used for identifying the drum type.
 */
enum class Drum : uint8_t {
  kOpenHiHat,
  kClosedHiHat,
  kHandClap,
  kRimShot,
  kSnareDrum,
  kBassDrum,
  kOutOfRange,
};

static constexpr int kNumDrums = static_cast<int>(Drum::kOutOfRange);

typedef struct bass_drum {
  uint8_t status;
} bass_drum_t;

typedef struct snare_drum {
  uint8_t status;
} snare_drum_t;

typedef struct rim_shot {
  uint8_t status;
} rim_shot_t;

typedef struct hand_clap {
  uint8_t status;
} hand_clap_t;

typedef struct hi_hat {
  uint8_t status;
  uint16_t pcm_address;
  uint16_t pcm_address_limit;  // PCM stops at this address
  uint8_t pcm_phase : 1;       // 0 : update, 1: latch
  uint8_t pcm_update_ready : 1;

  /*
   * Timer2 management for hi-hat tone control
   */
  // TCNT2 value to set by the Timer2 overflow interrupt handler.
  // Starting with 0 gives the slowest PCM clock, which gets faster as the value increases.
  uint8_t tcnt2_on_overflow;
} hi_hat_t;

extern bass_drum_t g_bass_drum;
extern snare_drum_t g_snare_drum;
extern rim_shot_t g_rim_shot;
extern hand_clap_t g_hand_clap;
extern hi_hat_t g_hi_hat;

/**
 * Sets Timer2 management configuration for a hi-hat tune value.
 * The range of tune value is 1024 / 16 = 64.
 */
inline static void SetHiHatTune(uint8_t tune) { g_hi_hat.tcnt2_on_overflow = tune + 168; }

/**
 * Places PCM value for hi-hat. The value is reflected to the output
 * port immediately but it is not propagated to the external DAC until
 * The latch bit is turned on.
 * The two LSBs of the value must be zeros. The function does not check
 * it for execution speed, so the caller must take care of clearing the
 * two LSBs.
 */
inline static void PlaceHiHatPcmValue(uint8_t value) { PORT_HI_HAT_PCM_VALUE = value; }

/**
 * Turns on the "PCM latch" bit to propagate the PCM value to the
 * external ADC where the clock port is connected to the latch bit.
 * The function turns on the latch bit but does not turn off by itself.
 * Another function DoneCommitHiHatPcmValue() must be called explicitly
 * afterwards.
 */
inline static void LatchHiHatPcmValue() { PORT_HI_HAT_PCM_VALUE |= _BV(BIT_HI_HAT_PCM_LATCH); }

/**
 * Turns off the "PCM latch" bit. When CommitHiHatPcmValue() is called,
 * this function must be called but no earlier than 3.5 microseconds.
 */
inline void UnlatchHiHatPcmValue() { PORT_HI_HAT_PCM_VALUE &= ~_BV(BIT_HI_HAT_PCM_LATCH); }

inline void StartPcmClock() {
  // normal mode (WGM21, WGM20 = 00), 1/8 prescale (CS21)
  TCCR2 = _BV(CS21);
}

inline void StopPcmClock() { TCCR2 = 0; }

void HitBassDrum(int8_t velocity) {
  REGISTER_VELOCITY_BASS_DRUM = velocity << 1;
  SetBit(PORT_TRIG_BASS_DRUM, BIT_TRIG_BASS_DRUM);
  if (FlashOnHitEnabled()) {
    SetBit(PORT_LED_BASS_DRUM, BIT_LED_BASS_DRUM);
  }
  g_bass_drum.status = 255;
}

void HitSnareDrum(int8_t velocity) {
  REGISTER_VELOCITY_SNARE_DRUM = velocity << 1;
  SetBit(PORT_TRIG_SNARE_DRUM, BIT_TRIG_SNARE_DRUM);
  if (FlashOnHitEnabled()) {
    SetBit(PORT_LED_SNARE_DRUM, BIT_LED_SNARE_DRUM);
  }
  g_snare_drum.status = 255;
}

void HitRimShot(int8_t velocity) {
  REGISTER_VELOCITY_RIM_SHOT = velocity << 1;
  SetBit(PORT_TRIG_RIM_SHOT, BIT_TRIG_RIM_SHOT);
  if (FlashOnHitEnabled()) {
    SetBit(PORT_LED_RIM_SHOT, BIT_LED_RIM_SHOT);
  }
  g_rim_shot.status = 255;
}

void HitHandClap(int8_t velocity) {
  REGISTER_VELOCITY_HAND_CLAP = velocity;
  SetBit(PORT_TRIG_HAND_CLAP, BIT_TRIG_HAND_CLAP);
  if (FlashOnHitEnabled()) {
    SetBit(PORT_LED_HAND_CLAP, BIT_LED_HAND_CLAP);
  }
  g_hand_clap.status = 255;
}

// Triggering Hi-Hats
template <void (*OpenHiHatLedFunc)(volatile uint8_t&, const uint8_t),
          void (*ClosedHiHatLedFunc)(volatile uint8_t&, const uint8_t),
          void (*HiHatSelectFunc)(volatile uint8_t&, const uint8_t), uint16_t pcm_start,
          uint16_t pcm_end>
void HitHiHat(int8_t velocity) {
  REGISTER_VELOCITY_HI_HAT = velocity + 128;
  SetBit(PORT_TRIG_HI_HAT, BIT_TRIG_HI_HAT);
  if (FlashOnHitEnabled()) {
    OpenHiHatLedFunc(PORT_LED_OPEN_HI_HAT, BIT_LED_OPEN_HI_HAT);
    ClosedHiHatLedFunc(PORT_LED_CLOSED_HI_HAT, BIT_LED_CLOSED_HI_HAT);
  }
  HiHatSelectFunc(PORT_SELECT_HI_HAT, BIT_SELECT_HI_HAT);
  g_hi_hat.status = 255;
  g_hi_hat.pcm_address = pcm_start;
  g_hi_hat.pcm_address_limit = pcm_end;
  g_hi_hat.pcm_phase = 0;
  g_hi_hat.pcm_update_ready = 0;
  StartPcmClock();
}

inline void HitOpenHiHat(int8_t velocity) {
  HitHiHat<SetBit, ClearBit, SetBit, 0, ADDRESS_CLOSED_HI_HAT_START>(velocity);
}

inline void HitClosedHiHat(int8_t velocity) {
  HitHiHat<ClearBit, SetBit, ClearBit, ADDRESS_CLOSED_HI_HAT_START, ADDRESS_END>(velocity);
}

#endif /* INSTRUMENTS_H_ */