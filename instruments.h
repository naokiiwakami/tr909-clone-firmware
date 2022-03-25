/*
 * instruments.h
 *
 * Created: 1/1/2022 1:11:57 PM
 *  Author: naoki
 */

#ifndef INSTRUMENTS_H_
#define INSTRUMENTS_H_

/**
 * Enum used for identifying the drum type.
 */
enum class Drum : uint8_t {
  kBassDrum,
  kSnareDrum,
  kRimShot,
  kHandClap,
  kClosedHiHat,
  kOpenHiHat,
  kOutOfRange,
};

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

#endif /* INSTRUMENTS_H_ */