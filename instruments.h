/*
 * instruments.h
 *
 * Created: 1/1/2022 1:11:57 PM
 *  Author: naoki
 */ 


#ifndef INSTRUMENTS_H_
#define INSTRUMENTS_H_

typedef struct bass_drum {
  uint8_t status;
}
bass_drum_t;

typedef struct rim_shot {
  uint8_t status;
}
rim_shot_t;

typedef struct hi_hat {
  uint8_t status;
  uint16_t pcm_address;
  uint16_t pcm_address_limit;  // PCM stops at this address
  uint8_t pcm_phase : 1;  // 0 : update, 1: latch
  uint8_t pcm_update_ready : 1;
}
hi_hat_t;

#endif /* INSTRUMENTS_H_ */