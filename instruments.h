/*
 * instruments.h
 *
 * Created: 1/1/2022 1:11:57 PM
 *  Author: naoki
 */ 


#ifndef INSTRUMENTS_H_
#define INSTRUMENTS_H_

typedef struct rim_shot {
  uint8_t status;
}
rim_shot_t;

typedef struct hi_hat {
  uint8_t status;
  uint16_t pcm_address;
  uint16_t pcm_address_limit;  // PCM stops at this address
  uint8_t pcm_value;  // TODO: Remove this. Only for testing.
}
hi_hat_t;

#endif /* INSTRUMENTS_H_ */