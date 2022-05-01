/*
 * din_sync.h
 */

#include "ports.hpp"
#include "utils.hpp"

#ifndef DIN_SYNC_HPP_
#define DIN_SYNC_HPP_

class DinSync {
 private:
  uint8_t clock_countdown_ = 0;

 public:
  inline void Start() { ClearBit(PORT_DIN_START, BIT_DIN_START); }

  inline void Stop() { SetBit(PORT_DIN_START, BIT_DIN_START); }

  inline void Clock() {
    ClearBit(PORT_DIN_CLOCK, BIT_DIN_CLOCK);
    clock_countdown_ = 8;
  }

  inline void Update() {
    if (clock_countdown_ && --clock_countdown_ == 0) {
      SetBit(PORT_DIN_CLOCK, BIT_DIN_CLOCK);
    }
  }
};

#endif /* DIN_SYNC_HPP_ */