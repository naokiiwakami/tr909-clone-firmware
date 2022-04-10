/*
 * utils.hpp
 */ 


#ifndef UTILS_HPP_
#define UTILS_HPP_

inline static void SetBit(volatile uint8_t& port, const uint8_t bit) { port |= _BV(bit); }

inline static void ClearBit(volatile uint8_t& port, const uint8_t bit) { port &= ~_BV(bit); }

inline static void ToggleBit(volatile uint8_t& port, const uint8_t bit) { port ^= _BV(bit); }

inline static void SetBit(volatile uint8_t& port, const uint8_t bit, bool onOrOff) {
  if (onOrOff) {
    SetBit(port, bit);
    } else {
    ClearBit(port, bit);
  }
}

#endif /* UTILS_HPP_ */