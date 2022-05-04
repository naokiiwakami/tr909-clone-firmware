/*
 * utils.hpp
 */
#ifndef UTILS_HPP_
#define UTILS_HPP_

#include "ports.hpp"

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

/**
 * Overflow aware subtraction method
 */
template <typename T>
inline static T Subtract(T x, T y) {
  return x >= y ? x - y : x + ~y + 1;
}

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

template <void (*Func)()>
void CheckSwitch(uint8_t prev_switches, uint8_t new_switches, uint8_t switch_bit) {
  if ((prev_switches & _BV(switch_bit)) && !(new_switches & _BV(switch_bit))) {
    Func();
  }
}

#endif /* UTILS_HPP_ */