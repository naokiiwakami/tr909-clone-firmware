/*
 * eeprom.h
 */

#ifndef EEPROM_HPP_
#define EEPROM_HPP_

#include <avr/eeprom.h>

static constexpr uint8_t* E_MAGIC = reinterpret_cast<uint8_t*>(0);
static constexpr uint8_t* E_MIDI_CH = reinterpret_cast<uint8_t*>(1);

#endif /* EEPROM_HPP_ */