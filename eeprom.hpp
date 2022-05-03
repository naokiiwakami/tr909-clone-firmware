/*
 * eeprom.h
 */
#ifndef EEPROM_HPP_
#define EEPROM_HPP_

#include <avr/eeprom.h>

static constexpr uint8_t* E_MAGIC = reinterpret_cast<uint8_t*>(0);
static constexpr uint8_t* E_MIDI_CH = reinterpret_cast<uint8_t*>(1);
static constexpr uint16_t E_TEMPO = 2;
static constexpr uint8_t* E_PATTERN_ID = reinterpret_cast<uint8_t*>(4);
static constexpr uint16_t E_PATTERN = 6;

extern "C" {
void eeprom_write_async(uint16_t address, uint8_t data);
}

#endif /* EEPROM_HPP_ */