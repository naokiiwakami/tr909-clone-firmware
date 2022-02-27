/*
 * eeprom.h
 *
 * Created: 2/27/2022 9:32:57 PM
 *  Author: naoki
 */

#ifndef EEPROM_H_
#define EEPROM_H_

#include <avr/eeprom.h>

static constexpr uint8_t* E_MAGIC = reinterpret_cast<uint8_t*>(0);
static constexpr uint8_t* E_MIDI_CH = reinterpret_cast<uint8_t*>(1);

#endif /* EEPROM_H_ */