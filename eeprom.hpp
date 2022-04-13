/*
 * eeprom.h
 */

#ifndef EEPROM_HPP_
#define EEPROM_HPP_

#include <avr/eeprom.h>

static constexpr uint8_t* E_MAGIC = reinterpret_cast<uint8_t*>(0);
static constexpr uint8_t* E_MIDI_CH = reinterpret_cast<uint8_t*>(1);
static constexpr uint16_t* E_TEMPO = reinterpret_cast<uint16_t*>(2);
static constexpr uint8_t* E_PATTERN1 = reinterpret_cast<uint8_t*>(4);

inline void EEPROM_WRITE(uint8_t* address, uint8_t data) {
  /* Wait for completion of previous write */
  while (EECR & (1 << EEWE))
    ;
  /* Set up address and data registers */
  EEAR = reinterpret_cast<unsigned>(address);
  EEDR = data;
  // Prohibit interrupts since the EEMWE bit would be timed out in 4 clocks
  cli();
  /* Write logical one to EEMWE */
  EECR |= (1 << EEMWE);
  /* Start eeprom write by setting EEWE */
  EECR |= (1 << EEWE);
  sei();
}

inline void eeprom_write_async(uint8_t* address, uint8_t data) {
  /* Set up address and data registers */
  EEAR = reinterpret_cast<unsigned>(address);
  EEDR = data;
  // Prohibit interrupts since the EEMWE bit would be timed out in 4 clocks
  cli();
  /* Write logical one to EEMWE */
  EECR |= (1 << EEMWE);
  /* Start eeprom write by setting EEWE */
  EECR |= (1 << EEWE);
  sei();
}

#endif /* EEPROM_HPP_ */