
/*
 * eeprom_write.S
 */ 
 
 #define __SFR_OFFSET 0
 #include <avr/io.h>

.global eeprom_write_async
eeprom_write_async:
    ; Disable interrupts
    cli
    ; Set up the address (r18:r17) to address register
    out     EEARH, r18
    out     EEARL, r17
    ; Write data (r16) to Data register
    out     EEDR, r16
    ; Write logical one to EEMWE
    sbi     EECR, EEMWE
    ; Start eeprom write by setting EEWE
    sbi     0x1C, EEWE
    ; Enable interrupts
    sei
    ret