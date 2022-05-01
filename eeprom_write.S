
/*
 * eeprom_write.S
 */ 
 
 #define __SFR_OFFSET 0
 #include <avr/io.h>

.global eeprom_write_async
eeprom_write_async:
    ; Disable interrupts
    cli
    ; Set up the address (r24:r25) to address register
    out     EEARH, r25
    out     EEARL, r24
    ; Write data (r22) to Data register
    out     EEDR, r22
    ; Write logical one to EEMWE
    sbi     EECR, EEMWE
    ; Start eeprom write by setting EEWE
    sbi     EECR, EEWE
    ; Enable interrupts
    sei
    ret