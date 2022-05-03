/*
 * ports.h
 */
#ifndef PORTS_HPP_
#define PORTS_HPP_

/*
 * I/O Ports:
 *  PA0 --- TEST
 *  PA1 out HH latch
 *  PA2 out HH D2
 *  PA3 out HH D3
 *  PA4 out HH D4
 *  PA5 out HH D5
 *  PA6 out HH D6
 *  PA7 out HH D7
 */
#define PORT_HI_HAT_PCM_VALUE PORTA
#define BIT_HI_HAT_PCM_LATCH PA1

/*
 *  PB0 out DIN Start
 *  PB1 out DIN Clock
 *  PB2 out LED DM
 *  PB3 out LED Open Hi-Hat
 *  PB4 out PWM SD Velocity  OC0
 *  PB5 out PWM SD Tune      OC1A
 *  PB6 out PWM BD Tune      OC1B
 *  PB7 out PWM BD Velocity  OC2/OC1C
 */
#define PORT_DIN_START PORTB
#define BIT_DIN_START PB0
#define PORT_DIN_CLOCK PORTB
#define BIT_DIN_CLOCK PB1
#define PORT_LED_DIN_MUTE PORTB
#define BIT_LED_DIN_MUTE PB2
#define PORT_LED_OPEN_HI_HAT PORTB
#define BIT_LED_OPEN_HI_HAT PB3

#define REGISTER_TUNE_BASS_DRUM OCR1BL
#define REGISTER_TUNE_SNARE_DRUM OCR1AL
#define REGISTER_VELOCITY_BASS_DRUM OCR1CL
#define REGISTER_VELOCITY_SNARE_DRUM OCR0

/*
 *  PC0 in  SW shift
 *  PC1 in  SW Open Hi-Hat
 *  PC2 in  SW Closed Hi-Hat
 *  PC3 in  SW Hand Clap
 *  PC4 in  SW Rim Shot
 *  PC5 in  SW Snare Drum
 *  PC6 in  SW Bass Drum
 *  PC7 in  SW DIN Mute
 */
#define PORT_SWITCHES PINC
#define PORT_SW_SHIFT PORT_SWITCHES
#define BIT_SW_SHIFT PINC0
#define PORT_SW_OPEN_HI_HAT PORT_SWITCHES
#define BIT_SW_OPEN_HI_HAT PINC1
#define PORT_SW_CLOSED_HI_HAT PORT_SWITCHES
#define BIT_SW_CLOSED_HI_HAT PINC2
#define PORT_SW_HAND_CLAP PORT_SWITCHES
#define BIT_SW_HAND_CLAP PINC3
#define PORT_SW_RIM_SHOT PORT_SWITCHES
#define BIT_SW_RIM_SHOT PINC4
#define PORT_SW_SNARE_DRUM PORT_SWITCHES
#define BIT_SW_SNARE_DRUM PINC5
#define PORT_SW_BASS_DRUM PORT_SWITCHES
#define BIT_SW_BASS_DRUM PINC6
#define PORT_SW_DIN_MUTE PORT_SWITCHES
#define BIT_SW_DIN_MUTE PINC7

/*
 *  PD0 out Trigger Rim Shot
 *  PD1 out Trigger Hand Clap
 *  PD2 in  MIDI IN
 *  PD3 out Trigger Hi-Hat
 *  PD4 out Trigger Snare Drum
 *  PD5 out Hi-Hat Select
 *  PD6 out Trigger Bass Drum
 *  PD7 --- TEST
 */

#define PORT_TRIG_RIM_SHOT PORTD
#define BIT_TRIG_RIM_SHOT PD0
#define PORT_TRIG_HAND_CLAP PORTD
#define BIT_TRIG_HAND_CLAP PD1

#define PORT_MIDI_IN PIND
#define BIT_MIDI_IN PIND2

#define PORT_TRIG_HI_HAT PORTD
#define BIT_TRIG_HI_HAT PD3
#define PORT_TRIG_SNARE_DRUM PORTD
#define BIT_TRIG_SNARE_DRUM PD4
#define PORT_SELECT_HI_HAT PORTD
#define BIT_SELECT_HI_HAT PD5
#define PORT_TRIG_BASS_DRUM PORTD
#define BIT_TRIG_BASS_DRUM PD6

/*
 *  PE0 out LED Rim Shot
 *  PE1 out LED Closed Hi-Hat
 *  PE2 out LED Hand Clap
 *  PE3 out PWM Hi-Hat Velocity     OC3A
 *  PE4 out PWM Rim Short Velocity  OC3B
 *  PE5 out PWM Hand Clap Velocity  OC3C
 *  PE6 out LED Snare Drum
 *  PE7 out LED Bass Drum
 */

#define PORT_LED_RIM_SHOT PORTE
#define BIT_LED_RIM_SHOT PE0
#define PORT_LED_CLOSED_HI_HAT PORTE
#define BIT_LED_CLOSED_HI_HAT PE1
#define PORT_LED_HAND_CLAP PORTE
#define BIT_LED_HAND_CLAP PE2
#define PORT_LED_SNARE_DRUM PORTE
#define BIT_LED_SNARE_DRUM PE6
#define PORT_LED_BASS_DRUM PORTE
#define BIT_LED_BASS_DRUM PE7

#define REGISTER_VELOCITY_HI_HAT OCR3AL
#define REGISTER_VELOCITY_RIM_SHOT OCR3BL
#define REGISTER_VELOCITY_HAND_CLAP OCR3CL

/*
 *  PF0 in  AD Hi-Hat Tune      ADC0
 *  PF1 in  AD Snare Drum Tune  ADC1
 *  PF2 in  AD Bass Drum Tune   ADC2
 *  PF3 --- TEST
 *  PF4 TCK
 *  PF5 TMS
 *  PF6 TDO
 *  PF7 TDI
 */
static constexpr uint8_t ADMUX_HI_HAT = 0;              // ADC0
static constexpr uint8_t ADMUX_SNARE_DRUM = _BV(MUX0);  // ADC1
static constexpr uint8_t ADMUX_BASS_DRUM = _BV(MUX1);   // ADC2
static constexpr uint8_t ADMUX_NUM_CHANNELS = 3;

/*
 *  PG0 --- TEST
 *  PG1 --- TEST
 *  PG2 --- TEST
 *  PG3 out Noise
 *  PG4 --- TEST
 */
#define PORT_NOISE PORTG
#define BIT_NOISE PG3

#endif /* PORTS_HPP_ */