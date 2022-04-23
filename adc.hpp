/*
 * adc.h
 *
 * ADC controller
 */

#include <avr/io.h>

#include "instruments.hpp"
#include "ports.hpp"

#ifndef ADC_HPP_
#define ADC_HPP_

class Adc {
 private:
  uint8_t current_adc_channel_ = 0;
  bool ready_to_read_ = false;

 public:
  Adc() {}

  void Initialize() {
    current_adc_channel_ = ADMUX_HI_HAT;
    /*
     * ADC Multiplexer Selection Register
     */
    // voltage reference: AREF (REFS = 00)
    // ADC Left Adjust Result: off (ADLAR = 0)
    // Channel is set f| or hi-hat (ADC0)
    ADMUX = current_adc_channel_;

    /*
     * ADC Control and Status Register A
     */
    // clang-format off
  ADCSRA = _BV(ADEN)  // ADC ENable
    // | _BV(ADSC)    // ADC Start Conversion
    // | _BV(ADATE)   // ADC Auto Trigger Enable
    // | _BV(ADIF)    // ADC Interrupt Flag
    // | _BV(ADIE)    // ADC Interrupt Enable
    | _BV(ADPS2)      // ADC Prescaler Select Bits. '111' is 128 that gives 125 kHz for 16MHz clock
    | _BV(ADPS1)
    | _BV(ADPS0);
    // clang-format on

    ready_to_read_ = false;
  }

  void Update() {
    if (!ready_to_read_) {
      // set the next ADC channel
      ADMUX &= 0xF0;
      ADMUX |= current_adc_channel_;
      // start ADC
      ADCSRA |= _BV(ADSC);
      ready_to_read_ = true;  // the value would be read in the next cycle
    } else if ((ADCSRA & _BV(ADSC)) == 0) {
      switch (current_adc_channel_) {
        case ADMUX_HI_HAT:
          SetHiHatTune(ADC >> 4);
          break;
        case ADMUX_SNARE_DRUM:
          REGISTER_TUNE_SNARE_DRUM = (ADC >> 4) + 38;  // 38 to 102, CV range about 0.7V to 2V
          break;
        case ADMUX_BASS_DRUM:
          REGISTER_TUNE_BASS_DRUM = (ADC >> 2);  // CV rail to rail, range 0V to 5V
          break;
      }
      ready_to_read_ = false;
      current_adc_channel_ = (current_adc_channel_ + 1) % ADMUX_NUM_CHANNELS;
    }
  }
};

extern Adc g_adc;

#endif /* ADC_HPP_ */