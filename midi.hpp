/*
 * midi_message.h
 */
#ifndef MIDI_MESSAGE_HPP_
#define MIDI_MESSAGE_HPP_

#include <avr/io.h>

#include "eeprom.hpp"
#include "sequencer.hpp"
#include "system.hpp"

// serial interface setup
static constexpr uint32_t USART_BAUD_RATE = 31250;
static constexpr uint8_t USART_BUF_SIZE = 16;  // must be power of 2
static constexpr uint16_t USART_BAUD_SELECT = (F_OSC / (USART_BAUD_RATE * 16) - 1);

// MIDI messages
static constexpr uint8_t MIDI_NOTE_OFF = 0x80;
static constexpr uint8_t MIDI_NOTE_ON = 0x90;
static constexpr uint8_t MIDI_CONTROL_CHANGE = 0xb0;
static constexpr uint8_t MIDI_PROGRAM_CHANGE = 0xc0;
static constexpr uint8_t MIDI_CHANNEL_PRESSURE = 0xd0;

static constexpr uint8_t MIDI_REALTIME_START = 0xfa;
static constexpr uint8_t MIDI_REALTIME_STOP = 0xfc;
static constexpr uint8_t MIDI_REALTIME_CLOCK = 0xf8;

static constexpr uint8_t MIDI_NOTE_BASS_DRUM = 36;      // C1
static constexpr uint8_t MIDI_NOTE_SNARE_DRUM = 38;     // D1
static constexpr uint8_t MIDI_NOTE_RIM_SHOT = 37;       // C#1
static constexpr uint8_t MIDI_NOTE_HAND_CLAP = 39;      // D#1
static constexpr uint8_t MIDI_NOTE_CLOSED_HI_HAT = 42;  // F#1
static constexpr uint8_t MIDI_NOTE_OPEN_HI_HAT = 46;    // A#1

class MidiReceiver {
 private:
  uint8_t status_ = 0;
  uint8_t current_channel_ = 0;
  uint8_t data_[2] = {0, 0};
  uint8_t data_length_mask_ = 0;
  uint8_t data_pointer_ = 0;

  uint8_t listening_channel_ = 0;

 public:
  MidiReceiver() {}

  void Initialize();

  inline uint8_t GetChannel() const { return listening_channel_; }
  inline void SetChannel(uint8_t channel) { listening_channel_ = channel; }

  void ParseMidiInput(uint8_t next_byte) {
    if (next_byte >= 0xf0) {
      switch (next_byte) {
        case MIDI_REALTIME_START:
          g_sequencer.StartRecording();
          break;
        case MIDI_REALTIME_CLOCK:
          g_sequencer.StepForwardRecording();
          break;
        case MIDI_REALTIME_STOP:
          g_sequencer.EndRecording();
          break;
      }
      return;
    }
    if (next_byte > 0x7f) {  // is status byte
      status_ = next_byte & 0xf0;
      current_channel_ = next_byte & 0x0f;
      switch (status_) {
        case MIDI_PROGRAM_CHANGE:
        case MIDI_CHANNEL_PRESSURE:
          data_length_mask_ = 0;
          break;
        default:
          data_length_mask_ = 1;
      }
      data_pointer_ = 0;
    } else {  // data byte
      data_[data_pointer_ & 0x1] = next_byte;
      if ((++data_pointer_ & data_length_mask_) == 0) {
        if (current_channel_ == listening_channel_) {
          ProcessMidiChannelMessage();
        }
      }
    }
  }

  void ProcessMidiChannelMessage() {
    switch (status_) {
      case MIDI_NOTE_ON: {
        auto& velocity = data_[1];
        if (velocity == 0) {
          // this is actually a note-off message
          break;
        }
        auto& note = data_[0];
        switch (note) {
          case MIDI_NOTE_BASS_DRUM:
            g_sequencer.Trigger<Drum::kBassDrum>(velocity);
            break;
          case MIDI_NOTE_SNARE_DRUM:
            g_sequencer.Trigger<Drum::kSnareDrum>(velocity);
            break;
          case MIDI_NOTE_RIM_SHOT:
            g_sequencer.Trigger<Drum::kRimShot>(velocity);
            break;
          case MIDI_NOTE_HAND_CLAP:
            g_sequencer.Trigger<Drum::kHandClap>(velocity);
            break;
          case MIDI_NOTE_CLOSED_HI_HAT:
            g_sequencer.Trigger<Drum::kClosedHiHat>(velocity);
            break;
          case MIDI_NOTE_OPEN_HI_HAT:
            g_sequencer.Trigger<Drum::kOpenHiHat>(velocity);
            break;
        }
      } break;
    }
  }
};

extern MidiReceiver g_midi_receiver;

void MidiReceiver::Initialize() {
  // Set baud rate
  UBRR1H = static_cast<uint8_t>((USART_BAUD_SELECT >> 8) & 0xff);
  UBRR1L = static_cast<uint8_t>(USART_BAUD_SELECT & 0xff);

  // Enable receiver
  UCSR1B = _BV(RXEN1);

  // Set frame format: asynchronous operation, parity disabled, 8 data, 1 stop bit */
  UCSR1C = _BV(UCSZ11) | _BV(UCSZ10);

  SetChannel(eeprom_read_byte(E_MIDI_CH));
}

#endif /* MIDI_MESSAGE_HPP_ */