/*
 * midi_message.h
 *
 * Created: 3/22/2022 10:52:29 PM
 *  Author: naoki
 */

#ifndef MIDI_MESSAGE_H_
#define MIDI_MESSAGE_H_

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

#endif /* MIDI_MESSAGE_H_ */