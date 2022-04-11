/*
 * sequencer.hpp
 */

#ifndef SEQUENCER_HPP_
#define SEQUENCER_HPP_

#include "instruments.hpp"
#include "utils.hpp"

class Sequencer {
 private:
  static constexpr int kNumDrums = static_cast<int>(Drum::kOutOfRange);
  static constexpr int kNumBars = 4;
  static constexpr int kTicksPerQuarterNote = 24;
  static constexpr int kTicksPerBar = kTicksPerQuarterNote * 4;
  static constexpr int kTotalClockTicks = kNumBars * kTicksPerBar;
  static constexpr int kPatternBytes = kTotalClockTicks / 8;

  uint32_t* tempo_clock_count_;
  uint16_t* tempo_wrap_;

  uint8_t patterns_[kNumDrums][kPatternBytes];
  uint8_t accents_[kNumDrums][kPatternBytes];
  uint32_t clock0_;
  uint32_t prev_clock_;
  uint32_t last_triggers_[kNumDrums];
  uint8_t last_accent_;

  int16_t position_;

  uint8_t state_;

  static constexpr void (*trigger_func_[])(int8_t) = {
      TriggerBassDrum, TriggerSnareDrum,   TriggerRimShot,
      TriggerHandClap, TriggerClosedHiHat, TriggerOpenHiHat,
  };

 public:
  static constexpr uint8_t kStandBy = 0;
  static constexpr uint8_t kRunning = 1;
  static constexpr uint8_t kStopping = 2;
  static constexpr uint8_t kStandByRecording = 4;
  static constexpr uint8_t kRecording = 8;

  inline uint8_t DrumIndex(Drum drum) { return static_cast<uint8_t>(drum); }

  Sequencer(uint32_t* tempo_clock_count, uint16_t* tempo_wrap)
      : tempo_clock_count_{tempo_clock_count},
        tempo_wrap_{tempo_wrap},
        position_{-1},
        state_{kStandBy} {
    Clear();
    for (auto idrum = 0; idrum < kNumDrums; ++idrum) {
      for (auto ipattern = 0; ipattern < kPatternBytes; ++ipattern) {
        auto* ptr = E_PATTERN1 + idrum * kPatternBytes + ipattern;
        patterns_[idrum][ipattern] = eeprom_read_byte(ptr);
        accents_[idrum][ipattern] = eeprom_read_byte(ptr + kNumDrums * kPatternBytes);
      }
    }
  }

  inline uint8_t GetState() const { return state_; }

  inline void Start() { state_ = kRunning; }

  inline void Stop() { state_ = kStopping; }

  inline void StopImmediately() { state_ = kStandBy; }

  inline void Toggle() {
    if (state_ == kStandBy) {
      Start();
    } else if (state_ == kRunning) {
      Stop();
    }
  }

  inline void StandByRecording() {
    position_ = -1;
    *tempo_clock_count_ = 0;
    state_ = kStandByRecording;
    Clear();
  }

  inline void StartRecording() {
    state_ = kRecording;
    prev_clock_ = *tempo_clock_count_;
    clock0_ = prev_clock_;
    ClearBit(PORT_LED_DIN_MUTE, BIT_LED_DIN_MUTE);
  }

  void Clear() {
    for (int i = 0; i < kNumDrums; ++i) {
      last_triggers_[i] = 0;
      for (int j = 0; j < kPatternBytes; ++j) {
        patterns_[i][j] = 0;
        accents_[i][j] = 0;
      }
    }
    last_accent_ = 0;
  }

  template <Drum drum>
  inline void Check(uint8_t index, uint8_t mask) {
    constexpr uint8_t drum_index = static_cast<uint8_t>(drum);
    constexpr auto trigger_func = trigger_func_[drum_index];
    if (patterns_[drum_index][index] & mask) {
      trigger_func((accents_[drum_index][index] & mask) ? 127 : 63);
    }
  }

  void StepForward() {
    if (state_ & kStandByRecording) {
      ToggleBit(PORT_LED_DIN_MUTE, BIT_LED_DIN_MUTE);
    }
    if ((state_ & (kRunning | kStopping)) == 0) {
      return;
    }
    if (++position_ == kTotalClockTicks) {
      position_ = 0;
    }
    auto mod = position_ % 24;
    if (mod == 0) {
      SetBit(PORT_LED_DIN_MUTE, BIT_LED_DIN_MUTE);
    } else if (mod == 3) {
      ClearBit(PORT_LED_DIN_MUTE, BIT_LED_DIN_MUTE);
    }
    uint8_t byte = position_ >> 3;
    uint8_t mask = _BV(7 - (position_ & 0x7));
    Check<Drum::kBassDrum>(byte, mask);
    Check<Drum::kSnareDrum>(byte, mask);
    Check<Drum::kRimShot>(byte, mask);
    Check<Drum::kHandClap>(byte, mask);
    Check<Drum::kClosedHiHat>(byte, mask);
    Check<Drum::kOpenHiHat>(byte, mask);

    if (state_ == kStopping && (position_ % kTicksPerBar) == (kTicksPerBar - 1)) {
      state_ = kStandBy;
    }
  }

  void StepForwardRecording() {
    if (state_ != kRecording) {
      return;
    }
    if (position_ >= 0) {
      *tempo_wrap_ = *tempo_clock_count_ - prev_clock_;
      auto margin = *tempo_wrap_ >> 1;
      uint8_t byte = position_ >> 3;
      uint8_t mask = _BV(7 - (position_ & 0x7));
      for (auto i = 0; i < kNumDrums; ++i) {
        auto trigger_clock = last_triggers_[i];
        if (trigger_clock >= prev_clock_ - margin && trigger_clock < prev_clock_ + margin) {
          patterns_[i][byte] |= mask;
          if (last_accent_ & _BV(i)) {
            accents_[i][byte] |= mask;
          }
        }
      }
    }

    prev_clock_ = *tempo_clock_count_;
    if (position_ == kTotalClockTicks - 1) {
      StopImmediately();
      *tempo_wrap_ = (*tempo_clock_count_ - clock0_) / kTotalClockTicks;
      for (auto idrum = 0; idrum < kNumDrums; ++idrum) {
        for (auto ipattern = 0; ipattern < kPatternBytes; ++ipattern) {
          if ((ipattern & 4) == 0) {
            ToggleBit(PORT_LED_DIN_MUTE, BIT_LED_DIN_MUTE);
          }
          auto* ptr = E_PATTERN1 + idrum * kPatternBytes + ipattern;
          eeprom_write_byte(ptr, patterns_[idrum][ipattern]);
          eeprom_write_byte(ptr + kNumDrums * kPatternBytes, accents_[idrum][ipattern]);
        }
      }
      eeprom_write_word(E_TEMPO, *tempo_wrap_);
      ClearBit(PORT_LED_DIN_MUTE, BIT_LED_DIN_MUTE);
      return;
    } else {
      ++position_;
    }

    // blink the tempo indicator
    auto mod = position_ % kTicksPerQuarterNote;
    if (mod == 0) {
      SetBit(PORT_LED_DIN_MUTE, BIT_LED_DIN_MUTE);
    } else if (mod == 3) {
      ClearBit(PORT_LED_DIN_MUTE, BIT_LED_DIN_MUTE);
    }
  }

  template <Drum drum>
  void Trigger(int8_t velocity) {
    constexpr uint8_t drum_index = static_cast<uint8_t>(drum);
    constexpr auto trigger_func = trigger_func_[drum_index];
    if (state_ != kStandByRecording) {
      trigger_func(velocity);
    }
    if (state_ == kRecording || state_ == kStandByRecording) {
      last_triggers_[drum_index] = *tempo_clock_count_;
      if (velocity >= 96) {
        last_accent_ |= _BV(drum_index);
      } else {
        last_accent_ &= ~_BV(drum_index);
      }
    }
  }
};

#endif /* SEQUENCER_HPP_ */