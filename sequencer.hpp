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
  static constexpr int kTotalPatternBytes = kNumDrums * kPatternBytes;

  uint16_t* tempo_clock_count_;
  uint16_t* tempo_interval_;

  uint8_t patterns_[kNumDrums][kPatternBytes];
  uint8_t accents_[kNumDrums][kPatternBytes];
  uint32_t clock0_;
  uint32_t prev_clock_;
  uint32_t prev_boundary_;
  uint32_t last_triggers_[kNumDrums];
  uint8_t last_accent_;

  int16_t position_;

  uint8_t state_;

  // [-3] - inactive, [-2:0] - write the tempo, [0:] - write the pattern and the accents
  int16_t data_index_;

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
  static constexpr uint8_t kFinishingRecording = 16;

  inline uint8_t DrumIndex(Drum drum) { return static_cast<uint8_t>(drum); }

  Sequencer(uint16_t* tempo_clock_count, uint16_t* tempo_wrap)
      : tempo_clock_count_{tempo_clock_count},
        tempo_interval_{tempo_wrap},
        position_{-1},
        state_{kStandBy},
        data_index_{-1} {
    Clear();
    for (auto idrum = 0; idrum < kNumDrums; ++idrum) {
      for (auto ipattern = 0; ipattern < kPatternBytes; ++ipattern) {
        auto* ptr = reinterpret_cast<uint8_t*>(E_PATTERN1) + idrum * kPatternBytes + ipattern;
        patterns_[idrum][ipattern] = eeprom_read_byte(ptr);
        accents_[idrum][ipattern] = eeprom_read_byte(ptr + kTotalPatternBytes);
      }
    }
  }

  inline uint8_t GetState() const { return state_; }

  inline uint16_t GetPosition() const { return position_; }

  inline void Start() {
    ClearBit(PORT_DIN_START, BIT_DIN_START);
    state_ = kRunning;
  }

  inline void Stop() { state_ = kStopping; }

  inline void HardStop() {
    SetBit(PORT_DIN_START, BIT_DIN_START);
    state_ = kStandBy;
  }

  inline void Toggle() {
    if (state_ == kStandBy) {
      Start();
    } else if (state_ == kRunning) {
      Stop();
    }
  }

  inline void StandByRecording() {
    position_ = -1;
    state_ = kStandByRecording;
    *tempo_clock_count_ = 0;
    prev_boundary_ = 0;
    SetBit(PORT_DIN_START, BIT_DIN_START);
    Clear();
  }

  inline void StartRecording() {
    state_ = kRecording;
    prev_clock_ = *tempo_clock_count_;
    clock0_ = prev_clock_;
    ClearBit(PORT_LED_DIN_MUTE, BIT_LED_DIN_MUTE);
  }

  inline void EndRecording() {
    state_ = kFinishingRecording;
    data_index_ = 0;
    position_ = -1;
  }

  inline void WriteTempoInterval() { data_index_ = kTotalPatternBytes * 2; }

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
  inline void PlayPatternAt(uint8_t index, uint8_t mask) {
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
    PlayPatternAt<Drum::kBassDrum>(byte, mask);
    PlayPatternAt<Drum::kSnareDrum>(byte, mask);
    PlayPatternAt<Drum::kRimShot>(byte, mask);
    PlayPatternAt<Drum::kHandClap>(byte, mask);
    PlayPatternAt<Drum::kClosedHiHat>(byte, mask);
    PlayPatternAt<Drum::kOpenHiHat>(byte, mask);

    if (state_ == kStopping && (position_ % kTicksPerBar) == (kTicksPerBar - 1)) {
      HardStop();
    }
  }

  void StepForwardRecording() {
    if (state_ != kRecording) {
      return;
    }
    if (position_ >= 0) {
      // quantize
      *tempo_interval_ = (*tempo_clock_count_ - prev_clock_) >> 1;
      auto margin = *tempo_interval_;
      if (prev_boundary_ == 0) {
        prev_boundary_ = prev_clock_ - margin;
      }
      uint32_t next_boundary = prev_clock_ + margin;
      uint8_t byte = position_ >> 3;
      uint8_t mask = _BV(7 - (position_ & 0x7));
      for (auto i = 0; i < kNumDrums; ++i) {
        auto last_trigger = last_triggers_[i];
        if (last_trigger >= prev_boundary_ && last_trigger < next_boundary) {
          patterns_[i][byte] |= mask;
          if (last_accent_ & _BV(i)) {
            accents_[i][byte] |= mask;
          }
        }
      }
      prev_boundary_ = next_boundary;
    }

    prev_clock_ = *tempo_clock_count_;
    if (position_ == kTotalClockTicks - 1) {
      // The position reached to the end, start saving the pattern.
      *tempo_interval_ = (*tempo_clock_count_ - clock0_) / kTotalClockTicks / 2;
      EndRecording();
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
    if (state_ != kStandByRecording && state_ != kFinishingRecording) {
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

  void Poll() {
    if (data_index_ < 0 || !eeprom_is_ready()) {
      return;
    }

    if ((data_index_ & 4) == 0) {
      ToggleBit(PORT_LED_DIN_MUTE, BIT_LED_DIN_MUTE);
    }
    if (data_index_ < kTotalPatternBytes) {
      uint8_t* ptr = &patterns_[0][0];
      eeprom_write_async(E_PATTERN1 + data_index_, ptr[data_index_]);
    } else if (data_index_ < kTotalPatternBytes * 2) {
      uint8_t* ptr = &accents_[0][0];
      eeprom_write_async(E_PATTERN1 + data_index_, ptr[data_index_ - kTotalPatternBytes]);
    } else if (data_index_ == kTotalPatternBytes * 2) {
      eeprom_write_async(E_TEMPO, (*tempo_interval_) & 0xff);
    } else if (data_index_ == kTotalPatternBytes * 2 + 1) {
      eeprom_write_async(E_TEMPO + 1, ((*tempo_interval_) >> 8) & 0xff);
    }

    if (++data_index_ == kTotalPatternBytes * 2 + 2) {
      data_index_ = -1;
      if (state_ == kFinishingRecording) {
        ClearBit(PORT_LED_DIN_MUTE, BIT_LED_DIN_MUTE);
        state_ = kStandBy;
      }
    }
  }
};

#endif /* SEQUENCER_HPP_ */