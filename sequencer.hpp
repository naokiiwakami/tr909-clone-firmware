/*
 * sequencer.hpp
 */

#ifndef SEQUENCER_HPP_
#define SEQUENCER_HPP_

#include "din_sync.hpp"
#include "eeprom.hpp"
#include "instruments.hpp"
#include "utils.hpp"

class Sequencer {
 private:
  static constexpr int kNumDrums = static_cast<int>(Drum::kOutOfRange);
  static constexpr int kNumBars = 4;
  static constexpr int kTicksPerQuarterNote = 24;
  static constexpr int kTicksPerBar = kTicksPerQuarterNote * 4;
  static constexpr int kTotalClockTicks = kNumBars * kTicksPerBar;
  static constexpr int kBitsPerStep = 2;
  static constexpr int kPatternBytes = (kTotalClockTicks / 8) * kBitsPerStep;
  static constexpr int kTotalPatternBytes = kNumDrums * kPatternBytes;

  static constexpr uint8_t kTapHistory = 2;

  uint32_t tempo_clock_count_;
  uint16_t tempo_interval_;
  uint8_t tap_count_;
  int8_t tap_current_;
  uint32_t tempo_clocks_[kTapHistory];
  uint8_t patterns_[kNumDrums][kPatternBytes];
  uint32_t clock0_;
  uint32_t prev_clock_;
  uint32_t prev_boundary_;
  uint32_t last_triggers_[kNumDrums];
  uint16_t last_levels_;

  int16_t position_;

  uint8_t state_;

  // eeprom control
  uint8_t eeprom_write_enabled_;
  // masks for eeprom_write_enabled_ bits
  static constexpr uint8_t kMaskMidiCh = 0x1;
  static constexpr uint8_t kMaskTempoL = 0x2;
  static constexpr uint8_t kMaskTempoH = 0x4;
  static constexpr uint8_t kMaskPattern1 = 0x8;

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

  Sequencer()
      : tempo_clock_count_{0},
        tap_count_{0},
        tap_current_{0},
        position_{-1},
        state_{kStandBy},
        eeprom_write_enabled_{0},
        data_index_{0} {
    Clear();
  }

  void Initialize() {
    eeprom_read_block(patterns_, reinterpret_cast<uint8_t*>(E_PATTERN1), kTotalPatternBytes);
    tempo_interval_ = eeprom_read_word(reinterpret_cast<uint16_t*>(E_TEMPO));
    SetBit(PORT_LED_DIN_MUTE, BIT_LED_DIN_MUTE);
  }

  inline uint8_t GetState() const { return state_; }

  inline uint16_t GetPosition() const { return position_; }

  inline uint8_t GetTapCount() const { return tap_count_; }
  inline void ResetTapCount() { tap_count_ = 0; }

  inline uint16_t GetTempoInterval() const { return tempo_interval_; }

  inline void IncrementClock() { ++tempo_clock_count_; }

  inline void Start() {
    g_din_sync.Start();
    state_ = kRunning;
  }

  inline void Stop() { state_ = kStopping; }

  inline void HardStop() {
    g_din_sync.Stop();
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
    tempo_clock_count_ = 0;
    prev_boundary_ = 0;
    g_din_sync.Start();
    Clear();
  }

  inline void StartRecording() {
    if (state_ == kStandByRecording) {
      state_ = kRecording;
      prev_clock_ = tempo_clock_count_;
      tap_count_ = 0;
      clock0_ = prev_clock_;
      ClearBit(PORT_LED_DIN_MUTE, BIT_LED_DIN_MUTE);
    } else {
      g_din_sync.Start();
    }
  }

  inline void EndRecording() {
    if (state_ == Sequencer::kRecording) {
      state_ = kFinishingRecording;
      position_ = -1;
      StartWritingTempo();
      StartWritingPattern();
    } else {
      g_din_sync.Stop();
    }
  }

  inline void StartWritingTempo() { eeprom_write_enabled_ |= kMaskTempoL | kMaskTempoH; }

  inline void StartWritingPattern() {
    eeprom_write_enabled_ |= kMaskPattern1;
    data_index_ = 0;
  }

  void Clear() {
    for (int i = 0; i < kNumDrums; ++i) {
      last_triggers_[i] = 0;
      for (int j = 0; j < kPatternBytes; ++j) {
        patterns_[i][j] = 0;
      }
    }
    last_levels_ = 0;
  }

  template <Drum drum>
  inline void PlayPatternAt(uint8_t index, uint8_t bit) {
    constexpr uint8_t drum_index = static_cast<uint8_t>(drum);
    constexpr auto trigger_func = trigger_func_[drum_index];
    uint8_t level = (patterns_[drum_index][index] >> bit) & 0x3;
    switch (level) {
      case 0x3:
        trigger_func(127);
        break;
      case 0x2:
        trigger_func(85);
        break;
      case 0x1:
        trigger_func(50);
        break;
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
    uint8_t byte = position_ >> 2;
    uint8_t bit = (position_ & 0x3) << 1;
    PlayPatternAt<Drum::kBassDrum>(byte, bit);
    PlayPatternAt<Drum::kSnareDrum>(byte, bit);
    PlayPatternAt<Drum::kRimShot>(byte, bit);
    PlayPatternAt<Drum::kHandClap>(byte, bit);
    PlayPatternAt<Drum::kClosedHiHat>(byte, bit);
    PlayPatternAt<Drum::kOpenHiHat>(byte, bit);

    if (state_ == kStopping && (position_ % kTicksPerBar) == (kTicksPerBar - 1)) {
      HardStop();
    }
  }

  void StepForwardRecording() {
    if (state_ != kRecording) {
      if (state_ == kStandBy) {
        // g_din_sync.Clock();
      }
      return;
    }
    if (position_ >= 0) {
      // quantize
      tempo_interval_ = (tempo_clock_count_ - prev_clock_) >> 1;
      auto margin = tempo_interval_;
      if (prev_boundary_ == 0) {
        prev_boundary_ = prev_clock_ - margin;
      }
      uint32_t next_boundary = prev_clock_ + margin;
      uint8_t byte = position_ >> 2;
      uint8_t bit = (position_ & 0x3) << 1;
      for (auto i = 0; i < kNumDrums; ++i) {
        auto last_trigger = last_triggers_[i];
        if (last_trigger >= prev_boundary_ && last_trigger < next_boundary) {
          patterns_[i][byte] |= ((last_levels_ >> (i * 2)) & 0x3) << bit;
        }
      }
      prev_boundary_ = next_boundary;
    }

    if (position_ == kTotalClockTicks - 1) {
      tempo_interval_ = (tempo_clock_count_ - clock0_) / 24;
      EndRecording();
      return;
    } else {
      ++position_;
    }
    prev_clock_ = tempo_clock_count_;

    // blink the tempo indicator
    auto mod = position_ % kTicksPerQuarterNote;
    if (mod == 0) {
      clock0_ = tempo_clock_count_;
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
      last_triggers_[drum_index] = tempo_clock_count_;
      if (velocity >= 102) {
        last_levels_ |= 0x3 << (drum_index * 2);
      } else if (velocity >= 76) {
        last_levels_ |= _BV(drum_index * 2 + 1);
        last_levels_ &= ~_BV(drum_index * 2);
      } else {
        last_levels_ &= ~_BV(drum_index * 2 + 1);
        last_levels_ |= _BV(drum_index * 2);
      }
    }
  }

  inline void Tap() {
    if (tap_count_ == 0) {
      tempo_clock_count_ = 0;
      tap_current_ = kTapHistory - 1;
      SetBit(PORT_LED_DIN_MUTE, BIT_LED_DIN_MUTE);
    } else {
      uint32_t diff = Subtract<uint32_t>(
          tempo_clock_count_, tempo_clocks_[(tap_current_ + (tap_count_ - 1)) % kTapHistory]);
      diff /= 24 * tap_count_;
      tempo_interval_ = diff;

      if (--tap_current_ < 0) {
        tap_current_ = kTapHistory - 1;
      }
    }
    tempo_clocks_[tap_current_] = tempo_clock_count_;
    if (tap_count_ < kTapHistory) {
      ++tap_count_;
    }
  }

  void Poll() {
    if (eeprom_write_enabled_ == 0 || !eeprom_is_ready()) {
      return;
    }

    if (eeprom_write_enabled_ & kMaskTempoL) {
      eeprom_write_async(E_TEMPO, (tempo_interval_)&0xff);
      eeprom_write_enabled_ &= ~kMaskTempoL;
      return;
    }
    if (eeprom_write_enabled_ & kMaskTempoH) {
      eeprom_write_async(E_TEMPO + 1, ((tempo_interval_) >> 8) & 0xff);
      eeprom_write_enabled_ &= ~kMaskTempoH;
      return;
    }

    if ((data_index_ & 8) == 0) {
      ToggleBit(PORT_LED_DIN_MUTE, BIT_LED_DIN_MUTE);
    }

    uint8_t* ptr = &patterns_[0][0];
    eeprom_write_async(E_PATTERN1 + data_index_, ptr[data_index_]);

    if (++data_index_ == kTotalPatternBytes) {
      data_index_ = 0;
      eeprom_write_enabled_ &= ~kMaskPattern1;
      if (state_ == kFinishingRecording) {
        ClearBit(PORT_LED_DIN_MUTE, BIT_LED_DIN_MUTE);
        state_ = kStandBy;
      }
    }
  }
};

extern Sequencer g_sequencer;

#endif /* SEQUENCER_HPP_ */