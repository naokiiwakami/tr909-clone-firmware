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
 public:
  static constexpr int kNumDrums = static_cast<int>(Drum::kOutOfRange);
  static constexpr int kNumBars = 4;
  static constexpr int kClocksPerQuarterNote = 24;
  static constexpr int kClocksPerBar = kClocksPerQuarterNote * 4;
  static constexpr int kTotalClocks = kNumBars * kClocksPerBar;
  static constexpr int kBitsPerStep = 2;
  static constexpr int kPatternBytes = (kTotalClocks / 8) * kBitsPerStep;
  static constexpr int kTotalPatternBytes = kNumDrums * kPatternBytes;

 private:
  static constexpr uint8_t kTapHistory = 2;

  uint32_t master_tempo_ticks_ = 0;

  // tap tempo
  uint8_t tap_count_ = 0;
  int8_t tap_current_ = 0;
  uint32_t ticks_history_[kTapHistory];

  // pattern recording
  uint32_t ticks_last_quarter_note_;
  uint32_t ticks_last_clock_;
  uint32_t prev_boundary_;

  uint32_t last_triggers_[kNumDrums];
  uint16_t last_levels_;

  // playing the pattern
  uint8_t patterns_[kNumDrums][kPatternBytes];
  int16_t position_ = -1;
  uint8_t pattern_id_;

  uint16_t tempo_interval_count_ = 0;
  uint16_t tempo_interval_;
  
  uint8_t drum_masks_;

  // sequencer state
  uint8_t state_ = kStandBy;

  // eeprom control
  // masks for eeprom_write_enabled_ bits
  static constexpr uint8_t kMaskMidiCh = 0x1;
  static constexpr uint8_t kMaskTempo = 0x2;
  static constexpr uint8_t kMaskPattern1 = 0x4;

  uint8_t eeprom_write_enabled_ = 0;
  int16_t data_index_ = 0;

  DinSync din_sync_;

  static constexpr void (*hit_[])(int8_t) = {
      HitOpenHiHat, HitClosedHiHat, HitHandClap, HitRimShot, HitSnareDrum, HitBassDrum, 
  };

 public:
  static constexpr uint8_t kStandBy = 0;
  static constexpr uint8_t kRunning = 1;
  static constexpr uint8_t kStopping = 2;
  static constexpr uint8_t kStandByRecording = 4;
  static constexpr uint8_t kRecording = 8;
  static constexpr uint8_t kFinishingRecording = 16;

  inline uint8_t DrumIndex(Drum drum) { return static_cast<uint8_t>(drum); }

  Sequencer() { Clear(); }

  void Initialize() {
    pattern_id_ = eeprom_read_byte(E_PATTERN_ID);
    LoadPattern();
    tempo_interval_ = eeprom_read_word(reinterpret_cast<uint16_t*>(E_TEMPO));
    drum_masks_ = 0xff;
  }

  void LoadPattern() {
    MapToLed(0x20 >> pattern_id_);
    eeprom_read_block(patterns_,
                      reinterpret_cast<uint8_t*>(E_PATTERN) + kTotalPatternBytes * pattern_id_,
                      kTotalPatternBytes);
    MapToLed(0);
  }

  inline uint8_t GetState() const { return state_; }
    
  inline bool IsPlaying() const { return state_ == kRunning || state_ == kStopping; }

  inline uint16_t GetPosition() const { return position_; }

  inline uint8_t GetTapCount() const { return tap_count_; }
  inline void ResetTapCount() { tap_count_ = 0; }

  inline uint16_t GetTempoInterval() const { return tempo_interval_; }

  inline uint8_t GetPatternId() const { return pattern_id_; }
  inline void SetPatternId(uint8_t pattern_id) {
    pattern_id_ = pattern_id;
    eeprom_update_byte(E_PATTERN_ID, pattern_id);
  }

  inline void IncrementClock() {
    din_sync_.Update();
    if (++tempo_interval_count_ >= tempo_interval_) {
      StepForward();
      tempo_interval_count_ = 0;
    }

    ++master_tempo_ticks_;
  }

  inline void Start() {
    din_sync_.Start();
    MapToLed(drum_masks_);
    state_ = kRunning;
  }

  inline void Stop() { state_ = kStopping; }

  inline void HardStop() {
    din_sync_.Stop();
    state_ = kStandBy;
    MapToLed(0);
  }

  inline void ToggleStartStop() {
    if (state_ == kStandBy) {
      Start();
    } else if (state_ == kRunning) {
      Stop();
    }
  }
  
  inline void ToggleMask(uint8_t drum_index) {
    drum_masks_ ^= _BV(drum_index);
    // TODO: Better to toggle only changed LED
    MapToLed(drum_masks_);
  }

  inline void StandByRecording() {
    position_ = -1;
    state_ = kStandByRecording;
    master_tempo_ticks_ = 0;
    prev_boundary_ = 0;
    din_sync_.Start();
    Clear();
    MapToLed(0x20 >> pattern_id_);
  }

  inline void StartRecording() {
    if (state_ == kStandByRecording) {
      state_ = kRecording;
      ticks_last_clock_ = master_tempo_ticks_;
      tap_count_ = 0;
      ticks_last_quarter_note_ = ticks_last_clock_;
      ClearBit(PORT_LED_BASS_DRUM, BIT_LED_BASS_DRUM);
      ClearBit(PORT_LED_SNARE_DRUM, BIT_LED_SNARE_DRUM);
      ClearBit(PORT_LED_RIM_SHOT, BIT_LED_RIM_SHOT);
      ClearBit(PORT_LED_DIN_MUTE, BIT_LED_DIN_MUTE);
    } else {
      din_sync_.Start();
    }
  }

  inline void EndRecording() {
    if (state_ == Sequencer::kRecording) {
      state_ = kFinishingRecording;
      position_ = -1;
      StartWritingTempo();
      StartWritingPattern();
    } else {
      din_sync_.Stop();
    }
  }

  inline void StartWritingTempo() { eeprom_write_enabled_ |= kMaskTempo; }

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
    if (!(drum_masks_ & _BV(drum_index))) {
      return;
    }
    constexpr auto hit = hit_[drum_index];
    uint8_t level = (patterns_[drum_index][index] >> bit) & 0x3;
    switch (level) {
      case 0x3:
        hit(127);
        break;
      case 0x2:
        hit(85);
        break;
      case 0x1:
        hit(50);
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
    din_sync_.Clock();
    if (++position_ == kTotalClocks) {
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

    if (state_ == kStopping && (position_ % kClocksPerBar) == (kClocksPerBar - 1)) {
      HardStop();
    }
  }

  void StepForwardRecording() {
    if (state_ != kRecording) {
      if (state_ == kStandBy) {
        din_sync_.Clock();
      }
      return;
    }
    if (position_ >= 0) {
      // quantize
      tempo_interval_ = master_tempo_ticks_ - ticks_last_clock_;
      auto margin = tempo_interval_ >> 1;
      if (prev_boundary_ == 0) {
        prev_boundary_ = ticks_last_clock_ - margin;
      }
      uint32_t next_boundary = ticks_last_clock_ + margin;
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

    if (position_ == kTotalClocks - 1) {
      tempo_interval_ = (master_tempo_ticks_ - ticks_last_quarter_note_) / 24;
      EndRecording();
      return;
    } else {
      ++position_;
    }
    ticks_last_clock_ = master_tempo_ticks_;

    // blink the tempo indicator
    auto mod = position_ % kClocksPerQuarterNote;
    if (mod == 0) {
      ticks_last_quarter_note_ = master_tempo_ticks_;
      SetBit(PORT_LED_DIN_MUTE, BIT_LED_DIN_MUTE);
    } else if (mod == 3) {
      ClearBit(PORT_LED_DIN_MUTE, BIT_LED_DIN_MUTE);
    }
  }

  /**
   * Triggers an action for a drum.
   *
   * The action is different by the sequencer state:
   *  - StandBy  : hit
   *  - Running  : toggle mask
   *  - Stopping : toggle mask
   *  - StandByRecording : update the last trigger timestamp
   *  - Recording : hit, update the last trigger timestamp
   *  - FinishRecording  : No operation
   */
  template <Drum drum, bool is_midi = false>
  void Trigger(int8_t velocity) {
    constexpr uint8_t drum_index = static_cast<uint8_t>(drum);
    constexpr auto hit = hit_[drum_index];
    if (state_ == kRunning || state_ == kStopping) {
       if (!is_midi) {
         ToggleMask(drum_index);       
       }
       return;
    }
    if (state_ != kStandByRecording && state_ != kFinishingRecording) {
      hit(velocity);
    }
    if (state_ == kRecording || state_ == kStandByRecording) {
      last_triggers_[drum_index] = master_tempo_ticks_;
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
      master_tempo_ticks_ = 0;
      tap_current_ = kTapHistory - 1;
      SetBit(PORT_LED_DIN_MUTE, BIT_LED_DIN_MUTE);
    } else {
      uint32_t diff = Subtract<uint32_t>(
          master_tempo_ticks_, ticks_history_[(tap_current_ + (tap_count_ - 1)) % kTapHistory]);
      diff /= 24 * tap_count_;
      tempo_interval_ = diff;

      if (--tap_current_ < 0) {
        tap_current_ = kTapHistory - 1;
      }
    }
    ticks_history_[tap_current_] = master_tempo_ticks_;
    if (tap_count_ < kTapHistory) {
      ++tap_count_;
    }
  }

  inline void Poll() {
    if (eeprom_write_enabled_ == 0 || !eeprom_is_ready()) {
      return;
    }

    if (eeprom_write_enabled_ & kMaskTempo) {
      eeprom_write_word(reinterpret_cast<uint16_t*>(E_TEMPO), tempo_interval_);
      eeprom_write_enabled_ &= ~kMaskTempo;
      return;
    }

    if ((data_index_ & 0xf) == 0) {
      ToggleBit(PORT_LED_DIN_MUTE, BIT_LED_DIN_MUTE);
    }

    uint8_t* ptr = &patterns_[0][0];
    eeprom_write_async(E_PATTERN + kTotalPatternBytes * pattern_id_ + data_index_,
                       ptr[data_index_]);

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