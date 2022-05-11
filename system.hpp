/*
 * tr909_clone system configuration
 */
#ifndef SYSTEM_HPP_
#define SYSTEM_HPP_

// System clock frequency
static constexpr uint32_t F_OSC = 16000000;  // 16 MHz

static constexpr uint8_t kOperationModeNormal = 0x1;
static constexpr uint8_t kOperationModeDirectPlay = 0x2;
static constexpr uint8_t kOperationModeRecording = 0x4;
static constexpr uint8_t kOperationModeChangePattern = 0x8;
static constexpr uint8_t kOperationModePatternChanged = 0x10;
static constexpr uint8_t kOperationModePatternTransiting = 0x20;
extern uint8_t g_operation_mode;
// to keep the previous mode before entering a temporary mode
extern uint8_t g_operation_mode_prev;
extern uint8_t g_pattern_changed_countdown;

inline bool FlashOnHitEnabled() {
  return (g_operation_mode & (kOperationModeDirectPlay | kOperationModeRecording)) &&
         !(g_operation_mode & kOperationModePatternTransiting);
}

#endif  // SYSTEM_HPP_
