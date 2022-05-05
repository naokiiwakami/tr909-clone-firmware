/*
 * tr909_clone system configuration
 */
#ifndef SYSTEM_HPP_
#define SYSTEM_HPP_

// System clock frequency
static constexpr uint32_t F_OSC = 16000000;  // 16 MHz

static constexpr uint8_t kOperationModeNormal = 0x1;
static constexpr uint8_t kOperationModeDirectPlay = 0x2;
extern uint8_t g_operation_mode;

#endif  // SYSTEM_HPP_
