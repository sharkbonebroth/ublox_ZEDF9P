#pragma once

#include "ublox_msgs/ublox_msgs_utils.hpp"

namespace ublox_msgs {

/**
 * @brief A ublox message for resetting the receiver
 * Do not expect this message to be acknowledged by the receiver.
 */
struct CfgRST {

  /**
   * @brief Writes the message contents into bytes.
   * @param data_stream where to write to
   * @param message the CfgRST object to write from
   */
  inline static void write_to_data_stream(uint8_t* data_stream, const CfgRST &message);

  /**
   * @brief The length of this message (when sent by this driver) in bytes
   * @param message the CfgRST object from which to obtain the serialized length
   */
  inline static uint32_t serialized_length(const CfgRST &message);

  uint16_t navBbrMask = 0;
  uint8_t resetMode = 0;
  uint8_t reserved0 = 0;

  enum {
    CLASS_ID = 6u,
    MESSAGE_ID = 4u,
    NAV_BBR_HOT_START = 0u,
    NAV_BBR_WARM_START = 1u,
    NAV_BBR_COLD_START = 65535u,
    NAV_BBR_EPH = 1u,
    NAV_BBR_ALM = 2u,
    NAV_BBR_HEALTH = 4u,
    NAV_BBR_KLOB = 8u,
    NAV_BBR_POS = 16u,
    NAV_BBR_CLKD = 32u,
    NAV_BBR_OSC = 64u,
    NAV_BBR_UTC = 128u,
    NAV_BBR_RTC = 256u,
    NAV_BBR_AOP = 32768u,
    RESET_MODE_HW_IMMEDIATE = 0u,
    RESET_MODE_SW = 1u,
    RESET_MODE_GNSS = 2u,
    RESET_MODE_HW_AFTER_SHUTDOWN = 4u,
    RESET_MODE_GNSS_STOP = 8u,
    RESET_MODE_GNSS_START = 9u,
  };

}; // struct CfgRST

void CfgRST::write_to_data_stream(uint8_t* data_stream, const CfgRST &message) {
  write_data_to_stream(message.navBbrMask, data_stream);
  write_data_to_stream(message.resetMode, data_stream);
  write_data_to_stream(message.reserved0, data_stream);
}

inline uint32_t CfgRST::serialized_length(const CfgRST &message) {
  return 4;
}

} // namespace ublox_msgs