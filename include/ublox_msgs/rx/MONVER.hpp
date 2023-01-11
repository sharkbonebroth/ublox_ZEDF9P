#pragma once

#include "ublox_msgs/ublox_msgs_utils.hpp"


namespace ublox_msgs {

/**
 * @brief A ublox message for polling the firmware and hardware version of the ublox ZEDF9P
 */
struct MONVER {

  /**
   * @brief Initialise the message from bytes.
   * @param data_stream the start of the byte string from which to decode
   * @param message the MONVER object to write to
   */
  inline static void initialize_from_stream(uint8_t* data_stream, MONVER &message);

  std::array<uint8_t, 30> swVersion;

}; // struct MONVER

} // namespace ublox_msgs