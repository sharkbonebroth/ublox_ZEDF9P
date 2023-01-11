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
  std::array<uint8_t, 10> hwVersion;
  std::vector<std::array<uint8_t, 30>> extension;

  enum {
    CLASS_ID = 10u,
    MESSAGE_ID = 4u,
  };

  MONVER() {
    swVersion.fill(0);
    hwVersion.fill(0);
  }

}; // struct MONVER

void MONVER::initialize_from_stream(uint8_t* data_stream, MONVER &message) {
  int num_bytes_to_read = received_message_length(data_stream) - 40;

  initialize_datafield_from_stream(message.swVersion, data_stream);
  initialize_datafield_from_stream(message.hwVersion, data_stream);

  while (num_bytes_to_read >= 30) {
    std::array<uint8_t, 30> extended_data;
    initialize_datafield_from_stream(extended_data, data_stream);
    message.extension.push_back(extended_data);

    num_bytes_to_read -= 30;
  }
}

} // namespace ublox_msgs