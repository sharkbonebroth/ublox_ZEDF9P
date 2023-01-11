#pragma once

#include "ublox_msgs/ublox_msgs_utils.hpp"

namespace ublox_msgs {

/**
 * @brief A ublox message for setting the rate of publishing of a ublox message
 * note that this is suppodedly deprecated, but seems like we still need to send this to get the
 * gps to publish
 */
struct CfgMSG {

  /**
   * @brief Writes the message contents into bytes.
   * @param data_stream where to write to
   * @param message the CfgMSG object to write from
   */
  inline static void write_to_data_stream(uint8_t* data_stream, const CfgMSG &message);

  /**
   * @brief The length of this message (when sent by this driver) in bytes
   * @param message the CfgMSG object from which to obtain the serialized length
   */
  inline static uint32_t serialized_length(const CfgMSG &message);

  uint8_t msgClass = 0;
  uint8_t msgID = 0;
  uint8_t rate = 0;

  enum {
    CLASS_ID = 6u,
    MESSAGE_ID = 1u,
  };

}; // struct CfgMSG

void CfgMSG::write_to_data_stream(uint8_t* data_stream, const CfgMSG &message) {
  write_data_to_stream(message.msgClass, data_stream);
  write_data_to_stream(message.msgID, data_stream);
  write_data_to_stream(message.rate, data_stream);
}

inline uint32_t CfgMSG::serialized_length(const CfgMSG &message) {
  return 3;
}

} // namespace ublox_msgs