#pragma once

#include "ublox_msgs/ublox_msgs_utils.hpp"


namespace ublox_msgs {

struct Ack {

  /**
   * @brief Initialise the message from bytes.
   * @param data_stream the start of the byte string from which to decode
   * @param message the message object to write to
   */
  inline static void initialize_from_stream(uint8_t* data_stream, Ack &message);

  uint8_t clsID = 0;
  uint8_t msgID = 0;

  enum {
    CLASS_ID = 5u,
    NACK_MESSAGE_ID = 0u,
    ACK_MESSAGE_ID = 1u,
  };
}; // struct Ack

void Ack::initialize_from_stream(uint8_t* data_stream, Ack &message) {
  initialize_datafield_from_stream(message.clsID, data_stream);
  initialize_datafield_from_stream(message.msgID, data_stream);
}

} // namespace ublox_msgs

