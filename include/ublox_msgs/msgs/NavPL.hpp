#pragma once

#include "ublox_msgs/ublox_msgs_utils.hpp"

namespace ublox_msgs {

struct NavPL {

  /**
   * @brief Initialise the NavPL message from bytes.
   * @param data_stream the start of the byte string from which to decode
   * @param message the NavPL message object to write to
   */
  inline static void initialize_from_stream(const uint8_t* data_stream, NavPL &message);

  uint8_t msgVersion = 0;
  uint8_t tmirCoeff = 0;
  int8_t tmirExp = 0;
  uint8_t plPosValid = 0;
  uint8_t plPosFrame = 0;
  uint8_t plVelValid = 0;
  uint8_t plVelFrame = 0;
  uint8_t plTimeValid = 0;
  std::array<uint8_t, 4> reserved0 = {0, 0, 0, 0};
  uint32_t iTow = 0;
  uint32_t plPos1 = 0;
  uint32_t plPos2 = 0;
  uint32_t plPos3 = 0;
  uint32_t plVel1 = 0;
  uint32_t plVel2 = 0;
  uint32_t plVel3 = 0;
  uint16_t plPosHorizOrient = 0;
  uint16_t plVelHorizOrient = 0;
  uint32_t plTime = 0;
  std::array<uint8_t, 4> reserved1 = {0, 0, 0, 0};

  enum {
    CLASS_ID = 1u,
    MESSAGE_ID = 98u,
    PLPOSFRAME_INVALID = 0u,
    PLPOSFRAME_NED = 1u,
    PLPOSFRAME_LAT_LON_VER = 2u,
    PLPOSFRAME_MAJ_MIN_VER = 3u
  };

}; // struct NavPL

void NavPL::initialize_from_stream(const uint8_t* data_stream, NavPL &message) {
  initialize_datafield_from_stream(message.msgVersion, data_stream);
  initialize_datafield_from_stream(message.tmirCoeff, data_stream);
  initialize_datafield_from_stream(message.tmirExp, data_stream);
  initialize_datafield_from_stream(message.plPosValid, data_stream);
  initialize_datafield_from_stream(message.plPosFrame, data_stream);
  initialize_datafield_from_stream(message.plVelValid, data_stream);
  initialize_datafield_from_stream(message.plVelFrame, data_stream);
  initialize_datafield_from_stream(message.plTimeValid, data_stream);
  initialize_datafield_from_stream(message.reserved0, data_stream);
  initialize_datafield_from_stream(message.iTow, data_stream);
  initialize_datafield_from_stream(message.plPos1, data_stream);
  initialize_datafield_from_stream(message.plPos2, data_stream);
  initialize_datafield_from_stream(message.plPos3, data_stream);
  initialize_datafield_from_stream(message.plVel1, data_stream);
  initialize_datafield_from_stream(message.plVel2, data_stream);
  initialize_datafield_from_stream(message.plVel3, data_stream);
  initialize_datafield_from_stream(message.plPosHorizOrient, data_stream);
  initialize_datafield_from_stream(message.plVelHorizOrient, data_stream);
  initialize_datafield_from_stream(message.plTime, data_stream);
  initialize_datafield_from_stream(message.reserved1, data_stream);
}

}// namespace ublox_msgs