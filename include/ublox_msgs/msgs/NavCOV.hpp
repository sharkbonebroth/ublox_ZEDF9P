#pragma once

#include "ublox_msgs/ublox_msgs_utils.hpp"

namespace ublox_msgs {

struct NavCOV {

  /**
   * @brief Initialise the NavCOV message from bytes. The covariance is given in NED frame
   * @param data_stream the start of the byte string from which to decode
   * @param message the NavCOV message object to write to
   */
  inline static void initialize_from_stream(uint8_t* data_stream, NavCOV &message);

  uint32_t iTOW = 0;
  uint8_t version = 0;
  uint8_t posCovValid = 0;
  uint8_t velCovValid = 0;
  std::array<uint8_t, 9> reserved0 = {0, 0, 0, 0, 0, 0, 0, 0, 0};
  float posCovNN = 0;
  float posCovNE = 0;
  float posCovND = 0;
  float posCovEE = 0;
  float posCovED = 0;
  float posCovDD = 0;
  float velCovNN = 0;
  float velCovNE = 0;
  float velCovND = 0;
  float velCovEE = 0;
  float velCovED = 0;
  float velCovDD = 0;

  enum {
    CLASS_ID = 1u,
    MESSAGE_ID = 54u,
  };

}; // struct NavCOV

void NavCOV::initialize_from_stream(uint8_t* data_stream, NavCOV &message) {
  initialize_datafield_from_stream(message.iTOW, data_stream);
  initialize_datafield_from_stream(message.version, data_stream);
  initialize_datafield_from_stream(message.posCovValid, data_stream);
  initialize_datafield_from_stream(message.velCovValid, data_stream);
  initialize_datafield_from_stream(message.reserved0, data_stream);
  initialize_datafield_from_stream(message.posCovNN, data_stream);
  initialize_datafield_from_stream(message.posCovNE, data_stream);
  initialize_datafield_from_stream(message.posCovND, data_stream);
  initialize_datafield_from_stream(message.posCovEE, data_stream);
  initialize_datafield_from_stream(message.posCovED, data_stream);
  initialize_datafield_from_stream(message.posCovDD, data_stream);
  initialize_datafield_from_stream(message.velCovNN, data_stream);
  initialize_datafield_from_stream(message.velCovNE, data_stream);
  initialize_datafield_from_stream(message.velCovND, data_stream);
  initialize_datafield_from_stream(message.velCovEE, data_stream);
  initialize_datafield_from_stream(message.velCovED, data_stream);
  initialize_datafield_from_stream(message.velCovDD, data_stream);
}

} // namespace ublox_msgs