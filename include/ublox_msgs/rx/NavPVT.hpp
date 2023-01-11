#pragma once

#include "ublox_msgs/ublox_msgs_utils.hpp"


namespace ublox_msgs {

struct NavPVT {

  /**
   * @brief Initialise the message from bytes.
   * @param data_stream the start of the byte string from which to decode
   * @param message the message object to write to
   */
  inline static void initialize_from_stream(uint8_t* data_stream, NavPVT &message);

  uint32_t iTOW = 0;
  uint16_t year = 0;
  uint8_t month = 0;
  uint8_t day = 0;
  uint8_t hour = 0;
  uint8_t min = 0;
  uint8_t sec = 0;
  uint8_t valid = 0;
  uint32_t tAcc = 0;
  int32_t nano = 0;
  uint8_t fixType = 0;
  uint8_t flags = 0;
  uint8_t flags2 = 0;
  uint8_t numSV = 0;
  int32_t lon = 0;
  int32_t lat = 0;
  int32_t height = 0;
  int32_t hMSL = 0;
  uint32_t hAcc = 0;
  uint32_t vAcc = 0;
  int32_t velN = 0;
  int32_t velE = 0;
  int32_t velD = 0;
  int32_t gSpeed = 0;
  int32_t heading = 0;
  uint32_t sAcc = 0;
  uint32_t headAcc = 0;
  uint16_t pDOP = 0;
  std::array<uint8_t, 6> reserved1[6] = {0, 0, 0, 0, 0, 0};
  int32_t headVeh = 0;
  int16_t magDec = 0;
  uint16_t magAcc = 0;

  enum {
    CLASS_ID = 1u,
    MESSAGE_ID = 7u,
    VALID_DATE = 1u,
    VALID_TIME = 2u,
    VALID_FULLY_RESOLVED = 4u,
    VALID_MAG = 8u,
    FIX_TYPE_NO_FIX = 0u,
    FIX_TYPE_DEAD_RECKONING_ONLY = 1u,
    FIX_TYPE_2D = 2u,
    FIX_TYPE_3D = 3u,
    FIX_TYPE_GNSS_DEAD_RECKONING_COMBINED = 4u,
    FIX_TYPE_TIME_ONLY = 5u,
    FLAGS_GNSS_FIX_OK = 1u,
    FLAGS_DIFF_SOLN = 2u,
    FLAGS_PSM_MASK = 28u,
    PSM_OFF = 0u,
    PSM_ENABLED = 4u,
    PSM_ACQUIRED = 8u,
    PSM_TRACKING = 12u,
    PSM_POWER_OPTIMIZED_TRACKING = 16u,
    PSM_INACTIVE = 20u,
    FLAGS_HEAD_VEH_VALID = 32u,
    FLAGS_CARRIER_PHASE_MASK = 192u,
    CARRIER_PHASE_NO_SOLUTION = 0u,
    CARRIER_PHASE_FLOAT = 64u,
    CARRIER_PHASE_FIXED = 128u,
    FLAGS2_CONFIRMED_AVAILABLE = 32u,
    FLAGS2_CONFIRMED_DATE = 64u,
    FLAGS2_CONFIRMED_TIME = 128u,
  };
}; // Struct NavPVT

void NavPVT::initialize_from_stream(uint8_t* data_stream, NavPVT &message) {
  initialize_datafield_from_stream(message.iTOW, data_stream);
  initialize_datafield_from_stream(message.year, data_stream);
  initialize_datafield_from_stream(message.month, data_stream);
  initialize_datafield_from_stream(message.day, data_stream);
  initialize_datafield_from_stream(message.hour, data_stream);
  initialize_datafield_from_stream(message.min, data_stream);
  initialize_datafield_from_stream(message.sec, data_stream);
  initialize_datafield_from_stream(message.valid, data_stream);
  initialize_datafield_from_stream(message.tAcc, data_stream);
  initialize_datafield_from_stream(message.nano, data_stream);
  initialize_datafield_from_stream(message.fixType, data_stream);
  initialize_datafield_from_stream(message.flags, data_stream);
  initialize_datafield_from_stream(message.flags2, data_stream);
  initialize_datafield_from_stream(message.numSV, data_stream);
  initialize_datafield_from_stream(message.lon, data_stream);
  initialize_datafield_from_stream(message.lat, data_stream);
  initialize_datafield_from_stream(message.height, data_stream);
  initialize_datafield_from_stream(message.hMSL, data_stream);
  initialize_datafield_from_stream(message.hAcc, data_stream);
  initialize_datafield_from_stream(message.vAcc, data_stream);
  initialize_datafield_from_stream(message.velN, data_stream);
  initialize_datafield_from_stream(message.velE, data_stream);
  initialize_datafield_from_stream(message.velD, data_stream);
  initialize_datafield_from_stream(message.gSpeed, data_stream);
  initialize_datafield_from_stream(message.heading, data_stream);
  initialize_datafield_from_stream(message.sAcc, data_stream);
  initialize_datafield_from_stream(message.headAcc, data_stream);
  initialize_datafield_from_stream(message.pDOP, data_stream);
  initialize_datafield_from_stream(message.reserved1, data_stream);
  initialize_datafield_from_stream(message.headVeh, data_stream);
  initialize_datafield_from_stream(message.magDec, data_stream);
  initialize_datafield_from_stream(message.magAcc, data_stream);
}

} // Namespace ublox_msgs