#pragma once

#include "ublox_msgs/msgs/Ack.hpp"
#include "ublox_msgs/msgs/NavPVT.hpp"
#include "ublox_msgs/msgs/MONVER.hpp"
#include "ublox_msgs/msgs/Valset.hpp"
#include "ublox_msgs/msgs/Valget.hpp"
#include "ublox_msgs/msgs/CfgRST.hpp"
#include "ublox_msgs/msgs/CfgMSG.hpp"

namespace ublox_msgs {

namespace Class {
  constexpr uint8_t NAV = 0x01; //!< Navigation Result Messages: Position, 
                                   //!< Speed, Time, Acceleration, Heading, 
                                   //!< DOP, SVs used
  constexpr uint8_t RXM = 0x02; //!< Receiver Manager Messages: 
                                   //!< Satellite Status, RTC Status
  constexpr uint8_t INF = 0x04; //!< Information Messages: 
                                   //!< Printf-Style Messages, with IDs such as
                                   //!< Error, Warning, Notice
  constexpr uint8_t ACK = 0x05; //!< Ack/Nack Messages: Acknowledge or Reject 
                                   //!< messages to CFG input messages
  constexpr uint8_t CFG = 0x06; //!< Configuration Input Messages: Set 
                                   //!< Dynamic Model, Set DOP Mask, Set Baud 
                                   //!< Rate, etc.
  constexpr uint8_t UPD = 0x09; //!< Firmware Update Messages: i.e. 
                                   //!< Memory/Flash erase/write, Reboot, Flash 
                                   //!< identification, etc.
                                   //!< Used to update the firmware and identify 
                                   //!< any attached flash device
  constexpr uint8_t MON = 0x0A; //!< Monitoring Messages: Communication 
                                   //!< Status, CPU Load, Stack Usage, 
                                   //!< Task Status
  constexpr uint8_t AID = 0x0B; //!< AssistNow Aiding Messages: Ephemeris, 
                                   //!< Almanac, other A-GPS data input
  constexpr uint8_t TIM = 0x0D; //!< Timing Messages: Timepulse Output, 
                                   //!< Timemark Results
  constexpr uint8_t ESF = 0x10; //!< External Sensor Fusion Messages: 
                                   //!< External sensor measurements and status 
                                   //!< information
  constexpr uint8_t MGA = 0x13; //!< Multiple GNSS Assistance Messages: 
                                   //!< Assistance data for various GNSS
  constexpr uint8_t LOG = 0x21; //!< Logging Messages: Log creation, 
                                   //!< deletion, info and retrieval
  constexpr uint8_t SEC = 0x27; //!< Security Feature Messages
  constexpr uint8_t HNR = 0x28; //!< High Rate Navigation Results Messages: 
                                   //!< High rate time, position, speed, heading
  constexpr uint8_t RTCM = 0xF5; //!< RTCM Configuration Messages
}

namespace Message {
  namespace NAV {
    constexpr uint8_t PVT = NavPVT::MESSAGE_ID;
  }

  namespace INF {
    constexpr uint8_t ERROR = 0x00;
    constexpr uint8_t WARNING = 0x01;
    constexpr uint8_t NOTICE = 0x02;
    constexpr uint8_t TEST = 0x03;
    constexpr uint8_t DEBUG = 0x04;
  }

  namespace ACK {
    constexpr uint8_t NACK = 0x00; 
    constexpr uint8_t ACK = 0x01; 
  }
  
}

}