/// This is a demo program that shows polling of a Ublox message

#include "ublox_ZEDF9P/ublox_ZEDF9P.hpp"
#include "ublox_ZEDF9P/logging.hpp"
#include "ublox_msgs/ublox_msgs.hpp"

int main(int argc, char** argv) {
  if (argc < 3) {
    std::cout << "Usage: demo_poll_MONVER {device} {baudrate}" << std::endl;
    return 1;
  }

  // Create the main gps object that interfaces with the ublox ZEDF9P sensor
  ublox_ZEDF9P::ublox_ZEDF9P gps_; 
  gps_.set_debug_level(2);
  gps_.initializeSerial(argv[1], std::stoi(argv[2]));

  // Note that the function below is essentially how the poll_MONVER function implemented in the ublox_ZEDF9P.cpp source file
  // We are just showing it here for demonstration purposes
  ublox_msgs::MONVER MONVER_msg;
  if (gps_.poll(MONVER_msg)) {
    std::cout << SUCCESS << "successfully polled MONVER msg" << RESET_FORMATTING << std::endl;
  }
  
  char swVersion_char[30];
  char hwVersion_char[10];
  std::memcpy(swVersion_char, MONVER_msg.swVersion.data(), 30);
  std::cout << YELLOW << "ublox_ZEDF9P software version: " << swVersion_char << RESET_FORMATTING << std::endl;

  std::memcpy(hwVersion_char, MONVER_msg.hwVersion.data(), 10);
  std::cout << YELLOW << "ublox_ZEDF9P hardware version: " << hwVersion_char << RESET_FORMATTING << std::endl;

  for (std::array<uint8_t, 30> extended_data: MONVER_msg.extension) {
    char extended_data_char[30];
    std::memcpy(extended_data_char, extended_data.data(), 30);
    std::cout << YELLOW << extended_data_char << RESET_FORMATTING << std::endl;
  }
}