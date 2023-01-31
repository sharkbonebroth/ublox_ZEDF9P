/// This is a demo program that shows how to configute the ublox ZEDF9P GNSS sensor using config messages

#include "ublox_ZEDF9P/ublox_ZEDF9P.hpp"
#include "ublox_msgs/ublox_msgs.hpp"

void NavPVT_callback(const ublox_msgs::NavPVT& pvt_msg) {
  bool fixOk = pvt_msg.flags & pvt_msg.FLAGS_GNSS_FIX_OK;
  spdlog::info("received nav pvt msg!");
  spdlog::info("lat: {}", pvt_msg.lat * 1e-7);
  spdlog::info("lon: {}", pvt_msg.lon * 1e-7);
  spdlog::info("alt: {}", pvt_msg.height * 1e-7);
  if (fixOk) {
    spdlog::info("VALID FIX");
  } else {
    spdlog::info("INVALID FIX");
  }
}

int main(int argc, char** argv) {
  if (argc < 3) {
    spdlog::error("Usage: demo_config {device} {baudrate}");
    return 1;
  }

  spdlog::info("Please enter your new desired baudrate");
  unsigned int baudrate;
  std::cin >> baudrate;

  // Check the baudrate to make sure that we are not setting the baudrate to some non-standard value
  std::array<int, 8> valid_baudrates = {4800, 9600, 19200, 38400, 57600, 115200, 230400, 460800};
  bool baudrate_is_valid = false;
  for (int valid_baudrate : valid_baudrates) {
    if (baudrate == valid_baudrate) {baudrate_is_valid = true; break;}
  }
  if (!baudrate_is_valid) {throw std::runtime_error("ublox: invalid baudrate! Valud baudrates are: 4800, 9600, 19200, 38400, 57600, 115200, 230400, 460800") ;}

  // Create the main gps object that interfaces with the ublox ZEDF9P sensor
  ublox_ZEDF9P::ublox_ZEDF9P gps_; 
  gps_.set_debug_level(1);
  gps_.initializeSerial(argv[1], std::stoi(argv[2]));
  // Subcribe to NavPVT messages
  gps_.subscribe<ublox_msgs::NavPVT>(NavPVT_callback);

  // Wait for 1 seconds to show that messages are being received
  std::chrono::milliseconds duration(1000);
  std::this_thread::sleep_for(duration);

  ublox_msgs::Valset cfg_msg;
  cfg_msg.layers = 3;
  uint32_t new_baudrate = baudrate;
  if (cfg_msg.add_config(0x40520001, new_baudrate)) {
    spdlog::info("Successfully added configuration for baudrate on uart1. Please remember to set the correct baudrate the next time you start the gps up");
  }
  gps_.send_config_msg(cfg_msg, true);
  // We need to change the baudrate of the gps object to read from the new baudrate
  gps_.change_baudrate(new_baudrate);

  // Wait for 1 seconds to show that messages are still being received
  std::this_thread::sleep_for(duration);
}