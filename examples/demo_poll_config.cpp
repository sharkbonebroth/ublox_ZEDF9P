/// This is a demo program that shows how to poll a configuration using Valget messages

#include "ublox_ZEDF9P/ublox_ZEDF9P.hpp"
#include "ublox_msgs/ublox_msgs.hpp"

int main(int argc, char** argv) {
  if (argc < 3) {
    spdlog::error("Usage: demo_poll_config {device} {baudrate}");
    return 1;
  }

  // Create the main gps object that interfaces with the ublox ZEDF9P sensor
  ublox_ZEDF9P::ublox_ZEDF9P gps_; 
  gps_.set_debug_level(1);
  gps_.initializeSerial(argv[1], std::stoi(argv[2]));

  ublox_msgs::Valget cfg_request_msg;
  cfg_request_msg.layer = ublox_msgs::Valget::RAM_LAYER_CODE; // Get the configuration from the RAM layer
  if (cfg_request_msg.add_config_request(0x40520001)) {
    spdlog::info("Successfully added config request");
  }
  gps_.poll_Valget(cfg_request_msg);

  uint32_t baudrate;

  // Wait for 1 seconds to show that messages are being received
  std::chrono::milliseconds duration(1000);
  std::this_thread::sleep_for(duration);

  if (cfg_request_msg.get_config_data(baudrate, 0x40520001)) {
    spdlog::info("Successfully polled baudrate! Baudrate: {}", static_cast<int>(baudrate));
  }
}