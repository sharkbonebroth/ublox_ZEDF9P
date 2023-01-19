/// This is a demo program that shows how to poll a configuration using Valget messages

#include "ublox_ZEDF9P/ublox_ZEDF9P.hpp"
#include "ublox_ZEDF9P/logging.hpp"
#include "ublox_msgs/ublox_msgs.hpp"

int main(int argc, char** argv) {
  if (argc < 3) {
    std::cout << "Usage: demo_poll_config {device} {baudrate}" << std::endl;
    return 1;
  }

  // Create the main gps object that interfaces with the ublox ZEDF9P sensor
  ublox_ZEDF9P::ublox_ZEDF9P gps_; 
  gps_.set_debug_level(3);
  gps_.initializeSerial(argv[1], std::stoi(argv[2]));

  ublox_msgs::Valget cfg_request_msg;
  cfg_request_msg.layer = ublox_msgs::Valget::RAM_LAYER_CODE; // Get the configuration from the RAM layer
  if (cfg_request_msg.add_config_request(0x20520002)) {
    std::cout << SUCCESS << "Successfully added config request" << RESET_FORMATTING << std::endl;
  }
  gps_.poll_Valget(cfg_request_msg);

  uint8_t baudrate;

  // Wait for 1 seconds to show that messages are being received
  std::chrono::milliseconds duration(1000);
  std::this_thread::sleep_for(duration);

  if (cfg_request_msg.get_config_data(baudrate, 0x20520002)) {
    std::cout << SUCCESS << "Successfully polled baudrate! Baudrate: " << baudrate << RESET_FORMATTING << std::endl;
  }
}