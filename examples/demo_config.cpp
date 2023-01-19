/// This is a demo program that shows how to configute the ublox ZEDF9P GNSS sensor using config messages

#include "ublox_ZEDF9P/ublox_ZEDF9P.hpp"
#include "ublox_ZEDF9P/logging.hpp"
#include "ublox_msgs/ublox_msgs.hpp"

void NavPVT_callback(const ublox_msgs::NavPVT& pvt_msg) {
  bool fixOk = pvt_msg.flags & pvt_msg.FLAGS_GNSS_FIX_OK;
  std::cout << SUCCESS << "received nav pvt msg! \n" << RESET_FORMATTING
            << "lat: " << pvt_msg.lat * 1e-7 << "\n"
            << "lon: " << pvt_msg.lon * 1e-7 << "\n"
            << "alt: " << pvt_msg.height * 1e-7 << std::endl;
  if (fixOk) {
    std::cout << SUCCESS << "valid fix!" << RESET_FORMATTING << std::endl;
  } else {
    std::cout << FAILURE << "invalid fix!" << RESET_FORMATTING << std::endl;
  }
}

int main(int argc, char** argv) {
  if (argc < 3) {
    std::cout << "Usage: demo_config {device} {baudrate}" << std::endl;
    return 1;
  }

  std::cout << "Please enter your new desired baudrate" << std::endl;
  unsigned int baudrate;
  std::cin >> baudrate;

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
    std::cout << SUCCESS << "Successfully added configuration for baudrate on uart1" << RESET_FORMATTING << std::endl;
  }
  gps_.send_config_msg(cfg_msg, true);
  // We need to change the baudrate of the gps object to read from the new baudrate
  gps_.change_baudrate(new_baudrate);

  // Wait for 1 seconds to show that messages are still being received
  std::this_thread::sleep_for(duration);
}