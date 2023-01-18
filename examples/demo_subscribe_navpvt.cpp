/// This is a demo program that shows how to subscribe to navpvt messages and call a callback function upon receiving them
/// It uses config messages and certain library functions

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
    std::cout << "Usage: demo_config_baudrate {device} {baudrate}" << std::endl;
    return 1;
  }

  // Create the main gps object that interfaces with the ublox ZEDF9P sensor
  ublox_ZEDF9P::ublox_ZEDF9P gps_; 
  gps_.set_debug_level(1);
  gps_.initializeSerial(argv[1], std::stoi(argv[2]));
  // Subcribe to NavPVT messages
  gps_.subscribe<ublox_msgs::NavPVT>(NavPVT_callback);

  // Wait for 1 seconds to show that messages are being received
  std::chrono::milliseconds duration(1000);
  std::this_thread::sleep_for(duration);
}