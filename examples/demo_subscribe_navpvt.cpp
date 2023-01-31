/// This is a demo program that shows how to subscribe to navpvt messages and call a callback function upon receiving them

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
    spdlog::error("Usage: demo_subscribe_navpvt {device} {baudrate}");
    return 1;
  }

  // Create the main gps object that interfaces with the ublox ZEDF9P sensor
  ublox_ZEDF9P::ublox_ZEDF9P gps_; 
  gps_.set_debug_level(0);
  gps_.initializeSerial(argv[1], std::stoi(argv[2]));
  // Subcribe to NavPVT messages
  gps_.subscribe<ublox_msgs::NavPVT>(NavPVT_callback);

  // Wait for 1 seconds to show that messages are being received
  std::chrono::milliseconds duration(50000);
  std::this_thread::sleep_for(duration);
}