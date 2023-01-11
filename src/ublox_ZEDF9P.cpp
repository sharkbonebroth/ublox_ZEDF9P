//==============================================================================
// Copyright (c) 2012, Johannes Meyer, TU Darmstadt
// All rights reserved.

// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//     * Redistributions of source code must retain the above copyright
//       notice, this list of conditions and the following disclaimer.
//     * Redistributions in binary form must reproduce the above copyright
//       notice, this list of conditions and the following disclaimer in the
//       documentation and/or other materials provided with the distribution.
//     * Neither the name of the Flight Systems and Automatic Control group,
//       TU Darmstadt, nor the names of its contributors may be used to
//       endorse or promote products derived from this software without
//       specific prior written permission.

// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND
// ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
// WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
// DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER BE LIABLE FOR ANY
// DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
// (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
// LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
// ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
// (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
// SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
//==============================================================================

#include "ublox_ZEDF9P/ublox_ZEDF9P.hpp"
#include <boost/version.hpp>

#define RESET_FORMATTING   "\033[0m"
#define SUCCESS            "\033[32m"
#define FAILURE            "\033[31m"

namespace ublox_ZEDF9P {

int debug = 3; //!< Used to determine which debug messages to display

using namespace ublox_msgs;

//! Sleep time [ms] after setting the baudrate
constexpr static int kSetBaudrateSleepMs = 500;

const boost::posix_time::time_duration ublox_ZEDF9P::default_timeout_ =
    boost::posix_time::milliseconds(
        static_cast<int>(ublox_ZEDF9P::kDefaultAckTimeout * 1000));

ublox_ZEDF9P::ublox_ZEDF9P() : configured_(false) {
  subscribeAcks();
}

ublox_ZEDF9P::~ublox_ZEDF9P() { close(); }

void ublox_ZEDF9P::setWorker(const boost::shared_ptr<Worker>& worker) {
  if (worker_) return;
  worker_ = worker;
  worker_->setCallback(boost::bind(&CallbackHandlers::readCallback,
                                   &callbacks_, _1, _2));
  configured_ = static_cast<bool>(worker);
}

void ublox_ZEDF9P::subscribeAcks() {
  // Set NACK handler
  subscribeId<ublox_msgs::Ack>(boost::bind(&ublox_ZEDF9P::processNack, this, _1),
                               ublox_msgs::Message::ACK::NACK);
  // Set ACK handler
  subscribeId<ublox_msgs::Ack>(boost::bind(&ublox_ZEDF9P::processAck, this, _1),
                               ublox_msgs::Message::ACK::ACK);
}

void ublox_ZEDF9P::processAck(const ublox_msgs::Ack &m) {
  // Process ACK/NACK messages
  Ack ack;
  ack.type = ACK;
  ack.class_id = m.clsID;
  ack.msg_id = m.msgID;
  // store the ack atomically
  ack_.store(ack, boost::memory_order_seq_cst);
  if (debug >= 2) {
    std::cout << SUCCESS
              << "U-blox: received ACK: " 
              << std::hex << static_cast<int>(m.clsID ) 
              << " / " 
              << std::hex << static_cast<int>(m.msgID) 
              << RESET_FORMATTING
              << std::endl;
  }
}

void ublox_ZEDF9P::processNack(const ublox_msgs::Ack &m) {
  // Process ACK/NACK messages
  Ack ack;
  ack.type = NACK;
  ack.class_id = m.clsID;
  ack.msg_id = m.msgID;
  // store the ack atomically
  ack_.store(ack, boost::memory_order_seq_cst);
  std::cout << FAILURE 
            << "U-blox: received NACK: " 
            << std::hex << static_cast<int>(m.clsID) 
            << " / " 
            << std::hex << static_cast<int>(m.msgID) 
            << RESET_FORMATTING
            << std::endl;
}

bool ublox_ZEDF9P::test_serial() {
  Valget test_msg;
  test_msg.add_config_request(0x40520001);
  return send_config_msg(test_msg, true);
}

void ublox_ZEDF9P::initializeSerial(std::string port, unsigned int baudrate) {
  port_ = port;
  boost::shared_ptr<boost::asio::io_service> io_service(
      new boost::asio::io_service);
  boost::shared_ptr<boost::asio::serial_port> serial(
      new boost::asio::serial_port(*io_service));

  // open serial port
  try {
    serial->open(port);
  } catch (std::runtime_error& e) {
    throw std::runtime_error("U-Blox: initializeSerial: Could not open serial port :"
                             + port + " " + e.what());
  }

  std::cout << ("U-Blox: Opened serial port %s", port.c_str()) << std::endl;
    
  if(BOOST_VERSION < 106600)
  {
    // NOTE(Kartik): Set serial port to "raw" mode. This is done in Boost but
    // until v1.66.0 there was a bug which didn't enable the relevant code,
    // fixed by commit: https://github.com/boostorg/asio/commit/619cea4356
    int fd = serial->native_handle();
    termios tio;
    tcgetattr(fd, &tio);
    cfmakeraw(&tio);
    tcsetattr(fd, TCSANOW, &tio);
  }

  // Set the I/O worker
  if (worker_) return;
  setWorker(boost::shared_ptr<Worker>(
      new AsyncWorker<boost::asio::serial_port>(serial, io_service)));

  configured_ = false;

  // Set the baudrate
  boost::asio::serial_port_base::baud_rate current_baudrate;
  serial->get_option(current_baudrate);
  // Incrementally increase the baudrate to the desired value
  for (int i = 0; i < sizeof(kBaudrates)/sizeof(kBaudrates[0]); i++) {
    if (current_baudrate.value() == baudrate)
      break;
    // Don't step down, unless the desired baudrate is lower
    if(current_baudrate.value() > kBaudrates[i] && baudrate > kBaudrates[i])
      continue;
    serial->set_option(
        boost::asio::serial_port_base::baud_rate(kBaudrates[i]));
    boost::this_thread::sleep(
        boost::posix_time::milliseconds(kSetBaudrateSleepMs));
    serial->get_option(current_baudrate);
    std::cout << "U-Blox: Set ASIO baudrate to " << current_baudrate.value() << std::endl;
  }

  worker_->baudrate_ = current_baudrate.value();

  if (!test_serial()) {
    throw std::runtime_error("U-Blox: Could not read/ write from serial port :" + port);
  }

  configured_ = true;
}

void ublox_ZEDF9P::resetSerial(std::string port, unsigned int baudrate) {
  boost::shared_ptr<boost::asio::io_service> io_service(
      new boost::asio::io_service);
  boost::shared_ptr<boost::asio::serial_port> serial(
      new boost::asio::serial_port(*io_service));

  // open serial port
  try {
    serial->open(port);
  } catch (std::runtime_error& e) {
    throw std::runtime_error("U-Blox: reset:serial: Could not open serial port :"
                             + port + " " + e.what());
  }

  std::cout << ("U-Blox: Reset serial port %s", port.c_str()) << std::endl;

  // Set the I/O worker
  if (worker_) return;
  setWorker(boost::shared_ptr<Worker>(
      new AsyncWorker<boost::asio::serial_port>(serial, io_service)));
  configured_ = false;

  // Set the baudrate
  boost::asio::serial_port_base::baud_rate current_baudrate;
  serial->get_option(current_baudrate);
  // Incrementally increase the baudrate to the desired value
  for (int i = 0; i < sizeof(kBaudrates)/sizeof(kBaudrates[0]); i++) {
    if (current_baudrate.value() == baudrate)
      break;
    // Don't step down, unless the desired baudrate is lower
    if(current_baudrate.value() > kBaudrates[i] && baudrate > kBaudrates[i])
      continue;
    serial->set_option(
        boost::asio::serial_port_base::baud_rate(kBaudrates[i]));
    boost::this_thread::sleep(
        boost::posix_time::milliseconds(kSetBaudrateSleepMs));
    serial->get_option(current_baudrate);
    std:: cout << ("U-Blox: Reset serial port: Set ASIO baudrate to %u", current_baudrate.value()) << std::endl;
  }

  if (test_serial()) {
    throw std::runtime_error("U-Blox: Reset serial port: Could not read/ write from serial port :" + port);
  }

  configured_ = true;
}

void ublox_ZEDF9P::initializeTcp(std::string host, std::string port) {
  host_ = host;
  port_ = port;
  boost::shared_ptr<boost::asio::io_service> io_service(
      new boost::asio::io_service);
  boost::asio::ip::tcp::resolver::iterator endpoint;

  try {
    boost::asio::ip::tcp::resolver resolver(*io_service);
    endpoint =
        resolver.resolve(boost::asio::ip::tcp::resolver::query(host, port));
  } catch (std::runtime_error& e) {
    throw std::runtime_error("U-Blox: Could not resolve" + host + " " +
                             port + " " + e.what());
  }

  boost::shared_ptr<boost::asio::ip::tcp::socket> socket(
    new boost::asio::ip::tcp::socket(*io_service));

  try {
    socket->connect(*endpoint);
  } catch (std::runtime_error& e) {
    throw std::runtime_error("U-Blox: Could not connect to " +
                             endpoint->host_name() + ":" +
                             endpoint->service_name() + ": " + e.what());
  }

  std::cout << ("U-Blox: Connected to %s:%s.", endpoint->host_name().c_str(),
           endpoint->service_name().c_str()) << std::endl;

  if (worker_) return;
  setWorker(boost::shared_ptr<Worker>(
      new AsyncWorker<boost::asio::ip::tcp::socket>(socket,
                                                    io_service)));
}

void ublox_ZEDF9P::initializeUdp(std::string host, std::string port) {
  host_ = host;
  port_ = port;
  boost::shared_ptr<boost::asio::io_service> io_service(
      new boost::asio::io_service);
  boost::asio::ip::udp::resolver::iterator endpoint;

  try {
    boost::asio::ip::udp::resolver resolver(*io_service);
    endpoint =
        resolver.resolve(boost::asio::ip::udp::resolver::query(host, port));
  } catch (std::runtime_error& e) {
    throw std::runtime_error("U-Blox: Could not resolve" + host + " " +
                             port + " " + e.what());
  }

  boost::shared_ptr<boost::asio::ip::udp::socket> socket(
    new boost::asio::ip::udp::socket(*io_service));

  try {
    socket->connect(*endpoint);
  } catch (std::runtime_error& e) {
    throw std::runtime_error("U-Blox: Could not connect to " +
                             endpoint->host_name() + ":" +
                             endpoint->service_name() + ": " + e.what());
  }

  std::cout << ("U-Blox: Connected to %s:%s.", endpoint->host_name().c_str(),
           endpoint->service_name().c_str()) << std::endl;

  if (worker_) return;
  setWorker(boost::shared_ptr<Worker>(
      new AsyncWorker<boost::asio::ip::udp::socket>(socket,
                                                    io_service)));
}

bool ublox_ZEDF9P::setRate(uint8_t class_id, uint8_t message_id, uint8_t rate) {
  if (debug >= 2) {
    std::cout << "Setting rate to "
              << std::dec << static_cast<int>(rate)
              << " for "
              << std::hex << static_cast<int>(message_id)
              << " / "
              << std::hex << static_cast<int>(class_id)
              << std::endl;
  }
  ublox_msgs::CfgMSG cfg_msg;
  cfg_msg.msgClass = class_id;
  cfg_msg.msgID = message_id;
  cfg_msg.rate = rate;
  return send_config_msg(cfg_msg, true);
}

void ublox_ZEDF9P::close() {
  worker_.reset();
  configured_ = false;
}

void ublox_ZEDF9P::reset(const boost::posix_time::time_duration& wait) {
  unsigned int baudrate = worker_->baudrate_;
  worker_.reset();
  configured_ = false;
  // sleep because of undefined behavior after I/O reset
  boost::this_thread::sleep(wait);
  if (host_ == "")
    resetSerial(port_, baudrate);
  else
    initializeTcp(host_, port_);
}

bool ublox_ZEDF9P::configReset(uint16_t nav_bbr_mask, uint16_t reset_mode) {
  std::cout << ("Resetting u-blox. If device address changes, %s",
           "node must be relaunched.") << std::endl;

  CfgRST rst;
  rst.navBbrMask = nav_bbr_mask;
  rst.resetMode = reset_mode;

  // Don't wait for ACK, return if it fails
  if (!send_config_msg(rst, false))
    return false;
  return true;
}

bool ublox_ZEDF9P::poll(uint8_t class_id, uint8_t message_id,
               const std::vector<uint8_t>& payload) {
  if (!worker_) return false;

  std::vector<unsigned char> out(kWriterSize);
  ublox::Writer writer(out.data(), out.size());
  if (!writer.write(payload.data(), payload.size(), class_id, message_id))
    return false;
  worker_->send(out.data(), writer.end() - out.data());

  return true;
}

bool ublox_ZEDF9P::waitForAcknowledge(const boost::posix_time::time_duration& timeout,
                             uint8_t class_id, uint8_t msg_id) {
  if (debug >= 2) {
    std::cout << "Waiting for ACK " 
              << std::hex << static_cast<int>(class_id)
              << " / "
              << std::hex << static_cast<int>(msg_id)
              << std::endl;
  }
  boost::posix_time::ptime wait_until(
      boost::posix_time::second_clock::local_time() + timeout);

  Ack ack = ack_.load(boost::memory_order_seq_cst);
  while (boost::posix_time::second_clock::local_time() < wait_until
         && (ack.class_id != class_id
             || ack.msg_id != msg_id
             || ack.type == WAIT)) {
    worker_->wait(timeout);
    ack = ack_.load(boost::memory_order_seq_cst);
  }
  bool result = ack.type == ACK
                && ack.class_id == class_id
                && ack.msg_id == msg_id;
  return result;
}

void ublox_ZEDF9P::setRawDataCallback(const Worker::Callback& callback) {
  if (! worker_) return;
  worker_->setRawDataCallback(callback);
}

}  // namespace ublox_ZEDF9P
