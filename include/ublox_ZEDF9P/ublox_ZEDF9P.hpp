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

#pragma once

// STL
#include <map>
#include <vector>
#include <locale>
#include <stdexcept>

// Boost
#include <boost/asio/ip/tcp.hpp>
#include <boost/asio/ip/udp.hpp>
#include <boost/asio/serial_port.hpp>
#include <boost/asio/io_service.hpp>
#include <boost/atomic.hpp>

// u-blox ZEDF9P
#include "ublox_ZEDF9P/async_worker.hpp"
#include "ublox_ZEDF9P/callback.hpp"

// u-blox msgs
#include "ublox_msgs/ublox_msgs.hpp"

/**
 * @namespace ublox_ZEDF9P
 * This namespace is for I/O communication with the u-blox ZEDF9P device, including
 * read callbacks.
 */
namespace ublox_ZEDF9P {
//! Possible baudrates for u-blox devices
constexpr static unsigned int kBaudrates[] = { 4800,
                                               9600,
                                               19200,
                                               38400,
                                               57600,
                                               115200,
                                               230400,
                                               460800 };
/**
 * @brief Handles communication with and configuration of the u-blox device
 */
class ublox_ZEDF9P {
 public:
  //! Default timeout for ACK messages in seconds
  constexpr static double kDefaultAckTimeout = 1.0;
  //! Size of write buffer for output messages
  constexpr static int kWriterSize = 2056;

  ublox_ZEDF9P();
  virtual ~ublox_ZEDF9P();

  /**
   * @brief Initialize TCP I/O.
   * @param host the TCP host
   * @param port the TCP port
   */
  void initializeTcp(std::string host, std::string port);

  /**
   * @brief Initialize UDP I/O.
   * @param host the UDP host
   * @param port the UDP port
   */
  void initializeUdp(std::string host, std::string port);

  /**
   * @brief Test if the serial port can be read and write from by polling a VALGET message
   * and checking if we get an ACK
   * @return true if the port can be read and write from
   * @return false if polling fails
   */
  bool test_serial();

  /**
   * @brief Initialize the Serial I/O port.
   * @param port the device port address
   * @param baudrate the desired baud rate of the port
   */
  void initializeSerial(std::string port, unsigned int baudrate);

  /**
   * @brief Reset the Serial I/O port after u-blox reset.
   * @param port the device port address
   * @param baudrate the desired baud rate of the port
   */
  void resetSerial(std::string port, unsigned int baudrate);

  /**
   * @brief Set the rate at which the U-Blox device sends the given message
   * @param class_id the class identifier of the message
   * @param message_id the message identifier
   * @param rate the updated rate in Hz
   * @return true on ACK, false on other conditions.
   */
  bool setRate(uint8_t class_id, uint8_t message_id, uint8_t rate);

  /**
   * @brief Closes the I/O port, and initiates save on shutdown procedure
   * if enabled.
   */
  void close();

  /**
   * @brief Reset I/O communications.
   * @param wait Time to wait before restarting communications
   */
  void reset(const boost::posix_time::time_duration& wait);

  /**
   * @brief Send a reset message to the u-blox device.
   * @param nav_bbr_mask The BBR sections to clear, see CfgRST message
   * @param reset_mode The reset type, see CfgRST message
   * @return true if the message was successfully sent, false otherwise
   */
  bool configReset(uint16_t nav_bbr_mask, uint16_t reset_mode);
 
  /**
   * @brief Configure the U-Blox send rate of the message & subscribe to the
   * given message
   * @param the callback handler for the message
   * @param rate the rate in Hz of the message
   */
  template <typename T>
  void subscribe(typename CallbackHandler_<T>::Callback callback,
                 unsigned int rate);
  /**
   * @brief Subscribe to the given Ublox message.
   * @param the callback handler for the message
   */
  template <typename T>
  void subscribe(typename CallbackHandler_<T>::Callback callback);

  /**
   * @brief Subscribe to the given Ublox message.
   * @param the callback handler for the message
   */
  void subscribe_nmea(boost::function<void(const std::string&)> callback) { callbacks_.set_nmea_callback(callback); }

  /**
   * @brief Subscribe to the message with the given ID. This is used for
   * messages which have the same format but different message IDs,
   * e.g. INF messages.
   * @param the callback handler for the message
   * @param message_id the U-Blox message ID
   */
  template <typename T>
  void subscribeId(typename CallbackHandler_<T>::Callback callback,
                   unsigned int message_id);

  /**
   * Read a u-blox message of the given type.
   * @param message the received u-blox message
   * @param timeout the amount of time to wait for the desired message
   */
  template <typename T>
  bool read(T& message,
            const boost::posix_time::time_duration& timeout = default_timeout_);

  bool isInitialized() const { return worker_ != 0; }
  bool isConfigured() const { return isInitialized() && configured_; }
  bool isOpen() const { return worker_->isOpen(); }

  /**
   * Poll a u-blox message of the given type.
   * @param message the received u-blox message output
   * @param payload the poll message payload sent to the device
   * defaults to empty
   * @param timeout the amount of time to wait for the desired message
   */
  template <typename ConfigT>
  bool poll(ConfigT& message,
            const std::vector<uint8_t>& payload = std::vector<uint8_t>(),
            const boost::posix_time::time_duration& timeout = default_timeout_);
  /**
   * Poll a u-blox message.
   * @param class_id the u-blox message class id
   * @param message_id the u-blox message id
   * @param payload the poll message payload sent to the device,
   * defaults to empty
   * @param timeout the amount of time to wait for the desired message
   */
  bool poll(uint8_t class_id, uint8_t message_id,
            const std::vector<uint8_t>& payload = std::vector<uint8_t>());

  /**
   * @brief Send the given configuration message.
   * @param message the configuration message
   * @param wait if true, wait for an ACK
   * @return true if message sent successfully and either ACK was received or
   * wait was set to false
   */
  template <typename ConfigT>
  bool send_config_msg(const ConfigT& message, bool wait = true);

  /**
   * @brief Wait for an acknowledge message until the timeout
   * @param timeout maximum time to wait in seconds
   * @param class_id the expected class ID of the ACK
   * @param msg_id the expected message ID of the ACK
   * @return true if expected ACK received, false otherwise
   */
  bool waitForAcknowledge(const boost::posix_time::time_duration& timeout,
                          uint8_t class_id, uint8_t msg_id);

  /**
   * @brief Set the callback function which handles raw data.
   * @param callback the write callback which handles raw data
   */
  void setRawDataCallback(const Worker::Callback& callback);

  /**
   * @brief Polls the MONVER message to check hw and sw version. 
   * @return a ublox_msg::MONVER message containing the hw and
   */
  ublox_msgs::MONVER poll_MONVER();

  inline void set_debug_level(const int debug_level) {
    debug_level_ = debug_level;
    callbacks_.set_debug_level(debug_level);
  }

 private:
  //! Types for ACK/NACK messages, WAIT is used when waiting for an ACK
  enum AckType {
    NACK, //! Not acknowledged
    ACK, //! Acknowledge
    WAIT //! Waiting for ACK
  };

  //! Stores ACK/NACK messages
  struct Ack {
    AckType type; //!< The ACK type
    uint8_t class_id; //!< The class ID of the ACK
    uint8_t msg_id; //!< The message ID of the ACK
  };

  /**
   * @brief Set the I/O worker
   * @param an I/O handler
   */
  void setWorker(const boost::shared_ptr<Worker>& worker);

  /**
   * @brief Subscribe to ACK/NACK messages and UPD-SOS-ACK messages.
   */
  void subscribeAcks();

  /**
   * @brief Callback handler for UBX-ACK message.
   * @param m the message to process
   */
  void processAck(const ublox_msgs::Ack &m);

  /**
   * @brief Callback handler for UBX-NACK message.
   * @param m the message to process
   */
  void processNack(const ublox_msgs::Ack &m);

  //! Processes I/O stream data
  boost::shared_ptr<Worker> worker_;
  //! Whether or not the I/O port has been configured
  bool configured_;

  //! Defaults to 0
  int debug_level_ = 0;

  //! The default timeout for ACK messages
  static const boost::posix_time::time_duration default_timeout_;
  //! Stores last received ACK accessed by multiple threads
  mutable boost::atomic<Ack> ack_;

  //! Callback handlers for u-blox messages
  CallbackHandlers callbacks_;

  std::string host_, port_;
};


template <typename T>
void ublox_ZEDF9P::subscribe(typename CallbackHandler_<T>::Callback callback) {
  callbacks_.insert<T>(callback);
}

template <typename T>
void ublox_ZEDF9P::subscribeId(typename CallbackHandler_<T>::Callback callback,
                      unsigned int message_id) {
  callbacks_.insert<T>(callback, message_id);
}

template <typename ConfigT>
bool ublox_ZEDF9P::poll(ConfigT& message,
               const std::vector<uint8_t>& payload,
               const boost::posix_time::time_duration& timeout) {
  if (!poll(ConfigT::CLASS_ID, ConfigT::MESSAGE_ID, payload)) return false;
  return read(message, timeout);
}

template <typename T>
bool ublox_ZEDF9P::read(T& message, const boost::posix_time::time_duration& timeout) {
  if (!worker_) return false;
  return callbacks_.read(message, timeout);
}

template <typename ConfigT>
bool ublox_ZEDF9P::send_config_msg(const ConfigT& message, bool wait) {
  if (!worker_) return false;

  // Reset ack
  Ack ack;
  ack.type = WAIT;
  ack_.store(ack, boost::memory_order_seq_cst);

  // Encode the message
  std::vector<unsigned char> out(kWriterSize);
  ublox::Writer writer(out.data(), out.size());
  if (!writer.write(message)) {
    std::cout << ("Failed to encode config message 0x%02x / 0x%02x",
              message.CLASS_ID, message.MESSAGE_ID) << std::endl;
    return false;
  }
  // Send the message to the device
  worker_->send(out.data(), writer.end() - out.data());

  if (!wait) return true;

  // Wait for an acknowledgment and return whether or not it was received
  return waitForAcknowledge(default_timeout_,
                            message.CLASS_ID,
                            message.MESSAGE_ID);
}

}  // namespace ublox_ZEDF9P
