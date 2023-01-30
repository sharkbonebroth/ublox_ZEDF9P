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
#include "ublox_ZEDF9P/logging.hpp"

namespace ublox_ZEDF9P {

using namespace ublox_msgs;

//! Sleep time [ms] after setting the baudrate
constexpr static int kSetBaudrateSleepMs = 500;

ublox_ZEDF9P::ublox_ZEDF9P() : configured_(false) {
  stopping_.store(false);
  subscribeAcks();

  test_serial_thread_ = std::thread([this](){ periodicTestSerial(); });
}

ublox_ZEDF9P::~ublox_ZEDF9P() { 
  close();
  test_serial_thread_.join();
}

void ublox_ZEDF9P::setWorker(const std::shared_ptr<Worker>& worker) {
  if (worker_) return;
  worker_ = worker;
  worker_->set_debug_level(debug_level_);
  worker_->setCallback(std::bind(&CallbackHandlers::readCallback, &callbacks_, std::placeholders::_1, std::placeholders::_2));
  configured_ = static_cast<bool>(worker);
}

void ublox_ZEDF9P::subscribeAcks() {
  // Set NACK handler
  subscribeId<ublox_msgs::Ack>(std::bind(&ublox_ZEDF9P::processNack, this, std::placeholders::_1),
                               ublox_msgs::Message::ACK::NACK);
  // Set ACK handler
  subscribeId<ublox_msgs::Ack>(std::bind(&ublox_ZEDF9P::processAck, this, std::placeholders::_1),
                               ublox_msgs::Message::ACK::ACK);
}

void ublox_ZEDF9P::processAck(const ublox_msgs::Ack &m) {
  // Process ACK/NACK messages
  Ack ack;
  ack.type = ACK;
  ack.class_id = m.clsID;
  ack.msg_id = m.msgID;
  // store the ack atomically
  ack_.store(ack, std::memory_order_seq_cst);
  if (debug_level_ >= 2) {
    spdlog::info(
      "U-blox: received ACK: {0:x} / {1:x} ",
      m.clsID,
      m.msgID
    );
  }
}

void ublox_ZEDF9P::processNack(const ublox_msgs::Ack &m) {
  // Process ACK/NACK messages
  Ack ack;
  ack.type = NACK;
  ack.class_id = m.clsID;
  ack.msg_id = m.msgID;
  // store the ack atomically
  ack_.store(ack, std::memory_order_seq_cst);
    spdlog::error(
      "U-blox: received NACK: {0:x} / {1:x} ",
      m.clsID,
      m.msgID
    );
}

bool ublox_ZEDF9P::test_serial() {
  if (!worker_->isOpen()) {
    return false;
  }
  Valget test_msg;
  test_msg.add_config_request(0x40520001);
  bool success = send_config_msg(test_msg, true);
  return success;
}

void ublox_ZEDF9P::initializeSerial(std::string port, unsigned int baudrate) {
  port_ = port;
  baudrate_ = baudrate;

  // Set the I/O worker
  if (worker_) return;
  setWorker(std::shared_ptr<Worker>(new Worker(baudrate, port)));

  if (!test_serial()) {
    throw std::runtime_error("U-Blox: Could not read/ write from serial port :" + port);
  }

  configured_ = true;
}

void ublox_ZEDF9P::resetSerial(std::string port, unsigned int baudrate) {
  // Set the I/O worker
  if (worker_) return;
  setWorker(std::shared_ptr<Worker>(new Worker(baudrate, port)));

  if (!test_serial()) {
    throw std::runtime_error("U-Blox: Reset serial port: Could not read/ write from serial port :" + port);
  }

  configured_ = true;
}

bool ublox_ZEDF9P::setRate(uint8_t class_id, uint8_t message_id, uint8_t rate) {
  if (debug_level_ >= 2) {
    spdlog::info(
      "Setting rate to {0:d} for {1:x} / {2:x}",
      rate,
      message_id,
      class_id
    );
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
  stopping_.store(true);
}

void ublox_ZEDF9P::reset(const unsigned int timeout_milliseconds) {
  worker_.reset();
  configured_ = false;
  std::chrono::milliseconds duration(timeout_milliseconds);
  // sleep because of undefined behavior after I/O reset
  std::this_thread::sleep_for(duration);
  resetSerial(port_, baudrate_);
}

void ublox_ZEDF9P::change_baudrate(unsigned int new_baudrate) {
  spdlog::info("Resetting u-blox and changing baudrate used");
  worker_.reset();
  configured_ = false;
  std::chrono::milliseconds duration(1000);
  // sleep because of undefined behavior after I/O reset
  std::this_thread::sleep_for(duration);
  initializeSerial(port_, new_baudrate);
}


bool ublox_ZEDF9P::configReset(uint16_t nav_bbr_mask, uint16_t reset_mode) {
  spdlog::info("Resetting u-blox. If device address changes, node must be relaunched.");

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

bool ublox_ZEDF9P::waitForAcknowledge(const unsigned int timeout_milliseconds,
                             uint8_t class_id, uint8_t msg_id) {
  if (debug_level_ >= 2) {
    spdlog::info(
      "U-blox: Waiting for ACK: {0:x} / {1:x} ",
      class_id,
      msg_id
    );
  }

  std::chrono::time_point<std::chrono::steady_clock> wait_until_time = std::chrono::steady_clock::now();
  std::chrono::milliseconds duration(timeout_milliseconds);
  wait_until_time += duration;

  Ack ack = ack_.load(std::memory_order_seq_cst);
  while (std::chrono::steady_clock::now() < wait_until_time
         && (ack.class_id != class_id
             || ack.msg_id != msg_id
             || ack.type == WAIT)) {
    worker_->wait(timeout_milliseconds);
    ack = ack_.load(std::memory_order_seq_cst);
  }
  bool result = ack.type == ACK
                && ack.class_id == class_id
                && ack.msg_id == msg_id;
  return result;
}


ublox_msgs::MONVER ublox_ZEDF9P::poll_MONVER() {
  ublox_msgs::MONVER MONVER_msg;
  poll(MONVER_msg);

  if (debug_level_ >= 1) {
    char swVersion_char[30];
    char hwVersion_char[10];
    std::memcpy(swVersion_char, MONVER_msg.swVersion.data(), 30);
    spdlog::info("ublox_ZEDF9P software version: {}", swVersion_char);

    std::memcpy(hwVersion_char, MONVER_msg.hwVersion.data(), 10);
    spdlog::info("ublox_ZEDF9P hardware version: {}", hwVersion_char);    
    for (std::array<uint8_t, 30> extended_data: MONVER_msg.extension) {
      char extended_data_char[30];
      std::memcpy(extended_data_char, extended_data.data(), 30);
      spdlog::info(extended_data_char);
    }

  }

  return MONVER_msg;
}

void ublox_ZEDF9P::poll_Valget(ublox_msgs::Valget& Valget_msg) {
  int payload_length = static_cast<int>(ublox_msgs::Valget::serialized_length(Valget_msg));

  uint8_t payload[payload_length];
  ublox_msgs::Valget::write_to_data_stream(payload, Valget_msg);

  std::vector<uint8_t> payload_vec;
  payload_vec.insert(payload_vec.end(), payload, payload + payload_length);

  poll(Valget_msg, payload_vec);
}

void ublox_ZEDF9P::periodicTestSerial() {
  while (!stopping_.load()) {
    if (worker_) {
      if (!test_serial()) {
        spdlog::error("Ping Ublox failed... Attempting to reset serial");
        if (worker_->resetSerialConnection()) {
          std::this_thread::sleep_for(std::chrono::milliseconds(1000));
          for (std::array<int,3> subscribed_message: subscribed_) {
            setRate(subscribed_message[0], subscribed_message[1], subscribed_message[2]);
          }
          spdlog::info("Successfully reset serial");
        } else {
          spdlog::error("failed to reset serial");
        }
      } 
    } else {
      if (debug_level_ > 1)
        spdlog::info("Ublox: Serial not initialized yet...");
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(1000));
  }
}

}  // namespace ublox_ZEDF9P
