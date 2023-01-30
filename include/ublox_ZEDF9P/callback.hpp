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
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE 
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE 
// ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER BE LIABLE FOR ANY
// DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
// (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
// LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
// ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
// (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
// SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
//==============================================================================

#pragma once

#include <mutex>
#include <memory>
#include <functional>
#include <spdlog/spdlog.h>

#include "ublox_ZEDF9P/serialization.hpp"

namespace ublox_ZEDF9P {

/**
 * @brief A callback handler for a u-blox message.
 */
class CallbackHandler {
 public:
  /**
   * @brief Decode the u-blox message.
   */
  virtual void handle(ublox::Reader& reader) = 0;

  /**
   * @brief Wait for on the condition.
   */
  bool wait(const unsigned int timeout_milliseconds) {
    std::chrono::milliseconds duration(timeout_milliseconds);
    std::unique_lock<std::mutex> lock(mutex_);
    if (condition_.wait_for(lock, duration) == std::cv_status::timeout) {
      return false;
    }
    return true;
  }

  inline void set_debug_level(const int level) {
    debug_level_ = level;
  }

 protected:
  std::mutex mutex_; //!< Lock for the handler
  std::condition_variable condition_; //!< Condition for the handler lock

  int debug_level_ = 0;
};

/**
 * @brief A callback handler for a u-blox message.
 * @typedef T the message type
 */
template <typename T>
class CallbackHandler_ : public CallbackHandler {
 public:
  typedef std::function<void(const T&)> Callback; //!< A callback funtion

  /** 
   * @brief Initialize the Callback Handler with a callback function
   * @param func a callback function for the message, defaults to none
   */
  CallbackHandler_(const Callback& func = Callback()) : func_(func) {}
  
  /**
   * @brief Get the last received message.
   */
  virtual const T& get() { return message_; }

  /**
   * @brief Decode the U-Blox message & call the callback function if it exists.
   * @param reader a reader to decode the message buffer
   */
  void handle(ublox::Reader& reader) {
    std::lock_guard<std::mutex> lock(mutex_);
    try {
      if (!reader.read<T>(message_)) {
        if (debug_level_ >= 2) {
          spdlog::error(
            "U-blox Decoder error for {0:x} / {1:x} {2:d} bytes",
            reader.classId(),
            reader.messageId(),
            reader.length()
          );
        }
        condition_.notify_all();
        return;
      }
    } catch (std::runtime_error& e) {
        if (debug_level_ >= 2) {
          spdlog::error(
            "U-blox Decoder error for {0:x} / {1:x} {2:d} bytes",
            reader.classId(),
            reader.messageId(),
            reader.length()
          );
        }
      condition_.notify_all();
      return;
    }

    if (func_) func_(message_);
    condition_.notify_all();
  }
  
 private:
  Callback func_; //!< the callback function to handle the message
  T message_; //!< The last received message
};

/**
 * @brief Callback handlers for incoming u-blox messages.
 */
class CallbackHandlers {
 public:
  /**
   * @brief Add a callback handler for the given message type.
   * @param callback the callback handler for the message
   * @typedef.a ublox_msgs message with CLASS_ID and MESSAGE_ID constants
   */
  template <typename T>
  void insert(typename CallbackHandler_<T>::Callback callback) {
    std::lock_guard<std::mutex> lock(callback_mutex_);
    CallbackHandler_<T>* handler = new CallbackHandler_<T>(callback);
    handler->set_debug_level(debug_level_);
    callbacks_.insert(
      std::make_pair(std::make_pair(T::CLASS_ID, T::MESSAGE_ID),
                     std::shared_ptr<CallbackHandler>(handler)));
  }

  /**
   * @brief Add a callback handler for the given message type and ID. This is 
   * used for messages in which have the same structure (and therefore msg file)
   * and same class ID but different message IDs. (e.g. INF, ACK)
   * @param callback the callback handler for the message
   * @param message_id the ID of the message
   * @typedef.a ublox_msgs message with a CLASS_ID constant
   */
  template <typename T>
  void insert(
      typename CallbackHandler_<T>::Callback callback, 
      unsigned int message_id) {
    std::lock_guard<std::mutex> lock(callback_mutex_);
    CallbackHandler_<T>* handler = new CallbackHandler_<T>(callback);
    handler->set_debug_level(debug_level_);
    callbacks_.insert(
      std::make_pair(std::make_pair(T::CLASS_ID, message_id),
                     std::shared_ptr<CallbackHandler>(handler)));
  }

  /**
   * @brief Calls the callback handler for the message in the reader.
   * @param reader a reader containing a u-blox message
   */
  void handle(ublox::Reader& reader) {
    // Find the callback handlers for the message & decode it
    std::lock_guard<std::mutex> lock(callback_mutex_);
    Callbacks::key_type key =
        std::make_pair(reader.classId(), reader.messageId());
    for (Callbacks::iterator callback = callbacks_.lower_bound(key);
         callback != callbacks_.upper_bound(key); ++callback) {
      callback->second->handle(reader);
    }
  }

  /**
   * @brief Read a u-blox message of the given type.
   * @param message the received u-blox message
   * @param timeout_milliseconds the amount of time to wait for the desired message in milliseconds
   */
  template <typename T>
  bool read(T& message, const unsigned int timeout_milliseconds) {
    bool result = false;
    // Create a callback handler for this message
    callback_mutex_.lock();
    CallbackHandler_<T>* handler = new CallbackHandler_<T>();
    handler->set_debug_level(debug_level_);
    Callbacks::iterator callback = callbacks_.insert(
      (std::make_pair(std::make_pair(T::CLASS_ID, T::MESSAGE_ID),
                      std::shared_ptr<CallbackHandler>(handler))));
    callback_mutex_.unlock();

    // Wait for the message
    if (handler->wait(timeout_milliseconds)) {
      message = handler->get();
      result = true;
    }
    
    // Remove the callback handler
    callback_mutex_.lock();
    callbacks_.erase(callback);
    callback_mutex_.unlock();
    return result;
  }

  /**
   * @brief Processes u-blox messages in the given buffer & clears the read
   * messages from the buffer.
   * @param data the buffer of u-blox messages to process
   * @param size the size of the buffer
   */
  void readCallback(unsigned char* data, std::size_t& size) {
    ublox::Reader reader(data, size);
    // Read all U-Blox messages in buffer
    while (reader.search() != reader.end() && reader.found()) {
      if (debug_level_ >= 3) {
        // Print the received bytes
        spdlog::info(
          "U-Blox driver: received {0:d} bytes: {1}",
          reader.length() + 8,
          spdlog::to_hex(reader.pos(), reader.pos() + reader.length() + 8)
        );
      }

      handle(reader);
    }

    // delete read bytes from ASIO input buffer
    std::copy(reader.pos(), reader.end(), data);
    size -= reader.pos() - data;
  }

  inline void set_debug_level(int debug_level) {
    debug_level_ = debug_level;
  }

 private:
  typedef std::multimap<std::pair<uint8_t, uint8_t>,
                        std::shared_ptr<CallbackHandler> > Callbacks;

  // Call back handlers for u-blox messages
  Callbacks callbacks_;
  std::mutex callback_mutex_;

  int debug_level_ = 0;
};

}  // namespace ublox_gps