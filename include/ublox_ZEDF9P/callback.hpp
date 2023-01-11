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

#include "ublox_ZEDF9P/serialization.hpp"
#include <boost/format.hpp>
#include <boost/function.hpp>
#include <boost/thread.hpp>

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
  bool wait(const boost::posix_time::time_duration& timeout) {
    boost::mutex::scoped_lock lock(mutex_);
    return condition_.timed_wait(lock, timeout);
  }

  inline void set_debug_level(const int level) {
    debug_level_ = level;
  }

 protected:
  boost::mutex mutex_; //!< Lock for the handler
  boost::condition_variable condition_; //!< Condition for the handler lock

  int debug_level_ = 0;
};

/**
 * @brief A callback handler for a u-blox message.
 * @typedef T the message type
 */
template <typename T>
class CallbackHandler_ : public CallbackHandler {
 public:
  typedef boost::function<void(const T&)> Callback; //!< A callback function

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
    boost::mutex::scoped_lock lock(mutex_);
    try {
      if (!reader.read<T>(message_)) {
        if (debug_level_ >= 2) {
          std::cout << "U-Blox Decoder error for " 
                    << std::hex << static_cast<int>(reader.classId())
                    << " / "
                    << std::hex << static_cast<int>(reader.messageId())
                    << reader.length()
                    << " bytes "
                    << std::endl;
        }
        condition_.notify_all();
        return;
      }
    } catch (std::runtime_error& e) {
        if (debug_level_ >= 2) {
          std::cout << "U-Blox Decoder error for " 
                    << std::hex << static_cast<int>(reader.classId())
                    << " / "
                    << std::hex << static_cast<int>(reader.messageId())
                    << reader.length()
                    << " bytes "
                    << std::endl;
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
    boost::mutex::scoped_lock lock(callback_mutex_);
    CallbackHandler_<T>* handler = new CallbackHandler_<T>(callback);
    handler->set_debug_level(debug_level_);
    callbacks_.insert(
      std::make_pair(std::make_pair(T::CLASS_ID, T::MESSAGE_ID),
                     boost::shared_ptr<CallbackHandler>(handler)));
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
    boost::mutex::scoped_lock lock(callback_mutex_);
    CallbackHandler_<T>* handler = new CallbackHandler_<T>(callback);
    handler->set_debug_level(debug_level_);
    callbacks_.insert(
      std::make_pair(std::make_pair(T::CLASS_ID, message_id),
                     boost::shared_ptr<CallbackHandler>(handler)));
  }

  /**
   * @brief Add a callback handler for nmea messages
   * @param callback the callback handler for the message
   */
  void set_nmea_callback(boost::function<void(const std::string&)> callback) {
    boost::mutex::scoped_lock lock(callback_mutex_);
    callback_nmea_ = callback;
  }

  /**
   * @brief Calls the callback handler for the message in the reader.
   * @param reader a reader containing a u-blox message
   */
  void handle(ublox::Reader& reader) {
    // Find the callback handlers for the message & decode it
    boost::mutex::scoped_lock lock(callback_mutex_);
    Callbacks::key_type key =
        std::make_pair(reader.classId(), reader.messageId());
    for (Callbacks::iterator callback = callbacks_.lower_bound(key);
         callback != callbacks_.upper_bound(key); ++callback) {
      callback->second->handle(reader);
    }
  }

  /**
   * @brief Calls the callback handler for the nmea messages in the reader.
   * @param reader a reader containing an nmea message
   */
  void handle_nmea(ublox::Reader& reader) {
    boost::mutex::scoped_lock lock(callback_mutex_);
    if(callback_nmea_.empty())
        return;

    const std::string& buffer = reader.getUnusedData();
    size_t nmea_start = buffer.find('$', 0);
    size_t nmea_end = buffer.find('\n', nmea_start);
    while(nmea_start != std::string::npos && nmea_end != std::string::npos) {
        std::string sentence = buffer.substr(nmea_start, nmea_end - nmea_start + 1);
        callback_nmea_(sentence);

        nmea_start = buffer.find('$', nmea_end+1);
        nmea_end = buffer.find('\n', nmea_start);
    }
  }

  /**
   * @brief Read a u-blox message of the given type.
   * @param message the received u-blox message
   * @param timeout the amount of time to wait for the desired message
   */
  template <typename T>
  bool read(T& message, const boost::posix_time::time_duration& timeout) {
    bool result = false;
    // Create a callback handler for this message
    callback_mutex_.lock();
    CallbackHandler_<T>* handler = new CallbackHandler_<T>();
    handler->set_debug_level(debug_level_);
    Callbacks::iterator callback = callbacks_.insert(
      (std::make_pair(std::make_pair(T::CLASS_ID, T::MESSAGE_ID),
                      boost::shared_ptr<CallbackHandler>(handler))));
    callback_mutex_.unlock();

    // Wait for the message
    if (handler->wait(timeout)) {
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
        std::ostringstream oss;
        for (ublox::Reader::iterator it = reader.pos();
             it != reader.pos() + reader.length() + 8; ++it)
          oss << boost::format("%02x") % static_cast<unsigned int>(*it) << " ";
        std::cout << "U-blox driver: reading " 
                  << std::dec <<  reader.length() + 8 
                  << " bytes\n" << oss.str().c_str() 
                  << std::endl;
      }

      handle(reader);
    }
    handle_nmea(reader);

    // delete read bytes from ASIO input buffer
    std::copy(reader.pos(), reader.end(), data);
    size -= reader.pos() - data;
  }

  inline void set_debug_level(int debug_level) {
    debug_level_ = debug_level;
  }

 private:
  typedef std::multimap<std::pair<uint8_t, uint8_t>,
                        boost::shared_ptr<CallbackHandler> > Callbacks;

  // Call back handlers for u-blox messages
  Callbacks callbacks_;
  boost::mutex callback_mutex_;
  
  //! Callback handler for nmea messages
  boost::function<void(const std::string&)> callback_nmea_;

  int debug_level_ = 0;
};

}  // namespace ublox_gps