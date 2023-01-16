# pragma once

#include <libserial/SerialStream.h>

#include <memory>
#include <thread>
#include <mutex>
#include <condition_variable>
#include <chrono>
#include <iostream>
#include <iomanip>

#include <boost/function.hpp>

#include "ublox_ZEDF9P/logging.hpp"

namespace ublox_ZEDF9P {

/**
 * @brief Handles Asynchronous I/O reading and writing.
 */
class Worker {
 public:
  typedef std::mutex Mutex;
  typedef boost::function<void(unsigned char*, std::size_t&)> Callback;

  /**
   * @brief Construct an Asynchronous I/O worker.
   * @param baudrate the baudrate of the worker
   * @param port a string describing the port
   * @param buffer_size the size of the input and output buffers
   */
  inline Worker(unsigned int baudrate, std::string port, std::size_t buffer_size = 8192);
  inline ~Worker();

  /**
   * @brief Set the callback function which handles input messages.
   * @param callback the read callback which handles received messages
   */
  inline void setCallback(const Callback& callback) { read_callback_ = callback; }

  /**
   * @brief Send the data bytes via the I/O stream.
   * @param data the buffer of data bytes to send
   * @param size the size of the buffer
   */
  inline bool send(const unsigned char* data, const unsigned int size);
  /**
   * @brief Wait for incoming messages.
   * @param timeout_milliseconds the maximum time in milliseconds to wait
   */

  inline void wait(unsigned int timeout_milliseconds);

  inline bool isOpen() { return stream_.IsOpen(); }

  inline void set_debug_level(const int debug_level) {
    debug_level_ = debug_level;
  }

 protected:
  /**
   * @brief Read the input strea and process messages read from the input stream.
   */
  inline void doRead();

  /**
   * @brief Close the I/O stream.
   */
  inline void doClose();

  LibSerial::SerialStream stream_; //!< The I/O stream

  Mutex read_mutex_; //!< Lock for the input buffer
  std::condition_variable read_condition_;
  std::vector<unsigned char> in_; //!< The input buffer
  std::size_t in_buffer_size_; //!< number of bytes currently in the input buffer

  Mutex write_mutex_; //!< Lock for the output buffer
  std::condition_variable write_condition_;
  std::vector<unsigned char> out_; //!< The output buffer

  std::thread background_thread_; //!<Thread for background io service

  Callback read_callback_; //!< Callback function to handle received messages

  bool stopping_; //!< Whether or not the I/O service is closed
  int debug_level_ = 0;
};

Worker::Worker(unsigned int baudrate, std::string port, std::size_t buffer_size)
    : stopping_(false){

  // open serial port
  try {
    stream_.Open(port);
  } catch (const LibSerial::OpenFailed& e) {
    throw std::runtime_error("U-Blox: Worker: Could not open serial port :" + port + " " + e.what());
  }

  std::cout << SUCCESS << "U-Blox: Opened serial port  " << port << RESET_FORMATTING << std::endl;

  stream_.SetBaudRate(LibSerial::BaudRate::BAUD_115200);
  stream_.SetCharacterSize(LibSerial::CharacterSize::CHAR_SIZE_8);
  stream_.SetParity(LibSerial::Parity::PARITY_NONE);
  stream_.SetStopBits(LibSerial::StopBits::STOP_BITS_1);
  stream_.SetFlowControl(LibSerial::FlowControl::FLOW_CONTROL_NONE) ;

  in_.resize(buffer_size);
  in_buffer_size_ = 0;

  out_.reserve(buffer_size);

  background_thread_ = std::thread([this](){ doRead(); });
}

Worker::~Worker() {
  doClose();
  background_thread_.join();
}


bool Worker::send(const unsigned char* data, const unsigned int size) {
  const std::lock_guard<std::mutex> lock(write_mutex_);

  if(size == 0) {
    std::cout << "Ublox AsyncWorker::send: Size of message to send is 0" << std::endl;
    return true;
  }

  if (out_.capacity() - out_.size() < size) {
    std::cout << "Ublox AsyncWorker::send: Output buffer too full to send message" << std::endl;
    return false;
  }
  out_.insert(out_.end(), data, data + size);

  if (out_.size() == 0) {
    return true;
  }
  // TODO: im doing this because the rest of this library treats buffers as uint8 arrays. We should refactor this LOL
  stream_.write(const_cast<char *>(reinterpret_cast<char *>(out_.data())), out_.size());

  if (debug_level_ >= 2) {
    // Print the data that was sent
    std::ostringstream oss;
    for (std::vector<unsigned char>::iterator it = out_.begin();
         it != out_.end(); ++it)
      oss << std::setfill('0') << std::setw(2) << static_cast<unsigned int>(*it) << " ";
    std::cout << "U-Blox driver: sent " 
              << std::dec << static_cast<int>(out_.size()) 
              << " bytes: \n" 
              << oss.str().c_str() 
              << std::endl;
  }

  out_.clear();

  return true;
}

void Worker::doRead() {
  while (!stopping_) {
    const std::lock_guard<std::mutex> lock(read_mutex_);

    int num_bytes_available = stream_.GetNumberOfBytesAvailable();
    int max_bytes_transferrable = in_.size() - in_buffer_size_;
    int bytes_transfered;

    stream_.read(reinterpret_cast<char *>(in_.data()) + in_buffer_size_, in_.size() - in_buffer_size_);

    if (num_bytes_available < max_bytes_transferrable) {
      bytes_transfered = num_bytes_available;
    } else {
      bytes_transfered = max_bytes_transferrable;
    }
    in_buffer_size_ += bytes_transfered;

    if (debug_level_ >= 4) {
      std::ostringstream oss;
      for (std::vector<unsigned char>::iterator it =
                in_.begin() + in_buffer_size_ - bytes_transfered;
            it != in_.begin() + in_buffer_size_; ++it)
        oss << std::setfill('0') << std::setw(2) << static_cast<unsigned int>(*it) << " ";
      std::cout << "U-Blox driver: received " << bytes_transfered << " bytes\n" << oss.str().c_str() << std::endl;
    }

    if (read_callback_)
      read_callback_(in_.data(), in_buffer_size_);

    read_condition_.notify_all();
  }
}


void Worker::doClose() {
  std::cout << "Ublox: Worker: Closing serial port" << std::endl;
  const std::lock_guard<std::mutex> lock(read_mutex_);
  stopping_ = true;
  stream_.Close();
}

void Worker::wait(unsigned int timeout_milliseconds) {
  std::chrono::milliseconds duration(timeout_milliseconds);
  std::mutex mtx;
  std::unique_lock<std::mutex> lck(mtx);
  read_condition_.wait_for(lck, duration);
}

}  // namespace ublox_ZEDF9P

