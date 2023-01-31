# pragma once

#include <libserial/SerialStream.h>

#include <memory>
#include <thread>
#include <mutex>
#include <condition_variable>
#include <chrono>
#include <iostream>
#include <iomanip>
#include <functional>
#include <spdlog/spdlog.h>
#include <spdlog/fmt/bin_to_hex.h>
#include <atomic>
#include <utility>


namespace ublox_ZEDF9P {

/**
 * @brief Handles Asynchronous I/O reading and writing.
 */
class Worker {
public:
  typedef std::mutex Mutex;
  typedef std::function<void(unsigned char*, std::size_t&)> Callback;

  /**
   * @brief Construct an Asynchronous I/O worker.
   * @param baudrate the baudrate of the worker
   * @param port a string describing the port
   * @param buffer_size the size of the input and output buffers
   */
  Worker(unsigned int baudrate, std::string port, std::size_t buffer_size = 8192);
  ~Worker();

  /**
   * @brief Set the callback function which handles input messages.
   * @param callback the read callback which handles received messages
   */
  void setCallback(Callback&& callback) { read_callback_ = std::move(callback); }

  /**
   * @brief Send the data bytes via the I/O stream.
   * @param data the buffer of data bytes to send
   * @param size the size of the buffer
   */
  bool send(const unsigned char* data, const unsigned int size);
  /**
   * @brief Wait for incoming messages.
   * @param timeout_milliseconds the maximum time in milliseconds to wait
   */

  void wait(const unsigned int timeout_milliseconds);

  [[nodiscard]] bool isOpen() { return stream_.IsOpen(); }

  void set_debug_level(const int debug_level) {
    debug_level_ = debug_level;
  }

  /**
   * @brief Resets the serial connection
   * 
   * @return true if successful, false otherwise
   */
  bool resetSerialConnection();
protected:
  /**
   * @brief Read the input strea and process messages read from the input stream.
   */
  void doRead();

  /**
   * @brief Close the I/O stream.
   */
  void doClose();

  /**
   * @brief Opens and setup the serial port with the relevant baudrate, character size, parity, stop bits and flow control
   * @return true if the port is successfully opened, false otherwise
   */
  bool initializeSerial();

protected:
  std::string port_;
  LibSerial::SerialStream stream_; //!< The I/O stream
  LibSerial::BaudRate libserial_baudrate_; 

  Mutex read_mutex_; //!< Lock for the input buffer
  std::condition_variable read_condition_;
  std::vector<unsigned char> in_; //!< The input buffer
  std::size_t in_buffer_size_; //!< number of bytes currently in the input buffer

  Mutex write_mutex_; //!< Lock for the output buffer
  std::condition_variable write_condition_;
  std::vector<unsigned char> out_; //!< The output buffer

  std::thread background_thread_; //!<Thread for background io service

  Callback read_callback_; //!< Callback function to handle received messages

  std::atomic<bool> stopping_; //!< Whether or not the I/O service is closed
  int debug_level_ = 0;
};

inline Worker::Worker(unsigned int baudrate, std::string port, std::size_t buffer_size) {
  stopping_.store(false);
  
  port_ = port;

  switch(baudrate) {
    case 4800:
      libserial_baudrate_ = LibSerial::BaudRate::BAUD_4800;
      break;
    case 9600:
      libserial_baudrate_ = LibSerial::BaudRate::BAUD_9600;
      break;
    case 19200:
      libserial_baudrate_ = LibSerial::BaudRate::BAUD_19200;
      break;
    case 38400:
      libserial_baudrate_ = LibSerial::BaudRate::BAUD_38400;
      break;
    case 57600:
      libserial_baudrate_ = LibSerial::BaudRate::BAUD_57600;
      break;
    case 115200:
      libserial_baudrate_ = LibSerial::BaudRate::BAUD_115200;
      break;
    case 230400:
      libserial_baudrate_ = LibSerial::BaudRate::BAUD_230400;
      break;
    case 460800:
      libserial_baudrate_ = LibSerial::BaudRate::BAUD_460800;
      break;
    default:
      throw std::runtime_error("ublox: invalid baudrate! Valud baudrates are: 4800, 9600, 19200, 38400, 57600, 115200, 230400, 460800") ;
  }

  initializeSerial();

  in_.resize(buffer_size);
  in_buffer_size_ = 0;

  out_.reserve(buffer_size);

  background_thread_ = std::thread([this](){ doRead(); });
}

inline Worker::~Worker() {
  doClose();
}


inline bool Worker::send(const unsigned char* data, const unsigned int size) {
  const std::lock_guard<std::mutex> lock(write_mutex_);

  if(size == 0) {
    spdlog::warn("Ublox AsyncWorker::send: Size of message to send is 0");
    return true;
  }

  if (out_.capacity() - out_.size() < size) {
    spdlog::warn("Ublox AsyncWorker::send: Output buffer too full to send message");
    return false;
  }
  out_.insert(out_.end(), data, data + size);

  if (out_.size() == 0) {
    return true;
  }

  try {
    stream_.write(const_cast<char *>(reinterpret_cast<char *>(out_.data())), out_.size());
    stream_.DrainWriteBuffer();
    if (debug_level_ >= 2) {
      // Print the data that was sent
      spdlog::info(
        "U-Blox driver: sent {0:d} bytes:c {1}",
        static_cast<int>(out_.size()),
        spdlog::to_hex(out_.begin(), out_.end())
      );
    }
  } catch (std::runtime_error) {
    spdlog::error(
      "U-Blox driver: Could not send {0:d} bytes:c {1}",
      static_cast<int>(out_.size()),
      spdlog::to_hex(out_.begin(), out_.end())
    );
    
    return false;
  }

  out_.clear();

  return true;
}

inline void Worker::doRead() {
  while (!stopping_.load()) {
    if (stream_.IsOpen()) {
      if (stream_.IsDataAvailable()) {
        const std::lock_guard<std::mutex> lock(read_mutex_);

        const int num_bytes_available = stream_.GetNumberOfBytesAvailable();
        const int max_bytes_transferrable = in_.size() - in_buffer_size_;
        int bytes_transfered;

        if (num_bytes_available < max_bytes_transferrable) {
          bytes_transfered = num_bytes_available;
        } else {
          bytes_transfered = max_bytes_transferrable;
        }

        stream_.read(reinterpret_cast<char *>(in_.data()) + in_buffer_size_, bytes_transfered);
        
        if (bytes_transfered > 0) {
          in_buffer_size_ += bytes_transfered;

          if (debug_level_ >= 4) {
            spdlog::info(
              "U-Blox driver: received {0:d} bytes: {1}",
              bytes_transfered,
              spdlog::to_hex(in_.begin() + in_buffer_size_ - bytes_transfered, in_.begin() + in_buffer_size_)
            );
          }

          if (read_callback_) { read_callback_(in_.data(), in_buffer_size_); }

          read_condition_.notify_all();
          stream_.FlushInputBuffer();
        }
      } else {
        std::this_thread::sleep_for(std::chrono::milliseconds(5));
      }
    } 
  }
}


inline void Worker::doClose() {
  if (stream_.IsOpen()) {
    spdlog::info("Ublox: Worker: Closing serial port");
    const std::lock_guard<std::mutex> lock(read_mutex_);
    stopping_.store(true);
    stream_.Close();
  }

  if (background_thread_.joinable()) {
    background_thread_.join();  
  }
}

inline void Worker::wait(const unsigned int timeout_milliseconds) {
  std::chrono::milliseconds duration(timeout_milliseconds);
  std::unique_lock<std::mutex> lck(read_mutex_);
  read_condition_.wait_for(lck, duration);
}

inline bool Worker::initializeSerial() {
  // open serial port
  try {
    stream_.Open(port_);
    spdlog::info("U-Blox: Opened serial port {}", port_);

    stream_.SetBaudRate(libserial_baudrate_);
    stream_.SetCharacterSize(LibSerial::CharacterSize::CHAR_SIZE_8);
    stream_.SetParity(LibSerial::Parity::PARITY_NONE);
    stream_.SetStopBits(LibSerial::StopBits::STOP_BITS_1);
    stream_.SetFlowControl(LibSerial::FlowControl::FLOW_CONTROL_NONE);

    return true;
  } catch (const LibSerial::OpenFailed& e) {
    spdlog::error("U-Blox: Worker: Could not open serial port : {}", port_);
    return false;
  }
}

inline bool Worker::resetSerialConnection() {
  doClose();
  if (initializeSerial()) {
    stopping_.store(false);
    background_thread_ = std::thread([this](){ doRead(); });
    return true;
  }

  return false;
}

}  // namespace ublox_ZEDF9P

