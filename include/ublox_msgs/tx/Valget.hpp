#pragma once

#include <unordered_map>
#include "ublox_msgs/ublox_msgs_utils.hpp"


namespace ublox_msgs {

/**
 * @brief A ublox message for polling for the configuration settings of a ublox GPS
 */
struct Valget {

  /**
   * @brief Writes the message contents into bytes.
   * @param data_stream where to write to
   * @param message the Valget object to write from
   */
  inline static void write_to_data_stream(uint8_t* data_stream, const Valget &message);

  /**
   * @brief The length of this message (when sent by this driver) in bytes
   * @param message the Valget object from which to obtain the serialized length
   */
  inline static uint32_t serialized_length(const Valget &message);

  inline bool add_config_request(uint32_t key);

  /**
   * @brief Initialise the message from bytes.
   * @param data_stream the start of the byte string from which to decode
   * @param message the message object to write to
   */
  inline static void initialize_from_stream(uint8_t* data_stream, Valget &message);

  uint8_t version = 0;
  uint8_t layer = 0;
  uint16_t position = 0;
  std::vector<uint32_t> configs_to_request;
  std::unordered_map<uint32_t, std::vector<uint8_t>> received_configs;

  enum {
    CLASS_ID = 6u,
    MESSAGE_ID = 139u
  };

}; // struct Valget

void Valget::write_to_data_stream(uint8_t* data_stream, const Valget & message) {
  write_data_to_stream(message.version, data_stream);
  write_data_to_stream(message.layer, data_stream);
  write_data_to_stream(message.position, data_stream);
  write_data_to_stream(message.configs_to_request, data_stream);
}

uint32_t Valget::serialized_length(const Valget &message) {
  return 4 + message.configs_to_request.size() * 4;
}

bool Valget::add_config_request(uint32_t key) {
  if (configs_to_request.size() >= 64) {
    return false;
  }

  configs_to_request.push_back(key);

  return true;
}

void Valget::initialize_from_stream(uint8_t* data_stream, Valget &message) {
  int num_bytes_to_read = received_message_length(data_stream) - 4;

  initialize_datafield_from_stream(message.version, data_stream);
  initialize_datafield_from_stream(message.layer, data_stream);
  initialize_datafield_from_stream(message.position, data_stream);

  while (num_bytes_to_read > 4) {
    uint32_t key = *reinterpret_cast<uint32_t*>(data_stream);

    data_stream += 4;
    num_bytes_to_read -= 4;

    int configuration_data_length = configuration_data_length_from_key(key);

    if (configuration_data_length <= num_bytes_to_read) {
      message.received_configs[key] = std::vector<uint8_t>(data_stream, data_stream + configuration_data_length);

      data_stream += configuration_data_length;
      num_bytes_to_read -= configuration_data_length;
    } else {
      break;
    }
  }
}

} // namespace ublox_msgs