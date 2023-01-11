#pragma once

#include "ublox_msgs/ublox_msgs_utils.hpp"


namespace ublox_msgs {

/**
 * @brief A ublox message for configuration of the ublox GPS. See Interface desc document for more info
 */
struct Valset {

  /**
   * @brief Writes the message contents into bytes.
   * @param data_stream where to write to
   * @param message the Valset object to write from
   */
  inline static void write_to_data_stream(uint8_t* data_stream, const Valset &message);

  /**
   * @brief The length of this message (when sent by this driver) in bytes
   * @param message the Valset object from which to obtain the serialized length
   */
  inline static uint32_t serialized_length(const Valset &message);

  /**
   * @brief Adds a configuration to the Valset message
   * @tparam config_data_type the data type of the configuration to be set
   * @param key the key of the configuration to be set
   * @param data the data containing the configuration
   * @return true if the configuration is successfully added
   */
  template <typename config_data_type>
  bool add_config(uint32_t key, config_data_type& data);

  Valset() {
    reserved.fill(0);
    config_ptr = cfgData.data();
  }

  uint8_t version = 0;
  uint8_t layers = 0;
  std::array<uint8_t, 2> reserved;
  std::vector<uint8_t> cfgData;

  int num_configs = 0;
  uint8_t* config_ptr;

  enum {
    CLASS_ID = 6u,
    MESSAGE_ID = 138u,
    RAM_LAYER_BIT = 1u,
    BBR_LAYER_BIT = 2u,
    FLASH_LAYER_BIT = 4u
  };

}; // Struct Valset

void Valset::write_to_data_stream(uint8_t* data_stream, const Valset &message) {
  write_data_to_stream(message.version, data_stream);
  write_data_to_stream(message.layers, data_stream);
  write_data_to_stream(message.reserved, data_stream);
  write_data_to_stream(message.cfgData, data_stream);
}

uint32_t Valset::serialized_length(const Valset &message) {
  return 4 + message.cfgData.size();
}

template <typename config_data_type>
bool Valset::add_config(uint32_t key, config_data_type& data) {
  // Verify that the data length matches what is mentioned in the key, and the max number of configs have not been added yet
  if (num_configs == 64 || configuration_data_length_from_key(key) != sizeof(config_data_type)) {
    // std::cout << "desired data len: " << std::dec << configuration_data_length_from_key(key) << " but " << static_cast<int>(sizeof(config_data_type)) << " bytes provided" << std::endl;
    return false;
  }

  // Copy the key
  uint8_t* key_data = reinterpret_cast<uint8_t*>(&key);
  cfgData.insert(cfgData.end(), key_data, key_data + 4);

  // Then copy the data
  cfgData.insert(cfgData.end(), reinterpret_cast<uint8_t*>(&data), reinterpret_cast<uint8_t*>(&data) + sizeof(config_data_type));

  num_configs++;
  return true;
}

}// Namespace ublox_msgs