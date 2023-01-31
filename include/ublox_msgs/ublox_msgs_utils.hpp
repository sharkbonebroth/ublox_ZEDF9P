#pragma once

#include <cinttypes>
#include <cstring>
#include <vector>
#include <array>

namespace ublox_msgs {

/**
 * @brief Writes data from a data stream to a data field, and moves the data_stream pointer to the next field to be read
 * @param datafield The datafield to be initialized from the stream
 * @param data_stream the data stream to initialize the datafield from
 */
template<typename datafield_type>
inline void initialize_datafield_from_stream(datafield_type & datafield, const uint8_t* & data_stream) {
  std::memcpy(&datafield, data_stream, sizeof(datafield_type));
  data_stream += sizeof(datafield_type); 
}

/**
 * @brief Writes data from a data stream to a data field of type std::array, and moves the data_stream pointer to the next field to be read
 * @param datafield The datafield to be initialized from the stream
 * @param data_stream the data stream to initialize the datafield from
 * @overload
 */
template<typename array_datafield_type, std::size_t array_length>
inline void initialize_datafield_from_stream(std::array<array_datafield_type, array_length> & datafield, const uint8_t* & data_stream) {
  std::memcpy(datafield.data(), data_stream, array_length * sizeof(array_datafield_type));
  data_stream += array_length * sizeof(array_datafield_type); 
}

/**
 * @brief Writes data to a data stream, and moves the data_stream pointer to the end of the data stream after writing
 * @param datafield variable containing the data to be written
 * @param data_stream the data stream to write to
 */
template <typename datafield_type>
inline void write_data_to_stream(const datafield_type& datafield, uint8_t* & data_stream) {
  std::memcpy(data_stream, &datafield, sizeof(datafield_type));
  data_stream += sizeof(datafield_type); 
}

/**
 * Writes std::vector data to a data stream, and moves the data_stream pointer to the end of the data stream after writing
 * @param datafield variable containing the data to be written
 * @param data_stream the data stream to write to
 * @overload
 */
template <typename vector_datafield_type>
inline void write_data_to_stream(const std::vector<vector_datafield_type>& datafield, uint8_t* & data_stream) {
  std::memcpy(data_stream, datafield.data(), datafield.size() * sizeof(vector_datafield_type));
  data_stream += datafield.size() * sizeof(vector_datafield_type); 
}

/**
 * Writes std::array data to a data stream, and moves the data_stream pointer to the end of the data stream after writing
 * @param datafield variable containing the data to be written
 * @param data_stream the data stream to write to
 * @overload
 */
template <typename array_datafield_type, std::size_t array_length>
inline void write_data_to_stream(const std::array<array_datafield_type, array_length>& datafield, uint8_t* & data_stream) {
  std::memcpy(data_stream, datafield.data(), array_length * sizeof(array_datafield_type));
  data_stream += array_length * sizeof(array_datafield_type); 
}

/**
 * Writes a normal c-style array to a data stream, and moves the data_stream pointer to the end of the data stream after writing
 * @param datafield variable containing the data to be written
 * @param data_stream the data stream to write to
 * @overload
 */
template <typename array_datafield_type, std::size_t array_length>
inline void write_data_to_stream(const array_datafield_type datafield[array_length], uint8_t* & data_stream) {
  std::memcpy(data_stream, datafield, array_length * sizeof(array_datafield_type));
  data_stream += array_length * sizeof(array_datafield_type); 
}

/**
 * @brief Get the length of this message (when received from the ublox gps) in bytes. 
 * Not to be confused with the serialized_length function in ublox_msgs that are to be sent to the ublox gps
 * @param data_stream the start of the byte string from which to decode
 */
inline int received_message_length(const uint8_t* data_stream) {
  return (*(data_stream - 2) + (*(data_stream - 1) << 8));
}

/**
 * @brief Get the length of configuration data from its key
 * @param key the key to evaluate
 * @return uint32_t The configuration data length in bytes
 */
inline int configuration_data_length_from_key(const uint32_t key) {
  uint32_t tmp = key >> 28;
  uint32_t length_key = tmp & 0x7;
  switch (length_key) {
    case 0x01: 
      return 1;
    case 0x02:
      return 1;
    case 0x03:
      return 2;
    case 0x04:
      return 4;
    case 0x05:
      return 8;
    default:
      return -1;
  } 
}

} // namespace ublox_msgs