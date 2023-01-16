cmake_minimum_required(VERSION 3.15)

project(ublox_ZEDF9P VERSION 1.0)

# this is temporary, just to link to libserial
add_subdirectory(${CMAKE_CURRENT_SOURCE_DIR}/../libserial/src ${CMAKE_BINARY_DIR}/3rd_party)

find_package(
  Boost REQUIRED COMPONENTS system regex thread
)

add_library(ublox_ZEDF9P
  ${CMAKE_CURRENT_SOURCE_DIR}/src/ublox_ZEDF9P.cpp
)

add_dependencies(ublox_ZEDF9P libserial_LIB)

add_library(ublox::ublox_ZEDF9P ALIAS ublox_ZEDF9P)

target_include_directories(ublox_ZEDF9P 
PUBLIC
  ${CMAKE_CURRENT_SOURCE_DIR}/include
)

target_link_libraries(ublox_ZEDF9P
PUBLIC
  Boost::system
  Boost::regex
  Boost::thread
  dash3rdparty::libserial
)