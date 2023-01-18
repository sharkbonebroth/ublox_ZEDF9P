cmake_minimum_required(VERSION 3.15)

project(ublox_ZEDF9P VERSION 1.0)

# this is temporary, just to link to libserial
add_subdirectory(${CMAKE_CURRENT_SOURCE_DIR}/3rd_party/libserial ${CMAKE_BINARY_DIR}/3rd_party)

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
  dash3rdparty::libserial
)

if (${CMAKE_CURRENT_SOURCE_DIR} STREQUAL ${CMAKE_SOURCE_DIR})
  add_subdirectory(${CMAKE_SOURCE_DIR}/examples ${CMAKE_BINARY_DIR}/examples)
endif()