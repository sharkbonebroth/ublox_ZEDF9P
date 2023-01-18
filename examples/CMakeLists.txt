add_executable(demo_config 
  ${CMAKE_CURRENT_SOURCE_DIR} demo_config.cpp
)

target_link_libraries(demo_config
PRIVATE
  ublox_ZEDF9P
)

add_dependencies(demo_config ublox::ublox_ZEDF9P)

add_executable(demo_subscribe_navpvt
  ${CMAKE_CURRENT_SOURCE_DIR} demo_subscribe_navpvt.cpp
)

target_link_libraries(demo_subscribe_navpvt
PRIVATE
  ublox_ZEDF9P
)

add_dependencies(demo_subscribe_navpvt ublox::ublox_ZEDF9P)