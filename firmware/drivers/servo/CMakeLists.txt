zephyr_include_directories(${CMAKE_CURRENT_SOURCE_DIR})

if(CONFIG_SERVO)
  zephyr_library()
  zephyr_library_sources(servo.c)
  zephyr_library_include_directories(${CMAKE_CURRENT_SOURCE_DIR})
endif()
