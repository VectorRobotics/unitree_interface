cmake_minimum_required(VERSION 3.8)
project(unitree_interface)

# Options
option(ENABLE_LOW_LEVEL_MODE "Enable LowLevelMode usage" ON)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
set(CMAKE_BUILD_TYPE Release)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(unitree_interface_msgs REQUIRED)

# unitree_sdk2
set(BUILD_EXAMPLES OFF CACHE BOOL "" FORCE) # Don't build unitree_sdk2 examples
add_subdirectory(third_party/unitree_sdk2)

# Create the library
add_library(unitree_interface_lib
  STATIC
    src/unitree_interface.cpp
    src/unitree_sdk_wrapper.cpp
    src/mode_transitions.cpp
)

target_include_directories(unitree_interface_lib
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)

# Not decreasing compile time for some reason
# target_precompile_headers(unitree_interface_lib
#   PRIVATE
#     include/unitree_interface/pch.hpp
# )

target_link_libraries(unitree_interface_lib
  unitree_sdk2
)

ament_target_dependencies(unitree_interface_lib
  rclcpp
  geometry_msgs
  sensor_msgs
  std_srvs
  unitree_interface_msgs
)

target_compile_definitions(unitree_interface_lib
  PUBLIC
    $<$<BOOL:${ENABLE_LOW_LEVEL_MODE}>:UNITREE_INTERFACE_ENABLE_LOW_LEVEL_MODE>
)

# Create the executable
add_executable(unitree_interface_node
  src/unitree_interface_node.cpp
)

target_link_libraries(unitree_interface_node
  unitree_interface_lib
)

install(TARGETS unitree_interface_node
  DESTINATION lib/unitree_interface
)

install(DIRECTORY launch
  DESTINATION share/unitree_interface
)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)

  # ========== Unit tests ==========
  # Test control modes
  ament_add_gtest(test_control_modes_unit
    test/unit/test_control_modes.cpp
  )

  target_link_libraries(test_control_modes_unit
    unitree_interface_lib
  )

  # Test mode transitions
  ament_add_gtest(test_mode_transitions_unit
    test/unit/test_mode_transitions.cpp
  )

  target_link_libraries(test_mode_transitions_unit
    unitree_interface_lib
  )

  # ========== Integration tests ==========
  # Test mode transitions
  ament_add_gtest(test_mode_transitions_integration
    test/integration/test_mode_transitions.cpp
  )

  target_link_libraries(test_mode_transitions_integration
    unitree_interface_lib
  )

  # Test the unitree interface
  ament_add_gtest(test_unitree_interface_integration
    test/integration/test_unitree_interface.cpp
  )

  target_link_libraries(test_unitree_interface_integration
    unitree_interface_lib
  )
endif()

ament_package()
