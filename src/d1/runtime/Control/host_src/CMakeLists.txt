cmake_minimum_required(VERSION 3.16)
project(d1_arm_control LANGUAGES CXX)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
if(NOT CMAKE_BUILD_TYPE AND NOT CMAKE_CONFIGURATION_TYPES)
    set(CMAKE_BUILD_TYPE RelWithDebInfo CACHE STRING "Build type" FORCE)
endif()

find_package(unitree_sdk2 CONFIG REQUIRED)
find_package(Boost 1.75 REQUIRED)
add_library(d1_protocol src/protocol.cpp)
target_include_directories(d1_protocol PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/src)
target_include_directories(d1_protocol SYSTEM PUBLIC ${Boost_INCLUDE_DIRS})
target_compile_options(d1_protocol PRIVATE -Wall -Wextra -Wpedantic)

add_executable(arm_control arm_control.cpp src/controller.cpp
    msg/ArmString_.cpp msg/PubServoInfo_.cpp)
target_include_directories(arm_control PRIVATE ${CMAKE_CURRENT_SOURCE_DIR})
target_link_libraries(arm_control PRIVATE d1_protocol unitree_sdk2 rt)
target_compile_options(arm_control PRIVATE -Wall -Wextra -Wpedantic)

# Transport-only timed trajectory bridge. Boost JSON definitions already live
# in d1_protocol; do not include boost/json/src.hpp in this executable.
add_executable(d1_timed_bridge timed_trajectory_bridge.cpp msg/ArmString_.cpp)
target_include_directories(d1_timed_bridge PRIVATE ${CMAKE_CURRENT_SOURCE_DIR})
target_link_libraries(d1_timed_bridge PRIVATE d1_protocol unitree_sdk2 rt pthread)
target_compile_options(d1_timed_bridge PRIVATE -Wall -Wextra -Wpedantic)

include(CTest)
if(BUILD_TESTING)
    add_executable(protocol_tests tests/protocol_tests.cpp)
    target_link_libraries(protocol_tests PRIVATE d1_protocol)
    add_test(NAME protocol COMMAND protocol_tests)
    add_executable(feedback_filter_tests tests/feedback_filter_tests.cpp)
    target_link_libraries(feedback_filter_tests PRIVATE d1_protocol)
    add_test(NAME feedback_filter COMMAND feedback_filter_tests)
    find_package(Python3 COMPONENTS Interpreter REQUIRED)
    add_test(NAME terminal_cli COMMAND ${Python3_EXECUTABLE}
        ${CMAKE_CURRENT_SOURCE_DIR}/tests/test_cli.py $<TARGET_FILE:arm_control>)
endif()

# 显式构建/执行的 loopback 集成测试；常规 ctest 完全离线。
if(BUILD_TESTING)
    add_executable(dds_test_peer EXCLUDE_FROM_ALL tests/dds_peer.cpp msg/ArmString_.cpp)
    target_include_directories(dds_test_peer PRIVATE ${CMAKE_CURRENT_SOURCE_DIR})
    target_link_libraries(dds_test_peer PRIVATE d1_protocol unitree_sdk2 rt)
    add_custom_target(dds_integration_test
        COMMAND ${Python3_EXECUTABLE} ${CMAKE_CURRENT_SOURCE_DIR}/tests/test_dds.py
            $<TARGET_FILE:arm_control> $<TARGET_FILE:dds_test_peer>
        DEPENDS arm_control dds_test_peer
        USES_TERMINAL)
endif()

if(BUILD_TESTING)
    add_custom_target(position_bootstrap_integration_test
        COMMAND ${Python3_EXECUTABLE} ${CMAKE_CURRENT_SOURCE_DIR}/tests/test_bootstrap_dds.py
            $<TARGET_FILE:arm_control> $<TARGET_FILE:dds_test_peer>
        DEPENDS arm_control dds_test_peer
        USES_TERMINAL)
endif()

if(BUILD_TESTING)
    add_custom_target(feedback_jitter_integration_test
        COMMAND ${Python3_EXECUTABLE} ${CMAKE_CURRENT_SOURCE_DIR}/tests/test_feedback_jitter_dds.py
            $<TARGET_FILE:arm_control> $<TARGET_FILE:dds_test_peer>
        DEPENDS arm_control dds_test_peer
        USES_TERMINAL)
endif()
