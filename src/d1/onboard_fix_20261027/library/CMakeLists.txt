cmake_minimum_required(VERSION 3.5)
project(D1_FashionStar_Checked VERSION 1.0 LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

find_path(CSERIALPORT_INCLUDE_DIR CSerialPort/SerialPort.h HINTS /usr/local/include)
find_library(CSERIALPORT_LIBRARY NAMES cserialport HINTS /usr/local/lib)
if(NOT CSERIALPORT_INCLUDE_DIR OR NOT CSERIALPORT_LIBRARY)
    message(FATAL_ERROR "CSerialPort headers/library are required; specify CSERIALPORT_INCLUDE_DIR and CSERIALPORT_LIBRARY")
endif()

# Build-private headers match the controller's existing include spelling.
# There are deliberately no install rules in this patched build tree.
file(MAKE_DIRECTORY "${CMAKE_BINARY_DIR}/include/FashionStar/UServo")
foreach(header FashionStar_UartServo.h FashionStar_UartServoProtocol.h)
    configure_file("${CMAKE_CURRENT_SOURCE_DIR}/include/${header}"
                   "${CMAKE_BINARY_DIR}/include/FashionStar/UServo/${header}" COPYONLY)
endforeach()
add_subdirectory(src)
