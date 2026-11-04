add_library(fsuartservo SHARED FashionStar_UartServoProtocol.cpp FashionStar_UartServo.cpp)
set_target_properties(fsuartservo PROPERTIES LIBRARY_OUTPUT_DIRECTORY "${PROJECT_BINARY_DIR}/lib")
target_include_directories(fsuartservo PRIVATE "${PROJECT_SOURCE_DIR}/include" "${CSERIALPORT_INCLUDE_DIR}")
target_link_libraries(fsuartservo PRIVATE "${CSERIALPORT_LIBRARY}")
