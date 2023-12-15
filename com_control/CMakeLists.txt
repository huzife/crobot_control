find_package(CSerialPort REQUIRED)

include_directories(include ${CSerialPort_INCLUDE_DIR})

add_compile_options(-O2)

add_library(com_control SHARED
    src/controller.cpp
    src/listener.cpp
    src/message/request.cpp
    src/message/response.cpp
    src/message/utils.cpp
)

target_link_libraries(com_control ${CSerialPort_LIBRARY})
