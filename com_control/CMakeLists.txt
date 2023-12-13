include_directories(include)
link_directories(lib)

add_compile_options(-O2)

add_library(com_control SHARED
    src/controller.cpp
    src/listener.cpp
    src/message/base.cpp
    src/message/request.cpp
    src/message/response.cpp
)

target_link_libraries(com_control cserialport)
