cmake_minimum_required(VERSION 3.0.0)

if(APPLE)
    find_library(IOKIT_LIBRARY IOKit)
    find_library(FOUNDATION_LIBRARY Foundation)
endif()

add_library(serial SHARED)

## Sources
set(serial_SRCS
    src/serial.cc
)

if(APPLE)
    # OSX
    list(APPEND serial_SRCS src/impl/unix.cc)
    list(APPEND serial_SRCS src/impl/list_ports/list_ports_osx.cc)
elseif(UNIX)
    # Linux
    list(APPEND serial_SRCS src/impl/unix.cc)
    list(APPEND serial_SRCS src/impl/list_ports/list_ports_linux.cc)
else()
    # Windows
    list(APPEND serial_SRCS src/impl/win.cc)
    list(APPEND serial_SRCS src/impl/list_ports/list_ports_win.cc)
endif()

target_sources(serial 
    PRIVATE
    ${serial_SRCS}
)

target_include_directories(serial
    PRIVATE
    ${CMAKE_CURRENT_LIST_DIR}/include
)

include_directories(
    ${CMAKE_CURRENT_LIST_DIR}/include
)

target_include_directories(serial
    PUBLIC
    include
)

## Link system libs
if(APPLE)
    target_link_libraries(serial ${FOUNDATION_LIBRARY} ${IOKIT_LIBRARY})
elseif(UNIX)
    target_link_libraries(serial rt pthread)
else()
    target_link_libraries(serial setupapi)
endif()

# 安装库文件
install(TARGETS serial
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION bin
)

# 安装头文件
install(DIRECTORY include/
  DESTINATION include
)
