cmake_minimum_required(VERSION 3.0.0)

if(APPLE)
    find_library(IOKIT_LIBRARY IOKit)
    find_library(FOUNDATION_LIBRARY Foundation)
endif()

add_library(serial SHARED)
# add_library(serial::lib ALIAS serial)

## Sources
set(serial_SRCS
    src/serial.cc
)

if(APPLE)
    # If OSX
    list(APPEND serial_SRCS src/impl/unix.cc)
    list(APPEND serial_SRCS src/impl/list_ports/list_ports_osx.cc)
elseif(UNIX)
    # If unix
    list(APPEND serial_SRCS src/impl/unix.cc)
    list(APPEND serial_SRCS src/impl/list_ports/list_ports_linux.cc)
else()
    # If windows
    list(APPEND serial_SRCS src/impl/win.cc)
    list(APPEND serial_SRCS src/impl/list_ports/list_ports_win.cc)
endif()

#add_library(serial SHARED ${serial_SRCS})

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

## Add serial library
if(APPLE)
    target_link_libraries(serial ${FOUNDATION_LIBRARY} ${IOKIT_LIBRARY})
elseif(UNIX)
    target_link_libraries(serial rt pthread)
else()
    target_link_libraries(serial setupapi)
endif()

install()

