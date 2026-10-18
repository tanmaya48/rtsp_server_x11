cmake_minimum_required(VERSION 3.10)
project(RtspScreenshare CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()

option(ENABLE_SCREEN_CAPTURE "Enable X11 screen capture (--source screen)" ON)

find_package(OpenCV REQUIRED)
find_package(Threads REQUIRED)

add_executable(rtsp_server rtsp_server.cpp)

# screen_capture.hpp lives next to rtsp_server.cpp
target_include_directories(rtsp_server PRIVATE
  ${OpenCV_INCLUDE_DIRS}
  ${CMAKE_CURRENT_SOURCE_DIR})

target_link_libraries(rtsp_server PRIVATE
  ${OpenCV_LIBS}
  Threads::Threads)

target_compile_options(rtsp_server PRIVATE -Wall -Wextra)

if(ENABLE_SCREEN_CAPTURE)
  find_package(X11 REQUIRED)
  target_compile_definitions(rtsp_server PRIVATE WITH_X11)
  target_include_directories(rtsp_server PRIVATE ${X11_INCLUDE_DIR})
  target_link_libraries(rtsp_server PRIVATE ${X11_LIBRARIES})
endif()
