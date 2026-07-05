cmake_minimum_required(VERSION 3.24)

project(MocapSlave)

file(
    GLOB_RECURSE SOURCES CONFIGURE_DEPENDS
    "../../slave/*.cpp"
    "../../slave/*.hpp"
    "../../lib/*.cpp"
    "../../lib/*.hpp"
)

find_package(Threads REQUIRED)
find_package(spdlog REQUIRED)
find_package(OpenCV REQUIRED)

add_executable(${PROJECT_NAME} ${SOURCES})
include_directories(${OpenCV_INCLUDE_DIRS})

target_link_libraries(${PROJECT_NAME} PRIVATE zmq Threads::Threads spdlog::spdlog ${OpenCV_LIBS})