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

add_executable(${PROJECT_NAME} ${SOURCES})

target_link_libraries(${PROJECT_NAME} PRIVATE zmq Threads::Threads spdlog::spdlog)