cmake_minimum_required(VERSION 3.24)

project(MocapServer)

file(
    GLOB_RECURSE SOURCES CONFIGURE_DEPENDS
    "../../server/*.cpp"
    "../../server/*.hpp"
    "../../lib/*.cpp"
    "../../lib/*.hpp"
)

find_package(Threads REQUIRED)
find_package(spdlog REQUIRED)

add_executable(${PROJECT_NAME} ${SOURCES})

target_link_libraries(${PROJECT_NAME} PRIVATE zmq Threads::Threads spdlog::spdlog)