cmake_minimum_required(VERSION 3.24)

project(MocapSlave)

file(
    GLOB_RECURSE SOURCES CONFIGURE_DEPENDS
    "../../slave/src/*.cpp"
    "../../slave/header/*.hpp"
    "../../lib/src/*.cpp"
    "../../lib/header/*.hpp"
)

find_package(Threads REQUIRED)
find_package(OpenCV REQUIRED)

add_executable(${PROJECT_NAME} ${SOURCES})

target_include_directories(${PROJECT_NAME} PRIVATE ${OpenCV_INCLUDE_DIRS})

target_link_libraries(${PROJECT_NAME} PRIVATE 
    Threads::Threads 
    ${OpenCV_LIBS}
)