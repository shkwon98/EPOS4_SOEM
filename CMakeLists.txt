cmake_minimum_required(VERSION 2.8)
project(robot_hand)

set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -Wall -Wextra -Werror")
set(CMAKE_POSITION_INDEPENDENT_CODE true)
set(OS_LIBS pthread m)

# SOEM
include_directories(/usr/include/soem)

# Add the headers from the <include> directory
include_directories(include/${PROJECT_NAME})
include_directories(src/${PROJECT_NAME})

# Add the source files from the <src> directory
add_library(${PROJECT_NAME}_src
        src/${PROJECT_NAME}/servo_def.cpp
        src/${PROJECT_NAME}/tcpPacket.cpp
        src/${PROJECT_NAME}/udpPacket.cpp
        )

target_link_libraries(${PROJECT_NAME}_src soem ${OS_LIBS})

# Add the executable files
add_executable(${PROJECT_NAME}_simpletest src/${PROJECT_NAME}/main.cpp)
# Linking for libraries
target_link_libraries(${PROJECT_NAME}_simpletest
        ${PROJECT_NAME}_src
        soem
        ${OS_LIBS}
        )
