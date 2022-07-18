# cmake version
cmake_minimum_required(VERSION 3.0.2)

# compile options
add_compile_options(-Wall -Wextra)

# project information
project(ex_robot)

# SOEM package
find_package(soem REQUIRED)

# include
include_directories(RobotHand/include)
include_directories(RobotHand/src)
# include_directories(RobotCamera/include)
# include_directories(RobotCamera/src)

# reference directory
add_subdirectory(RobotHand)
# add_subdirectory(RobotCamera)



# adding the executable file & linking library
add_executable(${PROJECT_NAME} main.cpp)
target_link_libraries(${PROJECT_NAME}
    PUBLIC
        RobotHand
    )
