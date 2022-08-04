# cmake version
cmake_minimum_required(VERSION 3.4)

# compile options
add_compile_options(-Wall -Wextra)

# project information
project(ex_robot)

# open-source packages
find_package(soem REQUIRED)
# find_package(depthai CONFIG REQUIRED)
# find_package(OpenCV REQUIRED)

# reference directories
add_subdirectory(ansur_control)
add_subdirectory(ansur_ethercat)
add_subdirectory(ansur_socket)
add_subdirectory(ansur_thread)
# add_subdirectory(RobotCamera)

# adding the executable files & linking libraries
add_executable(${PROJECT_NAME} main.cpp)
target_link_libraries(${PROJECT_NAME}
        ansur_ethercat
        ansur_control
        # RobotCamera
    )
