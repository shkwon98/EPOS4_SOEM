# cmake version
cmake_minimum_required(VERSION 3.4)

# compile options
add_compile_options(-Wall -Wextra)

# project information
project(kistar)

# open-source packages
find_package(soem REQUIRED)
# find_package(depthai CONFIG REQUIRED)
# find_package(OpenCV REQUIRED)

# reference directories
add_subdirectory(kistar_control)
add_subdirectory(kistar_ethercat)
add_subdirectory(kistar_socket)
add_subdirectory(kistar_thread)
# add_subdirectory(RobotCamera)

# adding the executable files & linking libraries
add_executable(${PROJECT_NAME} kistar_main.cpp)
target_link_libraries(${PROJECT_NAME}
        kistar_ethercat
        kistar_control
        # RobotCamera
    )
