# /RobotCamera/CMakeLists.txt

# static library
file(GLOB_RECURSE SRC_FILES CONFIGURE_DEPENDS
    src/*.cpp
    )
add_library(RobotCamera
    ${SRC_FILES}
    )
target_link_libraries(RobotCamera
    depthai::opencv
    ${OpenCV_LIBS}
    )
