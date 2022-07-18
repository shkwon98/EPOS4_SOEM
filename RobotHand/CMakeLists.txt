# /RobotHand/CMakeLists.txt

set(OS_LIBS pthread m)

# static library
file(GLOB_RECURSE SRC_FILES CONFIGURE_DEPENDS
    src/*.cpp
    )
add_library(RobotHand
    ${SRC_FILES}
    )
target_link_libraries(RobotHand
    ${OS_LIBS}
    soem
    )
