# /ansur_thread/CMakeLists.txt
project(ansur_thread)

# static library
file(GLOB_RECURSE SRC_FILES CONFIGURE_DEPENDS
    src/*.cpp
    )
add_library(${PROJECT_NAME}
    ${SRC_FILES}
    )

target_include_directories(${PROJECT_NAME} PUBLIC include)