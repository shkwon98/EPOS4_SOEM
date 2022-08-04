# /ansur_socket/CMakeLists.txt
project(ansur_socket)

# static library
file(GLOB_RECURSE SRC_FILES CONFIGURE_DEPENDS
    src/*.cpp
    )
add_library(${PROJECT_NAME}
    ${SRC_FILES}
    )

target_include_directories(${PROJECT_NAME} PUBLIC include)

target_link_libraries(${PROJECT_NAME}
    soem
    )