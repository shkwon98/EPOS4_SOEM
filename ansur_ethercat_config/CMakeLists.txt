# /ansur_ethercat_config/CMakeLists.txt
project(ethercat_config)

set(OS_LIBS pthread m)

# static library
file(GLOB_RECURSE SRC_FILES CONFIGURE_DEPENDS
    src/*.cpp
    )
add_library(${PROJECT_NAME}
    ${SRC_FILES}
    )

target_include_directories(${PROJECT_NAME} PUBLIC include)

target_link_libraries(${PROJECT_NAME}
    ${OS_LIBS}
    soem
    socket_config
    )
