cmake_minimum_required(VERSION 3.22)


# Library core sources
set(SIMPLE_FRAME_SRCS
    ${CMAKE_CURRENT_LIST_DIR}/src/simple_frame.cpp
)

# Setup include directories
set(SIMPLE_FRAME_INCLUDE_DIRS
    ${CMAKE_CURRENT_LIST_DIR}/inc

    # 引用第三方库lwrb
    ${CMAKE_CURRENT_LIST_DIR}/../lwrb/src/include
)

# Register library to the system
add_library(simple_frame)
target_sources(simple_frame PRIVATE ${SIMPLE_FRAME_SRCS})
target_include_directories(simple_frame PUBLIC ${SIMPLE_FRAME_INCLUDE_DIRS})
target_compile_options(simple_frame PRIVATE ${SIMPLE_FRAME_COMPILE_OPTIONS})
target_compile_definitions(simple_frame PRIVATE ${SIMPLE_FRAME_COMPILE_DEFINITIONS})