cmake_minimum_required(VERSION 3.22)


# Library core sources
set(MAHONY_FILTER_SRCS
    ${CMAKE_CURRENT_LIST_DIR}/src/mahony_filter.cpp
)

# Setup include directories
set(MAHONY_FILTER_INCLUDE_DIRS
    ${CMAKE_CURRENT_LIST_DIR}/inc
)

# Register library to the system
add_library(mahony_filter)
target_sources(mahony_filter PRIVATE ${MAHONY_FILTER_SRCS})
target_include_directories(mahony_filter PUBLIC ${MAHONY_FILTER_INCLUDE_DIRS})
target_compile_options(mahony_filter PRIVATE ${MAHONY_FILTER_COMPILE_OPTIONS})
target_compile_definitions(mahony_filter PRIVATE ${MAHONY_FILTER_COMPILE_DEFINITIONS})