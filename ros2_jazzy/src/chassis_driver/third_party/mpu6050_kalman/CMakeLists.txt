cmake_minimum_required(VERSION 3.22)


# Library core sources
set(KALMAN_FILTER_SRCS
    ${CMAKE_CURRENT_LIST_DIR}/src/mpu6050_kalman.c
)

# Setup include directories
set(KALMAN_FILTER_INCLUDE_DIRS
    ${CMAKE_CURRENT_LIST_DIR}/inc
)

# Register library to the system
add_library(kalman_filter)
target_sources(kalman_filter PRIVATE ${KALMAN_FILTER_SRCS})
target_include_directories(kalman_filter PUBLIC ${KALMAN_FILTER_INCLUDE_DIRS})
target_compile_options(kalman_filter PRIVATE ${KALMAN_FILTER_COMPILE_OPTIONS})
target_compile_definitions(kalman_filter PRIVATE ${KALMAN_FILTER_COMPILE_DEFINITIONS})