cmake_minimum_required(VERSION 3.22)


# Library core sources
file(GLOB LD_LIDAR_DRIVER_SRCS 
    ${CMAKE_CURRENT_LIST_DIR}/src/*.cpp
)

# Setup include directories
set(LD_LIDAR_DRIVER_INCLUDE_DIRS
    ${CMAKE_CURRENT_LIST_DIR}/include
)

# Register library to the system
add_library(ld_lidar_driver)
target_sources(ld_lidar_driver PRIVATE ${LD_LIDAR_DRIVER_SRCS})
target_include_directories(ld_lidar_driver PUBLIC ${LD_LIDAR_DRIVER_INCLUDE_DIRS})
target_compile_options(ld_lidar_driver PRIVATE ${LD_LIDAR_DRIVER_COMPILE_OPTIONS})
target_compile_definitions(ld_lidar_driver PRIVATE ${LD_LIDAR_DRIVER_COMPILE_DEFINITIONS})