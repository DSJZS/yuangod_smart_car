# Set source files and include directories
set(srcs)
set(include_dirs "include")

if( CONFIG_USE_CAMPUS_NETWORK )
    list(APPEND srcs "campus_network.c" )
endif()

idf_component_register(
    SRCS "${srcs}"
    INCLUDE_DIRS "${include_dirs}"
    REQUIRES    esp_event log esp_wifi esp_common esp_http_client
)