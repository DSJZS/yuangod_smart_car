idf_component_register(
    SRCS "campus_network.c"
    INCLUDE_DIRS "include"
    REQUIRES    esp_event log esp_wifi esp_common esp_http_client
)