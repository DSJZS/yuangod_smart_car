idf_component_register(
    SRCS "softap_sta.c" "campus_network.c"
    INCLUDE_DIRS "include"
    REQUIRES    freertos esp_hw_support esp_wifi esp_event log esp_netif nvs_flash lwip esp_http_client json
)
