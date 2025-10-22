idf_component_register(
    SRCS "wifi_station.c"
    INCLUDE_DIRS "include"
    REQUIRES    freertos esp_system esp_wifi esp_event log nvs_flash lwip
)
