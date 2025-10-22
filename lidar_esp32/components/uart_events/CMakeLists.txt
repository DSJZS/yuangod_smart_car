idf_component_register(
    SRCS "uart_events.c"
    INCLUDE_DIRS "include"
    REQUIRES freertos driver log
)
