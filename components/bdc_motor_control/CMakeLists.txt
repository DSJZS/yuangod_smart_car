idf_component_register(
    SRCS "bdc_motor_control.c"
    INCLUDE_DIRS "."
    REQUIRES    log bdc_motor pid_ctrl driver freertos esp_timer
)