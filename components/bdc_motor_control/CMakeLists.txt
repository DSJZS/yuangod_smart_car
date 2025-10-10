idf_component_register(
    SRCS "bdc_motor_control.c"
    INCLUDE_DIRS "." "include"
    REQUIRES    log bdc_motor pid_ctrl driver freertos esp_timer
)