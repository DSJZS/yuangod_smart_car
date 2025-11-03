#ifndef __ACTIVE_BUZZER_H__
#define __ACTIVE_BUZZER_H__

#include "driver/gpio.h"

typedef void *active_buzzer_handle_t;
typedef uint32_t active_buzzer_state_t;

active_buzzer_handle_t active_buzzer_create(uint64_t gpio_num);
void active_buzzer_delete(active_buzzer_handle_t active_buzzer);
void active_buzzer_on(active_buzzer_handle_t active_buzzer);
void active_buzzer_off(active_buzzer_handle_t active_buzzer);
void active_buzzer_toggle(active_buzzer_handle_t active_buzzer);
void active_buzzer_write(active_buzzer_handle_t active_buzzer, active_buzzer_state_t state);
active_buzzer_state_t active_buzzer_state(active_buzzer_handle_t active_buzzer);

#endif
