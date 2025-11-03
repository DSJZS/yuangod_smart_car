#include "include/active_buzzer.h"

typedef struct {
    uint64_t gpio_num;
    uint32_t state;
}active_buzzer_dev_t;

active_buzzer_handle_t active_buzzer_create(uint64_t gpio_num)
{
    active_buzzer_dev_t *dev = (active_buzzer_dev_t *) calloc(1, sizeof(active_buzzer_dev_t));
    gpio_config_t config = {
        .pin_bit_mask = ( 1 << gpio_num ),
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE
    };
    gpio_config( &config);
    gpio_set_level( gpio_num, 0);

    dev->gpio_num = gpio_num;
    dev->state = 0;

    return (active_buzzer_handle_t)dev;
}

void active_buzzer_delete(active_buzzer_handle_t active_buzzer)
{
    active_buzzer_dev_t *dev = (active_buzzer_dev_t *) active_buzzer;
    free(dev);
}

void active_buzzer_on(active_buzzer_handle_t active_buzzer)
{
    active_buzzer_dev_t *dev = (active_buzzer_dev_t *) active_buzzer;
    dev->state = 1;
    gpio_set_level( dev->gpio_num, dev->state);
}

void active_buzzer_off(active_buzzer_handle_t active_buzzer)
{
    active_buzzer_dev_t *dev = (active_buzzer_dev_t *) active_buzzer;
    dev->state = 0;
    gpio_set_level( dev->gpio_num, dev->state);
}

void active_buzzer_toggle(active_buzzer_handle_t active_buzzer)
{
    active_buzzer_dev_t *dev = (active_buzzer_dev_t *) active_buzzer;
    dev->state = !(dev->state);
    gpio_set_level( dev->gpio_num, dev->state);
}

void active_buzzer_write(active_buzzer_handle_t active_buzzer, active_buzzer_state_t state)
{
    if( state != 0 )
        active_buzzer_on(active_buzzer);
    else
        active_buzzer_off(active_buzzer);
}

active_buzzer_state_t active_buzzer_state(active_buzzer_handle_t active_buzzer)
{
    active_buzzer_dev_t *dev = (active_buzzer_dev_t *) active_buzzer;
    return dev->state;
}
