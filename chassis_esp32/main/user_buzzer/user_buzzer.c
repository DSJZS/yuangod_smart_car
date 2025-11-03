#include "user_buzzer.h"
#include "active_buzzer.h"

static active_buzzer_handle_t buzzer = NULL;

void buzzer_init(void)
{
    if( buzzer == NULL ) {
        buzzer = active_buzzer_create(GPIO_NUM_45);
    }
}

void buzzer_on(void)
{
    active_buzzer_on( buzzer);
}

void buzzer_off(void)
{
    active_buzzer_off( buzzer);
}