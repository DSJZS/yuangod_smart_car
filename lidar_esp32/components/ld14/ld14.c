#include "ld14.h"
#include "uart_events.h"

static void ld14_uart_config(void)
{
    uart_events_init();
}

void ld14_init(void)
{
    ld14_uart_config();
}
