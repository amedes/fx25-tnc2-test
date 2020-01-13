#include <driver/gpio.h>

#define GPIO_CDT_LED GPIO_NUM_19

void gpio_init(void)
{
    gpio_set_direction(GPIO_CDT_LED, GPIO_MODE_OUTPUT);
}

void led_cdt(int on)
{
    gpio_set_level(GPIO_CDT_LED, on);
}
