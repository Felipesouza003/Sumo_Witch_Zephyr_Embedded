#include <zephyr/kernel.h>

#include "sensors.h"
#include "leds.h"
#include "motor.h"

int main(void)
{
    sensors_init();
    leds_init();
    motor_init();

    return 0;
}
