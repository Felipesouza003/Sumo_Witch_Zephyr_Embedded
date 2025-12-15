#include "leds.h"
#include "sensors.h"

#include <zephyr/kernel.h>
#include <zephyr/drivers/gpio.h>

/* =========================
 * Devicetree
 * ========================= */
#define LED1_NODE DT_ALIAS(led1)
#define LED2_NODE DT_ALIAS(led2)
#define LED3_NODE DT_ALIAS(led3)
#define LED4_NODE DT_ALIAS(led4)
#define LED5_NODE DT_ALIAS(led5)

static const struct gpio_dt_spec led1 = GPIO_DT_SPEC_GET(LED1_NODE, gpios);
static const struct gpio_dt_spec led2 = GPIO_DT_SPEC_GET(LED2_NODE, gpios);
static const struct gpio_dt_spec led3 = GPIO_DT_SPEC_GET(LED3_NODE, gpios);
static const struct gpio_dt_spec led4 = GPIO_DT_SPEC_GET(LED4_NODE, gpios);
static const struct gpio_dt_spec led5 = GPIO_DT_SPEC_GET(LED5_NODE, gpios);

/* =========================
 * Blink thread
 * ========================= */
void blink_thread(void)
{
    while (1) {
        gpio_pin_toggle_dt(&led1);
        k_sleep(K_MSEC(500));
    }
}

K_THREAD_DEFINE(blink_tid, 512,
                blink_thread, NULL, NULL, NULL,
                5, 0, 0);

/* =========================
 * Sensor LED thread
 * ========================= */
void leds_thread(void)
{
    while (1) {
        uint32_t ev = k_event_wait(&sensor_events,
            EVT_SP_FE | EVT_SP_FD | EVT_SP_LE | EVT_SP_LD,
            true, K_FOREVER);

        if (ev & EVT_SP_FE) gpio_pin_set_dt(&led2, 1);
        else gpio_pin_set_dt(&led2, 0);

        if (ev & EVT_SP_FD) gpio_pin_set_dt(&led3, 1);
        else gpio_pin_set_dt(&led3, 0);

        if (ev & EVT_SP_LE) gpio_pin_set_dt(&led4, 1);
        else gpio_pin_set_dt(&led4, 0);

        if (ev & EVT_SP_LD) gpio_pin_set_dt(&led5, 1);
        else gpio_pin_set_dt(&led5, 0);

    }
}

K_THREAD_DEFINE(leds_tid, 768,
                leds_thread, NULL, NULL, NULL,
                6, 0, 0);

/* =========================
 * Init
 * ========================= */
void leds_init(void)
{
    gpio_pin_configure_dt(&led1, GPIO_OUTPUT_INACTIVE);
    gpio_pin_configure_dt(&led2, GPIO_OUTPUT_INACTIVE);
    gpio_pin_configure_dt(&led3, GPIO_OUTPUT_INACTIVE);
    gpio_pin_configure_dt(&led4, GPIO_OUTPUT_INACTIVE);
    gpio_pin_configure_dt(&led5, GPIO_OUTPUT_INACTIVE);
}
