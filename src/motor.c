#include "motor.h"
#include "sensors.h"

#include <zephyr/kernel.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/pwm.h>

/* =========================
 * Devicetree
 * ========================= */
#define AIN_F_NODE DT_ALIAS(ain1_frente)
#define AIN_R_NODE DT_ALIAS(ain1_re)
#define BIN_F_NODE DT_ALIAS(bin1_frente)
#define BIN_R_NODE DT_ALIAS(bin1_re)
#define STBY_NODE  DT_ALIAS(stby)

#define PWMA_NODE  DT_ALIAS(pwm_a)
#define PWMB_NODE  DT_ALIAS(pwm_b)

static const struct gpio_dt_spec ain_f = GPIO_DT_SPEC_GET(AIN_F_NODE, gpios);
static const struct gpio_dt_spec ain_r = GPIO_DT_SPEC_GET(AIN_R_NODE, gpios);
static const struct gpio_dt_spec bin_f = GPIO_DT_SPEC_GET(BIN_F_NODE, gpios);
static const struct gpio_dt_spec bin_r = GPIO_DT_SPEC_GET(BIN_R_NODE, gpios);
static const struct gpio_dt_spec stby  = GPIO_DT_SPEC_GET(STBY_NODE, gpios);

static const struct pwm_dt_spec pwma = PWM_DT_SPEC_GET(PWMA_NODE);
static const struct pwm_dt_spec pwmb = PWM_DT_SPEC_GET(PWMB_NODE);

/* =========================
 * Motor thread
 * ========================= */
void motor_thread(void)
{
    while (1) {
        uint32_t ev = k_event_wait(&sensor_events,
            EVT_SP_FE | EVT_SP_FD | EVT_SP_LE | EVT_SP_LD,
            true, K_FOREVER);

        /* Exemplo simples */
        if ((ev & EVT_SP_FE) && (ev & EVT_SP_FD)) {
            gpio_pin_set_dt(&ain_f, 1);
            gpio_pin_set_dt(&ain_r, 0);
            gpio_pin_set_dt(&bin_f, 1);
            gpio_pin_set_dt(&bin_r, 0);
        } else {
            gpio_pin_set_dt(&ain_f, 0);
            gpio_pin_set_dt(&ain_r, 0);
            gpio_pin_set_dt(&bin_f, 0);
            gpio_pin_set_dt(&bin_r, 0);
        }

        if (ev & EVT_SP_LE) {
            gpio_pin_set_dt(&bin_f, 1);
            gpio_pin_set_dt(&bin_r, 0);
            gpio_pin_set_dt(&ain_r, 1);
            gpio_pin_set_dt(&ain_f, 0);
        } else {
            gpio_pin_set_dt(&ain_f, 0);
            gpio_pin_set_dt(&ain_r, 0);
            gpio_pin_set_dt(&bin_f, 0);
            gpio_pin_set_dt(&bin_r, 0);
        }

        if (ev & EVT_SP_LD) {
            gpio_pin_set_dt(&bin_f, 0);
            gpio_pin_set_dt(&bin_r, 1);
            gpio_pin_set_dt(&ain_r, 0);
            gpio_pin_set_dt(&ain_f, 1);
        } else {
            gpio_pin_set_dt(&ain_f, 0);
            gpio_pin_set_dt(&ain_r, 0);
            gpio_pin_set_dt(&bin_f, 0);
            gpio_pin_set_dt(&bin_r, 0);
        }
    }
}

K_THREAD_DEFINE(motor_tid, 1024,
                motor_thread, NULL, NULL, NULL,
                4, 0, 0);

/* =========================
 * Init
 * ========================= */
void motor_init(void)
{
    gpio_pin_configure_dt(&ain_f, GPIO_OUTPUT_INACTIVE);
    gpio_pin_configure_dt(&ain_r, GPIO_OUTPUT_INACTIVE);
    gpio_pin_configure_dt(&bin_f, GPIO_OUTPUT_INACTIVE);
    gpio_pin_configure_dt(&bin_r, GPIO_OUTPUT_INACTIVE);
    gpio_pin_configure_dt(&stby,  GPIO_OUTPUT_ACTIVE);

    pwm_set_dt(&pwma, PWM_USEC(250), PWM_USEC(125));
    pwm_set_dt(&pwmb, PWM_USEC(250), PWM_USEC(125));
}
