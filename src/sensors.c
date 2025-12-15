#include "sensors.h"

#include <zephyr/device.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/sys/util.h>

/* =========================
 * Event group
 * ========================= */
struct k_event sensor_events;

/* =========================
 * Devicetree
 * ========================= */
#define SP_FE_NODE DT_ALIAS(sp_fe)
#define SP_FD_NODE DT_ALIAS(sp_fd)
#define SP_LE_NODE DT_ALIAS(sp_le)
#define SP_LD_NODE DT_ALIAS(sp_ld)

BUILD_ASSERT(DT_NODE_HAS_STATUS(SP_FE_NODE, okay));
BUILD_ASSERT(DT_NODE_HAS_STATUS(SP_FD_NODE, okay));
BUILD_ASSERT(DT_NODE_HAS_STATUS(SP_LE_NODE, okay));
BUILD_ASSERT(DT_NODE_HAS_STATUS(SP_LD_NODE, okay));

/* =========================
 * GPIO specs
 * ========================= */
static const struct gpio_dt_spec sp_fe = GPIO_DT_SPEC_GET(SP_FE_NODE, gpios);
static const struct gpio_dt_spec sp_fd = GPIO_DT_SPEC_GET(SP_FD_NODE, gpios);
static const struct gpio_dt_spec sp_le = GPIO_DT_SPEC_GET(SP_LE_NODE, gpios);
static const struct gpio_dt_spec sp_ld = GPIO_DT_SPEC_GET(SP_LD_NODE, gpios);

/* =========================
 * Callbacks
 * ========================= */
static struct gpio_callback cb_sp_fe;
static struct gpio_callback cb_sp_fd;
static struct gpio_callback cb_sp_le;
static struct gpio_callback cb_sp_ld;

/* =========================
 * ISR
 * ========================= */
static void sensor_isr(const struct device *dev,
                       struct gpio_callback *cb,
                       uint32_t pins)
{
    if (cb == &cb_sp_fe) {
        k_event_set(&sensor_events, EVT_SP_FE);
    } else if (cb == &cb_sp_fd) {
        k_event_set(&sensor_events, EVT_SP_FD);
    } else if (cb == &cb_sp_le) {
        k_event_set(&sensor_events, EVT_SP_LE);
    } else if (cb == &cb_sp_ld) {
        k_event_set(&sensor_events, EVT_SP_LD);
    }
}

/* =========================
 * Init
 * ========================= */
void sensors_init(void)
{
    k_event_init(&sensor_events);

    gpio_pin_configure_dt(&sp_fe, GPIO_INPUT);
    gpio_pin_configure_dt(&sp_fd, GPIO_INPUT);
    gpio_pin_configure_dt(&sp_le, GPIO_INPUT);
    gpio_pin_configure_dt(&sp_ld, GPIO_INPUT);

    gpio_pin_interrupt_configure_dt(&sp_fe, GPIO_INT_EDGE_BOTH);
    gpio_pin_interrupt_configure_dt(&sp_fd, GPIO_INT_EDGE_BOTH);
    gpio_pin_interrupt_configure_dt(&sp_le, GPIO_INT_EDGE_BOTH);
    gpio_pin_interrupt_configure_dt(&sp_ld, GPIO_INT_EDGE_BOTH);

    gpio_init_callback(&cb_sp_fe, sensor_isr, BIT(sp_fe.pin));
    gpio_init_callback(&cb_sp_fd, sensor_isr, BIT(sp_fd.pin));
    gpio_init_callback(&cb_sp_le, sensor_isr, BIT(sp_le.pin));
    gpio_init_callback(&cb_sp_ld, sensor_isr, BIT(sp_ld.pin));

    gpio_add_callback(sp_fe.port, &cb_sp_fe);
    gpio_add_callback(sp_fd.port, &cb_sp_fd);
    gpio_add_callback(sp_le.port, &cb_sp_le);
    gpio_add_callback(sp_ld.port, &cb_sp_ld);
}
