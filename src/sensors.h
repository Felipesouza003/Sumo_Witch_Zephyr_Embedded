#pragma once
#include <zephyr/kernel.h>

/* Eventos dos sensores */
#define EVT_SP_FE BIT(0)
#define EVT_SP_FD BIT(1)
#define EVT_SP_LE BIT(2)
#define EVT_SP_LD BIT(3)

/* Event group global */
extern struct k_event sensor_events;

void sensors_init(void);
