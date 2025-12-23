#include "tasks_sensor.h"
#include "app_shared.h"
#include "rtos.h"
#include <math.h>

static float pseudo_noise(uint64_t t)
{
    /* deterministic pseudo noise */
    return (float)(sin((double)t * 0.001) * 0.05);
}

void Task_Sensor(void* arg)
{
    (void)arg;
    float alt = 300.0f;
    float airspeed = 0.0f;

    while (1) {
        uint64_t t_ms = rtos_tick_to_ms(rtos_tick_get());

        /* simple scenario: airspeed ramps up to 25 m/s then holds */
        if (airspeed < 25.0f) airspeed += 0.5f;

        /* altitude holds ~300m with tiny noise */
        alt += pseudo_noise(t_ms) * 0.1f;

        sensor_sample_t s = {
            .t_ms = t_ms,
            .ax = 0.0f + pseudo_noise(t_ms),
            .ay = 0.0f + pseudo_noise(t_ms + 7),
            .az = -9.81f + pseudo_noise(t_ms + 13),
            .gx = 0.001f + pseudo_noise(t_ms + 3),
            .gy = 0.002f + pseudo_noise(t_ms + 5),
            .gz = 0.003f + pseudo_noise(t_ms + 11),
            .airspeed_mps = airspeed,
            .alt_m = alt
        };

        (void)rtos_queue_send(&g_q_sensor, &s, 50);
        rtos_event_flags_set(&g_events, EV_SENSOR_READY);

        rtos_delay_ms(200);
    }
}
