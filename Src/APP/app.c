#include "app.h"
#include "delay.h"
#include "dashboard/ui_dashboard.h"
#include "vehicle/vehicle_state.h"
#include "motion/motion_control.h"

static u8 s_vehicle_div = 0;

void app_init(void)
{
    vehicle_init();
    motion_init();
    ui_init();
}

void app_update(void)
{
    s_vehicle_div++;
    if (s_vehicle_div >= (VEHICLE_SAMPLE_MS / UI_TASK_PERIOD_MS))
    {
        s_vehicle_div = 0;
        vehicle_update();
        ui_set_speed(vehicle_get_speed_cm_s());
        ui_set_mileage(vehicle_get_distance_cm());
    }

    // motion_update();
    ui_task();
    delay_ms(APP_TASK_PERIOD_MS);
}
