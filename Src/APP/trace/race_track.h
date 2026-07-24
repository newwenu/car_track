#ifndef __RACE_TRACK_H
#define __RACE_TRACK_H

#include "stm32f10x.h"

void race_track_init(void);
void race_track_update(void);
void race_track_get_wheel_targets(int *left_pct, int *right_pct);
float race_track_get_error(void);
u8 race_track_is_lost(void);
u8 race_track_is_all_black(void);
u8 race_track_is_lost_timeout(void);

#endif
