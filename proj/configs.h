#ifndef __CONFIGS_HEADER__
#define __CONFIGS_HEADER__

#include "types.h"

#include <math.h>

#define SQUARE_L 3.0

StateParam conf_squareccw[] = {
  { .state = ms_fwd, .speed = 0.3, .dist = SQUARE_L },
  { .state = ms_turn, .speed = 0.3, .angle = 90.0 / 180.0 * M_PI },
  { .state = ms_fwd, .speed = 0.3, .dist = SQUARE_L },
  { .state = ms_turn, .speed = 0.3, .angle = 90.0 / 180.0 * M_PI },
  { .state = ms_fwd, .speed = 0.3, .dist = SQUARE_L },
  { .state = ms_turn, .speed = 0.3, .angle = 90.0 / 180.0 * M_PI },
  { .state = ms_fwd, .speed = 0.3, .dist = SQUARE_L },
  { .state = ms_turn, .speed = 0.3, .angle = 90.0 / 180.0 * M_PI }
};

StateParam conf_squarecw[] = {
  { .state = ms_fwd, .speed = 0.4, .dist = SQUARE_L },
  { .state = ms_turn, .speed = 0.2, .angle = -90.0 / 180.0 * M_PI },
  { .state = ms_fwd, .speed = 0.4, .dist = SQUARE_L },
  { .state = ms_turn, .speed = 0.2, .angle = -90.0 / 180.0 * M_PI },
  { .state = ms_fwd, .speed = 0.4, .dist = SQUARE_L },
  { .state = ms_turn, .speed = 0.2, .angle = -90.0 / 180.0 * M_PI },
  { .state = ms_fwd, .speed = 0.4, .dist = SQUARE_L },
  { .state = ms_turn, .speed = 0.2, .angle = -90.0 / 180.0 * M_PI }
};

StateParam conf_turntest[] = {
  { .state = ms_turn, .speed = 0.3, .angle = 90.0 / 180.0 * M_PI },
  { .state = ms_turn, .speed = 0.3, .angle = -90.0 / 180.0 * M_PI },
  { .state = ms_turn, .speed = -0.3, .angle = 90.0 / 180.0 * M_PI },
  { .state = ms_turn, .speed = -0.3, .angle = -90.0 / 180.0 * M_PI }
};

/*
StateParam conf_followbm[] = {
  { .state = ms_followline, .speed = 0.35, .dist = 5.5, .is_black = 1, .line_to_follow = LINE_MIDDLE,
    .p_stop = p_line_cross }
};

StateParam conf_followwm[] = {
  { .state = ms_followline, .speed = 0.2, .dist = 3.5, .is_black = 0, .line_to_follow = LINE_LEFT, .turning_intensity = 0.5, 
    .p_stop = p_line_cross }
};
*/
int p_time(PredicateData dat) {
  printf("vals: %d %d %d %d %d %d %d %d\n", dat.lindat->raw_dat[0], dat.lindat->raw_dat[1], dat.lindat->raw_dat[2], dat.lindat->raw_dat[3], dat.lindat->raw_dat[4], dat.lindat->raw_dat[5], dat.lindat->raw_dat[6], dat.lindat->raw_dat[7]);
  return (dat.odo->time_curr - dat.odo->time_start > 2.0);
}
StateParam conf_calib[] = {
    { .state = ms_followline, .speed = 0.3, .dist = 10.0, .is_black = 0, .line_to_follow = LINE_MIDDLE,
    .p_stop = &p_time},
};




#endif