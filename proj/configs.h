#ifndef __CONFIGS_HEADER__
#define __CONFIGS_HEADER__

#include "types.h"

#include <math.h>

typedef struct {
  motiontype* mot;
  odotype* odo;
  double* laserpar;
} PredicateData;

typedef struct {
  int state; // mission state enum
  double speed;
  double dist;
  double angle;
  int is_black;
  int line_to_follow; // line to follow enum
  int (*p_stop)(PredicateData);
} StateParam;

StateParam conf_square[] = {
  { .state = ms_fwd, .speed = 0.3, .dist = 2.0 },
  { .state = ms_turn, .speed = 0.3, .angle = 90.0 / 180.0 * M_PI },
  { .state = ms_fwd, .speed = 0.3, .dist = 2.0 },
  { .state = ms_turn, .speed = 0.3, .angle = 90.0 / 180.0 * M_PI },
  { .state = ms_fwd, .speed = 0.3, .dist = 2.0 },
  { .state = ms_turn, .speed = 0.3, .angle = 90.0 / 180.0 * M_PI },
  { .state = ms_fwd, .speed = 0.3, .dist = 2.0 },
  { .state = ms_turn, .speed = 0.3, .angle = 90.0 / 180.0 * M_PI }
};

StateParam conf_followbm[] = {
  { .state = ms_turn, .speed = 0.3, .angle = 2.0 * M_PI },
  { .state = ms_followline, .speed = 0.3, .dist = 5.0, .is_black = 1, .line_to_follow = LINE_MIDDLE }
};

int p_stoponcross(PredicateData dat) {
  return dat.odo->crossing_line;
}
StateParam conf_stoponcross[] = {
  { .state = ms_fwd, .speed = 0.3, .dist = 0.1},
  { .state = ms_followline, .speed = 0.3, .dist = 10.0, .is_black = 1, .line_to_follow = LINE_RIGHT, 
    .p_stop = &p_stoponcross }
};

#endif