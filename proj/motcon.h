#ifndef __MOTCON_HEADER__
#define __MOTCON_HEADER__

#include "types.h"
#include "linesensor.h"

/********************************************
  Motion control
*/

#define MIN_VELOC 0.05
#define MAX_ACCEL 0.5

// #define PID_ANGLE_KP 0.050
// #define PID_ANGLE_KI 0.050
// #define PID_ANGLE_KD 0.005

#define PID_ANGLE_KP 0.30
#define PID_ANGLE_KI 0.20
#define PID_ANGLE_KD 0.08

#define PID_ANGLE_KR 1

#define DIST_LINESENSOR_FROM_CENTER 0.15

void update_motcon(motiontype *mot, odotype *odo, int *linesens_data);

int fwd(motiontype *mot, double dist, double speed, int time);
int turn(motiontype *mot, double angle, double speed, int time);
int followline(motiontype *mot, double dist, double speed, int time, int black_line, int line_to_follow);

double pid_angle(odotype *odo, double target);

#endif