#ifndef __MOTCON_HEADER__
#define __MOTCON_HEADER__

#include "types.h"
#include "linesensor.h"

/********************************************
  Motion control
*/

#define MIN_VELOC 0.05
#define MAX_ACCEL 0.5
#define REFRESH_RATE 20.0
#define SPEED_INCREMENT (MAX_ACCEL / REFRESH_RATE) // 0.5m/s^2 at 20hz

#define PID_ANGLE_KP 0.045
#define PID_ANGLE_KI 0.00006
#define PID_ANGLE_KD 0.0000045

#define DIST_LINESENSOR_FROM_CENTER 0.15

enum {LINE_LEFT = -1, LINE_MIDDLE = 0, LINE_RIGHT = 1};
enum {mot_stop = 1, mot_move, mot_turn, mot_followline};

void update_motcon(motiontype *p, odotype *po, int *linesens_data);

int fwd(motiontype *mot, double dist, double speed, int time);
int turn(motiontype *mot, double angle, double speed, int time);
int followline(motiontype *mot, double dist, double speed, int time, int black_line, int line_to_follow);

double pid_angle(odotype *odo, double target);

#endif