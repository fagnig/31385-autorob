

#ifndef __UTILITY_HEADER__
#define __UTILITY_HEADER__

double time_start = 0;
logentry log_arr[10000000];
int next_log_pos = 0;


typedef struct {
  double time;
  int mission_time;
  double motorspeed_r, motorspeed_l;
  double x, y, theta;
  double l[10];
} logentry;



double clamp(double d, double min, double max);
double get_time();



//Logging
void log_to_array(odotype *p, motiontype *mot, int mission_time, double *lin);
void save_array_log();

#endif