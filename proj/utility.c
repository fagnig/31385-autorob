#include <time.h>
#include "utility.h"

double clamp(double d, double min, double max) {
  const double t = d < min ? min : d;
  return t > max ? max : t;
}

double get_time() { 
  struct timespec time;
  clock_gettime(CLOCK_MONOTONIC, &time);
  return (double) ( (double) time.tv_sec + (double) time.tv_nsec/1000000000);
}

void init_log() {
  next_log_pos = 0;
}

void log_to_array(odotype *p, motiontype *mot, int mission_time, double *lin) {
  log_arr[next_log_pos].time = p->time_curr - p->time_start;
  log_arr[next_log_pos].mission_time = mission_time;
  log_arr[next_log_pos].motorspeed_r = mot->motorspeed_r;
  log_arr[next_log_pos].motorspeed_l = mot->motorspeed_l;
  log_arr[next_log_pos].x = p->x;
  log_arr[next_log_pos].y = p->y;
  log_arr[next_log_pos].theta = p->theta;
  for(int i = 0; i<10; ++i){
    log_arr[next_log_pos].l[i] = lin[i];
  }
  
  next_log_pos++;
}

void save_array_log() {
  FILE * f = fopen("output.dat","w");
  
  for(int i = 0; i<next_log_pos; ++i){
    fprintf(f, "%f %d %f %f %f\n",
      log_arr[i].time,
      log_arr[i].mission_time,
      log_arr[i].x,
      log_arr[i].y,
      log_arr[i].theta);
    //fprintf(f, "%f %f %f %f %f %f %f %f %f %f\n",
    //  log_arr[i].l[0],
    //  log_arr[i].l[1],
    //  log_arr[i].l[2],
    //  log_arr[i].l[3],
    //  log_arr[i].l[4],
    //  log_arr[i].l[5],
    //  log_arr[i].l[6],
    //  log_arr[i].l[7],
    // log_arr[i].l[8],
      // log_arr[i].l[9]);
  }

  fclose(f);  
}