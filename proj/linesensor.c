#include "linesensor.h"
#include "utility.h"

double convert_linesensor_val(double in, int i) {
  return convert_linesensor_val_internal(in, black_val[i], white_val[i]);
}

double convert_linesensor_val_internal(double in, double black_val, double white_val)
{
  return clamp((in - black_val) / (white_val - black_val),0.0,1.0);
}

int linesens_find_lowest(double * linesens_vals) {
  int lowest_index = 0;
  for (int i = 1; i < NUM_LINESENSORS; i++) {
    if (linesens_vals[i] <= linesens_vals[lowest_index])
      lowest_index = i;
  }
  return lowest_index;
}

int linesens_find_highest(double * linesens_vals) {
  int lowest_index = 0;
  for (int i = 1; i < NUM_LINESENSORS; i++) {
    if (linesens_vals[i] >= linesens_vals[lowest_index])
      lowest_index = i;
  }
  return lowest_index;
}

int linesens_find_line(double * linesens_vals, int is_black)
{
  if(is_black)
    return linesens_find_lowest(linesens_vals);
  return linesens_find_highest(linesens_vals);
}

int linesens_has_line(double * linesens_vals, int is_black){
  if(is_black){
    for(int i = 0; i < NUM_LINESENSORS; ++i){
      if(linesens_vals[i] < BLACK_THRESHOLD) return 1;
    }
  } else {
     for(int i = 0; i < NUM_LINESENSORS; ++i){
      if(linesens_vals[i] > WHITE_THRESHOLD) return 1;
    }
  }
  return 0;
}

int linesens_has_cross(double * linesens_vals, int is_black) {
  int num_sensors_line = 0;
  if(is_black){
    for(int i = 0; i < NUM_LINESENSORS; ++i){
      if(linesens_vals[i] < BLACK_THRESHOLD) num_sensors_line++;
    }
  } else {
     for(int i = 0; i < NUM_LINESENSORS; ++i){
      if(linesens_vals[i] > WHITE_THRESHOLD) num_sensors_line++;
    }
  }
  return num_sensors_line > MIN_LINES_FOR_CROSS;
}

int grav_lines(double *line_adj_vals, grav_line *out, int is_black) {
  int acqline = 0;
  int num_lines = 0;
  
  if (is_black) {
    for (int i = 0; i < NUM_LINESENSORS; i++) {
      if (line_adj_vals[i] < BLACK_THRESHOLD && !acqline) {
        acqline = 1;
        out[num_lines].first_sens = i;
      } else if (line_adj_vals[i] > BLACK_THRESHOLD && acqline) {
        acqline = 0;
        out[num_lines].last_sens = i;
        num_lines++;
      }
    }    
  } else {
    for (int i = 0; i < NUM_LINESENSORS; i++) {
      if (line_adj_vals[i] > WHITE_THRESHOLD && !acqline) {
        acqline = 1;
        out[num_lines].first_sens = i;
      } else if (line_adj_vals[i] < WHITE_THRESHOLD && acqline) {
        acqline = 0;
        out[num_lines].last_sens = i;
        num_lines++;
      }
    }    
  }
  
  if (acqline) {
    out[num_lines].last_sens = NUM_LINESENSORS;
    num_lines++;
  }
  
  return num_lines;
}

double center_of_gravity(double* linesens_vals, int is_black) {
  double sum_top = 0.0, sum_bot = 0.0;
  
  if (is_black) {
    for (int i = 0; i < NUM_LINESENSORS; i++) {
      sum_top += linesens_poss[i] * (1.0 - linesens_vals[i]);
      sum_bot += (1.0-linesens_vals[i]);
    }
  } else {
    for (int i = 0; i < NUM_LINESENSORS; i++) {
      sum_top += linesens_poss[i] * linesens_vals[i];
      sum_bot += linesens_vals[i];
    }
  }
  
  return sum_top / sum_bot;
}

double center_of_gravity_line(double* linesens_vals, int is_black, int from, int to) {
  double sum_top = 0.0, sum_bot = 0.0;
  
  if (is_black) {
    for (int i = from; i < to; i++) {
      sum_top += linesens_poss[i] * (1.0 - linesens_vals[i]);
      sum_bot += (1.0-linesens_vals[i]);
    }
  } else {
    for (int i = from; i < to; i++) {
      sum_top += linesens_poss[i] * linesens_vals[i];
      sum_bot += linesens_vals[i];
    }
  }
  return sum_top / sum_bot;
}
