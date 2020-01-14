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

double center_of_gravity(double* linesens_vals, int is_black) {
  double sum_top = 0.0, sum_bot = 0.0;
  
  if (is_black) {
    for (int i = 0; i < NUM_LINESENSORS; i++) {
      sum_top += linesens_poss[i] * (1.0 - linesens_vals[i]);
      sum_bot += linesens_vals[i];
    }
  } else {
    for (int i = 0; i < NUM_LINESENSORS; i++) {
      sum_top += linesens_poss[i] * linesens_vals[i];
      sum_bot += linesens_vals[i];
    }
  }
  
  return sum_top / sum_bot;
}
