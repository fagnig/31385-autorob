#include "linesensor.h"
#include "utility.h"

static double linesens_poss[8] = {-7.0, -5.0, -3.0, -1.0, 1.0, 3.0, 5.0, 7.0};

// static double calfacts[8]   = {0.0394,0.0340,0.0408,0.04,0.0402,0.0287,0.0115,0.0297};
// static double caloffsets[8] = {-1.9443,-1.685,-2.1994,-2.1112,-2.0748,-1.6768,-0.6277,-1.5307};
static double black_val[8] = {49.66,50.15,54.35,53.37,55.79,61.15,56.70,52.61};
//static double black_val[8] = {50.66,51.95,56.00,54.37,55.79,62.15,56.70,52.61};
static double gray_val[8] = {56.66,58.95,61.00,59.37,78.79,69.15,78.70,64.61};
static double white_val[8] = {72.61,78.20,78.12,77.35,134.11,91.40,138.32,95.49};


double convert_linesensor_val(double in, int i, int is_black) {
  //if(is_black){
  //  return convert_linesensor_val_internal(in, black_val[i], gray_val[i]);
  //} else{
  //  return convert_linesensor_val_internal(in, gray_val[i], white_val[i]);
  //}
  return convert_linesensor_val_internal(in, black_val[i], white_val[i]);
}

double convert_linesensor_val_internal(double in, double minval, double maxval)
{
  return clamp((in - minval) / (maxval - minval),0.0,1.0);
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

