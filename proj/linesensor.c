#include "linesensor.h"
#include "utility.h"

static double linesens_poss[8] = {-7.0, -5.0, -3.0, -1.0, 1.0, 3.0, 5.0, 7.0};

//static double black_val[8] = {49.66,50.15,54.35,53.37,55.79,61.15,56.70,52.61};
static double black_val[8] = {50.66,50.15,54.35,53.37,55.79,60.15,55.70,52.61};
static double gray_val[8] = {56.66,58.95,61.00,59.37,78.79,69.15,78.70,64.61};
//static double white_val[8] = {72.61,78.20,78.12,77.35,134.11,91.40,138.32,95.49};
//static double white_val[8] = {59.61,61.20,64.12,60.35,88.11,72.40,92.32,69.49};
static double white_val[8] = {75.61,80.20,79.12,79.35,135.11,94.40,140.32,96.49};

void update_linesensor(linedata *lindat){

  for(int i = 0; i < 8; ++i){
    lindat->adj_datb[i] = convert_linesensor_val(lindat->raw_dat[i], i, 1);
    lindat->adj_datw[i] = convert_linesensor_val(lindat->raw_dat[i], i, 0);
    //printf("Offset: %d Raw: %d, Adj: %f \n", i, linesens_data[i], linesens_adj_vals[i]);
  }

  grav_lines(lindat);

  lindat->crossing_line_b = 0;
  lindat->crossing_line_w = 0;
  for (int i = 0; i < lindat->numlines_b; i++) {
    if ((lindat->lines_b[i].last_sens - lindat->lines_b[i].first_sens) > MIN_LINES_FOR_CROSS) {
      lindat->crossing_line_b = 1;
    }
  }

  for (int i = 0; i < lindat->numlines_w; i++) {
    if ((lindat->lines_w[i].last_sens - lindat->lines_w[i].first_sens) > MIN_LINES_FOR_CROSS_WHITE) {
      lindat->crossing_line_w = 1;
    }
  }

}

double convert_linesensor_val(double in, int i, int is_black) {
  if(is_black){
    return convert_linesensor_val_internal(in, black_val[i], white_val[i]);
  } else{
    return convert_linesensor_val_internal(in, gray_val[i], white_val[i]);
  }

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

void grav_lines(linedata *lindat) {
  int acqlineb = 0;
  int acqlinew = 0;
  int num_linesb = 0;
  int num_linesw = 0;
  
  for (int i = 0; i < NUM_LINESENSORS; i++) {
    //Black lines
    if (lindat->adj_datb[i] < BLACK_THRESHOLD && !acqlineb) {
      acqlineb = 1;
      lindat->lines_b[num_linesb].first_sens = i;
    } else if (lindat->adj_datb[i] > BLACK_THRESHOLD && acqlineb) {
      acqlineb = 0;
      lindat->lines_b[num_linesb].last_sens = i;
      num_linesb++;
    }
    //White lines
    if (lindat->adj_datw[i] > WHITE_THRESHOLD && !acqlinew) {
      acqlinew = 1;
      lindat->lines_w[num_linesw].first_sens = i;
    } else if (lindat->adj_datw[i] < WHITE_THRESHOLD && acqlinew) {
      acqlinew = 0;
      lindat->lines_w[num_linesw].last_sens = i;
      num_linesw++;
    }
  }    
  
  if (acqlineb) {
    lindat->lines_b[num_linesb].last_sens = NUM_LINESENSORS;
    num_linesb++;
  }
  if (acqlinew) {
    lindat->lines_w[num_linesw].last_sens = NUM_LINESENSORS;
    num_linesw++;
  }

  lindat->numlines_b = num_linesb;
  lindat->numlines_w = num_linesw;
  
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


