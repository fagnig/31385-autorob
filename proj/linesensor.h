#ifndef __LINESENSOR_HEADER__
#define __LINESENSOR_HEADER__

#define NUM_LINESENSORS 8

#define BLACK_THRESHOLD 0.2
#define WHITE_THRESHOLD 0.6

#define MIN_LINES_FOR_CROSS 6
#define MIN_LINES_FOR_CROSS_WHITE 5

#include "types.h"

void update_linesensor(linedata *lindat);

double convert_linesensor_val(double in, int i, int is_black);
double convert_linesensor_val_internal(double in, double calval_factor, double calval_offset);

int linesens_find_lowest(double * linesens_vals);
int linesens_find_highest(double * linesens_vals);

int linesens_find_line(double * linesens_vals, int is_black);

int linesens_has_line(double * linesens_vals, int is_black);
int linesens_has_cross(double * linesens_vals, int is_black);

void grav_lines(linedata *lindat);
double center_of_gravity(double* linesens_vals, int is_black);
double center_of_gravity_line(double* linesens_vals, int is_black, int from, int to);

#endif
