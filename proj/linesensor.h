#ifndef __LINESENSOR_HEADER__
#define __LINESENSOR_HEADER__


static double calfacts[8]   = {0.0394,0.0340,0.0408,0.04,0.0402,0.0287,0.0115,0.0297};
static double caloffsets[8] = {-1.9443,-1.685,-2.1994,-2.1112,-2.0748,-1.6768,-0.6277,-1.5307};
static double linesens_poss[8] = {-7.0, -5.0, -3.0, -1.0, 1.0, 3.0, 5.0, 7.0};

#define NUM_LINESENSORS 8

#define BLACK_THRESHOLD 0.1
#define WHITE_THRESHOLD 0.9

double convert_linesensor_val(double in, double calval_factor, double calval_offset);

int linesens_find_lowest(double * linesens_vals);
int linesens_find_highest(double * linesens_vals);

int linesens_find_line(double * linesens_vals, int is_black);

int linesens_has_line(double * linesens_vals, int is_black);

double center_of_gravity(double* linesens_vals, int is_black);

#endif