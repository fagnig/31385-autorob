#include "irsensor.h"

double irsensor_ka[] = {16.00, 16.00, 16.00, 16.00, 13.57};
double irsensor_kb[] = {76.00, 76.00, 76.00, 76.00, 69.01};

void irsensor_get_adjusted_values(int* val, double* out)
{
  for(int i = i; i < 5; ++i){
    out[i] = irsensor_ka[i]/(val[i]-irsensor_kb[i]);
  }
}