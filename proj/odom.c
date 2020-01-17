#include "odom.h"

#include <math.h>

#include "utility.h"

void reset_odo(odotype * odo)
{
  odo->right_pos = odo->left_pos = 0.0;
  odo->right_enc_old = odo->right_enc;
  odo->left_enc_old = odo->left_enc;
  odo->x = odo->y = odo->theta = 0.0;
  odo->time_start = odo->time_prev = odo->time_curr = get_time();
  odo->crossing_line = 0;
}

void update_odo(odotype *odo)
{
  odo->time_prev = odo->time_curr;
  odo->time_curr = get_time();
  
  int delta;
  
  /*
     Routines to convert encoder values to positions.
     Encoder steps have to be converted to meters, and
     roll-over has to be detected and corrected.
  */

  delta = odo->right_enc - odo->right_enc_old;
  if (delta > 0x8000) delta -= 0x10000;
  else if (delta < -0x8000) delta += 0x10000;
  odo->right_enc_old = odo->right_enc;
  double disp_r_pos = delta * odo->cr;
  odo->right_pos += disp_r_pos;

  delta = odo->left_enc - odo->left_enc_old;
  if (delta > 0x8000) delta -= 0x10000;
  else if (delta < -0x8000) delta += 0x10000;
  odo->left_enc_old = odo->left_enc;
  double disp_l_pos = delta * odo->cl;
  odo->left_pos += disp_l_pos;
  
  // find final angle
  odo->theta += (disp_r_pos - disp_l_pos) / odo->w;
  
  // find robot center linear displacement delta
  double disp_c_pos = (disp_r_pos + disp_l_pos) / 2;  //Delta U(i)
  
  // find new x and y from angle and linear displacement
  odo->x += disp_c_pos * cos(odo->theta);
  odo->y += disp_c_pos * sin(odo->theta);
}