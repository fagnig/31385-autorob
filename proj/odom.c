#include "odom.h"

#include <math.h>

#include "utility.h"

void reset_odo(odotype * p)
{
  p->right_pos = p->left_pos = 0.0;
  p->right_enc_old = p->right_enc;
  p->left_enc_old = p->left_enc;
  p->x = p->y = p->theta = 0.0;
  p->time_start = p->time_prev = p->time_curr = get_time();
}

void update_odo(odotype *p)
{
  p->time_prev = p->time_curr;
  p->time_curr = get_time();
  
  int delta;
  
  /*
     Routines to convert encoder values to positions.
     Encoder steps have to be converted to meters, and
     roll-over has to be detected and corrected.
  */

  delta = p->right_enc - p->right_enc_old;
  if (delta > 0x8000) delta -= 0x10000;
  else if (delta < -0x8000) delta += 0x10000;
  p->right_enc_old = p->right_enc;
  double disp_r_pos = delta * p->cr;
  p->right_pos += disp_r_pos;

  delta = p->left_enc - p->left_enc_old;
  if (delta > 0x8000) delta -= 0x10000;
  else if (delta < -0x8000) delta += 0x10000;
  p->left_enc_old = p->left_enc;
  double disp_l_pos = delta * p->cl;
  p->left_pos += disp_l_pos;
  
  // find final angle
  p->theta += (disp_r_pos - disp_l_pos) / p->w;
  
  // find robot center linear displacement delta
  double disp_c_pos = (disp_r_pos + disp_l_pos) / 2;  //Delta U(i)
  
  // find new x and y from angle and linear displacement
  p->x += disp_c_pos * cos(p->theta);
  p->y += disp_c_pos * sin(p->theta);
}