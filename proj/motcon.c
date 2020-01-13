#include <math.h>

#include "motcon.h"

void update_motcon(motiontype *p, odotype *po, int *linesens_data) {

  if (p->cmd != 0) {

    p->finished = 0;
    switch (p->cmd) {
      case mot_stop:
        p->curcmd = mot_stop;
        break;
      case mot_move:
        p->startpos = (p->left_pos + p->right_pos) / 2;
        p->curcmd = mot_move;
        break;

      case mot_turn:
        p->startpos = po->theta;
        // if (p->angle > 0)
          // p->startpos = p->right_pos;
        // else
          // p->startpos = p->left_pos;
        p->curcmd = mot_turn;
        break;

      case mot_followline:
       p->startpos = (p->left_pos + p->right_pos) / 2;
       p->curcmd = mot_followline;
       break;

    }

    p->cmd = 0;
  }

  switch (p->curcmd) {
    case mot_stop:
      p->motorspeed_l = 0;
      p->motorspeed_r = 0;
      break;
    case mot_move:{
      double d = (p->right_pos + p->left_pos) / 2 - p->startpos - p->dist;
      double v_max = sqrt(2.0 * MAX_ACCEL * d);
      
      if (d >= 0) {
        p->finished = 1;
        p->motorspeed_l = 0;
        p->motorspeed_r = 0;
      }
      else {
        if (p->speedcmd > v_max) // decelerate early to avoid deceleration quicker than a_max
            p->speedcmd = v_max;
        if (p->speedcmd < MIN_VELOC)
            p->speedcmd = MIN_VELOC;
          
        if (p->motorspeed_l < p->speedcmd)
            p->motorspeed_l += SPEED_INCREMENT;
        if (p->motorspeed_l > p->speedcmd)
            p->motorspeed_l = p->speedcmd;
        
        if (p->motorspeed_r < p->speedcmd)
            p->motorspeed_r += SPEED_INCREMENT;
        if (p->motorspeed_r > p->speedcmd)
            p->motorspeed_r = p->speedcmd;
      }
      break;
    }
    case mot_turn:{
      double goal_angle = p->angle + p->startpos;
      double d = (p->w / 2) * (goal_angle - po->theta);
      double v_max = sqrt(2.0 * MAX_ACCEL * fabs(d));
      
      // can't go faster than v_max or speedcmd (whichever is smaller)
      // can't accelerate faster than 0.5m/s
      // can't decelerate (based on not going faster than v_max)
      // negative angle, (positive speed on left wheel)
      
      if (p->angle > 0) {
          if (d > 0) {
              p->motorspeed_r += 0.1*(goal_angle - po->theta)/*SPEED_INCREMENT*/;
          } else {
              p->motorspeed_r = 0;
              p->finished = 1;
          }
          p->motorspeed_l = -p->motorspeed_r;
          
      } else {
          if (d < 0) {
              p->motorspeed_l += 0.1*(goal_angle - po->theta);
          } else {
              p->motorspeed_l = 0;
              p->finished = 1;
          }
          p->motorspeed_r = -p->motorspeed_l;
      }
      
      // limit top speed to v_max, to decelerate properly at end.
      if (p->speedcmd > v_max) {
          p->speedcmd = v_max;
      }
      if (p->speedcmd < MIN_VELOC) {
          p->speedcmd = MIN_VELOC;
      }
      
      // limit speed to top speed
      if (p->motorspeed_r > p->speedcmd) {
          p->motorspeed_r = p->speedcmd;
          p->motorspeed_l = -p->speedcmd;
      } else if (p->motorspeed_l > p->speedcmd) {
          p->motorspeed_l = p->speedcmd;
          p->motorspeed_r = -p->speedcmd;
      }
      
      break;
    }

    case mot_followline:{
      double max_speed_inc = MAX_ACCEL * (po->time_curr - po->time_prev);
      
      double d = (p->right_pos + p->left_pos) / 2 - p->startpos - p->dist;
      double v_max = sqrt(2.0 * MAX_ACCEL * d);
      
      if (d >= 0) {
        p->finished = 1;
        p->motorspeed_l = 0;
        p->motorspeed_r = 0;
        break;
      }
    
      if (p->speedcmd > v_max)
        p->speedcmd = v_max;
      if (p->speedcmd < MIN_VELOC)
        p->speedcmd = MIN_VELOC;
        
      if (p->motorspeed_l < p->speedcmd)
        p->motorspeed_l += max_speed_inc;
      if (p->motorspeed_l > p->speedcmd)
        p->motorspeed_l = p->speedcmd;
      
      if (p->motorspeed_r < p->speedcmd)
        p->motorspeed_r += max_speed_inc;
      if (p->motorspeed_r > p->speedcmd)
        p->motorspeed_r = p->speedcmd;
      
      double linesens_adj_vals[8];
      for(int i = 0; i < 8; ++i){
        linesens_adj_vals[i] = convert_linesensor_val(linesens_data[i], i);
      }
      if(linesens_has_line(linesens_adj_vals, p->black_line)){
        //double line_pos = linesens_poss[linesens_find_line(linesens_adj_vals, p->black_line)];
        double line_pos = center_of_gravity(linesens_adj_vals, p->black_line);
        double turn_delta_v = 0.01*line_pos + (3.5 * p->line_to_follow);
        
        int turn_dir = turn_delta_v > 0;
        turn_delta_v = fabs(turn_delta_v);
        
        if (turn_delta_v > max_speed_inc)
          turn_delta_v = max_speed_inc;
      
        if (turn_dir) {
          p->motorspeed_l -= turn_delta_v;
        } else {
          p->motorspeed_r -= turn_delta_v;
        }  
      }
      
      break;
    }
  }
}

int fwd(motiontype *mot, double dist, double speed, int time) {
  if (time == 0) {
    mot->cmd = mot_move;
    mot->speedcmd = speed;
    mot->dist = dist;
    return 0;
  }
  else
    return mot->finished;
}

int turn(motiontype *mot, double angle, double speed, int time) {
  if (time == 0) {
    mot->cmd = mot_turn;
    mot->speedcmd = speed;
    mot->angle = angle;
    return 0;
  }
  else
    return mot->finished;
}

int followline(motiontype *mot, double dist, double speed, int time, int black_line, int line_to_follow)
{
  if (time == 0){
    mot->cmd = mot_followline;
    mot->speedcmd = speed;
    mot->dist = dist;
    mot->black_line = black_line;
    mot->line_to_follow = line_to_follow;

    return 0;
  } else {
    return mot->finished;
  }
}