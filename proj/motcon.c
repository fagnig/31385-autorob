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
       p->angle = po->theta;
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
        //printf("Offset: %d Raw: %d, Adj: %f \n", i, linesens_data[i], linesens_adj_vals[i]);
      }
      grav_line lines[4];
      int numlines = grav_lines(linesens_adj_vals, lines, p->black_line);
      
      if(numlines > 0){
        int selected = 0;
        
        switch (p->line_to_follow) {
          case LINE_LEFT: {
            selected = numlines-1;
            break;
          }
          case LINE_MIDDLE: {
            // determine which line is most in the middle (avg sensor closest to middle sensor)
            int bestval = 9;
            for (int i = 0; i < numlines; i++) {
              int val = abs((lines[i].first_sens + lines[i].last_sens)-7);
              if (val < bestval) {
                bestval = val;
                selected = i;
              }
            }
            break;
          }
          case LINE_RIGHT: {
            selected = 0;
            break;
          }
        }
        
        if (po->time_curr - po->time_last_cross > 0.5) {
          for (int i = 0; i < numlines; i++) {
            if ((lines[i].last_sens - lines[i].first_sens) > 4) {
              po->time_last_cross = po->time_curr;
              po->num_crossed++;
              printf("line crossed %d\n", po->num_crossed);
            }
          }
        }
        
        // for (int i = 0; i < numlines; i++) {
          // printf("line %d: start: %d, end: %d", i, lines[i].first_sens, lines[i].last_sens);
        // }
        // printf("selected line: %d\n", selected);
        
        //double line_pos = center_of_gravity(linesens_adj_vals, p->black_line) - 4.0*p->line_to_follow;
        double line_pos = center_of_gravity_line(linesens_adj_vals, p->black_line, lines[selected].first_sens, lines[selected].last_sens);
        
        double turn_angle = atan((line_pos/100.0)/DIST_LINESENSOR_FROM_CENTER);
        // printf("Line_pos: %f, turn_angle: %f\n", line_pos, turn_angle);
        double goal_angle = po->theta + turn_angle;

        double turn_delta = pid_angle(po, goal_angle);
        //printf("TRYING TO TURN WITH DV: %f\n", turn_delta_v);
        
        //d = (p->w / 2) * (turn_delta);
        //v_max = sqrt(2.0 * MAX_ACCEL * fabs(d));
        
        p->motorspeed_r -= turn_delta;
        p->motorspeed_l += turn_delta;
        break;
      }
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

double pid_angle(odotype *odo, double target) {
  double p, i, d;
  
  // Proportional
  p = odo->theta - target;
  
  // Integral
  i = odo->i_sum += p; // can put a clamp on this
  
  // Derivative
  d = (odo->theta - odo->angle_prev) / (odo->time_curr - odo->time_prev);
  // update_odo saves time_curr and time_prev
  
  return PID_ANGLE_KP*p + PID_ANGLE_KI*i + PID_ANGLE_KD*d;
}
