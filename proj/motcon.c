#include <stdlib.h>
#include <math.h>

#include "utility.h"
#include "motcon.h"

void update_motcon(motiontype *mot, odotype *odo, linedata *lindat) {

  if (mot->cmd != 0) {

    mot->finished = 0;
    switch (mot->cmd) {
      case mot_stop:{
        mot->curcmd = mot_stop;
        break;
      }
      case mot_move: {
        mot->startpos = (mot->left_pos + mot->right_pos) / 2;
        mot->curcmd = mot_move;
        break;
      }

      case mot_turn:{
        mot->startpos = odo->theta;
        // if (mot->angle > 0)
          // mot->startpos = mot->right_pos;
        // else
          // mot->startpos = mot->left_pos;
        mot->curcmd = mot_turn;
        break;
      }

      case mot_followline:{
        mot->startpos = (mot->left_pos + mot->right_pos) / 2;
        mot->angle = odo->theta;
        mot->curcmd = mot_followline;
      break;
      }

      case mot_wait:{
        mot->curcmd = mot_wait;
        mot->delay = odo->time_curr + mot->delay;
        break;
      }


    }

    mot->cmd = 0;
  }

  switch (mot->curcmd) {
    case mot_stop:
      mot->motorspeed_l = 0;
      mot->motorspeed_r = 0;
      break;
    case mot_move:{

      double speedsign = copysign(1.0,mot->speedcmd);

      double max_speed_inc = MAX_ACCEL * (odo->time_curr - odo->time_prev)*speedsign;

      
      double d = ((mot->right_pos + mot->left_pos) / 2 - mot->startpos)*speedsign - mot->dist;
      double v_max = sqrt(2.0 * MAX_ACCEL * fabs(d));

      if (d >= 0) {
        mot->finished = 1;
        mot->motorspeed_l = 0;
        mot->motorspeed_r = 0;
      }
      else {
        if (fabs(mot->speedcmd) > v_max) // decelerate early to avoid deceleration quicker than MAX_ACCEL
            mot->speedcmd = v_max*speedsign;
        if (fabs(mot->speedcmd) < MIN_VELOC)
            mot->speedcmd = MIN_VELOC*speedsign;
          
        if (mot->motorspeed_l < mot->speedcmd)
            mot->motorspeed_l += max_speed_inc;
        if (mot->motorspeed_l > mot->speedcmd)
            mot->motorspeed_l = mot->speedcmd;
        
        if (mot->motorspeed_r < mot->speedcmd)
            mot->motorspeed_r += max_speed_inc;
        if (mot->motorspeed_r > mot->speedcmd)
            mot->motorspeed_r = mot->speedcmd;
      }
      break;
    }
    case mot_turn:{
      double goal_angle = mot->angle + mot->startpos;
      double d = (mot->w / 2) * (goal_angle - odo->theta);
      double v_max = sqrt(2.0 * MAX_ACCEL * fabs(d));
      
      // can't go faster than v_max or speedcmd (whichever is smaller)
      // can't accelerate faster than 0.5m/s
      // can't decelerate (based on not going faster than v_max)
      // negative angle, (positive speed on left wheel)
      
      if (mot->angle > 0) {
          if (d > 0) {
              mot->motorspeed_r += fabs(0.1*(goal_angle - odo->theta));
          } else {
              mot->motorspeed_r = 0;
              mot->finished = 1;
          }
          mot->motorspeed_l = -mot->motorspeed_r;
          
      } else {
          if (d < 0) {
              mot->motorspeed_l += fabs(0.1*(goal_angle - odo->theta));
          } else {
              mot->motorspeed_l = 0;
              mot->finished = 1;
          }
          mot->motorspeed_r = -mot->motorspeed_l;
      }
      
      // limit top speed to v_max, to decelerate properly at end.
      if (mot->speedcmd > v_max) {
          mot->speedcmd = v_max;
      }
      if (mot->speedcmd < MIN_VELOC) {
          mot->speedcmd = MIN_VELOC;
      }
      
      // limit speed to top speed
      if (mot->motorspeed_r > mot->speedcmd) {
          mot->motorspeed_r = mot->speedcmd;
          mot->motorspeed_l = -mot->speedcmd;
      } else if (mot->motorspeed_l > mot->speedcmd) {
          mot->motorspeed_l = mot->speedcmd;
          mot->motorspeed_r = -mot->speedcmd;
      }
      
      break;
    }
    case mot_followline:{
      double max_speed_inc = MAX_ACCEL * (odo->time_curr - odo->time_prev);
      
      double d = (mot->right_pos + mot->left_pos) / 2 - mot->startpos - mot->dist;
      double v_max = sqrt(2.0 * MAX_ACCEL * d);
      
      if (d >= 0) {
        mot->finished = 1;
        mot->motorspeed_l = 0;
        mot->motorspeed_r = 0;
        break;
      }
    
      if (mot->speedcmd > v_max)
        mot->speedcmd = v_max;
      if (mot->speedcmd < MIN_VELOC)
        mot->speedcmd = MIN_VELOC;
        
      if (mot->motorspeed_l < mot->speedcmd)
        mot->motorspeed_l += max_speed_inc;
      if (mot->motorspeed_l > mot->speedcmd)
        mot->motorspeed_l = mot->speedcmd;
      
      if (mot->motorspeed_r < mot->speedcmd)
        mot->motorspeed_r += max_speed_inc;
      if (mot->motorspeed_r > mot->speedcmd)
        mot->motorspeed_r = mot->speedcmd;
      
      int numlines = mot->black_line ? lindat->numlines_b : lindat->numlines_w;
      grav_line* lines = mot->black_line ? lindat->lines_b : lindat->lines_w;

      if(numlines > 0 && !lindat->crossing_line_b){
        int selected = 0;
        
        switch (mot->line_to_follow) {
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

        
        double line_pos = 0.0;
        line_pos = center_of_gravity_line(  ( mot->black_line ? lindat->adj_datb : lindat->adj_datw )  , mot->black_line, lines[selected].first_sens, lines[selected].last_sens);
        double turn_angle = atan((line_pos/100.0)/DIST_LINESENSOR_FROM_CENTER);
        // printf("Line_pos: %f, turn_angle: %f\n", line_pos, turn_angle);
        double goal_angle = odo->theta + turn_angle;

        double turn_delta = (mot->turning_intensity+1)*pid_angle(odo, goal_angle);

        double turn_vel = mot->w*turn_delta/2;
        
        mot->motorspeed_r -= turn_vel;
        mot->motorspeed_l += turn_vel;
      }
      break;
    }

    case mot_wait:{
      if(odo->time_curr > mot->delay){
        mot->finished = 1;
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

int followline(motiontype *mot, odotype *odo, double dist, double speed, int time, int black_line, int line_to_follow, double turning_intensity)
{
  if (time == 0){
    mot->cmd = mot_followline;
    mot->speedcmd = speed;
    mot->dist = dist;
    mot->black_line = black_line;
    mot->line_to_follow = line_to_follow;
    mot->turning_intensity = turning_intensity;

    odo->i_sum = 0;
    odo->p_prev = 0;
    
    return 0;
  } else {
    return mot->finished;
  }
}

int wait(motiontype *mot, double delay, int time){
    if (time == 0){
    mot->cmd = mot_wait;
    mot->delay = delay;
    
    return 0;
  } else {
    return mot->finished;
  }
}

double pid_angle(odotype *odo, double target) {
  double p, i, d;
  
  double time_diff = odo->time_curr - odo->time_prev;
  
  // Proportional
  p = odo->theta - target;
  
  // Integral
  i = odo->i_sum += p * time_diff;


  // Derivative
  d = (p - odo->p_prev) / time_diff;
  odo->p_prev = p;
  
  return (PID_ANGLE_KP*p) + (PID_ANGLE_KI*i) + (PID_ANGLE_KD*d);
}
