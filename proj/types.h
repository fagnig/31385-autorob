#ifndef __TYPES_HEADER__
#define __TYPES_HEADER__

/////////////////////////////////////////////
// Structs
/////////////////////////////////////////////


typedef struct { 
  //input signals
  int left_enc, right_enc; // encoderticks
  // parameters
  double w; // wheel separation
  double cr, cl;  // meters per encodertick
  //output signals
  double right_pos, left_pos;
  double x, y, theta;
  double time_start;
  double time_prev;
  double time_curr;
  // internal variables
  int left_enc_old, right_enc_old;
} odotype;


typedef struct { 
  //input
  int cmd;
  int curcmd;
  double speedcmd;
  double dist;
  double angle;
  int black_line;
  int line_to_follow;
  double left_pos, right_pos;
  // parameters
  double w;
  //output
  double motorspeed_l, motorspeed_r;
  int finished;
  // internal variables
  double startpos;
} motiontype;


typedef struct {
  int state, oldstate;
  int time;
} smtype;

#endif