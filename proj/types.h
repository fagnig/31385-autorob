#ifndef __TYPES_HEADER__
#define __TYPES_HEADER__

/////////////////////////////////////////////
// Enums
/////////////////////////////////////////////

enum {ms_init, ms_nextstate, ms_fwd, ms_turn, ms_followline, ms_end};
enum {LINE_LEFT = -1, LINE_MIDDLE = 0, LINE_RIGHT = 1};
enum {mot_stop = 1, mot_move, mot_turn, mot_followline};

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
  int crossing_line;
  // internal variables
  int left_enc_old, right_enc_old;
  double i_sum, p_prev;
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

typedef struct {
  int first_sens;
  int last_sens;
} grav_line;

#endif