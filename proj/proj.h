//Structs

#ifndef __PROJ_HEADER__
#define __PROJ_HEADER__

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

/////////////////////////////////////////////
// Function Prototypes
/////////////////////////////////////////////

symTableElement * getinputref (const char *sym_name, symTableElement *tab);

symTableElement * getoutputref (const char *sym_name, symTableElement *tab);

void serverconnect(componentservertype *s);
void xml_proc(struct xml_in *x);
void xml_proca(struct xml_in *x);

void reset_odo(odotype *p);
void update_odo(odotype *p);

void update_motcon(motiontype *p, odotype *po);

int fwd(double dist, double speed, int time);
int turn(double angle, double speed, int time);
int followline(double dist, double speed, int time, int black_line, int line_to_follow);

double convert_linesensor_val(double in, double calval_factor, double calval_offset);
int find_lowest_linesens_index();

void sm_update(smtype *p);

#endif