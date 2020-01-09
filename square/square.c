/*
   An example SMR program.

*/
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include <unistd.h>
#include <string.h>
#include <stdlib.h>
#include <signal.h>
#include <sys/ioctl.h>
#include <sys/socket.h>
#include <sys/time.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <fcntl.h>

#include <sys/ioctl.h>
#include "rhd.h"
#include "componentserver.h"
#include "xmlio.h"

#define MIN_VELOC 0.05
#define MAX_ACCEL 0.5
#define REFRESH_RATE 20.0
#define SPEED_INCREMENT (MAX_ACCEL / REFRESH_RATE) // 0.5m/s^2 at 20hz

struct xml_in *xmldata;
struct xml_in *xmllaser;
struct {
  double x, y, z, omega, phi, kappa, code, id, crc;
} gmk;
double visionpar[10];
double laserpar[10];

void serverconnect(componentservertype *s);
void xml_proc(struct xml_in *x);
void xml_proca(struct xml_in *x);

componentservertype lmssrv, camsrv;

symTableElement *
getinputref (const char *sym_name, symTableElement * tab)
{
  int i;
  for (i = 0; i < getSymbolTableSize('r'); i++)
    if (strcmp (tab[i].name, sym_name) == 0)
      return &tab[i];
  return 0;
}

symTableElement *
getoutputref (const char *sym_name, symTableElement * tab)
{
  int i;
  for (i = 0; i < getSymbolTableSize('w'); i++)
    if (strcmp (tab[i].name, sym_name) == 0)
      return &tab[i];
  return 0;
}
/*****************************************
  odometry
*/
#define WHEEL_DIAMETER   0.06522  /* m */
#define WHEEL_SEPARATION 0.26 /* m */
#define DELTA_M (M_PI * WHEEL_DIAMETER / 2000)
#define ROBOTPORT 8000 //24902


typedef struct { //input signals
  int left_enc, right_enc; // encoderticks
  // parameters
  double w; // wheel separation
  double cr, cl;  // meters per encodertick
  //output signals
  double right_pos, left_pos;
  double x, y, theta;
  // internal variables
  int left_enc_old, right_enc_old;
} odotype;

void reset_odo(odotype *p);
void update_odo(odotype *p);


/********************************************
  Motion control
*/

typedef struct { //input
  int cmd;
  int curcmd;
  double speedcmd;
  double dist;
  double angle;
  double left_pos, right_pos;
  // parameters
  double w;
  //output
  double motorspeed_l, motorspeed_r;
  int finished;
  // internal variables
  double startpos;
} motiontype;

enum {mot_stop = 1, mot_move, mot_turn};

void update_motcon(motiontype *p, odotype *po);


int fwd(double dist, double speed, int time);
int turn(double angle, double speed, int time);

static double calfacts[8]   = {0.0394,0.0340,0.0408,0.04,0.0402,0.0287,0.0115,0.0297};
static double caloffsets[8] = {-1.9443,-1.685,-2.1994,-2.1112,-2.0748,-1.6768,-0.6277,-1.5307};

double convert_linesensor_val(double in, double calval_factor, double calval_offset);
int find_lowest_linesens_index();

typedef struct {
  int state, oldstate;
  int time;
} smtype;

void sm_update(smtype *p);

// SMR input/output data

symTableElement *  inputtable, *outputtable;
symTableElement *lenc, *renc, *linesensor, *irsensor, *speedl, *speedr, *resetmotorr, *resetmotorl;

odotype odo;
smtype mission;
motiontype mot;

enum {ms_init, ms_fwd, ms_turn, ms_followline, ms_end};

typedef struct {
  int mission_time;
  double motorspeed_r, motorspeed_l;
  double x, y, theta;
  double l[10];
} logentry;

logentry log_arr[10000000];
int next_log_pos = 0;

void log_to_array(int mission_time, double motorspeed_r, double motorspeed_l, double x, double y, double theta, double *lin) {
  log_arr[next_log_pos].mission_time = mission_time;
  log_arr[next_log_pos].motorspeed_r = motorspeed_r;
  log_arr[next_log_pos].motorspeed_l = motorspeed_l;
  log_arr[next_log_pos].x = x;
  log_arr[next_log_pos].y = y;
  log_arr[next_log_pos].theta = theta;
  for(int i = 0; i<10; ++i){
	log_arr[next_log_pos].l[i] = lin[i];
  }
  
  next_log_pos++;
}
void save_array_log() {
  FILE * f = fopen("output.dat","w");
  
  for(int i = 0; i<next_log_pos; ++i){
    //fprintf(f, "%d %f %f %f %f %f\n",
	//  log_arr[i].mission_time,
	//  log_arr[i].motorspeed_r,
	//  log_arr[i].motorspeed_l,
	//  log_arr[i].x,
	//  log_arr[i].y,
	//  log_arr[i].theta);
	fprintf(f, "%f %f %f %f %f %f %f %f %f %f\n",
	  log_arr[i].l[0],
	  log_arr[i].l[1],
	  log_arr[i].l[2],
	  log_arr[i].l[3],
	  log_arr[i].l[4],
	  log_arr[i].l[5],
	  log_arr[i].l[6],
	  log_arr[i].l[7],
	  log_arr[i].l[8],
      log_arr[i].l[9]);
  }

  fclose(f);  
}

double clamp(double d, double min, double max) {
  const double t = d < min ? min : d;
  return t > max ? max : t;
}

double convert_linesensor_val(double in, double calval_factor, double calval_offset)
{
  return clamp(in * calval_factor + calval_offset,0.0,1.0);
}

int find_lowest_linesens_index()
{
  double l_val = 99999.9; // big value, since we're trying to find the smallest val
  int l_index = 0;

  for(int i = 0; i < 8; ++i){
    double curval = convert_linesensor_val(linesensor->data[i], calfacts[i], caloffsets[i]);

    if(curval < l_val){
      l_val = curval;
      l_index = i;
    }
  }

  return l_index;
}

#define LINESENSOR_TRESHHOLD 60.0

int can_see_line(int is_black /*unused for now*/)
{
  double val;
  for(int i = 0; i < 8; ++i){
    double curval = convert_linesensor_val(linesensor->data[i], calfacts[i], caloffsets[i]);
    if(curval < LINESENSOR_TRESHHOLD)
      return 1;
  }
  return 0;
}

int main()
{
  int running, n = 0, arg, time = 0;
  double dist = 0, angle = 0;

  /* Establish connection to robot sensors and actuators.
  */
  if (rhdConnect('w', "localhost", ROBOTPORT) != 'w') {
    printf("Can't connect to rhd \n");
    exit(EXIT_FAILURE);
  }

  printf("connected to robot \n");
  if ((inputtable = getSymbolTable('r')) == NULL) {
    printf("Can't connect to rhd \n");
    exit(EXIT_FAILURE);
  }
  if ((outputtable = getSymbolTable('w')) == NULL) {
    printf("Can't connect to rhd \n");
    exit(EXIT_FAILURE);
  }
  // connect to robot I/O variables
  lenc = getinputref("encl", inputtable);
  renc = getinputref("encr", inputtable);
  linesensor = getinputref("linesensor", inputtable);
  irsensor = getinputref("irsensor", inputtable);

  speedl = getoutputref("speedl", outputtable);
  speedr = getoutputref("speedr", outputtable);
  resetmotorr = getoutputref("resetmotorr", outputtable);
  resetmotorl = getoutputref("resetmotorl", outputtable);
  // **************************************************
  //  Camera server code initialization
  //

  /* Create endpoint */
  lmssrv.port = 24919;
  strcpy(lmssrv.host, "127.0.0.1");
  strcpy(lmssrv.name, "laserserver");
  lmssrv.status = 1;
  camsrv.port = 24920;
  strcpy(camsrv.host, "127.0.0.1");
  camsrv.config = 1;
  strcpy(camsrv.name, "cameraserver");
  camsrv.status = 1;

  if (camsrv.config) {
    int errno = 0;
    camsrv.sockfd = socket(AF_INET, SOCK_STREAM, 0);
    if ( camsrv.sockfd < 0 )
    {
      perror(strerror(errno));
      fprintf(stderr, " Can not make  socket\n");
      exit(errno);
    }

    serverconnect(&camsrv);

    xmldata = xml_in_init(4096, 32);
    printf(" camera server xml initialized \n");

  }




  // **************************************************
  //  LMS server code initialization
  //

  /* Create endpoint */
  lmssrv.config = 1;
  if (lmssrv.config) {
    char buf[256];
    int errno = 0, len;
    lmssrv.sockfd = socket(AF_INET, SOCK_STREAM, 0);
    if ( lmssrv.sockfd < 0 )
    {
      perror(strerror(errno));
      fprintf(stderr, " Can not make  socket\n");
      exit(errno);
    }

    serverconnect(&lmssrv);
    if (lmssrv.connected) {
      xmllaser = xml_in_init(4096, 32);
      printf(" laserserver xml initialized \n");
      //len = sprintf(buf, "push  t=0.2 cmd='mrcobst width=0.4'\n");
      len = sprintf(buf, "scanpush cmd='zoneobst'\n");
      send(lmssrv.sockfd, buf, len, 0);
    }

  }


  /* Read sensors and zero our position.
  */
  rhdSync();

  odo.w = 0.256;
  odo.cr = DELTA_M;
  odo.cl = odo.cr;
  odo.left_enc = lenc->data[0];
  odo.right_enc = renc->data[0];
  reset_odo(&odo);
  printf("position: %f, %f\n", odo.left_pos, odo.right_pos);
  mot.w = odo.w;
  running = 1;
  mission.state = ms_init;
  mission.oldstate = -1;
  while (running) {
    if (lmssrv.config && lmssrv.status && lmssrv.connected) {
      while ( (xml_in_fd(xmllaser, lmssrv.sockfd) > 0))
        xml_proca(xmllaser);
    }

    if (camsrv.config && camsrv.status && camsrv.connected) {
      while ( (xml_in_fd(xmldata, camsrv.sockfd) > 0))
        xml_proc(xmldata);
    }


    rhdSync();
    odo.left_enc = lenc->data[0];
    odo.right_enc = renc->data[0];
    update_odo(&odo);

    /****************************************
      / mission statemachine
    */
	
	log_to_array(mission.time, mot.motorspeed_r, mot.motorspeed_l, odo.x, odo.y, odo.theta, laserpar);
	
    sm_update(&mission);
    switch (mission.state) {
      case ms_init:
        n = 4; dist = 1; angle = 90.0 / 180 * M_PI;
        mission.state = ms_fwd;
        break;

      case ms_fwd:
        if (fwd(dist, 0.3, mission.time))  mission.state = ms_turn;
        break;

      case ms_turn:
        if (turn(angle, 0.3, mission.time)) {
          n = n - 1;
          if (n == 0)
            mission.state = ms_end;
          else
            mission.state = ms_fwd;
        }
        break;

      case ms_end:
        mot.cmd = mot_stop;
        running = 0;
        break;
    }
	save_array_log();
    /*  end of mission  */

    mot.left_pos = odo.left_pos;
    mot.right_pos = odo.right_pos;
    update_motcon(&mot, &odo);
    speedl->data[0] = 100 * mot.motorspeed_l;
    speedl->updated = 1;
    speedr->data[0] = 100 * mot.motorspeed_r;
    speedr->updated = 1;
    if (time  % 100 == 0)
      //    printf(" laser %f \n",laserpar[3]);
      time++;
    /* stop if keyboard is activated

    */
    ioctl(0, FIONREAD, &arg);
    if (arg != 0)  running = 0;

  }/* end of main control loop */
  speedl->data[0] = 0;
  speedl->updated = 1;
  speedr->data[0] = 0;
  speedr->updated = 1;
  rhdSync();
  rhdDisconnect();
  exit(0);
}


/*
   Routines to convert encoder values to positions.
   Encoder steps have to be converted to meters, and
   roll-over has to be detected and corrected.
*/



void reset_odo(odotype * p)
{
  p->right_pos = p->left_pos = 0.0;
  p->right_enc_old = p->right_enc;
  p->left_enc_old = p->left_enc;
  p->x = p->y = p->theta = 0.0;
}

void update_odo(odotype *p)
{
  int delta;

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


void update_motcon(motiontype *p, odotype *po) {

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
			  p->motorspeed_r += SPEED_INCREMENT;
		  } else {
			  p->motorspeed_r = 0;
			  p->finished = 1;
		  }
		  p->motorspeed_l = -p->motorspeed_r;
		  
	  } else {
		  if (d < 0) {
			  p->motorspeed_l += SPEED_INCREMENT;
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


      p->finished = 1;
    }
  }
}


int fwd(double dist, double speed, int time) {
  if (time == 0) {
    mot.cmd = mot_move;
    mot.speedcmd = speed;
    mot.dist = dist;
    return 0;
  }
  else
    return mot.finished;
}

int turn(double angle, double speed, int time) {
  if (time == 0) {
    mot.cmd = mot_turn;
    mot.speedcmd = speed;
    mot.angle = angle;
    return 0;
  }
  else
    return mot.finished;
}

int followline(double speed, int time)
{
  if (time == 0){
    mot.cmd = mot_followline;
    mot.speedcmd = speed;

    return 0;
  } else {
    return mot.finished;
  }
}


void sm_update(smtype *p) {
  if (p->state != p->oldstate) {
    p->time = 0;
    p->oldstate = p->state;
  }
  else {
    p->time++;
  }
}



