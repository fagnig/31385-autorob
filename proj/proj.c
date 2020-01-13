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

#include "proj.h"
#include "utility.h"
#include "linesensor.h"

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

componentservertype lmssrv, camsrv;

symTableElement * getinputref (const char *sym_name, symTableElement *tab)
{
  int i;
  for (i = 0; i < getSymbolTableSize('r'); i++)
    if (strcmp (tab[i].name, sym_name) == 0)
      return &tab[i];
  return 0;
}

symTableElement * getoutputref (const char *sym_name, symTableElement *tab)
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


/********************************************
  Motion control
*/

enum {LINE_LEFT = 1, LINE_MIDDLE = 0, LINE_RIGHT = -1};
enum {mot_stop = 1, mot_move, mot_turn, mot_followline};

// SMR input/output data

symTableElement *  inputtable, *outputtable;
symTableElement *lenc, *renc, *linesensor, *irsensor, *speedl, *speedr, *resetmotorr, *resetmotorl;

odotype odo;
smtype mission;
motiontype mot;

enum {ms_init, ms_fwd, ms_turn, ms_followline, ms_end};


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
    
    log_to_array(&odo, &mot, mission.time, laserpar);
    
    sm_update(&mission);
    switch (mission.state) {
      case ms_init:
        n = 0; dist = 0.9; angle = 35.0 / 180 * M_PI;
        mission.state = ms_followline;
        break;

      case ms_fwd:
        if (fwd(dist, 0.3, mission.time))  mission.state = ms_turn;
        break;

      case ms_turn:
        if (turn(angle, 0.6, mission.time)) {
          n = n - 1;
          if (n <= 0)
            mission.state = ms_followline;
          else
            mission.state = ms_fwd;
        }
        break;
        
      case ms_followline:
        if (followline(3.0, 0.2, mission.time, 1, LINE_MIDDLE))
          mission.state = ms_end;
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
  p->time_start = p->time_prev = p->time_curr = get_time();
}

void update_odo(odotype *p)
{
  p->time_prev = p->time_curr;
  p->time_curr = get_time();
  
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
        linesens_adj_vals[i] = convert_linesensor_val(linesensor->data[i], calfacts[i], caloffsets[i]);
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

int followline(double dist, double speed, int time, int black_line, int line_to_follow)
{
  if (time == 0){
    mot.cmd = mot_followline;
    mot.speedcmd = speed;
    mot.dist = dist;
    mot.black_line = black_line;
    mot.line_to_follow = line_to_follow;

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



