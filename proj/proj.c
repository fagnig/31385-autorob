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
#include "motcon.h"
#include "odom.h"
#include "linesensor.h"

struct xml_in *xmldata;
struct xml_in *xmllaser;
struct {
  double x, y, z, omega, phi, kappa, code, id, crc;
} gmk;


double visionpar[10];
double laserpar[10];

componentservertype lmssrv, camsrv;
void serverconnect(componentservertype *s);

void xml_proc(struct xml_in *x);
void xml_proca(struct xml_in *x);

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

#define ROBOTPORT 24902
#define SIMULPORT 8000

// SMR input/output data

symTableElement *  inputtable, *outputtable;
symTableElement *lenc, *renc, *linesensor, *irsensor, *speedl, *speedr, *resetmotorr, *resetmotorl;

odotype odo;
smtype mission;
motiontype mot;

enum {ms_init, ms_fwd, ms_turn, ms_followline, ms_end};


int main()
{
  init_log();
  
  int running, n = 0, arg, time = 0;
  double dist = 0, angle = 0;

  /* Establish connection to robot sensors and actuators.
  */
  if (rhdConnect('w', "localhost", SIMULPORT) != 'w') {
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
      case ms_init: {
        n = 0; dist = 0.9; angle = 35.0 / 180 * M_PI;
        mission.state = ms_followline;
        break;
      }
      
      case ms_fwd: {
        if (fwd(&mot, dist, 0.3, mission.time))  mission.state = ms_turn;
        break;
      }
      
      case ms_turn: {
        if (turn(&mot, angle, 0.6, mission.time)) {
          n = n - 1;
          if (n <= 0)
            mission.state = ms_followline;
          else
            mission.state = ms_fwd;
        }
        break;
      }
      
      case ms_followline: {
        if (followline(&mot, 3.0, 0.2, mission.time, 1, LINE_MIDDLE))
          mission.state = ms_end;
        break;
      }
      
      case ms_end: {
        mot.cmd = mot_stop;
        running = 0;
        break;
      }
      
      default: {
        printf("[ERROR] Invalid state entered: {%d} ", mission.state);
        mission.state = ms_end;
      }
      
    }
    save_array_log();
    /*  end of mission  */

    mot.left_pos = odo.left_pos;
    mot.right_pos = odo.right_pos;
    update_motcon(&mot, &odo, linesensor->data);
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

void sm_update(smtype *p) {
  if (p->state != p->oldstate) {
    p->time = 0;
    p->oldstate = p->state;
  }
  else {
    p->time++;
  }
}



