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
#include "configs.h"
#include "types.h"
#include "utility.h"
#include "motcon.h"
#include "odom.h"
#include "linesensor.h"

// EDIT THIS TO CHANGE PROGRAM
#define CONF_TO_RUN conf_stoponcross

#define ROBOTPORT 24902
#define SIMULPORT 8000

/////////////////////////////////////////////
// Robot data connection
/////////////////////////////////////////////

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

// SMR input/output data

symTableElement *  inputtable, *outputtable;
symTableElement *lenc, *renc, *linesensor, *irsensor, *speedl, *speedr, *resetmotorr, *resetmotorl;

/////////////////////////////////////////////
// Main
/////////////////////////////////////////////
odotype odo;
smtype mission;
motiontype mot;

int main()
{
  init_log();
  
  /////////////////////////////////////////////
  // Program driver
  /////////////////////////////////////////////
  StateParam *config = CONF_TO_RUN;
  int config_len = sizeof(CONF_TO_RUN) / sizeof(StateParam);
  int nextparam = 0;
  // printf("%d %p\n", config_len, (void*) config);
  
  int running, arg;

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
  
  // Predicate data
  PredicateData pred_data = { .mot=&mot, .odo=&odo, .laserpar=laserpar};
  
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
    
    for(int i = 0; i<5; i++){
      printf("%d ", irsensor->data[i]);
    }
    puts("\n");

    /****************************************
      / mission statemachine
    */
    
    log_to_array(&odo, &mot, mission.time, laserpar);
    
    // Skip/finish command when predicate is true.
    if(nextparam < config_len && config[nextparam].p_stop && config[nextparam].p_stop(pred_data)) {
      nextparam++;
      // printf("stop predicate was true, skipped to param %d\n", nextparam);
    }
    
    // No more commands
    if (nextparam >= config_len) {
      mission.state = ms_end;
    }
    
    sm_update(&mission);
    switch (mission.state) {
      case ms_init: {
        printf("started program with {%d} commands\n", config_len);
        mission.state = config[nextparam].state;
        break;
      }
      
      case ms_nextstate: {
        mission.state = config[++nextparam].state;
        break;
      }
      
      case ms_fwd: {
        if (fwd(&mot, config[nextparam].dist, config[nextparam].speed, mission.time)) {
          mission.state = ms_nextstate;
        }
        break;
      }
      
      case ms_turn: {
        if (turn(&mot, config[nextparam].angle, config[nextparam].speed, mission.time)) {
          mission.state = ms_nextstate;
        }
        break;
      }
      
      case ms_followline: {
        if (followline(&mot, config[nextparam].dist, config[nextparam].speed, mission.time, config[nextparam].is_black, config[nextparam].line_to_follow)) {
          mission.state = ms_nextstate;
        }
        break;
      }
      
      case ms_end: {
        mot.cmd = mot_stop;
        running = 0;
        break;
      }
      
      default: {
        printf("[ERROR] Invalid state entered: {%d} ", mission.state);
        StateParam sp = config[nextparam];
        printf("%d %f %f %f %d %d\n", sp.state, sp.speed, sp.dist, sp.angle, sp.is_black, sp.line_to_follow);
        mission.state = ms_end;
        break;
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
      
    // stop if keyboard is activated
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



