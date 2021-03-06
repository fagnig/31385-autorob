#ifndef __COMPCONFIGS_HEADER__
#define __COMPCONFIGS_HEADER__

#include "types.h"

#include <math.h>

#define DIST_AXLE_TO_IRSENSOR 0.24
#define DIST_AXLE_TO_LINESENSOR 0.22


#define SPEED_SLOW 0.2
#define SPEED_SAFE 0.35
#define SPEED_RECKLESS 0.5


#define DIM_GARAGE_WIDTH 0.485
#define DIM_GARAGE_DOOR 0.19
#define DIM_GARAGE_LENGTH 0.6
#define DIM_SMR_SIDELENGTH 0.28


int p_print_dist_to_box(PredicateData dat){
  printf("DIST TO BOX: %f\n", dat.laserpar[8]);
  return 1;
}
int p_stoponcross(PredicateData dat) {
  return dat.lindat->crossing_line_b;
}

const double max_dist_to_wall = 0.7;
double dist_to_wall = 0.0;
int p_findwall(PredicateData dat) {
  printf("LASERDIST: %f\n", dat.laserpar[8]);
  if (dat.laserpar[8] > 0.01 && dat.laserpar[8] < max_dist_to_wall) {
    dist_to_wall = dat.laserpar[8] + 0.05;
    printf("Wall found %f\n", dist_to_wall);
    return 1;
  }
  return 0;
}
int p_findopening(PredicateData dat) {
  if(dat.laserpar[8] > 0.5) printf("Wall found %f\n", dat.laserpar[8]);
  return dat.laserpar[8] > 0.5;
}

int p_stoponline(PredicateData dat){
  return dat.lindat->numlines_b;
}

#define STOPDISTFROMOBJECTS 0.25
int p_approachobject(PredicateData dat){
  if(dat.laserpar[4] < STOPDISTFROMOBJECTS) puts("STOPPED FROM OBJECT");
  return dat.laserpar[4] < STOPDISTFROMOBJECTS;
}

#define STOPDISTFROMWALL 0.32
int p_approachwall(PredicateData dat){
  if(dat.laserpar[4] < STOPDISTFROMWALL) puts("STOPPED FROM OBJECT");
  return dat.laserpar[4] < STOPDISTFROMWALL;
}

const double max_dist_to_gate = 0.1;
double dist_to_gate = 0.0;
int p_findgate(PredicateData dat){
  if (dat.irpar[4] > 0.01 && dat.irpar[4] < max_dist_to_wall) {
    dist_to_wall = dat.irpar[4] + 0.05;
    printf("Gate found %f\n", dist_to_wall);
    return 1;
  }
  return 0;
}



/////////////////////////////////////////////
// Followwall via config predicate hacks
/////////////////////////////////////////////
double followwall_x, followwall_y, followwall_s;
// Get first measurement of x and y and IR distance on the appropriate side
int p_followwall_1(PredicateData dat){
  followwall_x = dat.odo->x;
  followwall_y = dat.odo->y;
  followwall_s = dat.irpar[4];

  printf("First measurement taken: %f\n", followwall_s);
  
  return 1;
}
double followwall_angle;
// Get second measurement of x-y-IR dist, and save angle.
// Optionally, use distance driven parameter instead of recalculating with xy
int p_followwall_2(PredicateData dat) {
  double xdiff = dat.odo->x - followwall_x;
  double ydiff = dat.odo->x - followwall_y;
  double sdiff = dat.irpar[4] - followwall_s;
  double dist = sqrt(xdiff*xdiff + ydiff*ydiff);
  followwall_angle = atan(sdiff / dist);

  printf("Second measurement taken: dist: %f, ang_corr: %f\n", dist, followwall_angle);
  
  return 1;
}
// Hack the angle of a turn command, but only once.
int p_followwall_hackangle(PredicateData dat) {
  dat.mot->angle = -(followwall_angle/2);
  return 0;
}

#define DIST_WHITE_LINE_1 0.75
#define DIST_WHITE_LINE_2 1.1
#define DIST_WHITE_LINE_3 1.1
#define DIST_WHITE_LINE_4 0.5
#define DIST_WHITE_LINE_5 0.5

int p_line_cross(PredicateData dat) {
  if(dat.lindat->crossing_line_w) puts("CROSSED WHITE LINE");
  return dat.lindat->crossing_line_w;
}

StateParam conf_comp[] = {
  { .state = ms_wait, .delay = 1.0 }, // To make sure linesensors are alive
  
  
  //Measure box
  { .state = ms_fwd, .speed = SPEED_SAFE, .dist = 0.65},
  { .state = ms_wait, .delay = 0.2 },
  { .state = ms_wait, .delay = 0.0, .p_stop = p_print_dist_to_box },

  //First transit
  
  //{ .state = ms_fwd, .speed = SPEED_SAFE, .dist = DIST_AXLE_TO_LINESENSOR },
  //{ .state = ms_turn, .speed = SPEED_SLOW, .angle = -80.0 / 180.0 * M_PI },
  //{ .state = ms_fwd, .speed = SPEED_SLOW, .dist = 0.03 },
  //{ .state = ms_followline, .speed = SPEED_SAFE, .dist = 1.0, .is_black = 1, .line_to_follow = LINE_LEFT, .turning_intensity = -0.1},
  //{ .state = ms_followline, .speed = SPEED_SAFE, .dist = 1.3, .is_black = 1, .line_to_follow = LINE_MIDDLE, .p_stop = p_stoponcross, .turning_intensity = 0.3},
  //{ .state = ms_fwd, .speed = SPEED_SLOW, .dist = 0.1},
  //{ .state = ms_followline, .speed = SPEED_SAFE, .dist = 1.5, .is_black = 1, .line_to_follow = LINE_MIDDLE, .turning_intensity = 0.4},
  
  { .state = ms_fwd, .speed = SPEED_SAFE, .dist = 1.85},
  { .state = ms_turn, .speed = SPEED_SAFE, .angle = -87.0 / 180.0 * M_PI },
  { .state = ms_fwd, .speed = SPEED_SAFE, .dist = 1.0},

  //Push the box & go through gate
  { .state = ms_followline, .speed = SPEED_SAFE, .dist = 8.0, .is_black = 1, .line_to_follow = LINE_MIDDLE,
    .p_stop = p_stoponcross},
  { .state = ms_fwd, .speed = SPEED_SAFE, .dist = DIST_AXLE_TO_IRSENSOR-0.05},
  { .state = ms_fwd, .speed = -SPEED_SAFE, .dist = 1.0},
  { .state = ms_turn, .speed = SPEED_SAFE, .angle = -90.0 / 180.0 * M_PI },
  { .state = ms_fwd, .speed = SPEED_SAFE, .dist = 0.4},
  { .state = ms_fwd, .speed = SPEED_SAFE, .dist = 1.0, .p_stop = p_stoponcross},
  { .state = ms_fwd, .speed = SPEED_SAFE, .dist = 0.3},
  { .state = ms_turn, .speed = SPEED_SAFE, .angle = 90.0 / 180.0 * M_PI },
  { .state = ms_followline, .speed = SPEED_SLOW, .dist = 2.0, .is_black = 1, .line_to_follow = LINE_MIDDLE, .turning_intensity = 0.4, 
    .p_stop = p_stoponcross},
  { .state = ms_fwd, .speed = SPEED_SAFE, .dist = 0.2},
  
  //Transit to loose gate
  { .state = ms_followline, .speed = SPEED_SLOW, .dist = 2.0, .is_black = 1, .line_to_follow = LINE_MIDDLE,
    .p_stop = p_stoponcross},
  { .state = ms_fwd, .speed = SPEED_SAFE, .dist = 0.1},
  
  //Find gate and...
  { .state = ms_followline, .speed = SPEED_SLOW, .dist = 10.0, .is_black = 1, .line_to_follow = LINE_MIDDLE, .turning_intensity = -0.5,
    .p_stop = p_findwall},
  { .state = ms_followline, .speed = SPEED_SLOW, .dist = 10.0, .is_black = 1, .line_to_follow = LINE_MIDDLE, .turning_intensity = -0.3,
    .p_stop = p_findopening},

  //...Go through it
  { .state = ms_followline, .speed = SPEED_SLOW, .dist = 0.75, .is_black = 1, .line_to_follow = LINE_MIDDLE, .turning_intensity = -0.3},
  { .state = ms_turn, .speed = SPEED_SLOW, .angle = -87.0 / 180.0 * M_PI },
  { .state = ms_fwd, .speed = SPEED_SAFE, .dist = 2.0, .p_stop = p_approachwall },
  
  //Transit to wall following
  
  { .state = ms_turn, .speed = SPEED_SAFE, .angle = 93.0 / 180.0 * M_PI },
  { .state = ms_fwd, .speed = SPEED_SAFE, .dist = 2.0, .p_stop = p_stoponcross},
  { .state = ms_fwd, .speed = SPEED_SAFE, .dist = 0.25},
  { .state = ms_turn, .speed = SPEED_SLOW, .angle = -80.0 / 180.0 * M_PI },


  
  { .state = ms_followline, .speed = SPEED_SLOW, .dist = 2.0, .is_black = 1, .line_to_follow = LINE_MIDDLE, .turning_intensity = -0.5,
    .p_stop = p_findgate},
  { .state = ms_followline, .speed = SPEED_SLOW, .dist = 0.75, .is_black = 1, .line_to_follow = LINE_MIDDLE, .turning_intensity = -0.2},

  //Make sure we're stopped
  { .state = ms_wait, .delay = 0.5},    
  
  //Turn and...
  { .state = ms_turn, .speed = SPEED_SAFE, .angle = -90 / 180.0 * M_PI },

  //Follow the wall
  { .state = ms_fwd, .speed = SPEED_SAFE, .dist = DIST_AXLE_TO_LINESENSOR},
  //Align with wall
  { .state = ms_wait, .delay = 0.0, .p_stop = p_followwall_1 },
  { .state = ms_fwd, .speed = SPEED_SAFE, .dist = 0.5 },
  { .state = ms_wait, .delay = 0.0, .p_stop = p_followwall_2 },
  { .state = ms_turn, .speed = SPEED_SAFE, .angle = 1.0 / 180.0 * M_PI, .p_stop = p_followwall_hackangle},

  { .state = ms_fwd, .speed = SPEED_SLOW, .dist = 10.0, .p_stop = p_findwall},
  { .state = ms_fwd, .speed = SPEED_SLOW, .dist = 10.0, .p_stop = p_findopening},

  //Go through the gate
  { .state = ms_fwd, .speed = SPEED_SAFE, .dist = DIST_AXLE_TO_IRSENSOR + 0.18},
  { .state = ms_turn, .speed = SPEED_SAFE, .angle = -85.0 / 180.0 * M_PI },
  { .state = ms_fwd, .speed = SPEED_SAFE, .dist = 0.8 },
  { .state = ms_turn, .speed = SPEED_SAFE, .angle = -90.0 / 180.0 * M_PI },
  { .state = ms_fwd, .speed = SPEED_SAFE, .dist = 0.1 },

  //Align to wall
  { .state = ms_wait, .delay = 0.0, .p_stop = p_followwall_1 },
  { .state = ms_fwd, .speed = SPEED_SAFE, .dist = 0.5 },
  { .state = ms_wait, .delay = 0.0, .p_stop = p_followwall_2 },
  { .state = ms_turn, .speed = SPEED_SAFE, .angle = 1.0 / 180.0 * M_PI, .p_stop = p_followwall_hackangle},

  //Follow the wall back to the line
  { .state = ms_fwd, .speed = SPEED_SAFE, .dist = 10.0, .p_stop = p_stoponcross},

  { .state = ms_fwd, .speed = SPEED_SAFE, .dist = DIST_AXLE_TO_LINESENSOR },
  { .state = ms_turn, .speed = SPEED_SAFE, .angle = -90.0 / 180.0 * M_PI },


  //Find cross before white line
  { .state = ms_followline, .speed = SPEED_SAFE, .dist = 2.0, .is_black = 1, .line_to_follow = LINE_MIDDLE,
    .p_stop = p_stoponcross},
  
  //Traverse white line (odom)
  { .state = ms_fwd, .speed = SPEED_SAFE, .dist = 0.2},
  { .state = ms_turn, .speed = SPEED_SLOW, .angle = 85.0 / 180.0 * M_PI },
  { .state = ms_fwd, .speed = SPEED_SAFE, .dist = 0.15},

  //// REPLACE WITH ACTUAL TRAVERSING
  //{ .state = ms_fwd, .speed = SPEED_SAFE, .dist = 5.0, .p_stop = p_stoponcross},
  //// REPLACE WITH ACTUAL TRAVERSING

  //{ .state = ms_fwd, .speed = SPEED_SAFE, .dist = DIST_WHITE_LINE_1},
  //{ .state = ms_turn, .speed = SPEED_SLOW, .angle = 40.0 / 180.0 * M_PI },
  //{ .state = ms_fwd, .speed = SPEED_SAFE, .dist = DIST_WHITE_LINE_2 },
  //{ .state = ms_turn, .speed = SPEED_SAFE, .angle = -90.0 / 180.0 * M_PI },
  //{ .state = ms_fwd, .speed = SPEED_SAFE, .dist = DIST_WHITE_LINE_3 },
  //{ .state = ms_turn, .speed = SPEED_SAFE, .angle = 40.0 / 180.0 * M_PI },

  //Traverse white line (odom)

  { .state = ms_followline, .speed = 0.2, .dist = 3, .is_black = 0, .line_to_follow = LINE_MIDDLE, .turning_intensity = 0.5,
    .p_stop = p_line_cross },

  

  //Transit to goal
  { .state = ms_wait, .delay = 0.2 },
  { .state = ms_followline, .speed = SPEED_SAFE, .dist = 2.0, .is_black = 1, .line_to_follow = LINE_MIDDLE,
    .p_stop = p_stoponcross},
  { .state = ms_fwd, .speed = SPEED_SAFE, .dist = DIST_AXLE_TO_LINESENSOR},
  { .state = ms_turn, .speed = SPEED_SAFE, .angle = 90.0 / 180.0 * M_PI },
  

  //Do the goal dance
  
  { .state = ms_followline, .speed = SPEED_SAFE, .dist = 10.0, .is_black = 1, .line_to_follow = LINE_MIDDLE,
    .p_stop = p_approachobject},

  { .state = ms_turn, .speed = SPEED_SLOW, .angle = 90.0 / 180.0 * M_PI },

  { .state = ms_fwd, .speed = SPEED_SAFE, .dist = (DIM_GARAGE_WIDTH/2)+DIM_SMR_SIDELENGTH+(DIM_GARAGE_DOOR/2) + 0.05},

  { .state = ms_turn, .speed = SPEED_SLOW, .angle = -85.0 / 180.0 * M_PI },

  //noget med en afstand
  //DIST_AXLE_TO_LINESENSOR + STOPDISTFROMOBJECTS + (DIM_GARAGE_LENGTH/2)
  { .state = ms_fwd, .speed = SPEED_SAFE, .dist = 1.0 },
  
  { .state = ms_turn, .speed = SPEED_SLOW, .angle = 90.0 / 180.0 * M_PI },

  { .state = ms_fwd, .speed = -SPEED_SLOW, .dist = (0.60*DIM_GARAGE_DOOR)},

  { .state = ms_turn, .speed = SPEED_SLOW, .angle = 0.9*(90.0 / 180.0 * M_PI) },

  { .state = ms_fwd, .speed = SPEED_SAFE, .dist = (DIM_GARAGE_LENGTH/2)+(DIM_GARAGE_DOOR/2) + (DIST_AXLE_TO_LINESENSOR*2.0)},

  { .state = ms_turn, .speed = SPEED_SLOW, .angle = -90.0 / 180.0 * M_PI },

  { .state = ms_fwd, .speed = -SPEED_SAFE, .dist = DIM_SMR_SIDELENGTH + DIST_AXLE_TO_LINESENSOR*0.5},

  { .state = ms_turn, .speed = SPEED_SLOW, .angle = 90.0 / 180.0 * M_PI },

  { .state = ms_fwd, .speed = -SPEED_SAFE, .dist = DIM_SMR_SIDELENGTH + (DIST_AXLE_TO_LINESENSOR*1.5) + 0.05},
  
};


/*
StateParam conf_comp_old[] = {
  
  //Measure box
  { .state = ms_fwd, .speed = SPEED_SAFE, .dist = 0.4},
  { .state = ms_wait, .delay = 0.2 },
  { .state = ms_wait, .delay = 0.0, .p_stop = p_print_dist_to_box },

  //First transit
  { .state = ms_turn, .speed = SPEED_SAFE, .angle = 45.0 / 180.0 * M_PI },
  { .state = ms_followline, .speed = SPEED_SAFE, .dist = 1.3, .is_black = 1, .line_to_follow = LINE_RIGHT},
  { .state = ms_wait, .delay = 0.2 },
  { .state = ms_followline, .speed = SPEED_RECKLESS, .dist = 2.0, .is_black = 1, .line_to_follow = LINE_MIDDLE},

  //Push the box & go through gate
  { .state = ms_followline, .speed = SPEED_SAFE, .dist = 8.0, .is_black = 1, .line_to_follow = LINE_MIDDLE,
    .p_stop = p_stoponcross},
  { .state = ms_fwd, .speed = SPEED_SAFE, .dist = 0.5},
  { .state = ms_fwd, .speed = -SPEED_SAFE, .dist = 1.5},
  { .state = ms_turn, .speed = SPEED_SAFE, .angle = 90.0 / 180.0 * M_PI },
  { .state = ms_fwd, .speed = SPEED_SAFE, .dist = 1.5, .p_stop = p_stoponline},
  { .state = ms_fwd, .speed = SPEED_SAFE, .dist = 0.3},
  { .state = ms_turn, .speed = SPEED_SAFE, .angle = -90.0 / 180.0 * M_PI },
  { .state = ms_followline, .speed = SPEED_RECKLESS, .dist = 2.0, .is_black = 1, .line_to_follow = LINE_MIDDLE,
    .p_stop = p_stoponcross},
  { .state = ms_fwd, .speed = SPEED_SAFE, .dist = 0.3},


  //Transit to loose gate
  { .state = ms_followline, .speed = SPEED_SAFE, .dist = 1.0, .is_black = 1, .line_to_follow = LINE_LEFT},
  { .state = ms_followline, .speed = SPEED_RECKLESS, .dist = 2.0, .is_black = 1, .line_to_follow = LINE_MIDDLE,
    .p_stop = p_stoponcross},
  { .state = ms_fwd, .speed = SPEED_SAFE, .dist = 0.3},

  //Find gate and...
  { .state = ms_followline, .speed = SPEED_SAFE, .dist = 10.0, .is_black = 1, .line_to_follow = LINE_MIDDLE,
    .p_stop = p_findwall},
  { .state = ms_followline, .speed = SPEED_SAFE, .dist = 10.0, .is_black = 1, .line_to_follow = LINE_MIDDLE,
    .p_stop = p_findopening},

  //...Go through it
  { .state = ms_fwd, .speed = SPEED_SAFE, .dist = 0.5 },
  { .state = ms_turn, .speed = SPEED_SAFE, .angle = -90.0 / 180.0 * M_PI },
  { .state = ms_fwd, .speed = SPEED_SAFE, .dist = 1.0 },
  
  

  
  //Transit to wall following
  
  { .state = ms_turn, .speed = SPEED_SAFE, .angle = 80.0 / 180.0 * M_PI },
  { .state = ms_fwd, .speed = SPEED_SAFE, .dist = 2.0, .p_stop = p_stoponline},
  { .state = ms_fwd, .speed = SPEED_SAFE, .dist = 0.3},
  { .state = ms_turn, .speed = SPEED_SAFE, .angle = -80.0 / 180.0 * M_PI },
  
  { .state = ms_followline, .speed = SPEED_SAFE, .dist = 2.0, .is_black = 1, .line_to_follow = LINE_MIDDLE,
    .p_stop = p_stoponcross},
  { .state = ms_fwd, .speed = SPEED_SAFE, .dist = 0.3},

  //Make sure we're stopped
  { .state = ms_wait, .delay = 0.5},    
  
  //Turn and...
  { .state = ms_turn, .speed = SPEED_SAFE, .angle = -90 / 180.0 * M_PI },

  //Follow the wall
  { .state = ms_fwd, .speed = SPEED_SAFE, .dist = DIST_AXLE_TO_LINESENSOR},
  //Align with wall
  { .state = ms_wait, .delay = 0.0, .p_stop = p_followwall_1 },
  { .state = ms_fwd, .speed = SPEED_SAFE, .dist = 0.5 },
  { .state = ms_wait, .delay = 0.0, .p_stop = p_followwall_2 },
  { .state = ms_turn, .speed = SPEED_SAFE, .angle = 1.0 / 180.0 * M_PI, .p_stop = p_followwall_hackangle},

  { .state = ms_fwd, .speed = SPEED_SAFE, .dist = 10.0, .p_stop = p_findwall},
  { .state = ms_fwd, .speed = SPEED_SAFE, .dist = 10.0, .p_stop = p_findopening},

  //Go through the gate
  { .state = ms_fwd, .speed = SPEED_SAFE, .dist = DIST_AXLE_TO_IRSENSOR + 0.08},
  { .state = ms_turn, .speed = SPEED_SAFE, .angle = -90.0 / 180.0 * M_PI },
  { .state = ms_fwd, .speed = SPEED_SAFE, .dist = 1.0 },
  { .state = ms_turn, .speed = SPEED_SAFE, .angle = -90.0 / 180.0 * M_PI },

  //Align to wall
  { .state = ms_wait, .delay = 0.0, .p_stop = p_followwall_1 },
  { .state = ms_fwd, .speed = SPEED_SAFE, .dist = 0.5 },
  { .state = ms_wait, .delay = 0.0, .p_stop = p_followwall_2 },
  { .state = ms_turn, .speed = SPEED_SAFE, .angle = 1.0 / 180.0 * M_PI, .p_stop = p_followwall_hackangle},

  //Follow the wall back to the line
  { .state = ms_fwd, .speed = SPEED_SAFE, .dist = 10.0, .p_stop = p_stoponcross},

  { .state = ms_fwd, .speed = SPEED_SAFE, .dist = DIST_AXLE_TO_LINESENSOR },
  { .state = ms_turn, .speed = SPEED_SAFE, .angle = -90.0 / 180.0 * M_PI },

  //Find cross before white line
  { .state = ms_followline, .speed = 0.5, .dist = 2.0, .is_black = 1, .line_to_follow = LINE_MIDDLE,
    .p_stop = p_stoponcross},
  


  //Traverse white line (odom)
  //{ .state = ms_fwd, .speed = SPEED_SAFE, .dist = 1.5},
  //{ .state = ms_turn, .speed = SPEED_SAFE, .angle = -90.0 / 180.0 * M_PI },
  //{ .state = ms_fwd, .speed = SPEED_SAFE, .dist = 10.0, .p_stop = p_stoponcross },
  //{ .state = ms_fwd, .speed = SPEED_SAFE, .dist = DIST_AXLE_TO_LINESENSOR},
  //{ .state = ms_turn, .speed = SPEED_SAFE, .angle = 90.0 / 180.0 * M_PI },
  
  /// TEMP
  


  ///

  //Do the goal dance
  { .state = ms_followline, .speed = SPEED_SAFE, .dist = 10.0, .is_black = 1, .line_to_follow = LINE_MIDDLE,
    .p_stop = p_stoponcross},
  { .state = ms_fwd, .speed = SPEED_SLOW, .dist = 0.2},

  { .state = ms_followline, .speed = SPEED_SLOW, .dist = 10.0, .is_black = 1, .line_to_follow = LINE_MIDDLE,
    .p_stop = p_approachobject},

  { .state = ms_turn, .speed = SPEED_SLOW, .angle = 90.0 / 180.0 * M_PI },

  { .state = ms_fwd, .speed = SPEED_SAFE, .dist = (DIM_GARAGE_WIDTH/2)+DIM_SMR_SIDELENGTH+(DIM_GARAGE_DOOR/2) + 0.05},

  { .state = ms_turn, .speed = SPEED_SLOW, .angle = -90.0 / 180.0 * M_PI },

  //noget med en afstand
  //DIST_AXLE_TO_LINESENSOR + STOPDISTFROMOBJECTS + (DIM_GARAGE_LENGTH/2)
  { .state = ms_fwd, .speed = SPEED_SAFE, .dist = 1.0 },
  
  { .state = ms_turn, .speed = SPEED_SLOW, .angle = 90.0 / 180.0 * M_PI },

  { .state = ms_fwd, .speed = -SPEED_SLOW, .dist = (0.75*DIM_GARAGE_DOOR)},

  { .state = ms_turn, .speed = SPEED_SLOW, .angle = 0.9*(90.0 / 180.0 * M_PI) },

  { .state = ms_fwd, .speed = SPEED_SAFE, .dist = (DIM_GARAGE_LENGTH/2)+(DIM_GARAGE_DOOR/2) + (DIST_AXLE_TO_LINESENSOR*2.0)},

  { .state = ms_turn, .speed = SPEED_SLOW, .angle = -90.0 / 180.0 * M_PI },

  { .state = ms_fwd, .speed = -SPEED_SAFE, .dist = DIM_SMR_SIDELENGTH + DIST_AXLE_TO_LINESENSOR*0.5},

  { .state = ms_turn, .speed = SPEED_SLOW, .angle = 90.0 / 180.0 * M_PI },

  { .state = ms_fwd, .speed = -SPEED_SAFE, .dist = DIM_SMR_SIDELENGTH + (DIST_AXLE_TO_LINESENSOR*1.5) + 0.05},
  
};
*/

#endif


































