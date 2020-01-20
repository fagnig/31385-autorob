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
  printf("DIST TO BOX: %f\n", dat.laserpar[0]);
  return 1;
}
int p_stoponcross(PredicateData dat) {
  return dat.lindat->crossing_line_b;
}

const double max_dist_to_wall = 1.0;
double dist_to_wall = 0.0;
int p_findwall(PredicateData dat) {
  if (dat.irpar[4] > 0.01 && dat.irpar[4] < max_dist_to_wall) {
    dist_to_wall = dat.irpar[4] + 0.05;
    printf("Wall found %f\n", dist_to_wall);
    return 1;
  }
  return 0;
}
int p_findopening(PredicateData dat) {
  if(dat.irpar[4] > dist_to_wall) printf("Wall found %f\n", dist_to_wall);
  return dat.irpar[4] > dist_to_wall;
}

int p_stoponline(PredicateData dat){
  return dat.lindat->numlines_b;
}

#define STOPDISTFROMOBJECTS 0.2
int p_approachobject(PredicateData dat){
  return dat.irpar[2] < STOPDISTFROMOBJECTS;
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
  dat.mot->angle = -followwall_angle;
  return 0;
}

StateParam conf_comp[] = {
  /*
  //Measure box
  { .state = ms_fwd, .speed = 0.3, .dist = 0.4},
  { .state = ms_wait, .delay = 0.2 },
  { .state = ms_wait, .delay = 0.0, .p_stop = p_print_dist_to_box },

  //First transit
  { .state = ms_turn, .speed = 0.3, .angle = 45.0 / 180.0 * M_PI },
  { .state = ms_followline, .speed = 0.3, .dist = 1.0, .is_black = 1, .line_to_follow = LINE_RIGHT},
  { .state = ms_wait, .delay = 0.2 },
  { .state = ms_followline, .speed = 0.5, .dist = 2.0, .is_black = 1, .line_to_follow = LINE_MIDDLE},

  //Push the box & go through gate
  { .state = ms_followline, .speed = 0.3, .dist = 8.0, .is_black = 1, .line_to_follow = LINE_MIDDLE,
    .p_stop = p_stoponcross},
  { .state = ms_fwd, .speed = 0.3, .dist = 0.5},
  { .state = ms_fwd, .speed = -0.3, .dist = 1.5},
  { .state = ms_turn, .speed = 0.3, .angle = 90.0 / 180.0 * M_PI },
  { .state = ms_fwd, .speed = 0.3, .dist = 1.5, .p_stop = p_stoponline},
  { .state = ms_fwd, .speed = 0.3, .dist = 0.3},
  { .state = ms_turn, .speed = 0.3, .angle = -90.0 / 180.0 * M_PI },
  { .state = ms_followline, .speed = 0.5, .dist = 2.0, .is_black = 1, .line_to_follow = LINE_MIDDLE,
    .p_stop = p_stoponcross},
  { .state = ms_fwd, .speed = 0.3, .dist = 0.3},


  //Transit to loose gate
  { .state = ms_followline, .speed = 0.3, .dist = 1.0, .is_black = 1, .line_to_follow = LINE_LEFT},
  { .state = ms_followline, .speed = 0.5, .dist = 2.0, .is_black = 1, .line_to_follow = LINE_MIDDLE,
    .p_stop = p_stoponcross},
  { .state = ms_fwd, .speed = 0.3, .dist = 0.3},

  //Find gate and...
  { .state = ms_followline, .speed = 0.3, .dist = 10.0, .is_black = 1, .line_to_follow = LINE_MIDDLE,
    .p_stop = p_findwall},
  { .state = ms_followline, .speed = 0.3, .dist = 10.0, .is_black = 1, .line_to_follow = LINE_MIDDLE,
    .p_stop = p_findopening},

  //...Go through it
  { .state = ms_fwd, .speed = 0.3, .dist = 0.5 },
  { .state = ms_turn, .speed = 0.3, .angle = -90.0 / 180.0 * M_PI },
  { .state = ms_fwd, .speed = 0.3, .dist = 1.0 },
  
  */

  /*
  //Transit to wall following
  { .state = ms_turn, .speed = SPEED_SAFE, .angle = 80.0 / 180.0 * M_PI },
  { .state = ms_fwd, .speed = SPEED_SAFE, .dist = 2.0, .p_stop = p_stoponline},
  { .state = ms_fwd, .speed = SPEED_SAFE, .dist = 0.3},
  { .state = ms_turn, .speed = SPEED_SAFE, .angle = -80.0 / 180.0 * M_PI },
  { .state = ms_followline, .speed = 0.5, .dist = 2.0, .is_black = 1, .line_to_follow = LINE_MIDDLE,
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
  { .state = ms_fwd, .speed = SPEED_SAFE, .dist = DIST_AXLE_TO_IRSENSOR*2 },
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
  */
  
  //Traverse white line (odom)

    
  /// TEMP
  { .state = ms_followline, .speed = SPEED_SLOW, .dist = 10.0, .is_black = 1, .line_to_follow = LINE_MIDDLE,
    .p_stop = p_stoponcross},
  ///

  //Do the goal dance
  //{ .state = ms_followline, .speed = SPEED_SAFE, .dist = 10.0, .is_black = 1, .line_to_follow = LINE_MIDDLE,
  //  .p_stop = p_stoponcross},
  { .state = ms_fwd, .speed = SPEED_SLOW, .dist = 0.2},

  { .state = ms_followline, .speed = SPEED_SLOW, .dist = 10.0, .is_black = 1, .line_to_follow = LINE_MIDDLE,
    .p_stop = p_approachobject},

  { .state = ms_turn, .speed = SPEED_SLOW, .angle = 90.0 / 180.0 * M_PI },

  { .state = ms_fwd, .speed = SPEED_SAFE, .dist = (DIM_GARAGE_WIDTH/2)+DIM_SMR_SIDELENGTH+(DIM_GARAGE_DOOR/2) + 0.05},

  { .state = ms_turn, .speed = SPEED_SLOW, .angle = -90.0 / 180.0 * M_PI },

  { .state = ms_fwd, .speed = SPEED_SAFE, .dist = 1.0/*DIST_AXLE_TO_LINESENSOR + STOPDISTFROMOBJECTS + (DIM_GARAGE_LENGTH/2)*/},

  { .state = ms_turn, .speed = SPEED_SLOW, .angle = 90.0 / 180.0 * M_PI },

  { .state = ms_fwd, .speed = -SPEED_SLOW, .dist = (0.75*DIM_GARAGE_DOOR)},

  { .state = ms_turn, .speed = SPEED_SLOW, .angle = 0.9*(90.0 / 180.0 * M_PI) },

  { .state = ms_fwd, .speed = SPEED_SAFE, .dist = (DIM_GARAGE_LENGTH/2)+(DIM_GARAGE_DOOR/2) + (DIST_AXLE_TO_LINESENSOR*2.0)},

  { .state = ms_turn, .speed = SPEED_SLOW, .angle = -90.0 / 180.0 * M_PI },

  { .state = ms_fwd, .speed = -SPEED_SAFE, .dist = DIM_SMR_SIDELENGTH + DIST_AXLE_TO_LINESENSOR*0.5},

  { .state = ms_turn, .speed = SPEED_SLOW, .angle = 90.0 / 180.0 * M_PI },

  { .state = ms_fwd, .speed = -SPEED_SAFE, .dist = DIM_SMR_SIDELENGTH + (DIST_AXLE_TO_LINESENSOR*1.5) + 0.05},

};


#endif