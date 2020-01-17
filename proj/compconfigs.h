#ifndef __COMPCONFIGS_HEADER__
#define __COMPCONFIGS_HEADER__

#include "types.h"

#include <math.h>

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
  if (dat.laserpar[8] > 0.01 && dat.laserpar[8] < max_dist_to_wall) {
    dist_to_wall = dat.laserpar[8] + 0.05;
    printf("Wall found %f\n", dist_to_wall);
    return 1;
  }
  return 0;
}
int p_findopening(PredicateData dat) {
  if(dat.laserpar[8] > dist_to_wall) printf("Wall found %f\n", dist_to_wall);
  return dat.laserpar[8] > dist_to_wall;
}

int p_stoponline(PredicateData dat){
  return dat.lindat->numlines_b;
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

  { .state = ms_followline, .speed = 0.3, .dist = 10.0, .is_black = 1, .line_to_follow = LINE_MIDDLE,
    .p_stop = p_findwall},
  { .state = ms_followline, .speed = 0.3, .dist = 10.0, .is_black = 1, .line_to_follow = LINE_MIDDLE,
    .p_stop = p_findopening},

  { .state = ms_fwd, .speed = 0.3, .dist = 0.5 },
  { .state = ms_turn, .speed = 0.3, .angle = -90.0 / 180.0 * M_PI },
  { .state = ms_fwd, .speed = 0.3, .dist = 1.0 },
  */

  { .state = ms_turn, .speed = 0.3, .angle = 80.0 / 180.0 * M_PI },
  { .state = ms_fwd, .speed = 0.3, .dist = 2.0, .p_stop = p_stoponline},
  { .state = ms_fwd, .speed = 0.3, .dist = 0.3},
  { .state = ms_turn, .speed = 0.3, .angle = -80.0 / 180.0 * M_PI },
  { .state = ms_followline, .speed = 0.5, .dist = 2.0, .is_black = 1, .line_to_follow = LINE_MIDDLE,
    .p_stop = p_stoponcross},
  { .state = ms_fwd, .speed = 0.3, .dist = 0.3},


  { .state = ms_wait, .delay = 0.5},    

  { .state = ms_turn, .speed = 0.3, .angle = -90 / 180.0 * M_PI },

  { .state = ms_fwd, .speed = 0.3, .dist = 10.0, .p_stop = p_findwall},
  { .state = ms_fwd, .speed = 0.3, .dist = 10.0, .p_stop = p_findopening},

  { .state = ms_fwd, .speed = 0.3, .dist = 0.5 },
  { .state = ms_turn, .speed = 0.3, .angle = -90.0 / 180.0 * M_PI },
  { .state = ms_fwd, .speed = 0.3, .dist = 1.0 },
  { .state = ms_turn, .speed = 0.3, .angle = -90.0 / 180.0 * M_PI },

  { .state = ms_fwd, .speed = 0.3, .dist = 10.0, .p_stop = p_findwall},
  { .state = ms_fwd, .speed = 0.3, .dist = 10.0, .p_stop = p_findopening},

  { .state = ms_fwd, .speed = 0.3, .dist = 0.5 },
  { .state = ms_turn, .speed = 0.3, .angle = -90.0 / 180.0 * M_PI },
  { .state = ms_fwd, .speed = 0.3, .dist = 1.0 },



};


#endif