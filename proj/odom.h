#ifndef __ODOM_HEADER__
#define __ODOM_HEADER__

#include "types.h"

/*****************************************
  odometry
*/
#define WHEEL_DIAMETER   0.06522  /* m */
#define WHEEL_SEPARATION 0.26 /* m */
#define DELTA_M (M_PI * WHEEL_DIAMETER / 2000)

void reset_odo(odotype *p);
void update_odo(odotype *p);

#endif
