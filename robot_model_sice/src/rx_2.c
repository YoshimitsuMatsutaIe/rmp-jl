#include "rx_2.h"
#include <math.h>

void rx_2(double *q, double *out_8263844185774076682) {

   out_8263844185774076682[0] = -sin(q[0])*sin(q[1])*cos(q[2]) - sin(q[0])*sin(q[2])*cos(q[1]) - sin(q[1])*sin(q[2])*cos(q[0]) + cos(q[0])*cos(q[1])*cos(q[2]);
   out_8263844185774076682[1] = -sin(q[0])*sin(q[1])*sin(q[2]) + sin(q[0])*cos(q[1])*cos(q[2]) + sin(q[1])*cos(q[0])*cos(q[2]) + sin(q[2])*cos(q[0])*cos(q[1]);

}
