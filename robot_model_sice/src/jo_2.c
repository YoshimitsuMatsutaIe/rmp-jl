#include "jo_2.h"
#include <math.h>

void jo_2(double l1, double l2, double *q, double *out_1662876426189495411) {

   out_1662876426189495411[0] = -l1*sin(q[0]) - l2*sin(q[0])*cos(q[1]) - l2*sin(q[1])*cos(q[0]);
   out_1662876426189495411[1] = -l2*sin(q[0])*cos(q[1]) - l2*sin(q[1])*cos(q[0]);
   out_1662876426189495411[2] = 0;
   out_1662876426189495411[3] = 0;
   out_1662876426189495411[4] = l1*cos(q[0]) - l2*sin(q[0])*sin(q[1]) + l2*cos(q[0])*cos(q[1]);
   out_1662876426189495411[5] = -l2*sin(q[0])*sin(q[1]) + l2*cos(q[0])*cos(q[1]);
   out_1662876426189495411[6] = 0;
   out_1662876426189495411[7] = 0;

}
