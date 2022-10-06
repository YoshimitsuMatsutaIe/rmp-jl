#include "jry_1_dot.h"
#include <math.h>

void jry_1_dot(double *dq, double *q, double *out_4671415976930104851) {

   out_4671415976930104851[0] = sin(q[0])*cos(q[1])*dq[0] + sin(q[0])*cos(q[1])*dq[1] + sin(q[1])*cos(q[0])*dq[0] + sin(q[1])*cos(q[0])*dq[1];
   out_4671415976930104851[1] = sin(q[0])*cos(q[1])*dq[0] + sin(q[0])*cos(q[1])*dq[1] + sin(q[1])*cos(q[0])*dq[0] + sin(q[1])*cos(q[0])*dq[1];
   out_4671415976930104851[2] = 0;
   out_4671415976930104851[3] = 0;
   out_4671415976930104851[4] = sin(q[0])*sin(q[1])*dq[0] + sin(q[0])*sin(q[1])*dq[1] - cos(q[0])*cos(q[1])*dq[0] - cos(q[0])*cos(q[1])*dq[1];
   out_4671415976930104851[5] = sin(q[0])*sin(q[1])*dq[0] + sin(q[0])*sin(q[1])*dq[1] - cos(q[0])*cos(q[1])*dq[0] - cos(q[0])*cos(q[1])*dq[1];
   out_4671415976930104851[6] = 0;
   out_4671415976930104851[7] = 0;

}
