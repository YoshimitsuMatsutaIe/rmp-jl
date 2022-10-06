#include "jrx_1_dot.h"
#include <math.h>

void jrx_1_dot(double *dq, double *q, double *out_6337063650590123624) {

   out_6337063650590123624[0] = sin(q[0])*sin(q[1])*dq[0] + sin(q[0])*sin(q[1])*dq[1] - cos(q[0])*cos(q[1])*dq[0] - cos(q[0])*cos(q[1])*dq[1];
   out_6337063650590123624[1] = sin(q[0])*sin(q[1])*dq[0] + sin(q[0])*sin(q[1])*dq[1] - cos(q[0])*cos(q[1])*dq[0] - cos(q[0])*cos(q[1])*dq[1];
   out_6337063650590123624[2] = 0;
   out_6337063650590123624[3] = 0;
   out_6337063650590123624[4] = -sin(q[0])*cos(q[1])*dq[0] - sin(q[0])*cos(q[1])*dq[1] - sin(q[1])*cos(q[0])*dq[0] - sin(q[1])*cos(q[0])*dq[1];
   out_6337063650590123624[5] = -sin(q[0])*cos(q[1])*dq[0] - sin(q[0])*cos(q[1])*dq[1] - sin(q[1])*cos(q[0])*dq[0] - sin(q[1])*cos(q[0])*dq[1];
   out_6337063650590123624[6] = 0;
   out_6337063650590123624[7] = 0;

}
