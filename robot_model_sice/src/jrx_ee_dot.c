#include "jrx_ee_dot.h"
#include <math.h>

void jrx_ee_dot(double *dq, double *q, double *out_6659462614077130505) {

   out_6659462614077130505[0] = -sin(q[0])*sin(q[1])*sin(q[2])*sin(q[3])*dq[0] - sin(q[0])*sin(q[1])*sin(q[2])*sin(q[3])*dq[1] - sin(q[0])*sin(q[1])*sin(q[2])*sin(q[3])*dq[2] - sin(q[0])*sin(q[1])*sin(q[2])*sin(q[3])*dq[3] + sin(q[0])*sin(q[1])*cos(q[2])*cos(q[3])*dq[0] + sin(q[0])*sin(q[1])*cos(q[2])*cos(q[3])*dq[1] + sin(q[0])*sin(q[1])*cos(q[2])*cos(q[3])*dq[2] + sin(q[0])*sin(q[1])*cos(q[2])*cos(q[3])*dq[3] + sin(q[0])*sin(q[2])*cos(q[1])*cos(q[3])*dq[0] + sin(q[0])*sin(q[2])*cos(q[1])*cos(q[3])*dq[1] + sin(q[0])*sin(q[2])*cos(q[1])*cos(q[3])*dq[2] + sin(q[0])*sin(q[2])*cos(q[1])*cos(q[3])*dq[3] + sin(q[0])*sin(q[3])*cos(q[1])*cos(q[2])*dq[0] + sin(q[0])*sin(q[3])*cos(q[1])*cos(q[2])*dq[1] + sin(q[0])*sin(q[3])*cos(q[1])*cos(q[2])*dq[2] + sin(q[0])*sin(q[3])*cos(q[1])*cos(q[2])*dq[3] + sin(q[1])*sin(q[2])*cos(q[0])*cos(q[3])*dq[0] + sin(q[1])*sin(q[2])*cos(q[0])*cos(q[3])*dq[1] + sin(q[1])*sin(q[2])*cos(q[0])*cos(q[3])*dq[2] + sin(q[1])*sin(q[2])*cos(q[0])*cos(q[3])*dq[3] + sin(q[1])*sin(q[3])*cos(q[0])*cos(q[2])*dq[0] + sin(q[1])*sin(q[3])*cos(q[0])*cos(q[2])*dq[1] + sin(q[1])*sin(q[3])*cos(q[0])*cos(q[2])*dq[2] + sin(q[1])*sin(q[3])*cos(q[0])*cos(q[2])*dq[3] + sin(q[2])*sin(q[3])*cos(q[0])*cos(q[1])*dq[0] + sin(q[2])*sin(q[3])*cos(q[0])*cos(q[1])*dq[1] + sin(q[2])*sin(q[3])*cos(q[0])*cos(q[1])*dq[2] + sin(q[2])*sin(q[3])*cos(q[0])*cos(q[1])*dq[3] - cos(q[0])*cos(q[1])*cos(q[2])*cos(q[3])*dq[0] - cos(q[0])*cos(q[1])*cos(q[2])*cos(q[3])*dq[1] - cos(q[0])*cos(q[1])*cos(q[2])*cos(q[3])*dq[2] - cos(q[0])*cos(q[1])*cos(q[2])*cos(q[3])*dq[3];
   out_6659462614077130505[1] = -sin(q[0])*sin(q[1])*sin(q[2])*sin(q[3])*dq[0] - sin(q[0])*sin(q[1])*sin(q[2])*sin(q[3])*dq[1] - sin(q[0])*sin(q[1])*sin(q[2])*sin(q[3])*dq[2] - sin(q[0])*sin(q[1])*sin(q[2])*sin(q[3])*dq[3] + sin(q[0])*sin(q[1])*cos(q[2])*cos(q[3])*dq[0] + sin(q[0])*sin(q[1])*cos(q[2])*cos(q[3])*dq[1] + sin(q[0])*sin(q[1])*cos(q[2])*cos(q[3])*dq[2] + sin(q[0])*sin(q[1])*cos(q[2])*cos(q[3])*dq[3] + sin(q[0])*sin(q[2])*cos(q[1])*cos(q[3])*dq[0] + sin(q[0])*sin(q[2])*cos(q[1])*cos(q[3])*dq[1] + sin(q[0])*sin(q[2])*cos(q[1])*cos(q[3])*dq[2] + sin(q[0])*sin(q[2])*cos(q[1])*cos(q[3])*dq[3] + sin(q[0])*sin(q[3])*cos(q[1])*cos(q[2])*dq[0] + sin(q[0])*sin(q[3])*cos(q[1])*cos(q[2])*dq[1] + sin(q[0])*sin(q[3])*cos(q[1])*cos(q[2])*dq[2] + sin(q[0])*sin(q[3])*cos(q[1])*cos(q[2])*dq[3] + sin(q[1])*sin(q[2])*cos(q[0])*cos(q[3])*dq[0] + sin(q[1])*sin(q[2])*cos(q[0])*cos(q[3])*dq[1] + sin(q[1])*sin(q[2])*cos(q[0])*cos(q[3])*dq[2] + sin(q[1])*sin(q[2])*cos(q[0])*cos(q[3])*dq[3] + sin(q[1])*sin(q[3])*cos(q[0])*cos(q[2])*dq[0] + sin(q[1])*sin(q[3])*cos(q[0])*cos(q[2])*dq[1] + sin(q[1])*sin(q[3])*cos(q[0])*cos(q[2])*dq[2] + sin(q[1])*sin(q[3])*cos(q[0])*cos(q[2])*dq[3] + sin(q[2])*sin(q[3])*cos(q[0])*cos(q[1])*dq[0] + sin(q[2])*sin(q[3])*cos(q[0])*cos(q[1])*dq[1] + sin(q[2])*sin(q[3])*cos(q[0])*cos(q[1])*dq[2] + sin(q[2])*sin(q[3])*cos(q[0])*cos(q[1])*dq[3] - cos(q[0])*cos(q[1])*cos(q[2])*cos(q[3])*dq[0] - cos(q[0])*cos(q[1])*cos(q[2])*cos(q[3])*dq[1] - cos(q[0])*cos(q[1])*cos(q[2])*cos(q[3])*dq[2] - cos(q[0])*cos(q[1])*cos(q[2])*cos(q[3])*dq[3];
   out_6659462614077130505[2] = -sin(q[0])*sin(q[1])*sin(q[2])*sin(q[3])*dq[0] - sin(q[0])*sin(q[1])*sin(q[2])*sin(q[3])*dq[1] - sin(q[0])*sin(q[1])*sin(q[2])*sin(q[3])*dq[2] - sin(q[0])*sin(q[1])*sin(q[2])*sin(q[3])*dq[3] + sin(q[0])*sin(q[1])*cos(q[2])*cos(q[3])*dq[0] + sin(q[0])*sin(q[1])*cos(q[2])*cos(q[3])*dq[1] + sin(q[0])*sin(q[1])*cos(q[2])*cos(q[3])*dq[2] + sin(q[0])*sin(q[1])*cos(q[2])*cos(q[3])*dq[3] + sin(q[0])*sin(q[2])*cos(q[1])*cos(q[3])*dq[0] + sin(q[0])*sin(q[2])*cos(q[1])*cos(q[3])*dq[1] + sin(q[0])*sin(q[2])*cos(q[1])*cos(q[3])*dq[2] + sin(q[0])*sin(q[2])*cos(q[1])*cos(q[3])*dq[3] + sin(q[0])*sin(q[3])*cos(q[1])*cos(q[2])*dq[0] + sin(q[0])*sin(q[3])*cos(q[1])*cos(q[2])*dq[1] + sin(q[0])*sin(q[3])*cos(q[1])*cos(q[2])*dq[2] + sin(q[0])*sin(q[3])*cos(q[1])*cos(q[2])*dq[3] + sin(q[1])*sin(q[2])*cos(q[0])*cos(q[3])*dq[0] + sin(q[1])*sin(q[2])*cos(q[0])*cos(q[3])*dq[1] + sin(q[1])*sin(q[2])*cos(q[0])*cos(q[3])*dq[2] + sin(q[1])*sin(q[2])*cos(q[0])*cos(q[3])*dq[3] + sin(q[1])*sin(q[3])*cos(q[0])*cos(q[2])*dq[0] + sin(q[1])*sin(q[3])*cos(q[0])*cos(q[2])*dq[1] + sin(q[1])*sin(q[3])*cos(q[0])*cos(q[2])*dq[2] + sin(q[1])*sin(q[3])*cos(q[0])*cos(q[2])*dq[3] + sin(q[2])*sin(q[3])*cos(q[0])*cos(q[1])*dq[0] + sin(q[2])*sin(q[3])*cos(q[0])*cos(q[1])*dq[1] + sin(q[2])*sin(q[3])*cos(q[0])*cos(q[1])*dq[2] + sin(q[2])*sin(q[3])*cos(q[0])*cos(q[1])*dq[3] - cos(q[0])*cos(q[1])*cos(q[2])*cos(q[3])*dq[0] - cos(q[0])*cos(q[1])*cos(q[2])*cos(q[3])*dq[1] - cos(q[0])*cos(q[1])*cos(q[2])*cos(q[3])*dq[2] - cos(q[0])*cos(q[1])*cos(q[2])*cos(q[3])*dq[3];
   out_6659462614077130505[3] = -sin(q[0])*sin(q[1])*sin(q[2])*sin(q[3])*dq[0] - sin(q[0])*sin(q[1])*sin(q[2])*sin(q[3])*dq[1] - sin(q[0])*sin(q[1])*sin(q[2])*sin(q[3])*dq[2] - sin(q[0])*sin(q[1])*sin(q[2])*sin(q[3])*dq[3] + sin(q[0])*sin(q[1])*cos(q[2])*cos(q[3])*dq[0] + sin(q[0])*sin(q[1])*cos(q[2])*cos(q[3])*dq[1] + sin(q[0])*sin(q[1])*cos(q[2])*cos(q[3])*dq[2] + sin(q[0])*sin(q[1])*cos(q[2])*cos(q[3])*dq[3] + sin(q[0])*sin(q[2])*cos(q[1])*cos(q[3])*dq[0] + sin(q[0])*sin(q[2])*cos(q[1])*cos(q[3])*dq[1] + sin(q[0])*sin(q[2])*cos(q[1])*cos(q[3])*dq[2] + sin(q[0])*sin(q[2])*cos(q[1])*cos(q[3])*dq[3] + sin(q[0])*sin(q[3])*cos(q[1])*cos(q[2])*dq[0] + sin(q[0])*sin(q[3])*cos(q[1])*cos(q[2])*dq[1] + sin(q[0])*sin(q[3])*cos(q[1])*cos(q[2])*dq[2] + sin(q[0])*sin(q[3])*cos(q[1])*cos(q[2])*dq[3] + sin(q[1])*sin(q[2])*cos(q[0])*cos(q[3])*dq[0] + sin(q[1])*sin(q[2])*cos(q[0])*cos(q[3])*dq[1] + sin(q[1])*sin(q[2])*cos(q[0])*cos(q[3])*dq[2] + sin(q[1])*sin(q[2])*cos(q[0])*cos(q[3])*dq[3] + sin(q[1])*sin(q[3])*cos(q[0])*cos(q[2])*dq[0] + sin(q[1])*sin(q[3])*cos(q[0])*cos(q[2])*dq[1] + sin(q[1])*sin(q[3])*cos(q[0])*cos(q[2])*dq[2] + sin(q[1])*sin(q[3])*cos(q[0])*cos(q[2])*dq[3] + sin(q[2])*sin(q[3])*cos(q[0])*cos(q[1])*dq[0] + sin(q[2])*sin(q[3])*cos(q[0])*cos(q[1])*dq[1] + sin(q[2])*sin(q[3])*cos(q[0])*cos(q[1])*dq[2] + sin(q[2])*sin(q[3])*cos(q[0])*cos(q[1])*dq[3] - cos(q[0])*cos(q[1])*cos(q[2])*cos(q[3])*dq[0] - cos(q[0])*cos(q[1])*cos(q[2])*cos(q[3])*dq[1] - cos(q[0])*cos(q[1])*cos(q[2])*cos(q[3])*dq[2] - cos(q[0])*cos(q[1])*cos(q[2])*cos(q[3])*dq[3];
   out_6659462614077130505[4] = sin(q[0])*sin(q[1])*sin(q[2])*cos(q[3])*dq[0] + sin(q[0])*sin(q[1])*sin(q[2])*cos(q[3])*dq[1] + sin(q[0])*sin(q[1])*sin(q[2])*cos(q[3])*dq[2] + sin(q[0])*sin(q[1])*sin(q[2])*cos(q[3])*dq[3] + sin(q[0])*sin(q[1])*sin(q[3])*cos(q[2])*dq[0] + sin(q[0])*sin(q[1])*sin(q[3])*cos(q[2])*dq[1] + sin(q[0])*sin(q[1])*sin(q[3])*cos(q[2])*dq[2] + sin(q[0])*sin(q[1])*sin(q[3])*cos(q[2])*dq[3] + sin(q[0])*sin(q[2])*sin(q[3])*cos(q[1])*dq[0] + sin(q[0])*sin(q[2])*sin(q[3])*cos(q[1])*dq[1] + sin(q[0])*sin(q[2])*sin(q[3])*cos(q[1])*dq[2] + sin(q[0])*sin(q[2])*sin(q[3])*cos(q[1])*dq[3] - sin(q[0])*cos(q[1])*cos(q[2])*cos(q[3])*dq[0] - sin(q[0])*cos(q[1])*cos(q[2])*cos(q[3])*dq[1] - sin(q[0])*cos(q[1])*cos(q[2])*cos(q[3])*dq[2] - sin(q[0])*cos(q[1])*cos(q[2])*cos(q[3])*dq[3] + sin(q[1])*sin(q[2])*sin(q[3])*cos(q[0])*dq[0] + sin(q[1])*sin(q[2])*sin(q[3])*cos(q[0])*dq[1] + sin(q[1])*sin(q[2])*sin(q[3])*cos(q[0])*dq[2] + sin(q[1])*sin(q[2])*sin(q[3])*cos(q[0])*dq[3] - sin(q[1])*cos(q[0])*cos(q[2])*cos(q[3])*dq[0] - sin(q[1])*cos(q[0])*cos(q[2])*cos(q[3])*dq[1] - sin(q[1])*cos(q[0])*cos(q[2])*cos(q[3])*dq[2] - sin(q[1])*cos(q[0])*cos(q[2])*cos(q[3])*dq[3] - sin(q[2])*cos(q[0])*cos(q[1])*cos(q[3])*dq[0] - sin(q[2])*cos(q[0])*cos(q[1])*cos(q[3])*dq[1] - sin(q[2])*cos(q[0])*cos(q[1])*cos(q[3])*dq[2] - sin(q[2])*cos(q[0])*cos(q[1])*cos(q[3])*dq[3] - sin(q[3])*cos(q[0])*cos(q[1])*cos(q[2])*dq[0] - sin(q[3])*cos(q[0])*cos(q[1])*cos(q[2])*dq[1] - sin(q[3])*cos(q[0])*cos(q[1])*cos(q[2])*dq[2] - sin(q[3])*cos(q[0])*cos(q[1])*cos(q[2])*dq[3];
   out_6659462614077130505[5] = sin(q[0])*sin(q[1])*sin(q[2])*cos(q[3])*dq[0] + sin(q[0])*sin(q[1])*sin(q[2])*cos(q[3])*dq[1] + sin(q[0])*sin(q[1])*sin(q[2])*cos(q[3])*dq[2] + sin(q[0])*sin(q[1])*sin(q[2])*cos(q[3])*dq[3] + sin(q[0])*sin(q[1])*sin(q[3])*cos(q[2])*dq[0] + sin(q[0])*sin(q[1])*sin(q[3])*cos(q[2])*dq[1] + sin(q[0])*sin(q[1])*sin(q[3])*cos(q[2])*dq[2] + sin(q[0])*sin(q[1])*sin(q[3])*cos(q[2])*dq[3] + sin(q[0])*sin(q[2])*sin(q[3])*cos(q[1])*dq[0] + sin(q[0])*sin(q[2])*sin(q[3])*cos(q[1])*dq[1] + sin(q[0])*sin(q[2])*sin(q[3])*cos(q[1])*dq[2] + sin(q[0])*sin(q[2])*sin(q[3])*cos(q[1])*dq[3] - sin(q[0])*cos(q[1])*cos(q[2])*cos(q[3])*dq[0] - sin(q[0])*cos(q[1])*cos(q[2])*cos(q[3])*dq[1] - sin(q[0])*cos(q[1])*cos(q[2])*cos(q[3])*dq[2] - sin(q[0])*cos(q[1])*cos(q[2])*cos(q[3])*dq[3] + sin(q[1])*sin(q[2])*sin(q[3])*cos(q[0])*dq[0] + sin(q[1])*sin(q[2])*sin(q[3])*cos(q[0])*dq[1] + sin(q[1])*sin(q[2])*sin(q[3])*cos(q[0])*dq[2] + sin(q[1])*sin(q[2])*sin(q[3])*cos(q[0])*dq[3] - sin(q[1])*cos(q[0])*cos(q[2])*cos(q[3])*dq[0] - sin(q[1])*cos(q[0])*cos(q[2])*cos(q[3])*dq[1] - sin(q[1])*cos(q[0])*cos(q[2])*cos(q[3])*dq[2] - sin(q[1])*cos(q[0])*cos(q[2])*cos(q[3])*dq[3] - sin(q[2])*cos(q[0])*cos(q[1])*cos(q[3])*dq[0] - sin(q[2])*cos(q[0])*cos(q[1])*cos(q[3])*dq[1] - sin(q[2])*cos(q[0])*cos(q[1])*cos(q[3])*dq[2] - sin(q[2])*cos(q[0])*cos(q[1])*cos(q[3])*dq[3] - sin(q[3])*cos(q[0])*cos(q[1])*cos(q[2])*dq[0] - sin(q[3])*cos(q[0])*cos(q[1])*cos(q[2])*dq[1] - sin(q[3])*cos(q[0])*cos(q[1])*cos(q[2])*dq[2] - sin(q[3])*cos(q[0])*cos(q[1])*cos(q[2])*dq[3];
   out_6659462614077130505[6] = sin(q[0])*sin(q[1])*sin(q[2])*cos(q[3])*dq[0] + sin(q[0])*sin(q[1])*sin(q[2])*cos(q[3])*dq[1] + sin(q[0])*sin(q[1])*sin(q[2])*cos(q[3])*dq[2] + sin(q[0])*sin(q[1])*sin(q[2])*cos(q[3])*dq[3] + sin(q[0])*sin(q[1])*sin(q[3])*cos(q[2])*dq[0] + sin(q[0])*sin(q[1])*sin(q[3])*cos(q[2])*dq[1] + sin(q[0])*sin(q[1])*sin(q[3])*cos(q[2])*dq[2] + sin(q[0])*sin(q[1])*sin(q[3])*cos(q[2])*dq[3] + sin(q[0])*sin(q[2])*sin(q[3])*cos(q[1])*dq[0] + sin(q[0])*sin(q[2])*sin(q[3])*cos(q[1])*dq[1] + sin(q[0])*sin(q[2])*sin(q[3])*cos(q[1])*dq[2] + sin(q[0])*sin(q[2])*sin(q[3])*cos(q[1])*dq[3] - sin(q[0])*cos(q[1])*cos(q[2])*cos(q[3])*dq[0] - sin(q[0])*cos(q[1])*cos(q[2])*cos(q[3])*dq[1] - sin(q[0])*cos(q[1])*cos(q[2])*cos(q[3])*dq[2] - sin(q[0])*cos(q[1])*cos(q[2])*cos(q[3])*dq[3] + sin(q[1])*sin(q[2])*sin(q[3])*cos(q[0])*dq[0] + sin(q[1])*sin(q[2])*sin(q[3])*cos(q[0])*dq[1] + sin(q[1])*sin(q[2])*sin(q[3])*cos(q[0])*dq[2] + sin(q[1])*sin(q[2])*sin(q[3])*cos(q[0])*dq[3] - sin(q[1])*cos(q[0])*cos(q[2])*cos(q[3])*dq[0] - sin(q[1])*cos(q[0])*cos(q[2])*cos(q[3])*dq[1] - sin(q[1])*cos(q[0])*cos(q[2])*cos(q[3])*dq[2] - sin(q[1])*cos(q[0])*cos(q[2])*cos(q[3])*dq[3] - sin(q[2])*cos(q[0])*cos(q[1])*cos(q[3])*dq[0] - sin(q[2])*cos(q[0])*cos(q[1])*cos(q[3])*dq[1] - sin(q[2])*cos(q[0])*cos(q[1])*cos(q[3])*dq[2] - sin(q[2])*cos(q[0])*cos(q[1])*cos(q[3])*dq[3] - sin(q[3])*cos(q[0])*cos(q[1])*cos(q[2])*dq[0] - sin(q[3])*cos(q[0])*cos(q[1])*cos(q[2])*dq[1] - sin(q[3])*cos(q[0])*cos(q[1])*cos(q[2])*dq[2] - sin(q[3])*cos(q[0])*cos(q[1])*cos(q[2])*dq[3];
   out_6659462614077130505[7] = sin(q[0])*sin(q[1])*sin(q[2])*cos(q[3])*dq[0] + sin(q[0])*sin(q[1])*sin(q[2])*cos(q[3])*dq[1] + sin(q[0])*sin(q[1])*sin(q[2])*cos(q[3])*dq[2] + sin(q[0])*sin(q[1])*sin(q[2])*cos(q[3])*dq[3] + sin(q[0])*sin(q[1])*sin(q[3])*cos(q[2])*dq[0] + sin(q[0])*sin(q[1])*sin(q[3])*cos(q[2])*dq[1] + sin(q[0])*sin(q[1])*sin(q[3])*cos(q[2])*dq[2] + sin(q[0])*sin(q[1])*sin(q[3])*cos(q[2])*dq[3] + sin(q[0])*sin(q[2])*sin(q[3])*cos(q[1])*dq[0] + sin(q[0])*sin(q[2])*sin(q[3])*cos(q[1])*dq[1] + sin(q[0])*sin(q[2])*sin(q[3])*cos(q[1])*dq[2] + sin(q[0])*sin(q[2])*sin(q[3])*cos(q[1])*dq[3] - sin(q[0])*cos(q[1])*cos(q[2])*cos(q[3])*dq[0] - sin(q[0])*cos(q[1])*cos(q[2])*cos(q[3])*dq[1] - sin(q[0])*cos(q[1])*cos(q[2])*cos(q[3])*dq[2] - sin(q[0])*cos(q[1])*cos(q[2])*cos(q[3])*dq[3] + sin(q[1])*sin(q[2])*sin(q[3])*cos(q[0])*dq[0] + sin(q[1])*sin(q[2])*sin(q[3])*cos(q[0])*dq[1] + sin(q[1])*sin(q[2])*sin(q[3])*cos(q[0])*dq[2] + sin(q[1])*sin(q[2])*sin(q[3])*cos(q[0])*dq[3] - sin(q[1])*cos(q[0])*cos(q[2])*cos(q[3])*dq[0] - sin(q[1])*cos(q[0])*cos(q[2])*cos(q[3])*dq[1] - sin(q[1])*cos(q[0])*cos(q[2])*cos(q[3])*dq[2] - sin(q[1])*cos(q[0])*cos(q[2])*cos(q[3])*dq[3] - sin(q[2])*cos(q[0])*cos(q[1])*cos(q[3])*dq[0] - sin(q[2])*cos(q[0])*cos(q[1])*cos(q[3])*dq[1] - sin(q[2])*cos(q[0])*cos(q[1])*cos(q[3])*dq[2] - sin(q[2])*cos(q[0])*cos(q[1])*cos(q[3])*dq[3] - sin(q[3])*cos(q[0])*cos(q[1])*cos(q[2])*dq[0] - sin(q[3])*cos(q[0])*cos(q[1])*cos(q[2])*dq[1] - sin(q[3])*cos(q[0])*cos(q[1])*cos(q[2])*dq[2] - sin(q[3])*cos(q[0])*cos(q[1])*cos(q[2])*dq[3];

}