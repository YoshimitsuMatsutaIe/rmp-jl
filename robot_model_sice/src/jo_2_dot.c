#include "jo_2_dot.h"
#include <math.h>

void jo_2_dot(double *dq, double l1, double l2, double *q, double *out_6715139419794012175) {

   out_6715139419794012175[0] = -l1*cos(q[0])*dq[0] + l2*sin(q[0])*sin(q[1])*dq[0] + l2*sin(q[0])*sin(q[1])*dq[1] - l2*cos(q[0])*cos(q[1])*dq[0] - l2*cos(q[0])*cos(q[1])*dq[1];
   out_6715139419794012175[1] = l2*sin(q[0])*sin(q[1])*dq[0] + l2*sin(q[0])*sin(q[1])*dq[1] - l2*cos(q[0])*cos(q[1])*dq[0] - l2*cos(q[0])*cos(q[1])*dq[1];
   out_6715139419794012175[2] = 0;
   out_6715139419794012175[3] = 0;
   out_6715139419794012175[4] = -l1*sin(q[0])*dq[0] - l2*sin(q[0])*cos(q[1])*dq[0] - l2*sin(q[0])*cos(q[1])*dq[1] - l2*sin(q[1])*cos(q[0])*dq[0] - l2*sin(q[1])*cos(q[0])*dq[1];
   out_6715139419794012175[5] = -l2*sin(q[0])*cos(q[1])*dq[0] - l2*sin(q[0])*cos(q[1])*dq[1] - l2*sin(q[1])*cos(q[0])*dq[0] - l2*sin(q[1])*cos(q[0])*dq[1];
   out_6715139419794012175[6] = 0;
   out_6715139419794012175[7] = 0;

}
