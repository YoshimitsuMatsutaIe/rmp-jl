#include "jo_3.h"
#include <math.h>

void jo_3(double l1, double l2, double l3, double *q, double *out_8624575235484115301) {

   out_8624575235484115301[0] = -l1*sin(q[0]) - l2*sin(q[0])*cos(q[1]) - l2*sin(q[1])*cos(q[0]) + l3*sin(q[0])*sin(q[1])*sin(q[2]) - l3*sin(q[0])*cos(q[1])*cos(q[2]) - l3*sin(q[1])*cos(q[0])*cos(q[2]) - l3*sin(q[2])*cos(q[0])*cos(q[1]);
   out_8624575235484115301[1] = -l2*sin(q[0])*cos(q[1]) - l2*sin(q[1])*cos(q[0]) + l3*sin(q[0])*sin(q[1])*sin(q[2]) - l3*sin(q[0])*cos(q[1])*cos(q[2]) - l3*sin(q[1])*cos(q[0])*cos(q[2]) - l3*sin(q[2])*cos(q[0])*cos(q[1]);
   out_8624575235484115301[2] = l3*sin(q[0])*sin(q[1])*sin(q[2]) - l3*sin(q[0])*cos(q[1])*cos(q[2]) - l3*sin(q[1])*cos(q[0])*cos(q[2]) - l3*sin(q[2])*cos(q[0])*cos(q[1]);
   out_8624575235484115301[3] = 0;
   out_8624575235484115301[4] = l1*cos(q[0]) - l2*sin(q[0])*sin(q[1]) + l2*cos(q[0])*cos(q[1]) - l3*sin(q[0])*sin(q[1])*cos(q[2]) - l3*sin(q[0])*sin(q[2])*cos(q[1]) - l3*sin(q[1])*sin(q[2])*cos(q[0]) + l3*cos(q[0])*cos(q[1])*cos(q[2]);
   out_8624575235484115301[5] = -l2*sin(q[0])*sin(q[1]) + l2*cos(q[0])*cos(q[1]) - l3*sin(q[0])*sin(q[1])*cos(q[2]) - l3*sin(q[0])*sin(q[2])*cos(q[1]) - l3*sin(q[1])*sin(q[2])*cos(q[0]) + l3*cos(q[0])*cos(q[1])*cos(q[2]);
   out_8624575235484115301[6] = -l3*sin(q[0])*sin(q[1])*cos(q[2]) - l3*sin(q[0])*sin(q[2])*cos(q[1]) - l3*sin(q[1])*sin(q[2])*cos(q[0]) + l3*cos(q[0])*cos(q[1])*cos(q[2]);
   out_8624575235484115301[7] = 0;

}
