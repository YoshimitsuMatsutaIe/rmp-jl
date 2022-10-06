#include "jry_2.h"
#include <math.h>

void jry_2(double *q, double *out_6840658375946921982) {

   out_6840658375946921982[0] = sin(q[0])*sin(q[1])*cos(q[2]) + sin(q[0])*sin(q[2])*cos(q[1]) + sin(q[1])*sin(q[2])*cos(q[0]) - cos(q[0])*cos(q[1])*cos(q[2]);
   out_6840658375946921982[1] = sin(q[0])*sin(q[1])*cos(q[2]) + sin(q[0])*sin(q[2])*cos(q[1]) + sin(q[1])*sin(q[2])*cos(q[0]) - cos(q[0])*cos(q[1])*cos(q[2]);
   out_6840658375946921982[2] = sin(q[0])*sin(q[1])*cos(q[2]) + sin(q[0])*sin(q[2])*cos(q[1]) + sin(q[1])*sin(q[2])*cos(q[0]) - cos(q[0])*cos(q[1])*cos(q[2]);
   out_6840658375946921982[3] = 0;
   out_6840658375946921982[4] = sin(q[0])*sin(q[1])*sin(q[2]) - sin(q[0])*cos(q[1])*cos(q[2]) - sin(q[1])*cos(q[0])*cos(q[2]) - sin(q[2])*cos(q[0])*cos(q[1]);
   out_6840658375946921982[5] = sin(q[0])*sin(q[1])*sin(q[2]) - sin(q[0])*cos(q[1])*cos(q[2]) - sin(q[1])*cos(q[0])*cos(q[2]) - sin(q[2])*cos(q[0])*cos(q[1]);
   out_6840658375946921982[6] = sin(q[0])*sin(q[1])*sin(q[2]) - sin(q[0])*cos(q[1])*cos(q[2]) - sin(q[1])*cos(q[0])*cos(q[2]) - sin(q[2])*cos(q[0])*cos(q[1]);
   out_6840658375946921982[7] = 0;

}
