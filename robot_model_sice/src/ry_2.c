#include "ry_2.h"
#include <math.h>

void ry_2(double *q, double *out_7382729309337082544) {

   out_7382729309337082544[0] = sin(q[0])*sin(q[1])*sin(q[2]) - sin(q[0])*cos(q[1])*cos(q[2]) - sin(q[1])*cos(q[0])*cos(q[2]) - sin(q[2])*cos(q[0])*cos(q[1]);
   out_7382729309337082544[1] = -sin(q[0])*sin(q[1])*cos(q[2]) - sin(q[0])*sin(q[2])*cos(q[1]) - sin(q[1])*sin(q[2])*cos(q[0]) + cos(q[0])*cos(q[1])*cos(q[2]);

}
