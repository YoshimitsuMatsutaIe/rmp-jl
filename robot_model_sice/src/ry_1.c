#include "ry_1.h"
#include <math.h>

void ry_1(double *q, double *out_6190902170331897602) {

   out_6190902170331897602[0] = -sin(q[0])*cos(q[1]) - sin(q[1])*cos(q[0]);
   out_6190902170331897602[1] = -sin(q[0])*sin(q[1]) + cos(q[0])*cos(q[1]);

}
