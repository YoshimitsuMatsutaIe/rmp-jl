#include "rx_1.h"
#include <math.h>

void rx_1(double *q, double *out_6622624079233976775) {

   out_6622624079233976775[0] = -sin(q[0])*sin(q[1]) + cos(q[0])*cos(q[1]);
   out_6622624079233976775[1] = sin(q[0])*cos(q[1]) + sin(q[1])*cos(q[0]);

}
