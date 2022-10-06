#include "rx_ee.h"
#include <math.h>

void rx_ee(double *q, double *out_2144441752696152832) {

   out_2144441752696152832[0] = sin(q[0])*sin(q[1])*sin(q[2])*sin(q[3]) - sin(q[0])*sin(q[1])*cos(q[2])*cos(q[3]) - sin(q[0])*sin(q[2])*cos(q[1])*cos(q[3]) - sin(q[0])*sin(q[3])*cos(q[1])*cos(q[2]) - sin(q[1])*sin(q[2])*cos(q[0])*cos(q[3]) - sin(q[1])*sin(q[3])*cos(q[0])*cos(q[2]) - sin(q[2])*sin(q[3])*cos(q[0])*cos(q[1]) + cos(q[0])*cos(q[1])*cos(q[2])*cos(q[3]);
   out_2144441752696152832[1] = -sin(q[0])*sin(q[1])*sin(q[2])*cos(q[3]) - sin(q[0])*sin(q[1])*sin(q[3])*cos(q[2]) - sin(q[0])*sin(q[2])*sin(q[3])*cos(q[1]) + sin(q[0])*cos(q[1])*cos(q[2])*cos(q[3]) - sin(q[1])*sin(q[2])*sin(q[3])*cos(q[0]) + sin(q[1])*cos(q[0])*cos(q[2])*cos(q[3]) + sin(q[2])*cos(q[0])*cos(q[1])*cos(q[3]) + sin(q[3])*cos(q[0])*cos(q[1])*cos(q[2]);

}
