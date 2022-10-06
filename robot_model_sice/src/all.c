#include <assert.h>

#include "o_0.h"
#include "o_1.h"
#include "o_2.h"
#include "o_3.h"
#include "o_ee.h"
#include "jo_0.h"
#include "jo_1.h"
#include "jo_2.h"
#include "jo_3.h"
#include "jo_ee.h"
#include "jo_0_dot.h"
#include "jo_1_dot.h"
#include "jo_2_dot.h"
#include "jo_3_dot.h"
#include "jo_ee_dot.h"
#include "rx_0.h"
#include "rx_1.h"
#include "rx_2.h"
#include "rx_3.h"
#include "rx_ee.h"
#include "jrx_0.h"
#include "jrx_1.h"
#include "jrx_2.h"
#include "jrx_3.h"
#include "jrx_ee.h"
#include "jrx_0_dot.h"
#include "jrx_1_dot.h"
#include "jrx_2_dot.h"
#include "jrx_3_dot.h"
#include "jrx_ee_dot.h"
#include "ry_0.h"
#include "ry_1.h"
#include "ry_2.h"
#include "ry_3.h"
#include "ry_ee.h"
#include "jry_0.h"
#include "jry_1.h"
#include "jry_2.h"
#include "jry_3.h"
#include "jry_ee.h"
#include "jry_0_dot.h"
#include "jry_1_dot.h"
#include "jry_2_dot.h"
#include "jry_3_dot.h"
#include "jry_ee_dot.h"


void o(int n, double *q, double l1, double l2, double l3, double l4, double* out)
{
    switch (n)
    {
    case 1:
        o_0(out);
        break;
    case 2:
        o_1(l1, q, out);
        break;
    case 3:
        o_2(l1, l2, q, out);
        break;
    case 4:
        o_3(l1, l2, l3, q, out);
        break;
    case 5:
        o_ee(l1, l2, l3, l4, q, out);
        break;
    default:
        assert(0);
        break;
    }
}

void rx(int n, double *q, double l1, double l2, double l3, double l4, double* out)
{
    switch (n)
    {
    case 1:
        rx_0(q, out);
        break;
    case 2:
        rx_1(q, out);
        break;
    case 3:
        rx_2(q, out);
        break;
    case 4:
        rx_3(q, out);
        break;
    case 5:
        rx_ee(q, out);
        break;
    default:
        assert(0);
        break;
    }
}

void ry(int n, double *q, double l1, double l2, double l3, double l4, double* out)
{
    switch (n)
    {
    case 1:
        ry_0(q, out);
        break;
    case 2:
        ry_1(q, out);
        break;
    case 3:
        ry_2(q, out);
        break;
    case 4:
        ry_3(q, out);
        break;
    case 5:
        ry_ee(q, out);
        break;
    default:
        assert(0);
        break;
    }
}

void jo(int n, double *q, double l1, double l2, double l3, double l4, double* out)
{
    switch (n)
    {
    case 1:
        jo_0(out);
        break;
    case 2:
        jo_1(l1, q, out);
        break;
    case 3:
        jo_2(l1, l2, q, out);
        break;
    case 4:
        jo_3(l1, l2, l3, q, out);
        break;
    case 5:
        jo_ee(l1, l2, l3, l4, q, out);
        break;
    default:
        assert(0);
        break;
    }
}

void jrx(int n, double *q, double l1, double l2, double l3, double l4, double* out)
{
    switch (n)
    {
    case 1:
        jrx_0(q, out);
        break;
    case 2:
        jrx_1(q, out);
        break;
    case 3:
        jrx_2(q, out);
        break;
    case 4:
        jrx_3(q, out);
        break;
    case 5:
        jrx_ee(q, out);
        break;
    default:
        assert(0);
        break;
    }
}

void jry(int n, double *q, double l1, double l2, double l3, double l4, double* out)
{
    switch (n)
    {
    case 1:
        jry_0(q, out);
        break;
    case 2:
        jry_1(q, out);
        break;
    case 3:
        jry_2(q, out);
        break;
    case 4:
        jry_3(q, out);
        break;
    case 5:
        jry_ee(q, out);
        break;
    default:
        assert(0);
        break;
    }
}

void jo_dot(int n, double *q, double *q_dot, double l1, double l2, double l3, double l4, double* out)
{
    switch (n)
    {
    case 1:
        jo_0_dot(out);
        break;
    case 2:
        jo_1_dot(q_dot, l1, q, out);
        break;
    case 3:
        jo_2_dot(q_dot, l1, l2, q, out);
        break;
    case 4:
        jo_3_dot(q_dot, l1, l2, l3, q, out);
        break;
    case 5:
        jo_ee_dot(q_dot, l1, l2, l3, l4, q, out);
        break;
    default:
        assert(0);
        break;
    }
}

void jrx_dot(int n, double *q, double *q_dot, double l1, double l2, double l3, double l4, double* out)
{
    switch (n)
    {
    case 1:
        jrx_0_dot(q_dot, q, out);
        break;
    case 2:
        jrx_1_dot(q_dot, q, out);
        break;
    case 3:
        jrx_2_dot(q_dot, q, out);
        break;
    case 4:
        jrx_3_dot(q_dot, q, out);
        break;
    case 5:
        jrx_ee_dot(q_dot, q, out);
        break;
    default:
        assert(0);
        break;
    }
}

void jry_dot(int n, double *q, double* q_dot, double l1, double l2, double l3, double l4, double* out)
{
    switch (n)
    {
    case 1:
        jry_0_dot(q_dot, q, out);
        break;
    case 2:
        jry_1_dot(q_dot, q, out);
        break;
    case 3:
        jry_2_dot(q_dot, q, out);
        break;
    case 4:
        jry_3_dot(q_dot, q, out);
        break;
    case 5:
        jry_ee_dot(q_dot, q, out);
        break;
    default:
        assert(0);
        break;
    }
}


// int main(){
//     return 0;
// }