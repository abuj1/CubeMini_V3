/*  
    DiffCounter Class, differentiate encoder counts for cuboid, applies LP filter and unwrapping
    
              b*(1 - z^-1)                      s
      G(z) = -------------  <-- tustin --  ----------- = G(s)
              1 - a*z^-1                      T*s + 1
*/

#include "DiffCounter.h"
#define   pi 3.141592653589793
using namespace std;

DiffCounter::DiffCounter(float T, float Ts)
{   
    b = 2.0/(2.0*(double)T + (double)Ts);
    a = -(2.0*(double)T - (double)Ts)/(2.0*(double)T + (double)Ts);
    incPast = 0;
    vel = 0.0;
    inc2rad = 2.0*pi/(4.0*2048.0f);   // incr encoder with 2048inc/rev
}

DiffCounter::~DiffCounter() {}

void DiffCounter::reset(float initValue, short inc)
{
    vel = (double)initValue;
    incPast = inc;
}

float DiffCounter::doStep(short inc)
{
    long del = (long)(inc - incPast);
    incPast = inc;
    if(del < -16000)
        del += 0xFFFF;
    if(del > 16000)
        del -= 0xFFFF;
    vel = b*(double)del*inc2rad - a*vel;
    return (float)vel;
}