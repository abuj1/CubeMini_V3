/*  
    PI Controller class with anti windup reset in biquad transposed direct form 2
    see e.g.: https://www.dsprelated.com/freebooks/filters/Four_Direct_Forms.html
    everything is calculated in double
    
                  Tn*s + 1                      
      G(s) = Kp -------------  with s ~ (1 - z^-1)/Ts
                    Ts*s              
                    
#include "PI_Cntrl.h"
using namespace std;       

*/



/*
    PID-T1 Controller class

                      1           s
      G(s) = Kp + Ki --- + Kd ---------
                      s       T_f*s + p

    Eigther reseting the Nucleo via the black button or save a new software on 
    the Nucleo sets the analog output to zero. Zero is equal to -4 Ampere!!!
    Therefor: NEVER !!! reset or save a new software while the VC is powered on
    (the green button on the VC is glowing green)                      

*/

#include "PID_Cntrl.h"
using namespace std;

PID_Cntrl::PID_Cntrl(float Kp, float Ki, float Kd, float Tf, float Ts, float uMin, float uMax)
{
    // link member variables
    Kp_ = Kp;
    Ki_ = Ki;
    Kd_ = Kd;
    Tf_ = Tf;
    Ts_ = Ts;
    uMin_ = uMin;
    uMax_ = uMax;
    
    
    reset(0.0f);
}

PID_Cntrl::~PID_Cntrl() {}

void PID_Cntrl::reset(float initValue)
{

    // implement controller reset
    P_new = 0;
    I_new = 0;
    D_new = 0;
    PID_output = 0;
    P_old = 0;
    I_old = 0;
    D_old = 0;
    delta_error = 0;
    e_old = 0;
    
}

float PID_Cntrl::update(double e)
{
    
    // Controller Input Value --> e
    
    // controller update function 
    
    // Delta Error (for D-Part)
    delta_error = e - e_old;
    
    // calculate I-Part Output
    float I_new = I_old + Ts_ * Ki_ * e_old;
   
    // saturate uI, uMin <= uI <= uMax (anti-windup for the integrator part)
    if(I_new > uMax_) {
        I_new = uMax_;
    }
    if(I_new < uMin_) {
        I_new = uMin_;
    }
    
    // calculate uD
    //D_new = (Kd_*delta_error + Tf_*D_old - Ts_*D_old)/(Tf_);
    D_new = 0;
        
    // calculate u
    P_new = Kp_ * e;
     
    
    // update signal storage
    P_old = P_new;
    I_old = I_new;
    D_old = D_new;
    e_old = e;
    
    // PID Output
    PID_output = P_new + I_new + D_new;
    
    // saturate u, uMin <= u <= uMax
    if(PID_output > uMax_) {
        PID_output = uMax_;
    }
    if(PID_output < uMin_) {
        PID_output = uMin_;
    }
    
    return PID_output;
}

