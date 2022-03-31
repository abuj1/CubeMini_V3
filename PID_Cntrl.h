#ifndef PID_Cntrl_H_
#define PID_Cntrl_H_

// PID Controller Class
class PID_Cntrl
{
public:

    PID_Cntrl(float Kp, float Ki, float Kd, float Tf, float Ts, float uMin, float uMax);

    float operator()(float error) {
        return update((double)error);
     }

    virtual     ~PID_Cntrl();

    void        reset(float initValue);
    float       update(double error);
    
private:

    // controller parameters (member variables)
    float Kp_, Ki_, Kd_, Tf_, Ts_, uMin_, uMax_;
        
    // storage for signals (member variables)
    float P_new, I_new, D_new, PID_output, P_old, I_old, D_old, delta_error, error, e_old;
};
#endif