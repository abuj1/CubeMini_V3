#include "mbed.h"
#include "ThreadFlag.h"




//extern AnalogIn i_act2;

// This is the loop class, it is not a controller at first hand, it guarantees a cyclic call
class ControllerLoop
{
public:
    ControllerLoop(float Ts);
    virtual     ~ControllerLoop();
    void start_loop(void);
    void init_controllers(void);
    void reset_pids(void);


private:
    void loop(void);
    Thread thread;
    Ticker ticker;
    ThreadFlag threadFlag;
    Timer ti;
    float Ts;
    void sendSignal();
    bool is_initialized;
    void find_index(void);
    float pos_cntrl(float);
    float Kv;


};
