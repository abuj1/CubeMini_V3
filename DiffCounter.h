#ifndef DIFFCOUNTER_H_
#define DIFFCOUNTER_H_

class DiffCounter
{
public:

    DiffCounter(float T, float Ts);
    
    float operator()(short inc) {
        return doStep(inc);
    }
    
    virtual     ~DiffCounter();
    
    void        reset(float initValue, short inc);
    float       doStep(short inc);

private:

    double b;
    double a;
    short incPast;
    double vel;
    double inc2rad;

};

#endif /* DIFFCOUNTER_H_ */