#ifndef IIR_FILTER_H_
#define IIR_FILTER_H_

#include "mbed.h"

class IIR_filter{
     
     public:
     
        IIR_filter(float T, float Ts);
        IIR_filter(float T, float Ts, float K);
        IIR_filter(float w0, float D, float Ts, float K);
        IIR_filter(float *b, float *a, int nb, int na);
        IIR_filter() {};
                    
        float operator()(float u){
            return filter(u);
         }
         
        virtual ~IIR_filter();
        
        void    setup(float T, float Ts, float K);
        void    reset(float val);
        float   filter(float input);
        
    private:

        uint8_t nb;
        uint8_t na;
        float   *B;
        float   *A;
        float   *uk;
        float   *yk;
        float   K;
};
#endif