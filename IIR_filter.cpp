#include "IIR_filter.h"

/*
  IIR filter implemention for the following filter types:
  init for: first order differentiatior:   G(s) = s/(T*s + 1)
            first order lowpass with gain  G(s) = K/(T*s + 1)
            second order lowpass with gain G(s) = K*w0^2/(s^2 + 2*D*w0*s + w0^2)        
            nth order, with arbitrary values
  billinear transformation is used for s -> z
  reseting the filter only makes sence for static signals, whatch out if you're using the differnetiator, static corresponds to output null
*/

// G(s) = s/(T*s + 1)
IIR_filter::IIR_filter(float T, float Ts) {
           
    // filter orders
    nb = 1;
    na = 1;
    
    // filter coefficients
    B = (float*)malloc((nb+1)*sizeof(float));
    A = (float*)malloc(na*sizeof(float));    
    B[0] = 2.0f/(2.0f*T + Ts);
    B[1] = -B[0];
    A[0] = -(2.0f*T - Ts)/(2.0f*T + Ts);
    
    // signal arrays
    uk = (float*)malloc((nb+1)*sizeof(float));
    yk = (float*)malloc(na*sizeof(float));
    uk[0] = uk[1] = 0.0f;
    yk[0] = 0.0f;
    
    // dc-gain
    this->K = 0.0f;
}

// G(s) = K/(T*s + 1)
IIR_filter::IIR_filter(float T, float Ts, float K) {
    
    // filter orders
    nb = 1;
    na = 1;
    
    // filter coefficients
    B = (float*)malloc((nb+1)*sizeof(float));
    A = (float*)malloc(na*sizeof(float));      
    B[0] = Ts/(Ts + 2.0f*T);
    B[1] = B[0];
    A[0] = (Ts - 2.0f*T)/(Ts + 2.0f*T); 
    
    // signal arrays
    uk = (float*)malloc((nb+1)*sizeof(float));
    yk = (float*)malloc(na*sizeof(float));
    uk[0] = uk[1] = 0.0f;
    yk[0] = 0.0f;
    
    // dc-gain
    this->K = K;
}

// G(s) = K*w0^2/(s^2 + 2*D*w0*s + w0^2) 
IIR_filter::IIR_filter(float w0, float D, float Ts, float K) {
    
    // filter orders
    nb = 2;
    na = 2;
    
    // filter coefficients
    B = (float*)malloc((nb+1)*sizeof(float));
    A = (float*)malloc(na*sizeof(float));
    float k0 = Ts*Ts*w0*w0;
    float k1 = 4.0f*D*Ts*w0;
    float k2 = k0 + k1 + 4.0f;    
    B[0] = K*k0/k2;
    B[1] = 2.0f*B[0];
    B[2] = B[0]; 
    A[0] = (2.0f*k0 - 8.0f)/k2;
    A[1] = (k0 - k1 + 4.0f)/k2;
    
    // signal arrays
    uk = (float*)malloc((nb+1)*sizeof(float));
    yk = (float*)malloc(na*sizeof(float));
    uk[0] = uk[1] = uk[2] = 0.0f;
    yk[0] = yk[1] = 0.0f;
    
    // dc-gain
    this->K = K;
}

IIR_filter::IIR_filter(float *b, float *a, int nb, int na) {
    
    // filter orders
    this->nb = nb - 1;
    this->na = na;
    
    // filter coefficients
    B = (float*)malloc((nb+1)*sizeof(float));
    A = (float*)malloc(na*sizeof(float));
    uk = (float*)malloc((nb+1)*sizeof(float));
    yk = (float*)malloc(na*sizeof(float));
    
    for(uint8_t k=0;k<=nb;k++) {
        B[k] = b[k];
        uk[k] = 0.0f;
    }
    for(uint8_t k=0;k<na;k++) {
        A[k] = a[k];
        yk[k] = 0.0f;
    }
    
    // dc-gain
    this->K = 1.0f;
}

    
IIR_filter::~IIR_filter() {} 

void IIR_filter::setup(float T, float Ts, float K) {
    
    // filter orders
    nb = 1;
    na = 1;
    
    // filter coefficients
    B = (float*)malloc((nb+1)*sizeof(float));
    A = (float*)malloc(na*sizeof(float));      
    B[0] = Ts/(Ts + 2.0f*T);
    B[1] = B[0];
    A[0] = (Ts - 2.0f*T)/(Ts + 2.0f*T); 
    
    // signal arrays
    uk = (float*)malloc((nb+1)*sizeof(float));
    yk = (float*)malloc(na*sizeof(float));
    uk[0] = uk[1] = 0.0f;
    yk[0] = 0.0f;
    
    // dc-gain
    this->K = K;
}

void IIR_filter::reset(float val) {
    for(uint8_t k=0;k < nb;k++)
        uk[k] = val;
    for(uint8_t k=0;k < na;k++)
        yk[k] = val*K;
        
}

/* 
    the filter is operating as follows: 
    (B[0] + B[1]*z^-1 + ... + B[nb]*z^-nb)*U(z) = (1 + A[0]*z^-1 + ... + A[na-1]*z^-na))*Y(z)
    y(n) =  B[0]*u(k)   + B[1]*u(k-1) + ... + B[nb]*u(k-nb) + ...
          - A[0]*y(k-1) - A[1]*y(k-2) - ... - A[na]*y(n-na)
*/
float IIR_filter::filter(float input) {
    
    for(uint8_t k = nb;k > 0;k--)    // shift input values back
        uk[k] = uk[k-1];
    uk[0] = input;
    float ret = 0.0f;
    for(uint8_t k = 0;k <= nb;k++)
        ret += B[k] * uk[k];
    for(uint8_t k = 0;k < na;k++)
        ret -= A[k] * yk[k];
    for(uint8_t k = na;k > 1;k--)
        yk[k-1] = yk[k-2];
    yk[0] = ret;
    return ret;
}
