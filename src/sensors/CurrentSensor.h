#ifndef CURRENT_LIB_H
#define CURRENT_LIB_H

#define ADC_BUF_LEN_1 4
#define ADC_BUF_LEN_2 2
#define FILTER_LEN 128

#include "Arduino.h"
#include "../common/foc_utils.h"
#include "hardware_api.h"


class CurrentSensor{
 public:

    CurrentSensor();
    void init();
    
    /**  get current angular velocity (rad/s) */
    void updateCurrent(float elecAng);
    int test();
    float get_d();
    float get_q();
    
    float i_d_filt, i_q_filt = 0; 
    float i_d, i_q = 0;

  private:
    void dq0(float theta, float a, float b, float c, float *d, float *q); //!< function returning 1 if encoder has index pin and 0 if not.
    float convert2current(int on, int off);

    
    float U_0, V_0, W_0 = 0;
    uint16_t adcBuffer1[ADC_BUF_LEN_1]; // Buffer for store the results of the ADC conversion
    uint16_t adcBuffer2[ADC_BUF_LEN_2]; // Buffer for store the results of the ADC conversion
};


#endif
