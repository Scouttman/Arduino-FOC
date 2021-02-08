#ifndef CURRENT_LIB_H
#define CURRENT_LIB_H

#define ADC_BUF_LEN_1 4
#define ADC_BUF_LEN_2 2
#define FILTER_LEN 128
// low pass filter velocity
#define DEF_CUR_FILTER_Tf 0.005 //!< default velocity filter time constant
#define DEF_CUR_IDLE_Tf 0.1 //!< default current 0 value filter time constnat

#include "Arduino.h"
#include "../common/foc_utils.h"
#include "../common/lowpass_filter.h"
#include "hardware_api.h"


class CurrentSensor{
 public:

    CurrentSensor();
    void init();
    
    /**  get current angular velocity (rad/s) */
    void updateCurrent(float elecAng);
    float get_d();
    float get_q();
    int test(int val);
    int test2(int val);
    
    float i_d_filt, i_q_filt = 0; 
    float i_d, i_q = 0;

  private:
    void dq0(float theta, float a, float b, float c, float *d, float *q); //!< function returning 1 if encoder has index pin and 0 if not.
    float convert2current(int on, int off);

    
    float U_0, V_0, W_0 = 0;
    uint16_t adcBuffer1[ADC_BUF_LEN_1]; // Buffer for store the results of the ADC conversion
    uint16_t adcBuffer2[ADC_BUF_LEN_2]; // Buffer for store the results of the ADC conversion
    LowPassFilter LPF_d{DEF_CUR_FILTER_Tf};//!<  parameter determining the current Lpw pass filter configuration 
    LowPassFilter LPF_q{DEF_CUR_FILTER_Tf};//!<  parameter determining the current Lpw pass filter configuration 
    LowPassFilter LPF_U_0{DEF_CUR_IDLE_Tf};//!<  parameter determining the 0 current Lpw pass filter configuration 
    LowPassFilter LPF_V_0{DEF_CUR_IDLE_Tf};//!<  parameter determining the 0 current Lpw pass filter configuration 
    LowPassFilter LPF_W_0{DEF_CUR_IDLE_Tf};//!<  parameter determining the 0 current Lpw pass filter configuration 
};


#endif
