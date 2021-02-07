#include "CurrentSensor.h"

/*

*/
static ADC_HandleTypeDef hadc1;
static ADC_HandleTypeDef hadc2;
static OPAMP_HandleTypeDef hopamp1;
static OPAMP_HandleTypeDef hopamp2;
static OPAMP_HandleTypeDef hopamp3;

CurrentSensor::CurrentSensor(){
}

void CurrentSensor::init(){
    HAL_Init();
    MX_GPIO_Init();
    MX_DMA_Init(); 
    MX_ADC1_Init(&hadc1);
    MX_ADC2_Init(&hadc2);
    MX_OPAMP1_Init(&hopamp1);
    MX_OPAMP2_Init(&hopamp2);
    MX_OPAMP3_Init(&hopamp3);
    HAL_ADC_Start_DMA(&hadc1, (uint32_t*)adcBuffer1, ADC_BUF_LEN_1);
    HAL_ADC_Start_DMA(&hadc2, (uint32_t*)adcBuffer2, ADC_BUF_LEN_2);
    HAL_OPAMP_Start(&hopamp1);
    HAL_OPAMP_Start(&hopamp2);
    HAL_OPAMP_Start(&hopamp3);  
}

float CurrentSensor::get_d(){
    return i_d_filt;
}

float CurrentSensor::get_q(){
    return i_q_filt;
}

int CurrentSensor::test(int i){
    return adcBuffer1[i];
}

int CurrentSensor::test2(int i){
    return adcBuffer2[i];
}

void CurrentSensor::updateCurrent(float elecAng){
    // memcpy(adcBufferTmp1, adcBuffer1, sizeof(adcBuffer1));
    // memcpy(adcBufferTmp2, adcBuffer2, sizeof(adcBuffer2));
    elecAng = _normalizeAngle(elecAng);

    U_0 = (U_0*63.0+(float)adcBuffer1[2])/64.0; // #TODO replace with LPF
    V_0 = (V_0*63.0+(float)adcBuffer2[1])/64.0;
    W_0 = (W_0*63.0+(float)adcBuffer1[3])/64.0;
    //  currentU = convert2current(adcBufferTmp1[0], U_0);
    //  currentV = convert2current(adcBufferTmp2[0], V_0);
    //  currentW = convert2current(adcBufferTmp1[1], W_0);
    //  dq0(elecAng, currentU, currentV, currentW, &f_d, &f_q);
    dq0(elecAng, (float)adcBuffer1[0]-U_0, (float)adcBuffer2[0]-V_0, adcBuffer1[1]-W_0, &i_d, &i_q);
    // i_d_filt = (i_d_filt*(FILTER_LEN-1)+i_d)/FILTER_LEN;
    // i_q_filt = (i_q_filt*(FILTER_LEN-1)+i_q)/FILTER_LEN;
    i_d_filt = LPF_d(i_d);
    i_q_filt = LPF_q(i_q);
}

// From mit mini cheeta
void CurrentSensor::dq0(float theta, float a, float c, float b, float *d, float *q){
  /// DQ0 Transform ///
  ///Phase current amplitude = lengh of dq vector///
  ///i.e. iq = 1, id = 0, peak phase current of 1///
  
  float cf = _cos(theta);
  float sf = _sin(theta);

  *d = 0.6666667*(cf*a + (_SQRT3_2*sf-.5f*cf)*b + (-_SQRT3_2*sf-.5f*cf)*c);   ///Faster DQ0 Transform
  *q = 0.6666667*(-sf*a - (-_SQRT3_2*cf-.5f*sf)*b - (_SQRT3_2*cf-.5f*sf)*c);       
}