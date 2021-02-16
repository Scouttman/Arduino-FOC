#include "../hardware_api.h"
#include "stm32g4_hal.h"


#if defined(_STM32_DEF_) // or stm32
#define _ADC_VOLTAGE 3.3
#define _ADC_RESOLUTION 4096.0
#define ADC_BUF_LEN_1 4
#define ADC_BUF_LEN_2 2

static ADC_HandleTypeDef hadc1;
static ADC_HandleTypeDef hadc2;
static OPAMP_HandleTypeDef hopamp1;
static OPAMP_HandleTypeDef hopamp2;
static OPAMP_HandleTypeDef hopamp3;

uint16_t adcBuffer1[ADC_BUF_LEN_1]; // Buffer for store the results of the ADC conversion
uint16_t adcBuffer2[ADC_BUF_LEN_2]; // Buffer for store the results of the ADC conversion

// adc counts to voltage conversion ratio
// some optimizing for faster execution
#define _ADC_CONV ( (_ADC_VOLTAGE) / (_ADC_RESOLUTION) )

// function reading an ADC value and returning the read voltage
float _readADCVoltage(const int pin){
  uint32_t raw_adc =0;
  if(pin == 0)
    raw_adc = adcBuffer1[0];
  else if(pin == 1)
    raw_adc = adcBuffer2[1];
  else if(pin == 2)
    raw_adc = adcBuffer1[1];
  
  return raw_adc * _ADC_CONV;
}


// function reading an ADC value and returning the read voltage
void _configureADC(const int pinA,const int pinB,const int pinC){
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

#endif
