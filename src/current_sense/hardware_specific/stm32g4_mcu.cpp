#include "../hardware_api.h"
#include "stm32g4_hal.h"

#define _STM32_DEF_ //TODO Remove
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
// As DMA is being used just return the DMA result
// The DMA of adcBuffer1[2,3] and adcBuffer1[0] are the off duty cycle
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

void _configureOPAMP(OPAMP_HandleTypeDef *hopamp, OPAMP_TypeDef *OPAMPx_Def){
  // could this be replaced with LL_OPAMP calls??
  hopamp->Instance = OPAMPx_Def;
  hopamp->Init.PowerMode = OPAMP_POWERMODE_HIGHSPEED;
  hopamp->Init.Mode = OPAMP_PGA_MODE;
  hopamp->Init.NonInvertingInput = OPAMP_NONINVERTINGINPUT_IO0;
  hopamp->Init.InternalOutput = DISABLE;
  hopamp->Init.TimerControlledMuxmode = OPAMP_TIMERCONTROLLEDMUXMODE_DISABLE;
  hopamp->Init.PgaConnect = OPAMP_PGA_CONNECT_INVERTINGINPUT_IO0_BIAS;
  hopamp->Init.PgaGain = OPAMP_PGA_GAIN_16_OR_MINUS_15;
  hopamp->Init.UserTrimming = OPAMP_TRIMMING_FACTORY;
  if (HAL_OPAMP_Init(hopamp) != HAL_OK)
  {
    Error_Handler();
  }
}
void _configureOPAMPs(OPAMP_HandleTypeDef *OPAMPA, OPAMP_HandleTypeDef *OPAMPB, OPAMP_HandleTypeDef *OPAMPC){
  // Configure the opamps
  _configureOPAMP(OPAMPA, OPAMP1);
  _configureOPAMP(OPAMPB, OPAMP2);
  _configureOPAMP(OPAMPC, OPAMP3);
}



// function reading an ADC value and returning the read voltage
void _configureADC(const int pinA,const int pinB,const int pinC){
  HAL_Init();
  MX_GPIO_Init();
  MX_DMA_Init(); 
  MX_ADC1_Init(&hadc1);
  MX_ADC2_Init(&hadc2);
  _configureOPAMPs(&hopamp1, &hopamp3, &hopamp2);
  HAL_ADC_Start_DMA(&hadc1, (uint32_t*)adcBuffer1, ADC_BUF_LEN_1);
  HAL_ADC_Start_DMA(&hadc2, (uint32_t*)adcBuffer2, ADC_BUF_LEN_2);
  HAL_OPAMP_Start(&hopamp1);
  HAL_OPAMP_Start(&hopamp2);
  HAL_OPAMP_Start(&hopamp3); 
}

#endif
