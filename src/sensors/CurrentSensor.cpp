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

int CurrentSensor::test(){
    return adcBuffer1[0];
}

float CurrentSensor::get_d(){
    return i_d_filt;
}

float CurrentSensor::get_q(){
    return i_q_filt;
}

void CurrentSensor::updateCurrent(float elecAng){
    // memcpy(adcBufferTmp1, adcBuffer1, sizeof(adcBuffer1));
    // memcpy(adcBufferTmp2, adcBuffer2, sizeof(adcBuffer2));
    elecAng = _normalizeAngle(elecAng);

    U_0 = (U_0*63.0+adcBuffer1[2])/64.0;
    V_0 = (V_0*63.0+adcBuffer2[1])/64.0;
    W_0 = (W_0*63.0+adcBuffer1[3])/64.0;
    //  currentU = convert2current(adcBufferTmp1[0], U_0);
    //  currentV = convert2current(adcBufferTmp2[0], V_0);
    //  currentW = convert2current(adcBufferTmp1[1], W_0);
    //  dq0(elecAng, currentU, currentV, currentW, &f_d, &f_q);
    dq0(elecAng, adcBuffer1[0]-U_0, adcBuffer2[0]-V_0, adcBuffer1[1]-W_0, &i_d, &i_q);
    i_d_filt = (i_d_filt*(FILTER_LEN-1)+i_d)/FILTER_LEN;
    i_q_filt = (i_q_filt*(FILTER_LEN-1)+i_q)/FILTER_LEN;
}

// From mit mini cheeta
void CurrentSensor::dq0(float theta, float a, float b, float c, float *d, float *q){
  /// DQ0 Transform ///
  ///Phase current amplitude = lengh of dq vector///
  ///i.e. iq = 1, id = 0, peak phase current of 1///
  
  float cf = _cos(theta);
  float sf = _sin(theta);
  
  *d = 0.6666667*(cf*a + (_SQRT3_2*sf-.5f*cf)*b + (-_SQRT3_2*sf-.5f*cf)*c);   ///Faster DQ0 Transform
  *q = 0.6666667*(-sf*a - (-_SQRT3_2*cf-.5f*sf)*b - (_SQRT3_2*cf-.5f*sf)*c);       
}

/**
  * @brief ADC1 Initialization Function
  * @param None
  * @retval None
  */
static void OG_MX_ADC1_Init(void)
{

  /* USER CODE BEGIN ADC1_Init 0 */

  /* USER CODE END ADC1_Init 0 */

  ADC_MultiModeTypeDef multimode = {0};
  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC1_Init 1 */

  /* USER CODE END ADC1_Init 1 */
  /** Common config 
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ClockPrescaler = ADC_CLOCK_ASYNC_DIV256;
  hadc1.Init.Resolution = ADC_RESOLUTION_12B;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.GainCompensation = 0;
  hadc1.Init.ScanConvMode = ADC_SCAN_DISABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  hadc1.Init.LowPowerAutoWait = DISABLE;
  hadc1.Init.ContinuousConvMode = ENABLE;
  hadc1.Init.NbrOfConversion = 1;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.DMAContinuousRequests = ENABLE;
  hadc1.Init.Overrun = ADC_OVR_DATA_PRESERVED;
  hadc1.Init.OversamplingMode = DISABLE;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure the ADC multi-mode 
  */
  multimode.Mode = ADC_MODE_INDEPENDENT;
  if (HAL_ADCEx_MultiModeConfigChannel(&hadc1, &multimode) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure Regular Channel 
  */
  sConfig.Channel = ADC_CHANNEL_12;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_2CYCLES_5;
  sConfig.SingleDiff = ADC_SINGLE_ENDED;
  sConfig.OffsetNumber = ADC_OFFSET_NONE;
  sConfig.Offset = 0;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

}