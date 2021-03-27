
#include "../hardware_api.h"

#define _STM32_DEF_
#if defined(_STM32_DEF_)
// default pwm parameters
#define _PWM_RESOLUTION 12 // 12bit
#define _PWM_RANGE 4095.0// 2^12 -1 = 4095
#define _PWM_FREQUENCY 25000 // 25khz
#define _PWM_FREQUENCY_MAX 50000 // 50khz

// 6pwm parameters
#define _HARDWARE_6PWM 1
#define _SOFTWARE_6PWM 0
#define _ERROR_6PWM -1


// setting pwm to hardware pin - instead analogWrite()
void _setPwm(int ulPin, uint32_t value, int resolution)
{
  PinName pin = digitalPinToPinName(ulPin);
  TIM_TypeDef *Instance = (TIM_TypeDef *)pinmap_peripheral(pin, PinMap_PWM);
  uint32_t index = get_timer_index(Instance);
  HardwareTimer *HT = (HardwareTimer *)(HardwareTimer_Handle[index]->__this);

  uint32_t channel = STM_PIN_CHANNEL(pinmap_function(pin, PinMap_PWM));
  HT->setCaptureCompare(channel, value, (TimerCompareFormat_t)resolution);
}


// init pin pwm 
HardwareTimer* _initPinPWM(uint32_t PWM_freq, int ulPin)
{
  PinName pin = digitalPinToPinName(ulPin);
  TIM_TypeDef *Instance = (TIM_TypeDef *)pinmap_peripheral(pin, PinMap_PWM);
  
  uint32_t index = get_timer_index(Instance);
  if (HardwareTimer_Handle[index] == NULL) {
    HardwareTimer_Handle[index]->__this = new HardwareTimer((TIM_TypeDef *)pinmap_peripheral(pin, PinMap_PWM));
    HardwareTimer_Handle[index]->handle.Init.CounterMode = TIM_COUNTERMODE_CENTERALIGNED3;
    HAL_TIM_Base_Init(&(HardwareTimer_Handle[index]->handle));
  }
  HardwareTimer *HT = (HardwareTimer *)(HardwareTimer_Handle[index]->__this);
  uint32_t channel = STM_PIN_CHANNEL(pinmap_function(pin, PinMap_PWM));
  HT->setMode(channel, TIMER_OUTPUT_COMPARE_PWM1, pin);
  HT->setOverflow(PWM_freq, HERTZ_FORMAT);
  HT->pause();
  HT->refresh();
  return HT;
}


// init high side pin
HardwareTimer* _initPinPWMHigh(uint32_t PWM_freq, int ulPin)
{
  return _initPinPWM(PWM_freq, ulPin);
}

// init low side pin
HardwareTimer* _initPinPWMLow(uint32_t PWM_freq, int ulPin)
{
  PinName pin = digitalPinToPinName(ulPin);
  TIM_TypeDef *Instance = (TIM_TypeDef *)pinmap_peripheral(pin, PinMap_PWM);
  uint32_t index = get_timer_index(Instance);
    
  if (HardwareTimer_Handle[index] == NULL) {
    HardwareTimer_Handle[index]->__this = new HardwareTimer((TIM_TypeDef *)pinmap_peripheral(pin, PinMap_PWM));
    HardwareTimer_Handle[index]->handle.Init.CounterMode = TIM_COUNTERMODE_CENTERALIGNED3;
    HAL_TIM_Base_Init(&(HardwareTimer_Handle[index]->handle));
    TIM_OC_InitTypeDef sConfigOC = TIM_OC_InitTypeDef();
    sConfigOC.OCMode = TIM_OCMODE_PWM2;
    sConfigOC.Pulse = 100;
    sConfigOC.OCPolarity = TIM_OCPOLARITY_LOW;
    sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
    sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
    sConfigOC.OCIdleState = TIM_OCIDLESTATE_SET;
    sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
    uint32_t channel = STM_PIN_CHANNEL(pinmap_function(pin, PinMap_PWM));
    HAL_TIM_PWM_ConfigChannel(&(HardwareTimer_Handle[index]->handle), &sConfigOC, channel);
  }
  HardwareTimer *HT = (HardwareTimer *)(HardwareTimer_Handle[index]->__this);
  uint32_t channel = STM_PIN_CHANNEL(pinmap_function(pin, PinMap_PWM));
  HT->setMode(channel, TIMER_OUTPUT_COMPARE_PWM2, pin);
  HT->setOverflow(PWM_freq, HERTZ_FORMAT);
  HT->pause();
  HT->refresh();
  return HT;
}


// align the timers to end the init
void _alignPWMTimers(HardwareTimer *HT1,HardwareTimer *HT2,HardwareTimer *HT3)
{
  HT1->pause();
  HT1->refresh();
  HT2->pause();
  HT2->refresh();
  HT3->pause();
  HT3->refresh();
  HT1->resume();
  HT2->resume();
  HT3->resume();
}

// align the timers to end the init
void _alignPWMTimers(HardwareTimer *HT1,HardwareTimer *HT2,HardwareTimer *HT3,HardwareTimer *HT4)
{
  HT1->pause();
  HT1->refresh();
  HT2->pause();
  HT2->refresh();
  HT3->pause();
  HT3->refresh();
  HT4->pause();
  HT4->refresh();
  HT1->resume();
  HT2->resume();
  HT3->resume();
  HT4->resume();
}

// configure hardware 6pwm interface only one timer with inverted channels
HardwareTimer* _initHardware6PWMInterface(uint32_t PWM_freq, float dead_zone, int pinA_h, int pinA_l, int pinB_h, int pinB_l, int pinC_h, int pinC_l)
{
  PinName uhPinName = digitalPinToPinName(pinA_h);
  PinName ulPinName = digitalPinToPinName(pinA_l);
  PinName vhPinName = digitalPinToPinName(pinB_h);
  PinName vlPinName = digitalPinToPinName(pinB_l);
  PinName whPinName = digitalPinToPinName(pinC_h);
  PinName wlPinName = digitalPinToPinName(pinC_l);

  TIM_TypeDef *Instance = (TIM_TypeDef *)pinmap_peripheral(uhPinName, PinMap_PWM);
 
  uint32_t index = get_timer_index(Instance);
  
  if (HardwareTimer_Handle[index] == NULL) {
    HardwareTimer_Handle[index]->__this = new HardwareTimer((TIM_TypeDef *)pinmap_peripheral(uhPinName, PinMap_PWM));
    HardwareTimer_Handle[index]->handle.Init.CounterMode = TIM_COUNTERMODE_CENTERALIGNED3;
    HAL_TIM_Base_Init(&(HardwareTimer_Handle[index]->handle));
    ((HardwareTimer *)HardwareTimer_Handle[index]->__this)->setOverflow(PWM_freq, HERTZ_FORMAT);  
  }
  HardwareTimer *HT = (HardwareTimer *)(HardwareTimer_Handle[index]->__this);
     
  HT->setMode(STM_PIN_CHANNEL(pinmap_function(uhPinName, PinMap_PWM)), TIMER_OUTPUT_COMPARE_PWM1, uhPinName);
  HT->setMode(STM_PIN_CHANNEL(pinmap_function(ulPinName, PinMap_PWM)), TIMER_OUTPUT_COMPARE_PWM1, ulPinName);
  HT->setMode(STM_PIN_CHANNEL(pinmap_function(vhPinName, PinMap_PWM)), TIMER_OUTPUT_COMPARE_PWM1, vhPinName);
  HT->setMode(STM_PIN_CHANNEL(pinmap_function(vlPinName, PinMap_PWM)), TIMER_OUTPUT_COMPARE_PWM1, vlPinName);
  HT->setMode(STM_PIN_CHANNEL(pinmap_function(whPinName, PinMap_PWM)), TIMER_OUTPUT_COMPARE_PWM1, whPinName);
  HT->setMode(STM_PIN_CHANNEL(pinmap_function(wlPinName, PinMap_PWM)), TIMER_OUTPUT_COMPARE_PWM1, wlPinName);

  // dead time is set in nanoseconds
  uint32_t dead_time_ns = (float)(1e9/PWM_freq)*dead_zone;
  uint32_t dead_time = __LL_TIM_CALC_DEADTIME(SystemCoreClock, LL_TIM_GetClockDivision(HT->getHandle()->Instance), dead_time_ns);
  LL_TIM_OC_SetDeadTime(HT->getHandle()->Instance, dead_time); // deadtime is non linear!
  LL_TIM_CC_EnableChannel(HT->getHandle()->Instance, LL_TIM_CHANNEL_CH1 | LL_TIM_CHANNEL_CH1N | LL_TIM_CHANNEL_CH2 | LL_TIM_CHANNEL_CH2N | LL_TIM_CHANNEL_CH3 | LL_TIM_CHANNEL_CH3N);

  HT->pause();
  HT->refresh();
  HT->resume();  
  return HT;
}


// returns 0 if each pair of pwm channels has same channel
// returns 1 all the channels belong to the same timer - hardware inverted channels 
// returns -1 if neither - avoid configuring - error!!!
int _interfaceType(const int pinA_h, const int pinA_l,  const int pinB_h, const int pinB_l, const int pinC_h, const int pinC_l){
  PinName nameAH = digitalPinToPinName(pinA_h);
  PinName nameAL = digitalPinToPinName(pinA_l);
  PinName nameBH = digitalPinToPinName(pinB_h);
  PinName nameBL = digitalPinToPinName(pinB_l);
  PinName nameCH = digitalPinToPinName(pinC_h);
  PinName nameCL = digitalPinToPinName(pinC_l);
  int tim1 = get_timer_index((TIM_TypeDef *)pinmap_peripheral(nameAH, PinMap_PWM));
  int tim2 = get_timer_index((TIM_TypeDef *)pinmap_peripheral(nameAL, PinMap_PWM));
  int tim3 = get_timer_index((TIM_TypeDef *)pinmap_peripheral(nameBH, PinMap_PWM));
  int tim4 = get_timer_index((TIM_TypeDef *)pinmap_peripheral(nameBL, PinMap_PWM));
  int tim5 = get_timer_index((TIM_TypeDef *)pinmap_peripheral(nameCH, PinMap_PWM));
  int tim6 = get_timer_index((TIM_TypeDef *)pinmap_peripheral(nameCL, PinMap_PWM));
  if(tim1 == tim2 && tim2==tim3 && tim3==tim4  && tim4==tim5 && tim5==tim6)
    return _HARDWARE_6PWM; // hardware 6pwm interface - only on timer 
  else if(tim1 == tim2 && tim3==tim4  && tim5==tim6)
    return _SOFTWARE_6PWM; // software 6pwm interface - each pair of high-low side same timer
  else
    return _ERROR_6PWM; // might be error avoid configuration
}



// function setting the high pwm frequency to the supplied pins
// - Stepper motor - 2PWM setting
// - hardware speciffic
void _configure2PWM(long pwm_frequency,const int pinA, const int pinB) {
  if( !pwm_frequency || !_isset(pwm_frequency) ) pwm_frequency = _PWM_FREQUENCY; // default frequency 25khz
  else pwm_frequency = _constrain(pwm_frequency, 0, _PWM_FREQUENCY_MAX); // constrain to 50kHz max
  // center-aligned frequency is uses two periods
  pwm_frequency *=2;

  HardwareTimer* HT1 = _initPinPWM(pwm_frequency, pinA);
  HardwareTimer* HT2 = _initPinPWM(pwm_frequency, pinB);
  // allign the timers
  _alignPWMTimers(HT1, HT2, HT2);
}


// function setting the high pwm frequency to the supplied pins
// - BLDC motor - 3PWM setting
// - hardware speciffic
void _configure3PWM(long pwm_frequency,const int pinA, const int pinB, const int pinC) {
  if( !pwm_frequency || !_isset(pwm_frequency) ) pwm_frequency = _PWM_FREQUENCY; // default frequency 25khz
  else pwm_frequency = _constrain(pwm_frequency, 0, _PWM_FREQUENCY_MAX); // constrain to 50kHz max
  // center-aligned frequency is uses two periods
  pwm_frequency *=2;

  HardwareTimer* HT1 = _initPinPWM(pwm_frequency, pinA);
  HardwareTimer* HT2 = _initPinPWM(pwm_frequency, pinB);
  HardwareTimer* HT3 = _initPinPWM(pwm_frequency, pinC);
  // allign the timers
  _alignPWMTimers(HT1, HT2, HT3);
}

// function setting the high pwm frequency to the supplied pins
// - Stepper motor - 4PWM setting
// - hardware speciffic
void _configure4PWM(long pwm_frequency,const int pinA, const int pinB, const int pinC, const int pinD) {
  if( !pwm_frequency || !_isset(pwm_frequency) ) pwm_frequency = _PWM_FREQUENCY; // default frequency 25khz
  else pwm_frequency = _constrain(pwm_frequency, 0, _PWM_FREQUENCY_MAX); // constrain to 50kHz max
  // center-aligned frequency is uses two periods
  pwm_frequency *=2;

  HardwareTimer* HT1 = _initPinPWM(pwm_frequency, pinA);
  HardwareTimer* HT2 = _initPinPWM(pwm_frequency, pinB);
  HardwareTimer* HT3 = _initPinPWM(pwm_frequency, pinC);
  HardwareTimer* HT4 = _initPinPWM(pwm_frequency, pinD); 
  // allign the timers
  _alignPWMTimers(HT1, HT2, HT3, HT4);
}

// function setting the pwm duty cycle to the hardware
// - Stepper motor - 2PWM setting
//- hardware speciffic
void _writeDutyCycle2PWM(float dc_a,  float dc_b, int pinA, int pinB){
  // transform duty cycle from [0,1] to [0,4095]
  _setPwm(pinA, _PWM_RANGE*dc_a, _PWM_RESOLUTION);
  _setPwm(pinB, _PWM_RANGE*dc_b, _PWM_RESOLUTION);
}

// function setting the pwm duty cycle to the hardware
// - BLDC motor - 3PWM setting
//- hardware speciffic
void _writeDutyCycle3PWM(float dc_a,  float dc_b, float dc_c, int pinA, int pinB, int pinC){
  // transform duty cycle from [0,1] to [0,4095]
  _setPwm(pinA, _PWM_RANGE*dc_a, _PWM_RESOLUTION);
  _setPwm(pinB, _PWM_RANGE*dc_b, _PWM_RESOLUTION);
  _setPwm(pinC, _PWM_RANGE*dc_c, _PWM_RESOLUTION);
}


// function setting the pwm duty cycle to the hardware
// - Stepper motor - 4PWM setting
//- hardware speciffic
void _writeDutyCycle4PWM(float dc_1a,  float dc_1b, float dc_2a, float dc_2b, int pin1A, int pin1B, int pin2A, int pin2B){
  // transform duty cycle from [0,1] to [0,4095]
  _setPwm(pin1A, _PWM_RANGE*dc_1a, _PWM_RESOLUTION);
  _setPwm(pin1B, _PWM_RANGE*dc_1b, _PWM_RESOLUTION);
  _setPwm(pin2A, _PWM_RANGE*dc_2a, _PWM_RESOLUTION);
  _setPwm(pin2B, _PWM_RANGE*dc_2b, _PWM_RESOLUTION);
}




// Configuring PWM frequency, resolution and alignment
// - BLDC driver - 6PWM setting
// - hardware specific
int _configure6PWM(long pwm_frequency, float dead_zone, const int pinA_h, const int pinA_l,  const int pinB_h, const int pinB_l, const int pinC_h, const int pinC_l){
  if( !pwm_frequency || !_isset(pwm_frequency) ) pwm_frequency = _PWM_FREQUENCY; // default frequency 25khz
  else pwm_frequency = _constrain(pwm_frequency, 0, _PWM_FREQUENCY_MAX); // constrain to |%0kHz max
  // center-aligned frequency is uses two periods
  pwm_frequency *=2;

  // find configuration
  int config = _interfaceType(pinA_h, pinA_l,  pinB_h, pinB_l, pinC_h, pinC_l);
  // configure accordingly
  switch(config){
    case _ERROR_6PWM:
      return -1; // fail
      break;
    case _HARDWARE_6PWM:
      _initHardware6PWMInterface(pwm_frequency, dead_zone, pinA_h, pinA_l, pinB_h, pinB_l, pinC_h, pinC_l);
      break;
    case _SOFTWARE_6PWM:
      HardwareTimer* HT1 = _initPinPWMHigh(pwm_frequency, pinA_h);
      _initPinPWMLow(pwm_frequency, pinA_l);
      HardwareTimer* HT2 = _initPinPWMHigh(pwm_frequency,pinB_h);
      _initPinPWMLow(pwm_frequency, pinB_l);
      HardwareTimer* HT3 = _initPinPWMHigh(pwm_frequency,pinC_h);
      _initPinPWMLow(pwm_frequency, pinC_l);
      _alignPWMTimers(HT1, HT2, HT3);
      break;
  }
  return 0; // success
}

// Function setting the duty cycle to the pwm pin (ex. analogWrite())
// - BLDC driver - 6PWM setting
// - hardware specific
void _writeDutyCycle6PWM(float dc_a,  float dc_b, float dc_c, float dead_zone, int pinA_h, int pinA_l, int pinB_h, int pinB_l, int pinC_h, int pinC_l){
  // find configuration
  int config = _interfaceType(pinA_h, pinA_l,  pinB_h, pinB_l, pinC_h, pinC_l);
  // set pwm accordingly
  switch(config){
    case _HARDWARE_6PWM:
      _setPwm(pinA_h, _PWM_RANGE*dc_a, _PWM_RESOLUTION);
      _setPwm(pinB_h, _PWM_RANGE*dc_b, _PWM_RESOLUTION);
      _setPwm(pinC_h, _PWM_RANGE*dc_c, _PWM_RESOLUTION);
      break;
    case _SOFTWARE_6PWM:
      _setPwm(pinA_l, _constrain(dc_a + dead_zone/2, 0, 1)*_PWM_RANGE, _PWM_RESOLUTION); 
      _setPwm(pinA_h, _constrain(dc_a - dead_zone/2, 0, 1)*_PWM_RANGE, _PWM_RESOLUTION);
      _setPwm(pinB_l, _constrain(dc_b + dead_zone/2, 0, 1)*_PWM_RANGE, _PWM_RESOLUTION); 
      _setPwm(pinB_h, _constrain(dc_b - dead_zone/2, 0, 1)*_PWM_RANGE, _PWM_RESOLUTION);
      _setPwm(pinC_l, _constrain(dc_c + dead_zone/2, 0, 1)*_PWM_RANGE, _PWM_RESOLUTION); 
      _setPwm(pinC_h, _constrain(dc_c - dead_zone/2, 0, 1)*_PWM_RANGE, _PWM_RESOLUTION);
      break;
  }
}


#define TIM_U CCR2
#define TIM_V CCR1
#define TIM_W CCR3

void _writeDutyCycle6PWMtmp(float dc_a,  float dc_b, float dc_c){
  TIM1->TIM_U = (int)(255*dc_a);
	TIM1->TIM_V = (int)(255*dc_b);
	TIM1->TIM_W = (int)(255*dc_c);
}

// From STM32 cube IDE
// TODO remove and replace/create functions
/**
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2020 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  */

#define LED1_Pin GPIO_PIN_6
#define LED1_GPIO_Port GPIOC

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
void  MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOF_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LED1_GPIO_Port, LED1_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : PB12 */
  GPIO_InitStruct.Pin = GPIO_PIN_12;
  GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : LED1_Pin */
  GPIO_InitStruct.Pin = LED1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LED1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : PB3 PB4 */
  GPIO_InitStruct.Pin = GPIO_PIN_3|GPIO_PIN_4;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF7_USART2;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

}

/**
  * @brief TIM1 Initialization Function
  * @param None
  * @retval None
  */
void  MX_TIM1_Init(TIM_HandleTypeDef* htim1)
{

  /* USER CODE BEGIN TIM1_Init 0 */

  /* USER CODE END TIM1_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};
  TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = {0};

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
  htim1->Instance = TIM1;
  htim1->Init.Prescaler = 23;
  htim1->Init.CounterMode = TIM_COUNTERMODE_CENTERALIGNED1;
  htim1->Init.Period = 255;
  htim1->Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1->Init.RepetitionCounter = 0;
  htim1->Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(htim1, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_UPDATE;
  sMasterConfig.MasterOutputTrigger2 = TIM_TRGO2_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(htim1, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 123;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
  sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
  if (HAL_TIM_PWM_ConfigChannel(htim1, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.Pulse = 0;
  if (HAL_TIM_PWM_ConfigChannel(htim1, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.Pulse = 123;
  if (HAL_TIM_PWM_ConfigChannel(htim1, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
  {
    Error_Handler();
  }
  sBreakDeadTimeConfig.OffStateRunMode = TIM_OSSR_DISABLE;
  sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_DISABLE;
  sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_OFF;
  sBreakDeadTimeConfig.DeadTime = 130;
  sBreakDeadTimeConfig.BreakState = TIM_BREAK_DISABLE;
  sBreakDeadTimeConfig.BreakPolarity = TIM_BREAKPOLARITY_HIGH;
  sBreakDeadTimeConfig.BreakFilter = 0;
  sBreakDeadTimeConfig.BreakAFMode = TIM_BREAK_AFMODE_INPUT;
  sBreakDeadTimeConfig.Break2State = TIM_BREAK2_DISABLE;
  sBreakDeadTimeConfig.Break2Polarity = TIM_BREAK2POLARITY_HIGH;
  sBreakDeadTimeConfig.Break2Filter = 0;
  sBreakDeadTimeConfig.Break2AFMode = TIM_BREAK_AFMODE_INPUT;
  sBreakDeadTimeConfig.AutomaticOutput = TIM_AUTOMATICOUTPUT_DISABLE;
  if (HAL_TIMEx_ConfigBreakDeadTime(htim1, &sBreakDeadTimeConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM1_Init 2 */

  /* USER CODE END TIM1_Init 2 */
  HAL_TIM_MspPostInit(htim1);

}

// /** 
//   * Enable DMA controller clock
//   */
// void  MX_DMA_Init(void) 
// {

//   /* DMA controller clock enable */
//   __HAL_RCC_DMAMUX1_CLK_ENABLE();
//   __HAL_RCC_DMA1_CLK_ENABLE();

//   /* DMA interrupt init */
//   /* DMA1_Channel1_IRQn interrupt configuration */
//   HAL_NVIC_SetPriority(DMA1_Channel1_IRQn, 0, 0);
//   HAL_NVIC_EnableIRQ(DMA1_Channel1_IRQn);
//   /* DMA1_Channel2_IRQn interrupt configuration */
//   HAL_NVIC_SetPriority(DMA1_Channel2_IRQn, 0, 0);
//   HAL_NVIC_EnableIRQ(DMA1_Channel2_IRQn);

// }


// /**
//   * @brief ADC1 Initialization Function
//   * @param None
//   * @retval None
//   */
// void  MX_ADC1_Init(void)
// {

//   /* USER CODE BEGIN ADC1_Init 0 */

//   /* USER CODE END ADC1_Init 0 */

//   ADC_MultiModeTypeDef multimode = {0};
//   ADC_ChannelConfTypeDef sConfig = {0};

//   /* USER CODE BEGIN ADC1_Init 1 */

//   /* USER CODE END ADC1_Init 1 */
//   /** Common config 
//   */
//   hadc1.Instance = ADC1;
//   hadc1.Init.ClockPrescaler = ADC_CLOCK_ASYNC_DIV32;
//   hadc1.Init.Resolution = ADC_RESOLUTION_12B;
//   hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
//   hadc1.Init.GainCompensation = 0;
//   hadc1.Init.ScanConvMode = ADC_SCAN_ENABLE;
//   hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
//   hadc1.Init.LowPowerAutoWait = DISABLE;
//   hadc1.Init.ContinuousConvMode = DISABLE;
//   hadc1.Init.NbrOfConversion = 2;
//   hadc1.Init.DiscontinuousConvMode = DISABLE;
//   hadc1.Init.ExternalTrigConv = ADC_EXTERNALTRIG_T1_TRGO;
//   hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_RISING;
//   hadc1.Init.DMAContinuousRequests = ENABLE;
//   hadc1.Init.Overrun = ADC_OVR_DATA_PRESERVED;
//   hadc1.Init.OversamplingMode = DISABLE;
//   if (HAL_ADC_Init(&hadc1) != HAL_OK)
//   {
//     Error_Handler();
//   }
//   /** Configure the ADC multi-mode 
//   */
//   multimode.Mode = ADC_MODE_INDEPENDENT;
//   if (HAL_ADCEx_MultiModeConfigChannel(&hadc1, &multimode) != HAL_OK)
//   {
//     Error_Handler();
//   }
//   /** Configure Regular Channel 
//   */
//   sConfig.Channel = ADC_CHANNEL_12;
//   sConfig.Rank = ADC_REGULAR_RANK_1;
//   sConfig.SamplingTime = ADC_SAMPLETIME_2CYCLES_5;
//   sConfig.SingleDiff = ADC_SINGLE_ENDED;
//   sConfig.OffsetNumber = ADC_OFFSET_NONE;
//   sConfig.Offset = 0;
//   if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
//   {
//     Error_Handler();
//   }
//   /** Configure Regular Channel 
//   */
//   sConfig.Channel = ADC_CHANNEL_3;
//   sConfig.Rank = ADC_REGULAR_RANK_2;
//   if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
//   {
//     Error_Handler();
//   }
//   /* USER CODE BEGIN ADC1_Init 2 */

//   /* USER CODE END ADC1_Init 2 */

// }

// /**
//   * @brief ADC2 Initialization Function
//   * @param None
//   * @retval None
//   */
// void  MX_ADC2_Init(void)
// {

//   /* USER CODE BEGIN ADC2_Init 0 */

//   /* USER CODE END ADC2_Init 0 */

//   ADC_ChannelConfTypeDef sConfig = {0};

//   /* USER CODE BEGIN ADC2_Init 1 */

//   /* USER CODE END ADC2_Init 1 */
//   /** Common config 
//   */
//   hadc2.Instance = ADC2;
//   hadc2.Init.ClockPrescaler = ADC_CLOCK_ASYNC_DIV32;
//   hadc2.Init.Resolution = ADC_RESOLUTION_12B;
//   hadc2.Init.DataAlign = ADC_DATAALIGN_RIGHT;
//   hadc2.Init.GainCompensation = 0;
//   hadc2.Init.ScanConvMode = ADC_SCAN_DISABLE;
//   hadc2.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
//   hadc2.Init.LowPowerAutoWait = DISABLE;
//   hadc2.Init.ContinuousConvMode = DISABLE;
//   hadc2.Init.NbrOfConversion = 1;
//   hadc2.Init.DiscontinuousConvMode = DISABLE;
//   hadc2.Init.ExternalTrigConv = ADC_EXTERNALTRIG_T1_TRGO;
//   hadc2.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_RISING;
//   hadc2.Init.DMAContinuousRequests = ENABLE;
//   hadc2.Init.Overrun = ADC_OVR_DATA_PRESERVED;
//   hadc2.Init.OversamplingMode = DISABLE;
//   if (HAL_ADC_Init(&hadc2) != HAL_OK)
//   {
//     Error_Handler();
//   }
//   /** Configure Regular Channel 
//   */
//   sConfig.Channel = ADC_CHANNEL_3;
//   sConfig.Rank = ADC_REGULAR_RANK_1;
//   sConfig.SamplingTime = ADC_SAMPLETIME_2CYCLES_5;
//   sConfig.SingleDiff = ADC_SINGLE_ENDED;
//   sConfig.OffsetNumber = ADC_OFFSET_NONE;
//   sConfig.Offset = 0;
//   if (HAL_ADC_ConfigChannel(&hadc2, &sConfig) != HAL_OK)
//   {
//     Error_Handler();
//   }
//   /* USER CODE BEGIN ADC2_Init 2 */

//   /* USER CODE END ADC2_Init 2 */

// }

// /**
//   * @brief OPAMP1 Initialization Function
//   * @param None
//   * @retval None
//   */
// void  MX_OPAMP1_Init(void)
// {

//   /* USER CODE BEGIN OPAMP1_Init 0 */

//   /* USER CODE END OPAMP1_Init 0 */

//   /* USER CODE BEGIN OPAMP1_Init 1 */

//   /* USER CODE END OPAMP1_Init 1 */
//   hopamp1.Instance = OPAMP1;
//   hopamp1.Init.PowerMode = OPAMP_POWERMODE_HIGHSPEED;
//   hopamp1.Init.Mode = OPAMP_PGA_MODE;
//   hopamp1.Init.NonInvertingInput = OPAMP_NONINVERTINGINPUT_IO0;
//   hopamp1.Init.InternalOutput = DISABLE;
//   hopamp1.Init.TimerControlledMuxmode = OPAMP_TIMERCONTROLLEDMUXMODE_DISABLE;
//   hopamp1.Init.PgaConnect = OPAMP_PGA_CONNECT_INVERTINGINPUT_IO0_BIAS;
//   hopamp1.Init.PgaGain = OPAMP_PGA_GAIN_16_OR_MINUS_15;
//   hopamp1.Init.UserTrimming = OPAMP_TRIMMING_FACTORY;
//   if (HAL_OPAMP_Init(&hopamp1) != HAL_OK)
//   {
//     Error_Handler();
//   }
//   /* USER CODE BEGIN OPAMP1_Init 2 */

//   /* USER CODE END OPAMP1_Init 2 */

// }

// /**
//   * @brief OPAMP2 Initialization Function
//   * @param None
//   * @retval None
//   */
// void  MX_OPAMP2_Init(void)
// {

//   /* USER CODE BEGIN OPAMP2_Init 0 */

//   /* USER CODE END OPAMP2_Init 0 */

//   /* USER CODE BEGIN OPAMP2_Init 1 */

//   /* USER CODE END OPAMP2_Init 1 */TIM_HandleTypeDef*SPEED;
//   hopamp2.Init.Mode = OPAMP_PGA_MODE;
//   hopamp2.Init.NonInvertingInput = OPAMP_NONINVERTINGINPUT_IO0;
//   hopamp2.Init.InternalOutput = DISABLE;
//   hopamp2.Init.TimerControlledMuxmode = OPAMP_TIMERCONTROLLEDMUXMODE_DISABLE;
//   hopamp2.Init.PgaConnect = OPAMP_PGA_CONNECT_INVERTINGINPUT_IO0_BIAS;
//   hopamp2.Init.PgaGain = OPAMP_PGA_GAIN_16_OR_MINUS_15;
//   hopamp2.Init.UserTrimming = OPAMP_TRIMMING_FACTORY;
//   if (HAL_OPAMP_Init(&hopamp2) != HAL_OK)
//   {
//     Error_Handler();
//   }
//   /* USER CODE BEGIN OPAMP2_Init 2 */

//   /* USER CODE END OPAMP2_Init 2 */

// }

// /**
//   * @brief OPAMP3 Initialization Function
//   * @param None
//   * @retval None
//   */
// void  MX_OPAMP3_Init(void)
// {

//   /* USER CODE BEGIN OPAMP3_Init 0 */

//   /* USER CODE END OPAMP3_Init 0 */

//   /* USER CODE BEGIN OPAMP3_Init 1 */

//   /* USER CODE END OPAMP3_Init 1 */
//   hopamp3.Instance = OPAMP3;
//   hopamp3.Init.PowerMode = OPAMP_POWERMODE_HIGHSPEED;
//   hopamp3.Init.Mode = OPAMP_PGA_MODE;
//   hopamp3.Init.NonInvertingInput = OPAMP_NONINVERTINGINPUT_IO0;
//   hopamp3.Init.InternalOutput = DISABLE;
//   hopamp3.Init.TimerControlledMuxmode = OPAMP_TIMERCONTROLLEDMUXMODE_DISABLE;
//   hopamp3.Init.PgaConnect = OPAMP_PGA_CONNECT_INVERTINGINPUT_IO0_BIAS;
//   hopamp3.Init.PgaGain = OPAMP_PGA_GAIN_16_OR_MINUS_15;
//   hopamp3.Init.UserTrimming = OPAMP_TRIMMING_FACTORY;
//   if (HAL_OPAMP_Init(&hopamp3) != HAL_OK)
//   {
//     Error_Handler();
//   }
//   /* USER CODE BEGIN OPAMP3_Init 2 */

//   /* USER CODE END OPAMP3_Init 2 */

// }

void HAL_TIM_MspPostInit(TIM_HandleTypeDef* htim)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  if(htim->Instance==TIM1)
  {
  /* USER CODE BEGIN TIM1_MspPostInit 0 */

  /* USER CODE END TIM1_MspPostInit 0 */
  
    __HAL_RCC_GPIOC_CLK_ENABLE();
    __HAL_RCC_GPIOB_CLK_ENABLE();
    __HAL_RCC_GPIOA_CLK_ENABLE();
    /**TIM1 GPIO Configuration    
    PC13     ------> TIM1_CH1N
    PB15     ------> TIM1_CH3N
    PA8     ------> TIM1_CH1
    PA9     ------> TIM1_CH2
    PA10     ------> TIM1_CH3
    PA12     ------> TIM1_CH2N 
    */
    GPIO_InitStruct.Pin = GPIO_PIN_13;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    GPIO_InitStruct.Alternate = GPIO_AF4_TIM1;
    HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

    GPIO_InitStruct.Pin = GPIO_PIN_15;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    GPIO_InitStruct.Alternate = GPIO_AF4_TIM1;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

    GPIO_InitStruct.Pin = GPIO_PIN_8|GPIO_PIN_9|GPIO_PIN_10|GPIO_PIN_12;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    GPIO_InitStruct.Alternate = GPIO_AF6_TIM1;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /* USER CODE BEGIN TIM1_MspPostInit 1 */

  /* USER CODE END TIM1_MspPostInit 1 */
  }

}
#endif