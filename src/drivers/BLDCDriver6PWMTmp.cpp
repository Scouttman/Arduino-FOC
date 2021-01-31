#include "BLDCDriver6PWMTmp.h"

BLDCDriver6PWMTmp::BLDCDriver6PWMTmp(){
  // default power-supply value
  voltage_power_supply = DEF_POWER_SUPPLY;
  voltage_limit = NOT_SET;
}

// enable motor driver
void  BLDCDriver6PWMTmp::enable(){
    // set zero to PWM
    startPWM();     
    setPwm(0.5, 0.5, 0.5);
}

// disable motor driver
void BLDCDriver6PWMTmp::disable()
{
  // set zero to PWM
  setPwm(0, 0, 0);
}

// init hardware pins   
int BLDCDriver6PWMTmp::init() {
  // a bit of separation
  _delay(100);
  MyTim = new HardwareTimer(TIM1);  // TIM3 is MCU hardware peripheral instance, its definition is provided in CMSIS

  // PWM pins
  MX_GPIO_Init();
  MX_TIM1_Init(&htim1);

  // sanity check for the voltage limit configuration
  if(voltage_limit == NOT_SET || voltage_limit > voltage_power_supply) voltage_limit =  voltage_power_supply;

  // configure 6pwm 
  // hardware specific function - depending on driver and mcu
//   return _configure6PWM(pwm_frequency, dead_zone, pwmA_h,pwmA_l, pwmB_h,pwmB_l, pwmC_h,pwmC_l);
}

void BLDCDriver6PWMTmp::test(){
    // HardwareTimer *MyTim = new HardwareTimer(TIM1);  // TIM3 is MCU hardware peripheral instance, its definition is provided in CMSIS
    // TIM_HandleTypeDef htim1;
    MX_GPIO_Init();
    MX_TIM1_Init(&htim1);
    startPWM();
    delay(10000);
}


// Set voltage to the pwm pin
void BLDCDriver6PWMTmp::setPwm(float Ua, float Ub, float Uc) {  
  // limit the voltage in driver
  Ua = _constrain(Ua, 0, voltage_limit);
  Ub = _constrain(Ub, 0, voltage_limit);
  Uc = _constrain(Uc, 0, voltage_limit);    
  // calculate duty cycle
  // limited in [0,1]
  float dc_a = _constrain(Ua / voltage_power_supply, 0.0 , 1.0 );
  float dc_b = _constrain(Ub / voltage_power_supply, 0.0 , 1.0 );
  float dc_c = _constrain(Uc / voltage_power_supply, 0.0 , 1.0 );
  // hardware specific writing
  // hardware specific function - depending on driver and mcu
  _writeDutyCycle6PWMtmp(dc_a, dc_b, dc_c);
}

void BLDCDriver6PWMTmp::startPWM(){
  
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2);
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_3);
  HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_1);
  HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_2);
  HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_3);
  
  MyTim->resume();
}