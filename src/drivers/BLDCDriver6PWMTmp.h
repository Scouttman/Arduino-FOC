
#ifndef BLDCDriver6PWMTmp_h
#define BLDCDriver6PWMTmp_h

#include "../common/base_classes/BLDCDriver.h"
#include "../common/foc_utils.h"
#include "../common/time_utils.h"
#include "../common/defaults.h"
#include "hardware_api.h"

/**
 6 pwm bldc driver class
*/
class BLDCDriver6PWMTmp: public BLDCDriver
{
  public:
    /**
      BLDCDriver class constructor
      @param phA_h A phase pwm pin
      @param phA_l A phase pwm pin
      @param phB_h B phase pwm pin
      @param phB_l A phase pwm pin
      @param phC_h C phase pwm pin
      @param phC_l A phase pwm pin
      @param en enable pin (optional input)
    */
    BLDCDriver6PWMTmp();
    
    /**  Motor hardware init function */
  	int init() override;
    /** Motor disable function */
  	void disable() override;
    /** Motor enable function */
    void enable() override;

    // hardware variables
  	int pwmA_h,pwmA_l; //!< phase A pwm pin number
  	int pwmB_h,pwmB_l; //!< phase B pwm pin number
  	int pwmC_h,pwmC_l; //!< phase C pwm pin number
    int enable_pin; //!< enable pin number

    float dead_zone; //!< a percentage of dead-time(zone) (both high and low side in low) for each pwm cycle [0,1]

    HardwareTimer *MyTim;  // TIM3 is MCU hardware peripheral instance, its definition is provided in CMSIS
    TIM_HandleTypeDef htim1;

    /** 
     * Set phase voltages to the harware 
     * 
     * @param Ua - phase A voltage
     * @param Ub - phase B voltage
     * @param Uc - phase C voltage
    */
    void setPwm(float Ua, float Ub, float Uc) override;

    void startPWM();

    void test();

  private:
        
};


#endif
