
#ifndef HARDWARE_UTILS_H_2
#define HARDWARE_UTILS_H_2
#include "../common/foc_utils.h"
#include "../common/time_utils.h"

// void SET_HDMA(DMA_HandleTypeDef* _hdma_adc_m1, DMA_HandleTypeDef* _hdma_adc_m2);

void MX_GPIO_Init();

/** 
  * Enable DMA controller clock
  */
void  MX_DMA_Init(void) ;

/**
  * @brief ADC1 Initialization Function
  * @param None
  * @retval None
  */
void MX_ADC1_Init(ADC_HandleTypeDef* hadc1);

/**
  * @brief ADC2 Initialization Function
  * @param None
  * @retval None
  */
void  MX_ADC2_Init(ADC_HandleTypeDef* hadc2);

/**
  * @brief OPAMP1 Initialization Function
  * @param None
  * @retval None
  */
void  MX_OPAMP1_Init(OPAMP_HandleTypeDef* hopamp);
/**
  * @brief OPAMP2 Initialization Function
  * @param None
  * @retval None
  */
void  MX_OPAMP2_Init(OPAMP_HandleTypeDef* hopamp);

/**
  * @brief OPAMP3 Initialization Function
  * @param None
  * @retval None
  */
void  MX_OPAMP3_Init(OPAMP_HandleTypeDef* hopamp);

/**
 * Initializes the Global MSP.
 */
void HAL_MspInit(void);

/**
* @brief ADC MSP Initialization
* This function configures the hardware resources used in this example
* @param hadc: ADC handle pointer
* @retval None
*/
void HAL_ADC_MspInit(ADC_HandleTypeDef* hadc);

/**
* @brief ADC MSP De-Initialization
* This function freeze the hardware resources used in this example
* @param hadc: ADC handle pointer
* @retval None
*/
void HAL_ADC_MspDeInit(ADC_HandleTypeDef* hadc);

/**
* @brief OPAMP MSP Initialization
* This function configures the hardware resources used in this example
* @param hopamp: OPAMP handle pointer
* @retval None
*/
void HAL_OPAMP_MspInit(OPAMP_HandleTypeDef* hopamp);
/**
* @brief OPAMP MSP De-Initialization
* This function freeze the hardware resources used in this example
* @param hopamp: OPAMP handle pointer
* @retval None
*/
void HAL_OPAMP_MspDeInit(OPAMP_HandleTypeDef* hopamp);

/**
  * @brief This function handles DMA1 channel1 global interrupt.
  */
void DMA1_Channel1_IRQHandler(void);

/**
  * @brief This function handles DMA1 channel2 global interrupt.
  */
void DMA1_Channel2_IRQHandler(void);

#endif