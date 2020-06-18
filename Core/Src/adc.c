
/* -------------------------------- includes -------------------------------- */
#include <stdint.h>
#include <stddef.h>
#include "stm32g4xx_hal.h"
#include "adc.h"


/* -------------------------- function prototypes --------------------------- */


/* ---------------------------- module variables ---------------------------- */
static volatile float adc_val;


/* ---------------------------- extern functions ---------------------------- */
void HAL_ADC_ErrorCallback(ADC_HandleTypeDef *hadc) {
  // TODO reset the error (e.g. overrun)
  }


void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc) {
  adc_val = HAL_ADC_GetValue(hadc);
  }


/* ---------------------------- public functions ---------------------------- */
float adc_get(TE_ADC_VALUE idx) {
  if (idx == ADC_VAL_KEYPAD) {
    return (float)adc_val;
    }
  return 0;
  }

