
/* -------------------------------- includes -------------------------------- */
#include <stdint.h>
#include <stddef.h>
#include "stm32g4xx_hal.h"
#include "adc.h"
#include "main.h"

/* -------------------------- function prototypes --------------------------- */


/* ---------------------------- module variables ---------------------------- */
#define V_REF 2.5f
#define MAX_ADC_CHANNELS 4
struct {
	float v12v_input;
	float v12v_display;
	float v3v3_half;
	float v5v5_usb;
	float btn;
	float i12v;
	float stm_Vbat;
	float stm_Vref;
	float stm_temperature;
	int32_t btn_lpf;
}measurement;

static uint16_t adc1_data[MAX_ADC_CHANNELS],adc2_data[MAX_ADC_CHANNELS],adc3_data[MAX_ADC_CHANNELS];


/* ---------------------------- extern functions ---------------------------- */


/* ---------------------------- public functions ---------------------------- */
uint16_t* get_adc_raw_data(uint8_t adc_nr, uint8_t index)
{
	uint16_t *ret_val=NULL;
	if (index<MAX_ADC_CHANNELS)
	{
		switch (adc_nr)
		{
		case 1:
			ret_val=&adc1_data[index];
			break;
		case 2:
			ret_val=&adc2_data[index];
			break;
		case 3:
			ret_val=&adc3_data[index];
			break;
		default:
			ret_val=NULL;
			break;
		}
	}
	return (ret_val);
}

float adc_get(TE_ADC_VALUE idx) {
  if (idx == ADC_VAL_KEYPAD) {
    return (float)(measurement.btn_lpf>>4);
    }
  return 0;
  }


void update_measurements()
{
	//adc1 data
	measurement.v12v_input   = V_REF*0.001391602f*adc1_data[0];
	measurement.v12v_display = V_REF*0.001391602f*adc1_data[1];
	measurement.v3v3_half	 = V_REF*0.000244141f*adc1_data[2];
	//adc2 data
	measurement.v5v5_usb	 = V_REF*0.000537109f*adc2_data[0];
	measurement.btn			 = V_REF/4096*adc2_data[1];
	measurement.btn_lpf		 += (adc2_data[1]-(measurement.btn_lpf>>4));
	//adc3
	measurement.i12v		 = V_REF/(35*2048)*adc3_data[0];
	measurement.stm_Vbat	 = V_REF*3/4096*adc3_data[1];
}
