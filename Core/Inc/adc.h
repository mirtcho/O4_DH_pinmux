#ifndef ADC_H
#define ADC_H

/* -------------------------------- typedefs -------------------------------- */
typedef enum {
  ADC_VAL_KEYPAD = 0,

  NOF_ADC_CHANNELS
  } TE_ADC_VALUE;


/* ---------------------------- public functions ---------------------------- */
uint16_t* get_adc_raw_data(uint8_t adc_nr, uint8_t index);
float adc_get(TE_ADC_VALUE idx);


#endif
