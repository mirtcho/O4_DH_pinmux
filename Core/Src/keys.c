

/* -------------------------------- includes -------------------------------- */
#include <stdint.h>
#include <stdbool.h>
#include "stm32g4xx_hal.h"
#include "adc.h"
#include "keys.h"


/* -------------------------------- defines --------------------------------- */
/* Note: TIME_ADC_KEY_SETTLE should be as low as possible. 100ms is too high,
 * the UI isn't responsive. ADC sample period must be at least equal,
 * but less is better (prevent false button reads due to averaging).
 */

#define TIME_ADC_KEY_SETTLE (uint32_t)50
#define NOF_KEY_RANGES      (uint8_t)6

#define KEY_N_MIN (uint16_t)3666 // 4074
#define KEY_N_MAX (uint16_t)4095
#define KEY_C_MIN (uint16_t)2030 // 2051
#define KEY_C_MAX (uint16_t)2078 // 2057
#define KEY_R_MIN (uint16_t)2701 // 2728
#define KEY_R_MAX (uint16_t)2759 // 2737
#define KEY_L_MIN (uint16_t)3033 // 3064
#define KEY_L_MAX (uint16_t)3107 // 3076
#define KEY_U_MIN (uint16_t)3221 // 3254
#define KEY_U_MAX (uint16_t)3301 // 3268
#define KEY_D_MIN (uint16_t)3428 // 3463
#define KEY_D_MAX (uint16_t)3513 // 3478


/* -------------------------------- typedefs -------------------------------- */
typedef struct {
  uint16_t min;
  uint16_t max;
  } TS_RANGE;


/* -------------------------- function prototypes --------------------------- */
static void update_keys(uint16_t adc);


/* ---------------------------- module variables ---------------------------- */
static uint8_t key_pressed;


/* -----------------------------module constants ---------------------------- */
static const uint8_t key_lut[] = {0x00, KEY_CTR, KEY_RIGHT, KEY_LEFT, KEY_UP, KEY_DOWN};

static const TS_RANGE key_range[NOF_KEY_RANGES] =
{
  {
    KEY_N_MIN,
    KEY_N_MAX
  },
  {
    KEY_C_MIN,
    KEY_C_MAX
  },
  {
    KEY_R_MIN,
    KEY_R_MAX
  },
  {
    KEY_L_MIN,
    KEY_L_MAX
  },
  {
    KEY_U_MIN,
    KEY_U_MAX
  },
  {
    KEY_D_MIN,
    KEY_D_MAX
  }
};


/* ---------------------------- public functions ---------------------------- */
void keys_init(void) {
  }


void keys_exec(void) {
  static uint32_t tick;

  if (HAL_GetTick() != tick) {
    tick = HAL_GetTick();         // Every ms
    update_keys((uint16_t)adc_get(ADC_VAL_KEYPAD));
    }
  }


uint8_t keys_get(void) {
  return key_pressed;
  }


/* ---------------------------- module functions ---------------------------- */
static void update_keys(uint16_t adc) {
  static bool     is_key;
  static uint32_t tick;
  static uint8_t  key_det;

  /* Set global key_pressed to the key corresponding to an ADC value range
   * as defined in key_range and key_lut, when the ADC value has been in
   * range for more than TIME_ADC_KEY_SETTLE.
   */
  for (uint8_t key = 0; key < NOF_KEY_RANGES; key++) {
    if ((adc >= key_range[key].min) && (adc <= key_range[key].max)) {
      if (!is_key) {
        is_key = true;
        tick = HAL_GetTick();
        key_det = key;
        }
      else {
        if (key == key_det) {
          if ((HAL_GetTick() - tick) > TIME_ADC_KEY_SETTLE) {
            key_pressed = key_lut[key];
            is_key = false;
            }
          }
        else {
          is_key = false;
          }
        }
      break;
      }
    }
  }
