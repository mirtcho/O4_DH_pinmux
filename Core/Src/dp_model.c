
/* -------------------------------- includes -------------------------------- */
#include <stdint.h>
#include <stdbool.h>
#include "stm32g4xx_hal.h"
#include "keys.h"
#include "model.h"
#include "dp_model.h"


/* -------------------------------- defines --------------------------------- */



#define USER_KEY_RATE_MAX_MS    (uint32_t)200
#define TIME_HOLD_KEY_REPEAT    (uint32_t)500


/* -------------------------- function prototypes --------------------------- */
static void init(void);
static void exec(void);

static void disp_put(uint8_t tag);
static void update_key_ui(void);


/* ---------------------------- module variables ---------------------------- */
static TCD_KEY    key_down;
static TCD_KEY    key_up;


/* ---------------------------- public constants ---------------------------- */
const TS_MODEL dp_model =
{
  init,
  exec,
  NULL,
  NULL,
  NULL,
  NULL,
};


/* ---------------------------- public functions ---------------------------- */
static void init(void) {
  }


static void exec(void) {
  update_key_ui();
  }


/* ---------------------------- module functions ---------------------------- */
static void update_key_ui(void) {
  static uint32_t tick_repeat;
  static uint32_t tick_lockout;
  static uint8_t  key_pressed_1;
  static bool     enable_key;

  uint8_t key_pressed = keys_get();

  /* Send key up event */
  key_up = (key_pressed_1 ^ key_pressed) & (uint8_t)~key_pressed;
  if (key_up) {
    disp_put(MSG_USER_KEYUP);
    }

  /* Limit user rate. */
  if (!key_pressed) {
    if ((HAL_GetTick() - tick_lockout) >= USER_KEY_RATE_MAX_MS) {
      tick_lockout = HAL_GetTick();
      enable_key = true;
      }
    }

  /* Auto-repeat when holding key  */
  if ((HAL_GetTick() - tick_repeat) >= TIME_HOLD_KEY_REPEAT) {
    tick_repeat = HAL_GetTick();
    enable_key = true;
    }

  if (key_pressed && enable_key) {
    enable_key = false;
    tick_repeat = HAL_GetTick();
    tick_lockout = HAL_GetTick();

    key_down = key_pressed;
    disp_put(MSG_USER_KEYDOWN);
    }

  key_pressed_1 = key_pressed;
  }


static void disp_put(uint8_t tag) {
	TCD_KEY pl;

	switch (tag) {
	  case MSG_USER_KEYDOWN:
		pl = key_down;
		break;

	  case MSG_USER_KEYUP:
		pl = key_up;
		break;

	  default:
		break;
	  }

	TS_MESSAGE msg = {.tag = tag | DISPLAY_CMD_WRITE,
					  .size = sizeof(TCD_KEY),
					  .payload = &pl};

	model_send_request(DISPLAY, &msg);

}

