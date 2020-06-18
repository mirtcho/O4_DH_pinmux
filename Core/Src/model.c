
/*
 *  model.c
 *
 *  Concrete models for each logical endpoint are attached to model which
 *  implements the communications callback and passes commands to the correct
 *  model.
 *
 *  A command is 16 bits, of which bit 15-8 are unused, bit 7 determines the
 *  request type (1 = PUT, 0 = GET) and bits 6-0 are the command tag.
 *
 *
 *  For displayholder (DH) to display (D) communication there is one possible kind
 *  of transaction:
 *
 *  1) DisplayHolder initiates PUT request to D. D answers with result (no payload).
 *  2) Controller initiates PUT request (TODO)
 *
 * ATTENTION: There is no explicit (de)marshalling of data - we're all using the
 * native little-endian ordering of the STM32 platform, and the 'line' spec is
 * also LE. When this code needs to be ported to a big endian architecture, the
 * default case in model_update can't be used and every message's payload must
 * be from LE, as well as converted to LE in the consumer in model_exec.
 *
 */



/* -------------------------------- includes -------------------------------- */
#include <stdint.h>
#include <stdbool.h>
#include "stm32g4xx_hal.h"
#include "o4nodes.h"
#include "model.h"


/* -------------------------------- defines --------------------------------- */
#define MAX_CALLBACKS           (uint8_t)1

// TODO not sure now whether or not the display holder will have more models


/* -------------------------------- typedefs -------------------------------- */

/* -------------------------- function prototypes --------------------------- */
static void do_callbacks(const TS_MESSAGE *msg);
//static bool do_model_cmd(uint16_t group, TS_MESSAGE *msg);


/* ---------------------------- module variables ---------------------------- */
static const TS_MODEL       *models[NOF_MODELS];
static TF_MESSAGE_CB        callback[MAX_CALLBACKS];
static uint8_t              msg_buf[COMMS_MESSAGE_MAX_SIZE];
static struct CommsMessage  comms_msg = {msg_buf, 0};



/* ---------------------------- public functions ---------------------------- */
void model_attach(TE_MODEL tag, const TS_MODEL *model) {
  if (tag < NOF_MODELS) {
    models[tag] = model;
    if (models[tag]->set_callback) {
      models[tag]->set_callback(do_callbacks);
      }
    if (models[tag]->init) {
      models[tag]->init();
      }
    }
  }


void model_init(void) {
  }


void model_exec(void) {
  for (uint8_t i = 0; i < NOF_MODELS; i++) {
    if (models[i] && models[i]->exec) {
      models[i]->exec();
      }
    }
  }


bool model_register_callback(TF_MESSAGE_CB cb) {
  for (uint8_t i = 0; i < MAX_CALLBACKS; i++) {
    if (callback[i] == NULL) {
      callback[i] = cb;
      return true;
      }
    }
  return false;
  }


bool model_release_callback(TF_MESSAGE_CB cb) {
  for (uint8_t i = 0; i < MAX_CALLBACKS; i++) {
    if (callback[i] == cb) {
      callback[i] = NULL;
      return true;
      }
    }
  return false;
  }


void model_get(TE_MESSAGE_TAG tag) {
  for (uint8_t i = 0; i < NOF_MODELS; i++) {
    if (models[i] && models[i]->get) {
      models[i]->get(tag);
      }
    }
  }

void model_set(const TS_MESSAGE *msg) {
  for (uint8_t i = 0; i < NOF_MODELS; i++) {
    if (models[i] && models[i]->set) {
      models[i]->set(msg);
      }
    }
  }


bool model_cmd(TE_MODEL model, const TS_MESSAGE *msg) {
  if (model < NOF_MODELS) {
    if (models[model]->cmd) {
      return models[model]->cmd(msg);
      }
    }
  return false;
  }


void model_send_request(TE_MODEL model, const TS_MESSAGE *msg) {
  // TODO include the model parameter ( think about settling on a single entity used throughout the
  // code to refer to a destination or source i.e. now we have the enum Node, uint8_t source address,
  // or TE_MODEL
  // The model param should be in the TS_MSG_RECORD so that the consumer of the queue knows where to
  // send messages.


  struct CommsMessage m = {.payload = (uint8_t *)msg->payload, .size = msg->size};
  commsFrameEncodeRequest(&comms_msg, CMDGROUP_DH, msg->tag, &m);
  commsSendMessage(Node_DISPLAY_HOLDER, Node_DISPLAY, &comms_msg);


  }


#if 0
/* TODO this is enabled again when we process the lights coming from the CT
 *
 * DISPLAY (light sensor action) ---> CT ---> HOLDER
 * CT ---> HOLDER in case DISPLAY is disconnected
 */

enum CommsMsgHandlingState model_process_msg(struct CommsFrame *frame) {
  enum CommsMsgHandlingState state = CommsMsgHandlingState_Idle;

  TS_MESSAGE msg = {0};

  if (frame->type == CommsType_Request) {
    state = CommsMsgHandlingState_MsgPrepared;

    if ((frame->cmd & ~DISPLAY_CMD_WRITE) >= MSG_COUNT) {
      commsFrameEncodeResponse(frame, CommsResponseResult_NotImplemented);
      return state;
      }

    msg.tag = frame->cmd & 0xFF;

    /* Only a PUT request has a payload. */
    if (msg.tag & DISPLAY_CMD_WRITE) {
      msg.payload = &frame->msg.payload[frame->payloadIdx];
      msg.size    = frame->msg.size;

      /* Special case for MSG_TERM_PRINT */
      if (msg.tag == (MSG_TERM_PRINT | DISPLAY_CMD_WRITE)) {
        term_print(msg.payload);
        commsFrameEncodeResponse(frame, CommsResponseResult_Success);
        return state;
        }
      }

    if (do_model_cmd(frame->group, &msg)) {
      commsFrameEncodeResponse(frame, CommsResponseResult_Success);
      }
    else {
      commsFrameEncodeResponse(frame, CommsResponseResult_NotImplemented);
      }
    }

  else if (frame->type == CommsType_IntermediateResponse) {
    state = CommsMsgHandlingState_Processing;
    }

  else if (frame->type == CommsType_Response) {
    state = CommsMsgHandlingState_Done;

    if ((frame->cmd & ~DISPLAY_CMD_WRITE) >= MSG_COUNT) {
      return state;
      }

    /* Any response from the CT ( TODO, check it's the CT) is a sign we're
     * still connected.
     */
    ack_heartbeat();

      /* Response to a GET request. */
    msg.tag = frame->cmd & 0xFF;
    if (msg.tag != MSG_HEARTBEAT) {
      if (!(msg.tag & DISPLAY_CMD_WRITE)) {
        msg.tag     = msg.tag | DISPLAY_CMD_WRITE;
        msg.payload = &frame->msg.payload[frame->payloadIdx];
        msg.size =    frame->msg.size;
        do_model_cmd(frame->group, &msg);
        }
      }
    }
  return state;
  }
#endif

/* ---------------------------- module functions ---------------------------- */
static void do_callbacks(const TS_MESSAGE *msg) {
  for (uint32_t i = 0; i < MAX_CALLBACKS; i++) {
    if (callback[i]) {
      callback[i](msg);
      }
    }
  }


#if 0
static bool do_model_cmd(uint16_t group, TS_MESSAGE *msg) {
  // TODO decide to stick with groups or use the source address
  switch (group) {
    case CMDGROUP_CT:
      return model_cmd(CONTROLLER, msg);

    case CMDGROUP_DH:
      return model_cmd(DISPLAYHOLDER, msg);

    }
  return false;
  }
#endif


