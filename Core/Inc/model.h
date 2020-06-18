#ifndef MODEL_H
#define MODEL_H

/* -------------------------------- includes -------------------------------- */
#include "o4commsprotocol.h"


/* -------------------------------- defines --------------------------------- */
#define DISPLAY_CMD_WRITE 	(uint8_t)0x80
#define DISPLAY_CMD_MASK  	(uint8_t)0x7F

#define MSG_USER_KEYDOWN	(uint8_t)0x2B
#define MSG_USER_KEYUP   	(uint8_t)0x2C

#define CMDGROUP_CT       CommsCommandGroup_User1
#define CMDGROUP_DH       CommsCommandGroup_User2


/* -------------------------------- typedefs -------------------------------- */

typedef uint8_t         TCD_KEY;
typedef uint8_t 		TE_MESSAGE_TAG;

/* Message */
typedef struct {
  TE_MESSAGE_TAG tag;
  const void     *payload;
  uint8_t        size;
  } TS_MESSAGE;

typedef void (*TF_MESSAGE_CB)(const TS_MESSAGE *msg);

typedef enum {
  DISPLAY,
  NOF_MODELS
  } TE_MODEL;


typedef struct {
  void (*init)(void);
  void (*exec)(void);
  void (*set_callback)(TF_MESSAGE_CB cb);
  void (*get)(TE_MESSAGE_TAG tag);
  void (*set)(const TS_MESSAGE *msg);
  bool (*cmd)(const TS_MESSAGE *msg);
  } TS_MODEL;


/* ---------------------------- public functions ---------------------------- */
void model_init(void);
void model_exec(void);
bool model_register_callback(TF_MESSAGE_CB cb);
bool model_release_callback(TF_MESSAGE_CB cb);
void model_get(TE_MESSAGE_TAG tag);
void model_set(const TS_MESSAGE *msg);
bool model_cmd(TE_MODEL model, const TS_MESSAGE *msg);
void model_attach(TE_MODEL tag, const TS_MODEL *model);
void model_send_request(TE_MODEL model, const TS_MESSAGE *msg);

enum CommsMsgHandlingState model_process_msg(struct CommsFrame *frame);


#endif
