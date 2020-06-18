#ifndef KEYS_H
#define KEYS_H

/* -------------------------------- defines --------------------------------- */
#define KEY_LEFT  (uint8_t)0x01
#define KEY_RIGHT (uint8_t)0x02
#define KEY_UP    (uint8_t)0x04
#define KEY_DOWN  (uint8_t)0x08
#define KEY_CTR   (uint8_t)0x10


/* ---------------------------- public functions ---------------------------- */
void keys_init(void);
void keys_exec(void);
uint8_t keys_get(void);


#endif
