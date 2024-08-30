#ifndef __RFID_H
#define __RFID_H

#include "stdint.h"

extern uint8_t ic_data;

void rfid_init(void);
uint8_t rfid_ReadId(unsigned char *IDout);
//void decode_ic_data(void);



#endif
