#ifndef __DEAL_DATA_H
#define __DEAL_DATA_H

#include "deal_string.h"
#include "cmsis_os.h"
#include "gps.h"
#include "http.h"
#include "mqtt.h"
#include "rtk_L1.h"

void *my_malloc(unsigned int uSize, const char *pszFunc, unsigned int uLine);
void my_free(void *pPtr, const char *pszFunc, unsigned int uLine);

extern char Imei[30];

osStatus_t usart1_send_data_apply(uint8_t *data,uint16_t len);
void usart1_rec_data_apply(uint8_t *data,uint16_t len);
void EC600U_send_msg(char* Name,char* fun,char *Source,uint16_t len);

osStatus_t usart2_send_data_apply(uint8_t *data,uint16_t len);
void usart2_rec_data_apply(uint8_t *data,uint16_t len);

#endif

