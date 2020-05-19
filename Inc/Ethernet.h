
#ifndef __ETHERNET_H
#define __ETHERNET_H
#include "main.h"

#define MAC_ADDRESS_LEN 6

#define GET_PAYLOAD_SIZE(_req_) *(((uint16_t*)_req_->payloadSize))
#define SET_PAYLOAD_SIZE(_req_ ,_newval_) (*((uint16_t*)_req_->payloadSize) = _newval_)
#define INC_PAYLOAD_SIZE(_req_) SET_PAYLOAD_SIZE(_req_ ,GET_PAYLOAD_SIZE(_req_) + 1) 


typedef struct{
	__IO uint8_t destinationMac[MAC_ADDRESS_LEN];
	__IO uint8_t sourceMac[MAC_ADDRESS_LEN];
	__IO uint8_t* payload;
	__IO uint8_t payloadSize[2];
	__IO uint8_t typeLength[2];
	__IO uint8_t tagPort[4];
	__IO uint8_t vld;//Internal use only - do not touch!
} Ethernet_req;

typedef struct{
	__IO uint8_t destinationMac[MAC_ADDRESS_LEN];
	__IO uint8_t sourceMac[MAC_ADDRESS_LEN];
	__IO uint8_t* payload;
	__IO uint8_t payloadSize[2];
	__IO uint8_t typeLength[2];
	__IO uint8_t tagPort[4];
	__IO uint8_t syndrom;
} Ethernet_res;




#endif /* __ETHERNET_H */
