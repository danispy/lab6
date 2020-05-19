
#ifndef __TEMPLATE_H
#define __TEMPLATE_H

#include <stdlib.h>
#include "stm32f3xx_hal.h"
#include "Ethernet.h"


#define UART_STATE_GETTING_MAC 1
#define UART_STATE_IDLE 0
#define UART_STATE_DATA_REACIVE 2

#define EDGE_REST -1
#define EDGE_RISE 1
#define EDGE_FALL 0

#define NO_PRINT 0
#define PRINT_REQ_MAC 1
#define PRINT_DEST_MAC 2
#define PRINT_DLL_GOT_REQUEST 3
#define PRINT_SIZE_ERROR 4
#define PRINT_DLL_BUSY 5
#define PRINT_DLL_GOT_THE_REQUST 6

#define TX_BUFFER_SIZE 1500 

extern UART_HandleTypeDef huart2;


int fputc(int ch, FILE *f); 									//over-run pritf
int fgetc(FILE *f);  													//over-run scanf
void printer(void);

//Interrupts handlers
void EXTI9_5_IRQHandler(void);
void USART2_IRQHandler(void);

void readBus(uint8_t *DataBusToWrite, GPIO_PinState *setDataBusValidValue);

/* Dll Rx functions */
uint8_t isRxByteReady(void);									//check if there is new byte received from phy.
uint8_t getByte(void); 												//get byte from phy, it's mandatory to use "isRxByteReady()" before calling this function.

/* Dll Tx functions */
uint8_t isPhyTxReady(void);										//check if the phy_Tx ready to get another byte to send.
uint8_t sendByte(uint8_t data); 							//sent byte to phy, it's mandatory to use "isPhyTxReady()" before call this function
uint8_t isNewTxRequest(void); 								//check if the upper_layer sent us new data to transmit.
Ethernet_req* getTxRequeset(void);						//get from upper_layer data to transmit, it's mandatory to use "isNewTxRequest()" before call this function.

// Interface params
extern uint8_t newTxDataAvaliable; // flag to DLL interface, set buy USER to '1' when has some data to send (after newTxData has been set), set automaticly by Interface to '0' after sent.
extern uint8_t newTxData; // hold data to send to Phy, (relevant only when "newTxDataAvaliable" == 1), set by USER.
extern uint8_t newRxDataAvaliable; // flag from DLL interface, set buy Interface to '1' when  recieve somting in Rx from phy, as to be reset by USER after read the "newRxData" content 
extern uint8_t newRxData;// hold data received from Phy, (relevant only when "newRxDataAvaliable" == 1), set by Interface.
extern uint8_t DllAlive;
//

//req from uppre layer struct
extern Ethernet_req* req_desc;
//

extern uint8_t printer_flag;
extern uint8_t recieved_value;

#endif /* __TEMPLATE_H */
