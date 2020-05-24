#include "template.h"

uint8_t newTxDataAvaliable = 0; // flag to DLL interface, set buy USER to '1' when has some data to send (after newTxData has been set), set automaticly by Interface to '0' after sent.
uint8_t newTxData = 0; // hold data to send to Phy, (relevant only when "newTxDataAvaliable" == 1), set by USER.
uint8_t newRxDataAvaliable = 0; // flag from DLL interface, set buy Interface to '1' when  recieve somting in Rx from phy, as to be reset by USER after read the "newRxData" content 
uint8_t newRxData = 0;// hold data received from Phy, (relevant only when "newRxDataAvaliable" == 1), set by Interface.
uint8_t DllAlive = 0;
//req from uppre layer struct
Ethernet_req* req_desc = NULL;

uint8_t printer_flag = NO_PRINT;
uint8_t recieved_value = 0;


int fputc(int ch, FILE *f) // over-run pritf
{ 
/* Place your implementation of fputc here */ 
	HAL_UART_Transmit(&huart2,(uint8_t*)&ch,1,1); // 1 tick waiting (1ms) enough for 87us/byte at 115200
	return ch; 
}

int fgetc(FILE *f)  // over-run scanf
{ 
	uint8_t ch=0,status = 0;
	do
	{
		status = HAL_UART_Receive(&huart2,&ch,1,10);
	}while(status == HAL_ERROR || status == HAL_TIMEOUT);  // wait until the user will put the carecters.
	if(ch == 0x7E) //"char of value ~ will reset the board"
		NVIC_SystemReset();
	return (ch); 
}

inline void printer()
{
	switch(printer_flag)
	{
		case PRINT_REQ_MAC: // print "enter dest mac address:
			printf("Enter destination MAC address (FORMAT = XXXXXXXXXXXX):\r\n");
			printer_flag = NO_PRINT;
			break;
		case PRINT_DEST_MAC: // print dest mac
			printf("\r\nDestination MAC address is: %02X:%02X:%02X:%02X:%02X:%02X\r\n",
				req_desc->destinationMac[0],req_desc->destinationMac[1],req_desc->destinationMac[2],
				req_desc->destinationMac[3],req_desc->destinationMac[4],req_desc->destinationMac[5]);
			printf("Enter payload and then press 'Enter' (max size is: 1500 bytes!)\r\n");
			printer_flag = NO_PRINT;
			break;
		case PRINT_SIZE_ERROR:
			printf("\r\nError! packet overflow! - max size is %d bytes (press any key to restart)\r\n",TX_BUFFER_SIZE); 
			printer_flag = NO_PRINT;
			break;
		case PRINT_DLL_GOT_REQUEST:
			printf("\r\nRequest sent to DLL (wait for transmission to complete).\r\n"); 
			printer_flag = NO_PRINT;
			break;
		case PRINT_DLL_BUSY:
			printf("\rDll busy! wait for Dll to read the request!\r\n"); 
			printer_flag = NO_PRINT;	
			break;
		case PRINT_DLL_GOT_THE_REQUST:
			printf("\rDll got the request! (press any key to build a next request)\r\n"); 
			printer_flag = NO_PRINT;	
			break;
	}
}


/******************************************************************************/
/* STM32F3xx Peripheral Interrupt Handlers                                    */
/* Add here the Interrupt Handlers for the used peripherals.                  */
/* For the available peripheral interrupt handler names,                      */
/* please refer to the startup file (startup_stm32f3xx.s).                    */
/******************************************************************************/

/**
* @brief This function handles EXTI line[9:5] interrupts.
*/
void EXTI9_5_IRQHandler(void)
{
  /* USER CODE BEGIN EXTI9_5_IRQn 0 */
	static int8_t edge = EDGE_REST;
	static uint8_t FalseInterrupt_cut = RESET;
	static uint8_t DataBusToWrite;
	static GPIO_PinState setDataBusValidValue = GPIO_PIN_RESET;

	uint8_t PhyIsNotInReset = HAL_GPIO_ReadPin(Phy_reset_GPIO_Port, Phy_reset_Pin);
	uint8_t CurrentSampledEdge = HAL_GPIO_ReadPin(Phy_clock_GPIO_Port, Phy_clock_Pin);
	
	if (DllAlive) // wait for DLL init to finish.
	{	
		if ((edge == EDGE_REST) && (PhyIsNotInReset))
		{
			edge = CurrentSampledEdge; // INIT
			FalseInterrupt_cut = RESET;			
		}
		else if (!PhyIsNotInReset)
			edge = EDGE_REST; //RESET
		else if (FalseInterrupt_cut == RESET) // wait for next trunsaction
			edge^=1; 
		
			if ((edge != EDGE_REST) && (edge == CurrentSampledEdge)) // clear some false interrupt transaction hazarads
			{
				FalseInterrupt_cut = RESET;
				if (edge == EDGE_FALL) 
					readBus(&DataBusToWrite,&setDataBusValidValue);
				else //if (edge == EDGE_RISE) write..
				{
					HAL_GPIO_WritePin(Phy_tx_data_bus_pin0_GPIO_Port,(uint16_t)DataBusToWrite,GPIO_PIN_SET);
					HAL_GPIO_WritePin(Phy_tx_data_bus_pin0_GPIO_Port,(uint16_t)(~DataBusToWrite),GPIO_PIN_RESET);
					HAL_GPIO_WritePin(Phy_tx_valid_GPIO_Port,Phy_tx_valid_Pin,setDataBusValidValue);
				}				
			}	else if (edge != EDGE_REST)
				FalseInterrupt_cut = SET;
	}
  /* USER CODE END EXTI9_5_IRQn 0 */
  HAL_GPIO_EXTI_IRQHandler(GPIO_PIN_9);
  /* USER CODE BEGIN EXTI9_5_IRQn 1 */

  /* USER CODE END EXTI9_5_IRQn 1 */
}


/**
* @brief This function handles USART2 global interrupt / USART2 wake-up interrupt through EXTI line 26.
*/
void USART2_IRQHandler(void)
{
  /* USER CODE BEGIN USART2_IRQn 0 */
	static uint8_t uartState = UART_STATE_IDLE;
	static uint16_t index = 0;
	static char macVal[3] = {0};
	uint8_t data = huart2.Instance->RDR ;
	uint8_t* payload;
	uint8_t sendLoopBack = 1;
	switch (uartState)
	{
		case UART_STATE_IDLE:	
				if (!req_desc) // only if req is free! then free for getting new req.
				{ 
					req_desc =(Ethernet_req*) malloc (sizeof(Ethernet_req));
					payload = (uint8_t*) malloc (TX_BUFFER_SIZE);
					if (req_desc && payload)
					{
						req_desc->payload = payload;
						uartState = UART_STATE_GETTING_MAC ; 
						printer_flag = PRINT_REQ_MAC;
						index = 0;
					}else
					{
						if (req_desc)
						{
							free(req_desc);
							req_desc = NULL;
						}
						puts("Rx buffer calloc fail, wait for old requests to finish!");
					}
				} else {
					sendLoopBack = 0;
					printer_flag = PRINT_DLL_BUSY;
				}
			break;
		case UART_STATE_GETTING_MAC:
			macVal[index & 0x1] = data; // odd => first byte, even = > second byte , therd byte is for string end "\0'
			if (index & 0x1)
				req_desc->destinationMac[index>>1] = strtol(macVal,NULL,16);			//convert string to hex number.
			index++;
			if ((index>>1) == MAC_ADDRESS_LEN) // finished mac recieve.
			{
				uartState = UART_STATE_DATA_REACIVE;
				printer_flag = PRINT_DEST_MAC;
				SET_PAYLOAD_SIZE((req_desc),0);
			}		
		break;
		case UART_STATE_DATA_REACIVE:
			if (GET_PAYLOAD_SIZE((req_desc)) > TX_BUFFER_SIZE)
			{
				uartState = UART_STATE_IDLE ; 
				printer_flag = PRINT_SIZE_ERROR;
			}else if (data != '\r') // not finishd
				{
					req_desc->payload[GET_PAYLOAD_SIZE(req_desc)] = data;
					INC_PAYLOAD_SIZE(req_desc);
				}else
				{
					req_desc->payload[GET_PAYLOAD_SIZE(req_desc)] = '\0';
					uartState = UART_STATE_IDLE ; 
					req_desc->vld = SET;
					printer_flag = PRINT_DLL_GOT_REQUEST;
				}
			break;
	}
	if (sendLoopBack)
		HAL_UART_Transmit(&huart2,&data,1,10);
  /* USER CODE END USART2_IRQn 0 */
  HAL_UART_IRQHandler(&huart2);
  /* USER CODE BEGIN USART2_IRQn 1 */
	
  /* USER CODE END USART2_IRQn 1 */
}


//DO not touch! 
void readBus(uint8_t *DataBusToWrite, GPIO_PinState *setDataBusValidValue)
{
	uint8_t Phy_busy = HAL_GPIO_ReadPin(Phy_tx_busy_GPIO_Port, Phy_tx_busy_Pin); // check if Phy free for Tx request.
	uint8_t Rx_valid = HAL_GPIO_ReadPin(Phy_rx_valid_GPIO_Port, Phy_rx_valid_Pin); // check if Phy put some new Rx data on interface.

	//Rx
	if(Rx_valid) // new Rx data
	{
		newRxData = (uint8_t)((Phy_rx_data_bus_pin0_GPIO_Port->IDR) >> 8);
		newRxDataAvaliable = 1;
	}
	
	//Tx	
	if (*setDataBusValidValue == GPIO_PIN_SET) // Tx valid stay for one cycle.
		*setDataBusValidValue = GPIO_PIN_RESET;
	else if (!Phy_busy && newTxDataAvaliable) // phy ready for new word && we have someting to send
	{ // set values that will put to wire in next rising edge.
		*DataBusToWrite = newTxData;
		*setDataBusValidValue = GPIO_PIN_SET;
		newTxDataAvaliable = 0; // update data took by interface.
	}
		
}

uint8_t isRxByteReady(void)
{ // return the value of newRxDataAvaliable flag
	return newRxDataAvaliable;
}

uint8_t getByte(void)
{ // get byte from Interface and reset flag, IMPORTANT: read this funtion without check "isByteReady()" first will return false result.
	uint8_t newByte = newRxData;
	newRxDataAvaliable = 0;
	return newByte;
}

uint8_t isPhyTxReady(void)
{ // return '1' if last byte sent to phy ans '0' else
	return !newTxDataAvaliable;
}

uint8_t sendByte(uint8_t data)
{ // sent byte to interface, IMPORTANT: using this methode without use "isPhyTxReady()" mey reasult return '0' meen - fail (last byte didn't sent yet) , id success return '1'	
	if (newTxDataAvaliable)
		return 0; // fail! last byte didn't sent yet.
	newTxData = data;
	newTxDataAvaliable = 1;
	return 1;
}

uint8_t isNewTxRequest(){
	return req_desc && req_desc->vld;
}

Ethernet_req* getTxRequeset(){ 
	// Important! - after finish using result need to free the Ethernet_req & Ethernet_req.payload.
	Ethernet_req* temp = req_desc;
	req_desc = NULL;
	if (temp)
		printer_flag = PRINT_DLL_GOT_THE_REQUST;
	return temp;
}
