/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  ** This notice applies to any and all portions of this file
  * that are not between comment pairs USER CODE BEGIN and
  * USER CODE END. Other portions of this file, whether 
  * inserted by the user or by software development tools
  * are owned by their respective copyright owners.
  *
  * COPYRIGHT(c) 2020 STMicroelectronics
  *
  * Redistribution and use in source and binary forms, with or without modification,
  * are permitted provided that the following conditions are met:
  *   1. Redistributions of source code must retain the above copyright notice,
  *      this list of conditions and the following disclaimer.
  *   2. Redistributions in binary form must reproduce the above copyright notice,
  *      this list of conditions and the following disclaimer in the documentation
  *      and/or other materials provided with the distribution.
  *   3. Neither the name of STMicroelectronics nor the names of its contributors
  *      may be used to endorse or promote products derived from this software
  *      without specific prior written permission.
  *
  * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
  * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
  * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
  * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
  * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
  * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
  * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
  * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
  * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
  * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H__
#define __MAIN_H__

/* Includes ------------------------------------------------------------------*/

/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private define ------------------------------------------------------------*/
#define PSC_2KHZ 36000
#define ARR_2HZ 1000

#define Phy_tx_valid_Pin GPIO_PIN_7
#define Phy_tx_valid_GPIO_Port GPIOA
#define Phy_tx_data_bus_pin0_Pin GPIO_PIN_0
#define Phy_tx_data_bus_pin0_GPIO_Port GPIOB
#define Phy_tx_data_bus_pin1_Pin GPIO_PIN_1
#define Phy_tx_data_bus_pin1_GPIO_Port GPIOB
#define Phy_tx_data_bus_pin2_Pin GPIO_PIN_2
#define Phy_tx_data_bus_pin2_GPIO_Port GPIOB
#define Phy_rx_data_bus_pin2_Pin GPIO_PIN_10
#define Phy_rx_data_bus_pin2_GPIO_Port GPIOB
#define Phy_rx_data_bus_pin3_Pin GPIO_PIN_11
#define Phy_rx_data_bus_pin3_GPIO_Port GPIOB
#define Phy_rx_data_bus_pin4_Pin GPIO_PIN_12
#define Phy_rx_data_bus_pin4_GPIO_Port GPIOB
#define Phy_rx_data_bus_pin5_Pin GPIO_PIN_13
#define Phy_rx_data_bus_pin5_GPIO_Port GPIOB
#define Phy_rx_data_bus_pin6_Pin GPIO_PIN_14
#define Phy_rx_data_bus_pin6_GPIO_Port GPIOB
#define Phy_rx_data_bus_pin7_Pin GPIO_PIN_15
#define Phy_rx_data_bus_pin7_GPIO_Port GPIOB
#define Phy_rx_valid_Pin GPIO_PIN_6
#define Phy_rx_valid_GPIO_Port GPIOC
#define Phy_tx_busy_Pin GPIO_PIN_7
#define Phy_tx_busy_GPIO_Port GPIOC
#define Phy_reset_Pin GPIO_PIN_8
#define Phy_reset_GPIO_Port GPIOC
#define Phy_clock_Pin GPIO_PIN_9
#define Phy_clock_GPIO_Port GPIOC
#define Phy_clock_EXTI_IRQn EXTI9_5_IRQn
#define Phy_tx_data_bus_pin3_Pin GPIO_PIN_3
#define Phy_tx_data_bus_pin3_GPIO_Port GPIOB
#define Phy_tx_data_bus_pin4_Pin GPIO_PIN_4
#define Phy_tx_data_bus_pin4_GPIO_Port GPIOB
#define Phy_tx_data_bus_pin5_Pin GPIO_PIN_5
#define Phy_tx_data_bus_pin5_GPIO_Port GPIOB
#define Phy_tx_data_bus_pin6_Pin GPIO_PIN_6
#define Phy_tx_data_bus_pin6_GPIO_Port GPIOB
#define Phy_tx_data_bus_pin7_Pin GPIO_PIN_7
#define Phy_tx_data_bus_pin7_GPIO_Port GPIOB
#define Phy_rx_data_bus_pin0_Pin GPIO_PIN_8
#define Phy_rx_data_bus_pin0_GPIO_Port GPIOB
#define Phy_rx_data_bus_pin1_Pin GPIO_PIN_9
#define Phy_rx_data_bus_pin1_GPIO_Port GPIOB

/* ########################## Assert Selection ############################## */
/**
  * @brief Uncomment the line below to expanse the "assert_param" macro in the 
  *        HAL drivers code
  */
/* #define USE_FULL_ASSERT    1U */

/* USER CODE BEGIN Private defines */




/* USER CODE END Private defines */

#ifdef __cplusplus
 extern "C" {
#endif
void _Error_Handler(char *, int);

#define Error_Handler() _Error_Handler(__FILE__, __LINE__)
#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H__ */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
