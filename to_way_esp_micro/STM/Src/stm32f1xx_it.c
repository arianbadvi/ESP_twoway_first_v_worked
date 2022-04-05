/**
  ******************************************************************************
  * @file    stm32f1xx_it.c
  * @brief   Interrupt Service Routines.
  ******************************************************************************
  *
  * COPYRIGHT(c) 2021 STMicroelectronics
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
/* Includes ------------------------------------------------------------------*/
#include "stm32f1xx_hal.h"
#include "stm32f1xx.h"
#include "stm32f1xx_it.h"

/* USER CODE BEGIN 0 */
#define ACTIVE          1
#define INACTIVE        0

#define UART4_INX   0
#define UART5_INX   1
#define UART1_INX   2


#define SEN_DATAPACK_SIZE       33



extern long int sen_port_c [2];
extern long int sen_port_r_check_c [2];
extern long int bno_c, mlx_c;
extern long int gps_wire_connection_c;

extern const int GPS_GETDATA_SIZE;

extern UART_HandleTypeDef huart1;
extern  char    str_buffer [400];
extern  int     index;
extern  volatile unsigned char gps_data_ready;


extern  int     index_uart4;
extern  int     index_uart5;
extern  char    data_sen[2][SEN_DATAPACK_SIZE], data_sen2[2][SEN_DATAPACK_SIZE];
extern unsigned char uart4_rx_state;
extern unsigned char    uart4_rx_complete;
extern unsigned char    uart5_rx_state;
extern unsigned char    uart5_rx_complete;

char test = 0;
extern char test_char_gps;

/* USER CODE END 0 */

/* External variables --------------------------------------------------------*/
extern I2C_HandleTypeDef hi2c2;
extern SPI_HandleTypeDef hspi1;
extern TIM_HandleTypeDef htim2;
extern UART_HandleTypeDef huart4;
extern UART_HandleTypeDef huart5;
extern UART_HandleTypeDef huart1;

/******************************************************************************/
/*            Cortex-M3 Processor Interruption and Exception Handlers         */ 
/******************************************************************************/

/**
* @brief This function handles System tick timer.
*/
void SysTick_Handler(void)
{
  /* USER CODE BEGIN SysTick_IRQn 0 */

  /* USER CODE END SysTick_IRQn 0 */
  HAL_IncTick();
  HAL_SYSTICK_IRQHandler();
  /* USER CODE BEGIN SysTick_IRQn 1 */

  /* USER CODE END SysTick_IRQn 1 */
}

/******************************************************************************/
/* STM32F1xx Peripheral Interrupt Handlers                                    */
/* Add here the Interrupt Handlers for the used peripherals.                  */
/* For the available peripheral interrupt handler names,                      */
/* please refer to the startup file (startup_stm32f1xx.s).                    */
/******************************************************************************/

/**
* @brief This function handles TIM2 global interrupt.
*/
void TIM2_IRQHandler(void)
{
  /* USER CODE BEGIN TIM2_IRQn 0 */

  /* USER CODE END TIM2_IRQn 0 */
  HAL_TIM_IRQHandler(&htim2);
  /* USER CODE BEGIN TIM2_IRQn 1 */

  /* USER CODE END TIM2_IRQn 1 */
}

/**
* @brief This function handles I2C2 event interrupt.
*/
void I2C2_EV_IRQHandler(void)
{
  /* USER CODE BEGIN I2C2_EV_IRQn 0 */

  /* USER CODE END I2C2_EV_IRQn 0 */
  HAL_I2C_EV_IRQHandler(&hi2c2);
  /* USER CODE BEGIN I2C2_EV_IRQn 1 */

  /* USER CODE END I2C2_EV_IRQn 1 */
}

/**
* @brief This function handles SPI1 global interrupt.
*/
void SPI1_IRQHandler(void)
{
  /* USER CODE BEGIN SPI1_IRQn 0 */

  /* USER CODE END SPI1_IRQn 0 */
  HAL_SPI_IRQHandler(&hspi1);
  /* USER CODE BEGIN SPI1_IRQn 1 */

  /* USER CODE END SPI1_IRQn 1 */
}

/**
* @brief This function handles USART1 global interrupt.
*/
void USART1_IRQHandler(void)
{
  /* USER CODE BEGIN USART1_IRQn 0 */

  /* USER CODE END USART1_IRQn 0 */
  HAL_UART_IRQHandler(&huart1);
  /* USER CODE BEGIN USART1_IRQn 1 */
  //str_buffer[index] = (USART1 -> DR & 0xFF);
  test_char_gps = (USART1 -> DR & 0xFF);
  NVIC_ClearPendingIRQ(USART1_IRQn);
  index++;   
  if (index > GPS_GETDATA_SIZE + 10)      index = 0; 
  if ( (index > GPS_GETDATA_SIZE) && (index < GPS_GETDATA_SIZE + 10))     gps_data_ready = 1;

  /* USER CODE END USART1_IRQn 1 */
}

/**
* @brief This function handles UART4 global interrupt.
*/
void UART4_IRQHandler(void)
{
  /* USER CODE BEGIN UART4_IRQn 0 */
  
  /* USER CODE END UART4_IRQn 0 */
  HAL_UART_IRQHandler(&huart4);
  /* USER CODE BEGIN UART4_IRQn 1 */
  test = (UART4 -> DR & 0xFF);
  NVIC_ClearPendingIRQ(UART4_IRQn);
  
  if(uart4_rx_state == 1)
  {
    data_sen[0][index_uart4] = (UART4 -> DR & 0xFF);
    //NVIC_ClearPendingIRQ(USART4_IRQn);
    index_uart4++;
  
    if (data_sen[0][index_uart4 - 1] == ';')
    {
      uart4_rx_state = 0;
      uart4_rx_complete = 1;
      index_uart4 = 0;
    }
  }
  /* USER CODE END UART4_IRQn 1 */
}

/**
* @brief This function handles UART5 global interrupt.
*/
void UART5_IRQHandler(void)
{
  /* USER CODE BEGIN UART5_IRQn 0 */

  /* USER CODE END UART5_IRQn 0 */
  HAL_UART_IRQHandler(&huart5);
  /* USER CODE BEGIN UART5_IRQn 1 */
  test = (UART5 -> DR & 0xFF);
  NVIC_ClearPendingIRQ(UART5_IRQn);
  
  if(uart5_rx_state == 1)
  {
    data_sen[1][index_uart5] = (UART5 -> DR & 0x00FF);
    index_uart5++; 
    if (data_sen[1][index_uart5 - 1] == ';')
    {
      uart5_rx_state = 0;
      uart5_rx_complete = 1;
      index_uart5 = 0;
    }
  
  }
  /* USER CODE END UART5_IRQn 1 */
}

/* USER CODE BEGIN 1 */

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
  if (huart->Instance == UART4)
  {
   
  }
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  if (htim->Instance == TIM2)
  {
    sen_port_c [UART4_INX] ++;
    sen_port_r_check_c [UART4_INX] ++;
    //counter3++;
    sen_port_c [UART5_INX] ++;
    sen_port_r_check_c [UART5_INX] ++;
    bno_c ++;
    mlx_c ++;
    gps_wire_connection_c ++;
  }
}









/* USER CODE END 1 */
/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
