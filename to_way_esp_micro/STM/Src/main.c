/**
  ******************************************************************************
  * File Name          : main.c
  * Description        : Main program body
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

/* USER CODE BEGIN Includes */
#include <bno055.h>
#include <bno055_stm32.h>
#include <stdio.h>
#include "mlx90614.h"
#include <string.h>
//#include "confiq.h"
//#include "gps.h"

#define ACTIVE          1
#define INACTIVE        0

#define L80_EN_PORT     GPIOC
#define MLX_EN_PORT     GPIOC
#define BNO_EN_PORT     GPIOC
#define ESP_EN_PORT     GPIOC
#define SS_PORT         GPIOB

#define L80_EN_PIN      (1<<3)
#define MLX_EN_PIN      (1<<0)
#define BNO_EN_PIN      (1<<1)
#define ESP_EN_PIN      (1<<2)
#define SS_PIN          (1<<3)

#define SEN0_RTS_EN_PORT        GPIOA
#define SEN1_RTS_EN_PORT        GPIOB

#define SEN0_RTS_EN_PIN (1<<15)
#define SEN1_RTS_EN_PIN (1<<4)

#define UART4_INX   0
#define UART5_INX   1
#define MLX_INX     2
#define UART1_INX   3

#define TEMP_OBJ        0
#define TEMP_AMB        1

#define SEN_DATAPACK_SIZE       33

#define TIMER2_PERIOD   85     
/*
  14745600Hz / 800 Prescaller = 18432
  18432 / 85 = 216Hz
  1 / 216 = 4.6 ms

 mind to place this value in timer function in case of regenerating the project
*/

#define R_VALUE 2

#define RMS_MODE_COMMAND_PACK    "$PMTK314,0,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0*29\r\n\0" 
#define RMS_GPS_OUTPUT  "$GPRMC,073810.000,A,3546.0748,N,05119.6196,E,0.00,196.72,061021,,,D*62\0"



/* USER CODE END Includes */

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c1;
I2C_HandleTypeDef hi2c2;

SPI_HandleTypeDef hspi1;

TIM_HandleTypeDef htim2;

UART_HandleTypeDef huart4;
UART_HandleTypeDef huart5;
UART_HandleTypeDef huart1;
UART_HandleTypeDef huart2;


char gps_validation_test = 0;

/* USER CODE BEGIN PV */
/* Private variables ---------------------------------------------------------*/

char    data_sen[2][SEN_DATAPACK_SIZE], data_sen2[2][SEN_DATAPACK_SIZE];
char    sen_request[2] = {97, 0};                    //116 is ASCII code of 't'
char    error[7] = {'E', 'R', 'R', 'O', 'R', '\n', 0};
char    pack_ready[2];

int     sen_value[3];

volatile long int       sen_port_c [2] ;
volatile long int       sen_port_r_check_c [2];
volatile long int       bno_c = 0;
volatile long int       mlx_c = 0;
volatile long int       gps_wire_connection_c = 0;

unsigned char   sen_send [2];
unsigned char   ch=0;
unsigned char   get_data [3];
unsigned char   sen_data_get_request[2];
float    data_mlx[2];

const int GPS_GETDATA_SIZE = 200;

char    sprint_buffer[20];
char    str[100];
char    str_buffer [800];
char    sen_data_buffer[420];
char    utc_str[30], latitude_str [30], longitude_str[30];
char    gps_validation, gps_ns, gps_ew;
int     index=0;

float   latitude = 0.0, longitude = 0.0;
float   latitude_final = 0.0, longitude_final = 0.0;
unsigned char gps_hour = 0, gps_min = 0, gps_sec = 0;
volatile unsigned char gps_wire_connection = 1;

volatile unsigned char gps_data_ready = 0;

char    utc_str_test[30] = "073810.123\r\n\0";
char    latitude_str_test[30] = "3546.0748\r\n\0";
char    longitude_str_test[30] = "05119.6196\r\n\0";
//char    gps_validation_test = 'A';
char    gps_ns = 'N';
char    gps_ew = 'E';

unsigned char battery_percent = 0;

char first_udp[30] = "aa";
char second_udp[10] = "ccc";
char first_serial[10] = "bbbb";
char second_serial[10] = "OK";
unsigned char volatile start_state = 0;
char buffer[20];

unsigned char   uart4_rx_state = 0;
unsigned char   uart4_rx_complete = 0;
unsigned char   uart5_rx_state = 0;
unsigned char   uart5_rx_complete = 0;

int     index_uart4 = 0;
int     index_uart5 = 0;

unsigned char bno_state = 1;

char test_char_gps = 0;


/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_I2C1_Init(void);
static void MX_I2C2_Init(void);
static void MX_SPI1_Init(void);
static void MX_TIM2_Init(void);
static void MX_UART4_Init(void);
static void MX_UART5_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_USART2_UART_Init(void);

/* USER CODE BEGIN PFP */
/* Private function prototypes -----------------------------------------------*/

/* USER CODE END PFP */

/* USER CODE BEGIN 0 */
void put_str(char *Str, UART_HandleTypeDef *Huart);
void datapack_maker ( char *Des, char *Source, char Max_index, char End);
void two_way_esp_polling (void);


void gps_data_check_extrac_func (char *Str, char *Utc_time, char* Validation,
                                    char *Latitude, char *Ns, char *Longitude, 
                                    char *Ew);
void gps_conv_double (char *In, float *Out);
void gps_cor_final_conv_func(float In, float *Out);
void gps_utc_conv_func (char *Utc_time, unsigned char *Hour, unsigned char *Min,
                        unsigned char *Sec);

/* USER CODE END 0 */

int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration----------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* Configure the system clock */
  SystemClock_Config();

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_I2C1_Init();
  MX_I2C2_Init();
  MX_SPI1_Init();
  MX_TIM2_Init();
  MX_UART4_Init();
  MX_UART5_Init();
  MX_USART1_UART_Init();
  MX_USART2_UART_Init();

  /* USER CODE BEGIN 2 */
  RCC->APB2ENR |= RCC_APB2ENR_AFIOEN;           // Enable A.F. clock
  AFIO->MAPR |= AFIO_MAPR_SWJ_CFG_JTAGDISABLE; // JTAG is disabled, SWD is enabled
    
  HAL_GPIO_WritePin(L80_EN_PORT, L80_EN_PIN, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(MLX_EN_PORT, MLX_EN_PIN, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(GPIOC, BNO_EN_PIN, GPIO_PIN_RESET);
  
  HAL_Delay(1000); 
  
  HAL_GPIO_WritePin(L80_EN_PORT, L80_EN_PIN, GPIO_PIN_SET);
  HAL_GPIO_WritePin(MLX_EN_PORT, MLX_EN_PIN, GPIO_PIN_SET);
  HAL_GPIO_WritePin(BNO_EN_PORT, BNO_EN_PIN, GPIO_PIN_SET);
  HAL_GPIO_WritePin(ESP_EN_PORT, ESP_EN_PIN, GPIO_PIN_SET);
  HAL_GPIO_WritePin(SS_PORT, SS_PIN, GPIO_PIN_SET);
  HAL_GPIO_WritePin(SEN0_RTS_EN_PORT, SEN0_RTS_EN_PIN, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(SEN1_RTS_EN_PORT, SEN1_RTS_EN_PIN, GPIO_PIN_RESET);

  //HAL_Delay(2000);
  //MLX90614_WriteReg(MLX90614_DEFAULT_SA, MLX90614_CFG1, 0xB7C0);
  HAL_Delay(2000);
    
  two_way_esp_polling();
 
  HAL_Delay(2000);
 
  bno055_setup();
  bno055_setOperationModeNDOF();
  bno055_getCalibrationData();
 
  sen_data_buffer[300] = 0;
  sen_data_buffer[301] = '\n';   
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  HAL_TIM_Base_Start_IT(&htim2); 

  for (int i = 0; i<60; i++)
  {
    HAL_Delay(1000);
    sprintf(str,"%d secs\n",i);
    put_str(str, &huart2);
  }
 
  HAL_Delay(1000);
  put_str(RMS_MODE_COMMAND_PACK, &huart1);
  HAL_Delay(1000);
  __HAL_UART_ENABLE_IT(&huart1, UART_IT_RXNE);
  __HAL_UART_ENABLE_IT(&huart4, UART_IT_RXNE);
  __HAL_UART_ENABLE_IT(&huart5, UART_IT_RXNE);
  
  HAL_Delay(2000);
  sen_port_c [UART5_INX] = 0;
  sen_port_r_check_c [UART5_INX] = 0;
  bno_c = 0;
  mlx_c = 200;
  gps_wire_connection_c = 0;
  
   while (1)
    {
    if (sen_port_c[UART4_INX] > 9)
    {
      get_data [UART4_INX] = ACTIVE;
      sen_port_c [UART4_INX]= 0;      
    }
    
    if (sen_port_c[UART5_INX] > 9)
    {
      get_data [UART5_INX] = ACTIVE;
      sen_port_c [UART5_INX]= 0;
    }
    
    if (mlx_c > 199)
    {
      get_data [MLX_INX] = ACTIVE;
      mlx_c = 0;
    }

    
/*    
    if (bno_c > 9)
    {
      bno055_vector_t v = bno055_getVectorEuler();
      sprintf(str, "%0.2f;%0.2f;%0.2f\n", v.x, v.y, v.z);
      put_str(str, &huart2);
      bno_c = 0;   
    }
*/    
    if(get_data [UART4_INX] == ACTIVE)
    {
      HAL_GPIO_WritePin(GPIOA, GPIO_PIN_15, GPIO_PIN_SET);
      HAL_Delay(2);
      HAL_UART_Transmit(&huart4, (unsigned char *)sen_request, 1,10);
      
      HAL_Delay(2);
      HAL_GPIO_WritePin(GPIOA, GPIO_PIN_15, GPIO_PIN_RESET);

      uart4_rx_state = 1;
      index_uart4 = 0;
      
      sen_data_get_request [UART4_INX] = ACTIVE;
      get_data [UART4_INX] = INACTIVE;
    }

    
    if(get_data [UART5_INX] == ACTIVE)
    {
      HAL_GPIO_WritePin(GPIOB, GPIO_PIN_4, GPIO_PIN_SET);
      HAL_Delay(2);
      HAL_UART_Transmit(&huart5, (unsigned char *)sen_request, 1,10);
      
      HAL_Delay(2);
      HAL_GPIO_WritePin(GPIOB, GPIO_PIN_4, GPIO_PIN_RESET);
      
      uart5_rx_state = 1;
      index_uart5 = 0;
      
      sen_data_get_request [UART5_INX]= ACTIVE;
      get_data [UART5_INX] = INACTIVE;
    }
        
    if (get_data [MLX_INX] == ACTIVE)
    {
      data_mlx[TEMP_OBJ] = MLX90614_ReadTemp(MLX90614_DEFAULT_SA, MLX90614_TOBJ1);
      HAL_Delay(10);
      data_mlx[TEMP_AMB] = MLX90614_ReadTemp(MLX90614_DEFAULT_SA, MLX90614_TAMB);
      HAL_Delay(10);
      get_data [MLX_INX] = INACTIVE;
    }
            
    if(sen_data_get_request [UART4_INX] == ACTIVE)
    {
      
      if(uart4_rx_complete == 1)
      {
        sen_send [UART4_INX]= ACTIVE;
        sen_port_r_check_c [UART4_INX]= 0;
        uart4_rx_complete = 0;
      }
      
      /*
      if(sen_port_r_check_c [UART4_INX] > 9)
      {
        put_str(error, &huart2);
        get_data [UART4_INX]= ACTIVE;
        sen_port_r_check_c [UART4_INX]= 0;
      } 
      */
    }


    if(sen_data_get_request [UART5_INX]== ACTIVE)
    {
      if(uart5_rx_complete == 1)
      {
        sen_send [UART5_INX]= ACTIVE;
        sen_port_r_check_c [UART5_INX]= 0;
        uart5_rx_complete = 0;
      }
      
      /*
      if(sen_port_r_check_c [UART5_INX] > 9)
      {
        put_str(error, &huart2);
        get_data [UART5_INX]= ACTIVE;
        sen_port_r_check_c [UART5_INX]= 0;
      } 
      */
    }
 
     
    if (sen_send [UART4_INX] == ACTIVE)
    { 

      datapack_maker((char*)data_sen2, (char*)data_sen, (SEN_DATAPACK_SIZE - 1),0);
      sscanf ((char const*)data_sen2, "%d",&sen_value[0]);
      
      sen_send [UART4_INX] = INACTIVE;
      sen_data_get_request [UART4_INX]= INACTIVE;
      pack_ready[UART4_INX] = ACTIVE;
    }    
   
    if (sen_send [UART5_INX]== ACTIVE)
    {      
    
      datapack_maker((char*)data_sen2 + SEN_DATAPACK_SIZE, (char*)data_sen + SEN_DATAPACK_SIZE, (SEN_DATAPACK_SIZE - 1),0);
      sscanf ((char const*)data_sen2 + SEN_DATAPACK_SIZE, "%d",&sen_value[1]);
      
      sen_send [UART5_INX] = INACTIVE;
      sen_data_get_request [UART5_INX]= INACTIVE;
      pack_ready[UART5_INX] = ACTIVE;
    }    

    if (gps_data_ready == 1)
    {
      
      for (int i = 0; i < GPS_GETDATA_SIZE; i++)        sen_data_buffer[i] = str_buffer[i];
      sen_data_buffer[GPS_GETDATA_SIZE] = 0;
      gps_data_check_extrac_func(sen_data_buffer, utc_str, &gps_validation, latitude_str,
                               &gps_ns, longitude_str, &gps_ew);
      gps_conv_double(latitude_str, &latitude);
      gps_conv_double(longitude_str, &longitude);
      
      gps_utc_conv_func(utc_str, &gps_hour, &gps_min, &gps_sec);
      gps_cor_final_conv_func(latitude, &latitude_final);
      gps_cor_final_conv_func(longitude, &longitude_final);
      
      gps_data_ready = 0;
      gps_wire_connection = ACTIVE;
      gps_wire_connection_c = 0;
    }
    
    if ((pack_ready[UART4_INX] && pack_ready[UART5_INX] ) == ACTIVE)
    //if ((pack_ready[UART4_INX]) == ACTIVE)
    {
      pack_ready[UART4_INX] = INACTIVE;
      pack_ready[UART5_INX] = INACTIVE;
      HAL_GPIO_WritePin(SS_PORT, SS_PIN, GPIO_PIN_RESET);
      
      put_str((char *)data_sen2, &huart2);
      put_str((char *)((char *)data_sen2 +  SEN_DATAPACK_SIZE), &huart2);
      
      sen_value[R_VALUE] = (sen_value[0] + sen_value[1]);
      sprintf(str, "%d\t", sen_value[R_VALUE]); 
      put_str(str, &huart2);
      
      if(bno_state == 1)
      {
        bno055_vector_t v = bno055_getVectorEuler();
        sprintf(str, "%0.2f\t%0.2f\t%0.2f\t", v.z, v.y, v.x); //the x and the z were reverse in tests
        put_str(str, &huart2);      
      }
      else if (bno_state == 2)
      {
        sprintf(str, "BNO Problem\t");
        put_str(str, &huart2);
      }
      else
      {
        sprintf(str, "BNO unavailable\t");
        put_str(str, &huart2);      
      }
      
      
      sprintf(str, "%0.2f\t%0.2f\t", data_mlx[TEMP_OBJ], data_mlx[TEMP_AMB]);
      put_str(str, &huart2);
     
      switch (gps_wire_connection)
      {
      case ACTIVE:
        if(gps_wire_connection_c > 800)  gps_wire_connection = INACTIVE;        
        
          switch (gps_validation)
          {
            case 'V':
              put_str("GPS Data was invalid\t", &huart2);
            break;
      
            case 'A':
              sprintf(str, "%2d:%2d:%2d\t", gps_hour, gps_min, gps_sec);
              put_str(str, &huart2);
        
              sprintf(str, "%2.7f\t%c\t", latitude_final, gps_ns);
              put_str(str, &huart2);
        
              sprintf(str, "%2.7f\t%c\t", longitude_final, gps_ew);
              put_str(str, &huart2);
            break;
            default:
              //sprintf(str,"GPS checking\t",gps_validation);
              //put_str(str, &huart2);
              put_str("GPS Data was invalid\t", &huart2);
              
            break;
            
          }
      break;
      
      case INACTIVE:
        put_str("GPS_ERROR\t", &huart2);           
      break;            
      }
      
      battery_percent = 0;
      HAL_SPI_Receive(&hspi1,&battery_percent,1,10);
      sprintf(str, "%d%%\n", battery_percent);
      put_str(str, &huart2);
      HAL_GPIO_WritePin(SS_PORT, SS_PIN, GPIO_PIN_SET);        
    }
       
  /* USER CODE END WHILE */

  /* USER CODE BEGIN 3 */

  }
  /* USER CODE END 3 */

}

/** System Clock Configuration
*/
void SystemClock_Config(void)
{

  RCC_OscInitTypeDef RCC_OscInitStruct;
  RCC_ClkInitTypeDef RCC_ClkInitStruct;

  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  HAL_RCC_OscConfig(&RCC_OscInitStruct);

  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_SYSCLK;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSE;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;
  HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0);

  HAL_SYSTICK_Config(HAL_RCC_GetHCLKFreq()/1000);

  HAL_SYSTICK_CLKSourceConfig(SYSTICK_CLKSOURCE_HCLK);

  /* SysTick_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(SysTick_IRQn, 0, 0);
}

/* I2C1 init function */
void MX_I2C1_Init(void)
{

  hi2c1.Instance = I2C1;
  hi2c1.Init.ClockSpeed = 100000;
  hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLED;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLED;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLED;
  HAL_I2C_Init(&hi2c1);

}

/* I2C2 init function */
void MX_I2C2_Init(void)
{

  hi2c2.Instance = I2C2;
  hi2c2.Init.ClockSpeed = 400000;
  hi2c2.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c2.Init.OwnAddress1 = 0;
  hi2c2.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c2.Init.DualAddressMode = I2C_DUALADDRESS_DISABLED;
  hi2c2.Init.OwnAddress2 = 0;
  hi2c2.Init.GeneralCallMode = I2C_GENERALCALL_DISABLED;
  hi2c2.Init.NoStretchMode = I2C_NOSTRETCH_DISABLED;
  HAL_I2C_Init(&hi2c2);

}

/* SPI1 init function */
void MX_SPI1_Init(void)
{

  hspi1.Instance = SPI1;
  hspi1.Init.Mode = SPI_MODE_MASTER;
  hspi1.Init.Direction = SPI_DIRECTION_2LINES;
  hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi1.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi1.Init.NSS = SPI_NSS_SOFT;
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_128;
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLED;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLED;
  hspi1.Init.CRCPolynomial = 10;
  HAL_SPI_Init(&hspi1);

}

/* TIM2 init function */
void MX_TIM2_Init(void)
{

  TIM_ClockConfigTypeDef sClockSourceConfig;
  TIM_MasterConfigTypeDef sMasterConfig;

  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 799;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = TIMER2_PERIOD;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  HAL_TIM_Base_Init(&htim2);

  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig);

  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig);

}

/* UART4 init function */
void MX_UART4_Init(void)
{

  huart4.Instance = UART4;
  huart4.Init.BaudRate = 115200;
  huart4.Init.WordLength = UART_WORDLENGTH_8B;
  huart4.Init.StopBits = UART_STOPBITS_1;
  huart4.Init.Parity = UART_PARITY_NONE;
  huart4.Init.Mode = UART_MODE_TX_RX;
  huart4.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart4.Init.OverSampling = UART_OVERSAMPLING_16;
  HAL_UART_Init(&huart4);

}

/* UART5 init function */
void MX_UART5_Init(void)
{

  huart5.Instance = UART5;
  huart5.Init.BaudRate = 115200;
  huart5.Init.WordLength = UART_WORDLENGTH_8B;
  huart5.Init.StopBits = UART_STOPBITS_1;
  huart5.Init.Parity = UART_PARITY_NONE;
  huart5.Init.Mode = UART_MODE_TX_RX;
  huart5.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart5.Init.OverSampling = UART_OVERSAMPLING_16;
  HAL_UART_Init(&huart5);

}

/* USART1 init function */
void MX_USART1_UART_Init(void)
{

  huart1.Instance = USART1;
  huart1.Init.BaudRate = 9600;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  HAL_UART_Init(&huart1);

}

/* USART2 init function */
void MX_USART2_UART_Init(void)
{

  huart2.Instance = USART2;
  huart2.Init.BaudRate = 115200;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  HAL_UART_Init(&huart2);

}

/** Configure pins as 
        * Analog 
        * Input 
        * Output
        * EVENT_OUT
        * EXTI
*/
void MX_GPIO_Init(void)
{

  GPIO_InitTypeDef GPIO_InitStruct;

  /* GPIO Ports Clock Enable */
  __GPIOD_CLK_ENABLE();
  __GPIOC_CLK_ENABLE();
  __GPIOA_CLK_ENABLE();
  __GPIOB_CLK_ENABLE();

  /*Configure GPIO pins : PC0 PC1 PC2 PC3 */
  GPIO_InitStruct.Pin = GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_3;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Speed = GPIO_SPEED_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : PA15 */
  GPIO_InitStruct.Pin = GPIO_PIN_15;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Speed = GPIO_SPEED_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : PB3 */
  GPIO_InitStruct.Pin = GPIO_PIN_3;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Speed = GPIO_SPEED_HIGH;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : PB4 */
  GPIO_InitStruct.Pin = GPIO_PIN_4;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Speed = GPIO_SPEED_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */
void put_str(char *Str, UART_HandleTypeDef *Huart)
{
  while(*Str!=0)
  {
    HAL_UART_Transmit(Huart,(unsigned char*)Str, 1, 20);
    Str++;
  }
}

/*
*In case of datapack contains ';' at the beginning, replace 
*Source[j] => Source[j+1]
*/
void datapack_maker ( char *Des, char *Source, char Max_index,char End)
{
  
  for (int j = 0; j<Max_index; j++)
  {
    if (Source[j] != ';')
    {
      Des[j] = Source[j];
    }
    else
    {      
      Des[j] = (End == 1) ? '\n' : '\t';    
      Des[j+1] = 0;    
      break;
    }
  }
}

void two_way_esp_polling (void)
{
  while (start_state < 4)
  {
    switch (start_state)
    {
      case 0:
        for(int i = 0; i < 10; i++)     buffer[i] = 0;
        
        while(start_state == 0)
        {         
          HAL_UART_Receive(&huart2, (unsigned char*)buffer, 2,5000);
          if(strcmp(buffer, first_udp) == 0)
          {
            put_str(first_serial, &huart2);
            start_state = 1;
          }
          else
          {
            for(int i = 0; i < 10; i++)     buffer[i] = 0;
          }
        }
          
      break;
  
      case 1:
        
        for(int i = 0; i < 10; i++)     buffer[i] = 0;
        
        while(start_state == 1)
        {      
          HAL_UART_Receive(&huart2, (unsigned char*)buffer, 3,5000);
          if(strcmp(buffer, second_udp) == 0)
          {
            put_str(second_serial, &huart2);
            start_state = 5;
          }
          else
          {
            for(int i = 0; i < 10; i++)     buffer[i] = 0;              
          }          
        }
        
        break;
      }           
  }


}



#include "gps.h"

extern char gps_validation_test;

void  gps_data_check_extrac_func (char *Str, char *Utc_time, char* Validation,
                                  char *Latitude, char *Ns, char *Longitude, 
                                  char *Ew)
{ 
  
  struct INDEX
  {
    unsigned char       Utc;
    unsigned int        data;
    unsigned char       latitude;
    unsigned char       longitude;
  };
  
  enum Gps_ex_state       //main menu
  {
    waiting,
    start_of_datapack,
    gps_validation_state,
    data_extraction,
    data_error,
  };
  enum Gpmode
  {
    S,
    G,
    P,
    R,
    NR,   //NOT R, RMS
    ER,   //ERROR
  };
  enum Comma_count
  {
    zero,
    one_utc,
    two_validation,
    three_latitude,
    four_ns,
    five_longitude,
    six_ew,
    seven,
  };  
  
  volatile unsigned char Progress = ACTIVE;
  volatile static char Check_ch = 0;
  volatile static enum Gps_ex_state    Main_state = waiting;
  volatile static enum Gpmode  Gp_state = S;
  volatile static enum Comma_count      Comma_state;
  static struct INDEX  Index, Index_default = {0, 0, 0, 0};
  
  Main_state = waiting;
  Gp_state = S;
  Comma_state = zero;
  Index = Index_default;
  

  *Validation = 0; 
  /*
  this is for checking the connection of GPS
  the validation will update and check in each data extraction
  */
  
  while (Progress == ACTIVE)
  {
    if ( (*(Str + Index.data)) != 0 )   Check_ch = *(Str + Index.data);
    else
    {
      //put_str("it was zero\n", &huart2);
      Progress = INACTIVE;
      break;
    }
        
    //Data_index++;
    Index.data++;
    
    switch (Main_state)
    {
      case waiting:
        Main_state = (Check_ch == 36) ? start_of_datapack: waiting;   //36 => $     
      break;
      
      case start_of_datapack:
        switch(Gp_state)
        {
          case S:
            //put_str("THIS WAS S\n", &huart2);
            Gp_state = (Check_ch == 'G') ? G: ER;
          break;          
          
          case G:
            //put_str("THIS WAS G\n", &huart2);
            Gp_state = (Check_ch == 'P') ? P: ER;
          break;          
          
          case P:
            //put_str("THIS WAS P\n", &huart2);
            Gp_state = (Check_ch == 'R') ? R: NR;
          break;        
          
          case R:  
            //put_str("THIS WAS R\n", &huart2);
            Main_state = gps_validation_state;
          break;        
         
          case NR:
            //put_str("THIS WAS NR\n", &huart2);
            Main_state = waiting;
            Gp_state = S;
            //Progress = INACTIVE;
          break;        
          
          case ER:
            //put_str("THIS WAS ER\n", &huart2);
            Gp_state = S;
            Main_state = waiting;
            Progress = INACTIVE;
          break;       
        }
        
      break;
      
      case gps_validation_state:
        switch(Comma_state)
        {
        case zero:
          Comma_state = (Check_ch == 44) ? one_utc: zero;          //44 => ,
        break;
        
        case one_utc:
          Comma_state = (Check_ch == 44) ? two_validation: one_utc;
          *(Utc_time + Index.Utc) = (Comma_state == one_utc) ? Check_ch : '\t';
          Index.Utc ++;
          
        break;
        
        case two_validation:
          //sprintf(Utc_time, "%s\n", Utc_time);
          *(Utc_time + Index.Utc) = 0;         
          //put_str(Utc_time, &huart2);
          
          //test
          gps_validation_test = Check_ch;
          switch (Check_ch)
          {
            case 'V':
              Main_state = data_error;
              *Validation = 'V';
            break;
            
            case 'A':
              Main_state = data_extraction;
              *Validation = 'A';
            break; 
          }
          
        break;
        }   
        
      break; 
     
      case data_extraction:
        //put_str("THIS WAS data extracton\n", &huart2);        
        switch(Comma_state)
        {
        case two_validation:
          Comma_state = (Check_ch == 44) ? three_latitude : two_validation; 

        break;
        
        case three_latitude:
          Comma_state = (Check_ch == 44) ? four_ns: three_latitude;          //44 => ,
          *(Latitude + Index.latitude) = (Comma_state == three_latitude) ? Check_ch : '\t';
          Index.latitude ++;
          
        break;
        
        case four_ns:
          *(Latitude + Index.latitude) = 0;
          Comma_state = (Check_ch == 44) ? five_longitude: four_ns;
          switch (Check_ch)
          {
            case 'N':
              *Ns = 'N';            
            break;
            
            case 'S':
              *Ns = 'S';            
            break;            
          }
          
        break;
        
        case five_longitude:
          Comma_state = (Check_ch == 44) ? six_ew: five_longitude;
          *(Longitude + Index.longitude) = (Comma_state == five_longitude) ? Check_ch : '\t';
          Index.longitude ++;
        
        break;
        
        case six_ew:
          
          *(Longitude + Index.longitude) = 0;
          Comma_state = (Check_ch == 44) ? seven: six_ew;
          switch (Check_ch)
          {
            case 'E':
              *Ew = 'E';            
            break;
            
            case 'W':
              *Ew = 'W';            
            break;            
          }
          
        break;
        
        case seven:
            Progress = INACTIVE;
        break;
                
        }
         
      break;
    
      case data_error:
        //put_str("THIS WAS dataError\r\n", &huart2);
        Progress = INACTIVE;
         
      break;    
    }   //switch
  }     //while       
}

void gps_conv_double (char *In, float *Out)
{
  float Float_buffer = 0;
  sscanf(In, "%f", &Float_buffer);
  *Out = Float_buffer;
}

void gps_cor_final_conv_func(float In, float *Out)
{
  int Decimal_part = 0;
  float Min_part = 0.0;
  Decimal_part = (int)(In / 100);
  Min_part = In - (Decimal_part * 100) ;
  Min_part /= 60;
  *Out = Decimal_part + Min_part;
}

void gps_utc_conv_func (char *Utc_time, unsigned char *Hour, unsigned char *Min,
                        unsigned char *Sec)
{
  unsigned int time_buffer = 0;
  unsigned char Hour_buffer = 0, Min_buffer = 0;
  sscanf(Utc_time, "%d", &time_buffer);
  Hour_buffer = (unsigned char)(time_buffer / 10000);
  Min_buffer = (unsigned char) ((time_buffer - (Hour_buffer * 10000)) / 100);
  *Sec = (unsigned char) (time_buffer - ((Hour_buffer * 10000) + (Min_buffer * 100)));

  Min_buffer += 30;
  if (Min_buffer > 59)
  {
    Min_buffer -= 60;
    Hour_buffer += 1;
  }

  Hour_buffer += 3;
  if (Hour_buffer > 23) Hour_buffer -= 24;
  
  *Hour = Hour_buffer;
  *Min = Min_buffer;
}



/* USER CODE END 4 */

#ifdef USE_FULL_ASSERT

/**
   * @brief Reports the name of the source file and the source line number
   * where the assert_param error has occurred.
   * @param file: pointer to the source file name
   * @param line: assert_param error line source number
   * @retval None
   */
void assert_failed(uint8_t* file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
    ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */

}

#endif

/**
  * @}
  */ 

/**
  * @}
*/ 

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
