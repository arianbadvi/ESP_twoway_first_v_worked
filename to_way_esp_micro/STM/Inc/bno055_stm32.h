/*
 * bno055_stm32.h
 *
 *  Created on: Jun 20, 2021
 *      Author: AiTech.co
 */

#ifndef INC_BNO055_STM32_H_
#define INC_BNO055_STM32_H_



#endif /* INC_BNO055_STM32_H_ */


#ifndef BNO055_STM32_H_
#define BNO055_STM32_H_

#ifdef __cplusplus
  extern "C" {
#endif

//#include "i2c.h"

#ifdef FREERTOS_ENABLED
#include "FreeRTOS.h"
#include "task.h"
#include "cmsis_os.h"
#endif
#include "bno055.h"

// added by arian badvi
#include "stm32f1xx_hal.h"

extern UART_HandleTypeDef huart2;

extern unsigned char bno_state;
// added by arian badvi

extern  I2C_HandleTypeDef hi2c1;

#define  _bno055_i2c_port &hi2c1


//I2C_HandleTypeDef *_bno055_i2c_port;
//
//void bno055_assignI2C(I2C_HandleTypeDef *hi2c_device) {
//  _bno055_i2c_port = hi2c_device;
//}

void put_str2(char *str)
{
  while(*str!=0)
  {
    HAL_UART_Transmit(&huart2,(unsigned char*)str, 1, 20);
    str++;
  }
}


void bno055_delay(int time) {
//#ifdef FREERTOS_ENABLED
//  osDelay(time);
//#else
  HAL_Delay(time);
//#endif
}

void bno055_writeData(uint8_t reg, uint8_t data) {
  uint8_t txdata[2] = {reg, data};
  uint8_t status;
  //status = HAL_I2C_Master_Transmit(_bno055_i2c_port, BNO055_I2C_ADDR << 1, txdata, sizeof(txdata), 10);
  status = HAL_I2C_Mem_Write(_bno055_i2c_port, BNO055_I2C_ADDR << 1, reg, 1, &data, 1, 100);
  
  if (status == HAL_OK) {
    return;
  }

  if (status == HAL_ERROR) {
    //printf("HAL_I2C_Master_Transmit HAL_ERROR\r\n");
    //put_str2("HAL_I2C_Master_Transmit HAL_ERROR\r\n");
    put_str2("BNO055    PROBLEM\r\n");
    bno_state = 2;
    
  } else if (status == HAL_TIMEOUT) {
    //printf("HAL_I2C_Master_Transmit HAL_TIMEOUT\r\n");
    put_str2("HAL_I2C_Master_Transmit HAL_TIMEOUT\r\n");
    
  } else if (status == HAL_BUSY) {
    //printf("HAL_I2C_Master_Transmit HAL_BUSY\r\n");
    put_str2("HAL_I2C_Master_Transmit HAL_BUSY me \r\n");
    
  } else {
    //printf("Unknown status data %d", status);
     put_str2("Unknown status data");
    
  }
/*
  uint32_t error = HAL_I2C_GetError(_bno055_i2c_port);
  if (error == HAL_I2C_ERROR_NONE) {
    return;
  } else if (error == HAL_I2C_ERROR_BERR) {
    //printf("HAL_I2C_ERROR_BERR\r\n");
    put_str2("HAL_I2C_ERROR_BERR\r\n");
    
  } else if (error == HAL_I2C_ERROR_ARLO) {
    //printf("HAL_I2C_ERROR_ARLO\r\n");
    put_str2("HAL_I2C_ERROR_ARLO\r\n");
    
  } else if (error == HAL_I2C_ERROR_AF) {
    //printf("HAL_I2C_ERROR_AF\r\n");
    put_str2("HAL_I2C_ERROR_AF\r\n");
    
  } else if (error == HAL_I2C_ERROR_OVR) {
    //printf("HAL_I2C_ERROR_OVR\r\n");
    put_str2("HAL_I2C_ERROR_OVR\r\n");

  } else if (error == HAL_I2C_ERROR_DMA) {
    //printf("HAL_I2C_ERROR_DMA\r\n");
    put_str2("HAL_I2C_ERROR_DMA\r\n");

  } else if (error == HAL_I2C_ERROR_TIMEOUT) {
    //printf("HAL_I2C_ERROR_TIMEOUT\r\n");
    put_str2("HAL_I2C_ERROR_TIMEOUT\r\n");

  }

  HAL_I2C_StateTypeDef state = HAL_I2C_GetState(_bno055_i2c_port);
  if (state == HAL_I2C_STATE_RESET) {
    //printf("HAL_I2C_STATE_RESET\r\n");
    put_str2("HAL_I2C_STATE_RESET\r\n");
    
  } else if (state == HAL_I2C_STATE_READY) {
    //printf("HAL_I2C_STATE_RESET\r\n");
    put_str2("HAL_I2C_STATE_RESET\r\n");
    
  } else if (state == HAL_I2C_STATE_BUSY) {
    //printf("HAL_I2C_STATE_BUSY\r\n");
    put_str2("HAL_I2C_STATE_BUSY\r\n");
    
  } else if (state == HAL_I2C_STATE_BUSY_TX) {
    //printf("HAL_I2C_STATE_BUSY_TX\r\n");
    put_str2("HAL_I2C_STATE_BUSY_TX\r\n");
    
  } else if (state == HAL_I2C_STATE_BUSY_RX) {
    //printf("HAL_I2C_STATE_BUSY_RX\r\n");
    put_str2("HAL_I2C_STATE_BUSY_RX\r\n");
    
    //
  } else if (state == HAL_I2C_STATE_LISTEN) {
    printf("HAL_I2C_STATE_LISTEN\r\n");
  } else if (state == HAL_I2C_STATE_BUSY_TX_LISTEN) {
    printf("HAL_I2C_STATE_BUSY_TX_LISTEN\r\n");
  } else if (state == HAL_I2C_STATE_BUSY_RX_LISTEN) {
    printf("HAL_I2C_STATE_BUSY_RX_LISTEN\r\n");
  } else if (state == HAL_I2C_STATE_ABORT) {
    printf("HAL_I2C_STATE_ABORT\r\n");
    //
  
  } else if (state == HAL_I2C_STATE_TIMEOUT) {
    //printf("HAL_I2C_STATE_TIMEOUT\r\n");
    put_str2("HAL_I2C_STATE_TIMEOUT\r\n");
    
  } else if (state == HAL_I2C_STATE_ERROR) {
    //printf("HAL_I2C_STATE_ERROR\r\n");
    put_str2("HAL_I2C_STATE_ERROR\r\n");
    
  }
*/
  // while (HAL_I2C_GetState(_bno055_i2c_port) != HAL_I2C_STATE_READY) {}
  // return;
}

void bno055_readData(uint8_t reg, uint8_t *data, uint8_t len) {
  
  
  HAL_I2C_Mem_Read(_bno055_i2c_port, BNO055_I2C_ADDR << 1, reg, 1, data, len, 100);
  
  /*
  HAL_I2C_Master_Transmit(_bno055_i2c_port, BNO055_I2C_ADDR << 1, &reg, 1 ,100);
  put_str2("6\n");
  HAL_I2C_Master_Receive(_bno055_i2c_port, BNO055_I2C_ADDR << 1, data, len ,100);
  put_str2("7\n");
  */
  
  // HAL_I2C_Mem_Read(_bno055_i2c_port, BNO055_I2C_ADDR_LO<<1, reg, I2C_MEMADD_SIZE_8BIT, data, len, 100);
}

#ifdef __cplusplus
  }
#endif

#endif  // BNO055_STM32_H_
