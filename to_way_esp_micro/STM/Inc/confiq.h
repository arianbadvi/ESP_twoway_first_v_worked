#ifndef _CONFIG_H
#define _CONFIG_H

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

#endif