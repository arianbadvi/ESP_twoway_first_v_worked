#ifndef _GPS_H
#define _GPS_H

#include "stm32f1xx_hal.h"

#define ACTIVE          1
#define INACTIVE        0

void gps_data_check_extrac_func (char *Str, char *Utc_time, char* Validation,
                                    char *Latitude, char *Ns, char *Longitude, 
                                    char *Ew);
void gps_conv_double (char *In, float *Out);
void gps_cor_final_conv_func(float In, float *Out);
void gps_utc_conv_func (char *Utc_time, unsigned char *Hour, unsigned char *Min,
                        unsigned char *Sec);
#endif