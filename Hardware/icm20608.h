#ifndef __ICM20608_H
#define __ICM20608_H

#include "main.h"

#define ICM_I2C_ADDR        0x68

/* ICM-20608-G registers */
#define ICM_WHO_AM_I        0x75
#define ICM_PWR_MGMT_1      0x6B
#define ICM_SMPLRT_DIV      0x19
#define ICM_CONFIG          0x1A
#define ICM_GYRO_CONFIG     0x1B
#define ICM_ACCEL_CONFIG    0x1C
#define ICM_ACCEL_CONFIG2   0x1D
#define ICM_ACCEL_XOUT_H    0x3B
#define ICM_TEMP_OUT_H      0x41
#define ICM_GYRO_XOUT_H     0x43

typedef struct {
    int16_t accel_x;
    int16_t accel_y;
    int16_t accel_z;
    int16_t temp;
    int16_t gyro_x;
    int16_t gyro_y;
    int16_t gyro_z;
} icm20608_data_t;

/*
 * Sensitivity (raw → physical):
 *   Gyro  ±2000°/s  →  raw / 16.4  = °/s
 *   Accel ±16g      →  raw / 2048   = g
 *   Temp            →  (raw / 326.8) + 25  = °C
 */

uint8_t ICM_Init(void);
void    ICM_Task(void);

#endif
