#ifndef __ICM42688_H
#define __ICM42688_H

#include <stdint.h>

/*功能声明*********************/

/*初始化函数*/
void ICM_Init(void);

/*读ID函数*/
uint8_t ICM_ReadWHOAMI(void);

/*读原始加速度计（16位有符号）*/
void ICM_ReadAccelRaw(int16_t *ax, int16_t *ay, int16_t *az);

/*读原始陀螺仪（16位有符号）*/
void ICM_ReadGyroRaw(int16_t *gx, int16_t *gy, int16_t *gz);

/*读物理量：加速度，单位g*/
void ICM_GetAccel(float *ax, float *ay, float *az);

/*读物理量：角速度，单位dps（已减零偏）*/
void ICM_GetGyro(float *gx, float *gy, float *gz);

/*上电静止零偏标定（需保持静止约200ms）*/
void ICM_CalibGyroBias(void);

/*********************功能声明*/

#endif
