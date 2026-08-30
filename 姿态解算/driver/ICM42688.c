#include "stm32f10x.h"
#include "ICM42688.h"
#include "SPI.h"
#include "Delay.h"

/*参数宏定义（Bank 0）*********************/

#define ICM_DEVICE_CONFIG		0x11		//设备配置，bit0为软复位
#define ICM_WHO_AM_I			0x75		//设备ID
#define ICM_PWR_MGMT0			0x4E		//电源管理，bits3:2陀螺，bits1:0加计
#define ICM_GYRO_CONFIG0		0x4F		//陀螺满量程/ODR
#define ICM_ACCEL_CONFIG0		0x50		//加速度满量程/ODR
#define ICM_ACCEL_DATA_X1		0x1F		//加速度X高字节
#define ICM_GYRO_DATA_X1		0x25		//陀螺X高字节

#define ICM_WHO_AM_I_VALUE		0x47		//ICM-42688-P默认ID

/*量程与灵敏度*/
#define GYRO_FS_DPS				2000.0f		//陀螺满量程：±2000dps
#define ACCEL_FS_G				16.0f		//加速度满量程：±16g
#define INT16_HALF				32768.0f	//16位有符号满量程的一半

/*********************参数宏定义*/

/*零偏（上电静止标定得到）*/
static float gyro_x_bias = 0.0f;
static float gyro_y_bias = 0.0f;
static float gyro_z_bias = 0.0f;

/**
  * 函    数：写寄存器
  * 参    数：reg：寄存器地址；data：写入数据
  * 返 回 值：无
  */
static void ICM_WriteReg(uint8_t reg, uint8_t data)
{
	SPI_CS_Low();
	SPI_ReadWriteByte(reg & 0x7F);
	SPI_ReadWriteByte(data);
	SPI_CS_High();
}

/**
  * 函    数：读寄存器
  * 参    数：reg：寄存器地址
  * 返 回 值：读取到的数据
  */
static uint8_t ICM_ReadReg(uint8_t reg)
{
	uint8_t data;
	SPI_CS_Low();
	SPI_ReadWriteByte(reg | 0x80);
	data = SPI_ReadWriteByte(0x00);
	SPI_CS_High();
	return data;
}

/**
  * 函    数：ICM-42688-P初始化（按江南方案：陀螺±2000dps/ODR 200Hz，加计±16g/ODR 200Hz，均低噪声）
  * 参    数：无
  * 返 回 值：无
  */
void ICM_Init(void)
{
	SPI1_Init();

	/*软复位*/
	ICM_WriteReg(ICM_DEVICE_CONFIG, 0x01);
	Delay_ms(10);

	/*读设备ID确认（0x47正常）*/
	ICM_ReadWHOAMI();

	/*配置陀螺与加速度：FS/ODR见注释，0x07=FS最大，ODR=200Hz*/
	ICM_WriteReg(ICM_GYRO_CONFIG0, 0x07);		//陀螺±2000dps，ODR 200Hz
	ICM_WriteReg(ICM_ACCEL_CONFIG0, 0x07);		//加计±16g，ODR 200Hz

	/*开启陀螺+加速度低噪声模式*/
	ICM_WriteReg(ICM_PWR_MGMT0, 0x0F);
	Delay_ms(60);								//等待陀螺稳定（≥45ms）
}

/**
  * 函    数：读设备ID
  * 参    数：无
  * 返 回 值：WHO_AM_I值（应为0x47）
  */
uint8_t ICM_ReadWHOAMI(void)
{
	return ICM_ReadReg(ICM_WHO_AM_I);
}

/**
  * 函    数：读加速度计原始数据
  * 参    数：ax/ay/az：存放X/Y/Z原始计数的指针
  * 返 回 值：无
  */
void ICM_ReadAccelRaw(int16_t *ax, int16_t *ay, int16_t *az)
{
	uint8_t i;
	uint8_t data[6];

	SPI_CS_Low();
	SPI_ReadWriteByte(ICM_ACCEL_DATA_X1 | 0x80);
	for (i = 0; i < 6; i++)
	{
		data[i] = SPI_ReadWriteByte(0x00);
	}
	SPI_CS_High();

	*ax = (int16_t)((uint16_t)((data[0] << 8) | data[1]));
	*ay = (int16_t)((uint16_t)((data[2] << 8) | data[3]));
	*az = (int16_t)((uint16_t)((data[4] << 8) | data[5]));
}

/**
  * 函    数：读陀螺仪原始数据
  * 参    数：gx/gy/gz：存放X/Y/Z原始计数的指针
  * 返 回 值：无
  */
void ICM_ReadGyroRaw(int16_t *gx, int16_t *gy, int16_t *gz)
{
	uint8_t i;
	uint8_t data[6];

	SPI_CS_Low();
	SPI_ReadWriteByte(ICM_GYRO_DATA_X1 | 0x80);
	for (i = 0; i < 6; i++)
	{
		data[i] = SPI_ReadWriteByte(0x00);
	}
	SPI_CS_High();

	*gx = (int16_t)((uint16_t)((data[0] << 8) | data[1]));
	*gy = (int16_t)((uint16_t)((data[2] << 8) | data[3]));
	*gz = (int16_t)((uint16_t)((data[4] << 8) | data[5]));
}

/**
  * 函    数：读加速度（单位g）
  * 参    数：ax/ay/az：输出物理量指针
  * 返 回 值：无
  */
void ICM_GetAccel(float *ax, float *ay, float *az)
{
	int16_t rx, ry, rz;
	ICM_ReadAccelRaw(&rx, &ry, &rz);
	*ax = (float)rx * (ACCEL_FS_G / INT16_HALF);
	*ay = (float)ry * (ACCEL_FS_G / INT16_HALF);
	*az = (float)rz * (ACCEL_FS_G / INT16_HALF);
}

/**
  * 函    数：读角速度（单位dps，已减零偏）
  * 参    数：gx/gy/gz：输出物理量指针
  * 返 回 值：无
  */
void ICM_GetGyro(float *gx, float *gy, float *gz)
{
	int16_t rx, ry, rz;
	ICM_ReadGyroRaw(&rx, &ry, &rz);
	*gx = ((float)rx - gyro_x_bias) * (GYRO_FS_DPS / INT16_HALF);
	*gy = ((float)ry - gyro_y_bias) * (GYRO_FS_DPS / INT16_HALF);
	*gz = ((float)rz - gyro_z_bias) * (GYRO_FS_DPS / INT16_HALF);
}

/**
  * 函    数：上电静止零偏标定
  * 参    数：无
  * 返 回 值：无
  * 说    明：静止采样100次（约200ms）取平均，作为陀螺零偏，消除漂移
  */
void ICM_CalibGyroBias(void)
{
	int32_t sx = 0, sy = 0, sz = 0;
	int16_t gx, gy, gz;
	uint8_t i;

	for (i = 0; i < 100; i++)
	{
		ICM_ReadGyroRaw(&gx, &gy, &gz);
		sx += gx;
		sy += gy;
		sz += gz;
		Delay_ms(2);
	}
	gyro_x_bias = (float)sx / 100.0f;
	gyro_y_bias = (float)sy / 100.0f;
	gyro_z_bias = (float)sz / 100.0f;
}
