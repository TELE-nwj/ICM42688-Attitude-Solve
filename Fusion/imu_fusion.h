#ifndef __IMU_FUSION_H
#define __IMU_FUSION_H

/*功能声明*********************/

/*解算初始化：复位四元数、初始化滤波系数*/
void Imu_Fusion_Init(void);

/*姿态解算更新：输入陀螺dps、加速度g，运行一次Mahony+欧拉角*/
void Imu_Fusion_Update(float gx, float gy, float gz, float ax, float ay, float az);

/*********************功能声明*/

/*解算结果（度）*********************/
extern float Imu_Pitch;		//俯仰角
extern float Imu_Roll;		//横滚角
extern float Imu_Yaw;		//偏航角（±180°）
extern float Imu_Yaw_Total;	//累计偏航角（360°/圈）

/*********************解算结果*/

#endif
