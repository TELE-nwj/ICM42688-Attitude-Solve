#include "stm32f10x.h"                  // Device header
#include "Delay.h"
#include "OLED.h"
#include "ICM42688.h"
#include "imu_fusion.h"
#include "Timer.h"

int main(void)
{
	OLED_Init();			//OLED初始化
	ICM_Init();				//ICM-42688初始化（陀螺+加计，200Hz ODR）
	ICM_CalibGyroBias();	//上电静止零偏标定（期间保持静止）
	Imu_Fusion_Init();		//Mahony解算初始化
	Timer_200Hz_Init();		//启动200Hz定时中断，里面跑姿态解算

	OLED_Clear();
	OLED_ShowString(0, 0, "P:", OLED_8X16);
	OLED_ShowString(0, 16, "R:", OLED_8X16);
	OLED_ShowString(0, 32, "Y:", OLED_8X16);
	OLED_Update();

	while (1)
	{
		/*屏蔽中断，避免200Hz解算打断软件I2C导致OLED花屏*/
		__disable_irq();
		OLED_ShowFloatNum(36, 0, Imu_Pitch, 3, 1, OLED_8X16);
		OLED_ShowFloatNum(36, 16, Imu_Roll, 3, 1, OLED_8X16);
		OLED_ShowFloatNum(36, 32, Imu_Yaw, 3, 1, OLED_8X16);
		OLED_Update();
		__enable_irq();

		Delay_ms(100);		//显示约30Hz
	}
}
