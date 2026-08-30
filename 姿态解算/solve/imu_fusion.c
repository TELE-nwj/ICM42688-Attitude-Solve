#include "imu_fusion.h"
#include <math.h>
#include <stdint.h>

/*常量*********************/

#define PI			3.141592654f
#define Ang2Rad		0.01745329252f	//度转弧度
#define Rad2Ang		57.295779513f	//弧度转度

#define sampleFreq	200.0f			//采样频率(Hz)，必须与200Hz中断一致
#define twoKp		(2.0f * 0.5f)	//比例增益(=1)，调整响应速度
#define twoKi		(2.0f * 0.0f)	//积分增益(=0)，禁用积分避免yaw饱和

/*********************常量*/

/*全局解算结果*/
float Imu_Pitch = 0.0f;
float Imu_Roll  = 0.0f;
float Imu_Yaw   = 0.0f;
float Imu_Yaw_Total = 0.0f;

/*四元数状态*/
static volatile float q0 = 1.0f, q1 = 0.0f, q2 = 0.0f, q3 = 0.0f;
static volatile float integralFBx = 0.0f, integralFBy = 0.0f, integralFBz = 0.0f;

/*欧拉角(弧度)与累计用变量*/
static float Pitch_a_Pi = 0.0f, Roll_a_Pi = 0.0f, Yaw_a_Pi = 0.0f;
static float Yaw_AngleLast = 0.0f;
static int Yaw_RoundCount = 0;

/*Biquad二阶滤波状态（按方案保留，用于各自通道，不影响显示的欧拉角）*/
typedef struct
{
	float a0, a1, a2, a3, a4;
	float x1, x2, y1, y2;
}biquad_state_t;

static biquad_state_t Pitch_g_b, Roll_g_b, Yaw_g_b;
static biquad_state_t Pitch_acc_b, Roll_acc_b, Yaw_acc_b;

/*快速平方根倒数*/
static float invSqrt(float x)
{
	float halfx = 0.5f * x;
	float y = x;
	int32_t i = *(int32_t *)&y;
	i = 0x5f3759df - (i >> 1);
	y = *(float *)&i;
	y = y * (1.5f - (halfx * y * y));
	return y;
}

/*Biquad低通系数初始化*/
static void biquad_filter_init(biquad_state_t *s, int fs, float fc, float q)
{
	float w0 = 2.0f * PI * fc / (float)fs;
	float sin_w0 = sinf(w0);
	float cos_w0 = cosf(w0);
	float alpha = sin_w0 / (2.0f * q);
	float b0 = (1.0f - cos_w0) / 2.0f;
	float b1 = b0 * 2.0f;
	float b2 = b0;
	float a0 = 1.0f + alpha;
	float a1 = -2.0f * cos_w0;
	float a2 = 1.0f - alpha;

	s->a0 = b0 / a0;
	s->a1 = b1 / a0;
	s->a2 = b2 / a0;
	s->a3 = a1 / a0;
	s->a4 = a2 / a0;
	s->x1 = s->x2 = 0.0f;
	s->y1 = s->y2 = 0.0f;
}

/*Biquad二阶滤波处理*/
static float biquad(biquad_state_t *s, float data)
{
	float result = s->a0 * data + s->a1 * s->x1 + s->a2 * s->x2
				 - s->a3 * s->y1 - s->a4 * s->y2;
	s->x2 = s->x1;
	s->x1 = data;
	s->y2 = s->y1;
	s->y1 = result;
	return result;
}

/*Mahony四元数姿态解算（IMU版，陀螺+加速度，无磁力计）*/
static void MahonyAHRSupdateIMU(float gx, float gy, float gz, float ax, float ay, float az)
{
	static float recipNorm;
	static float halfvx, halfvy, halfvz;
	static float halfex, halfey, halfez;
	static float qa, qb, qc;

	if (!((ax == 0.0f) && (ay == 0.0f) && (az == 0.0f)))
	{
		gx *= Ang2Rad;		//dps -> rad/s
		gy *= Ang2Rad;
		gz *= Ang2Rad;

		recipNorm = invSqrt(ax * ax + ay * ay + az * az);	//归一化加速度
		ax *= recipNorm;
		ay *= recipNorm;
		az *= recipNorm;

		halfvx = q1 * q3 - q0 * q2;
		halfvy = q0 * q1 + q2 * q3;
		halfvz = q0 * q0 - 0.5f + q3 * q3;

		halfex = (ay * halfvz - az * halfvy);
		halfey = (az * halfvx - ax * halfvz);
		halfez = (ax * halfvy - ay * halfvx);

		if (twoKi > 0.0f)
		{
			integralFBx += twoKi * halfex * (1.0f / sampleFreq);
			integralFBy += twoKi * halfey * (1.0f / sampleFreq);
			integralFBz += twoKi * halfez * (1.0f / sampleFreq);
			gx += integralFBx;
			gy += integralFBy;
			gz += integralFBz;
		}
		else
		{
			integralFBx = integralFBy = integralFBz = 0.0f;
		}

		gx += twoKp * halfex;
		gy += twoKp * halfey;
		gz += twoKp * halfez;
	}

	gx *= (0.5f * (1.0f / sampleFreq));
	gy *= (0.5f * (1.0f / sampleFreq));
	gz *= (0.5f * (1.0f / sampleFreq));

	qa = q0; qb = q1; qc = q2;
	q0 += (-qb * gx - qc * gy - q3 * gz);
	q1 += (qa * gx + qc * gz - q3 * gy);
	q2 += (qa * gy - qb * gz + q3 * gx);
	q3 += (qa * gz + qb * gy - qc * gx);

	recipNorm = invSqrt(q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3);
	q0 *= recipNorm;
	q1 *= recipNorm;
	q2 *= recipNorm;
	q3 *= recipNorm;

	{
		float r11 = 2.0f * (q0 * q1 + q2 * q3);
		float r12 = 1.0f - 2.0f * (q1 * q1 + q2 * q2);
		float r21 = 2.0f * (q0 * q2 - q3 * q1);
		float r31 = 2.0f * (q0 * q3 + q1 * q2);
		float r32 = 1.0f - 2.0f * (q2 * q2 + q3 * q3);

		Yaw_a_Pi = atan2f(r31, r32);
		Pitch_a_Pi = -asinf(r21);
		Roll_a_Pi = atan2f(r11, r12);
	}
}

/**
  * 函    数：解算初始化
  * 参    数：无
  * 返 回 值：无
  * 说    明：复位四元数与角度，按方案初始化各通道Biquad低通系数
  */
void Imu_Fusion_Init(void)
{
	q0 = 1.0f; q1 = q2 = q3 = 0.0f;
	integralFBx = integralFBy = integralFBz = 0.0f;
	Imu_Pitch = Imu_Roll = Imu_Yaw = Imu_Yaw_Total = 0.0f;
	Pitch_a_Pi = Roll_a_Pi = Yaw_a_Pi = 0.0f;
	Yaw_AngleLast = 0.0f;
	Yaw_RoundCount = 0;

	/*按方案：角速度30/20Hz，加速度10Hz*/
	biquad_filter_init(&Pitch_g_b, 200, 30, 0.7071f);
	biquad_filter_init(&Roll_g_b, 200, 20, 0.5773f);
	biquad_filter_init(&Yaw_g_b, 200, 30, 0.5773f);
	biquad_filter_init(&Pitch_acc_b, 200, 10, 0.5773f);
	biquad_filter_init(&Roll_acc_b, 200, 10, 0.5773f);
	biquad_filter_init(&Yaw_acc_b, 200, 10, 0.5773f);
}

/**
  * 函    数：姿态解算更新（在200Hz中断里调用）
  * 参    数：gx/gy/gz：陀螺角速度(dps)；ax/ay/az：加速度(g)
  * 返 回 值：无
  * 说    明：轴向映射与符号按方案；Biquad滤波各通道(仅备用，不影响显示欧拉角)；Mahony出欧拉角；
  *           Yaw按±180°取模并按360°/圈累计总角度（顺时针加、逆时针减）
  */
void Imu_Fusion_Update(float gx, float gy, float gz, float ax, float ay, float az)
{
	/*轴向映射与符号（与方案一致）*/
	float Pitch_g = -gy;
	float Roll_g  =  gx;
	float Yaw_g   =  gz;
	float Pitch_acc = -ay;
	float Roll_acc  =  ax;
	float Yaw_acc   =  az;

	/*Biquad各通道（保留备用）*/
	biquad(&Yaw_acc_b, Yaw_acc);
	biquad(&Roll_acc_b, Roll_acc);
	biquad(&Pitch_acc_b, Pitch_acc);
	biquad(&Yaw_g_b, Yaw_g);
	biquad(&Roll_g_b, Roll_g);
	biquad(&Pitch_g_b, Pitch_g);

	/*Mahony：用原始物理量（偏航无磁力计补偿，会缓慢漂移）*/
	MahonyAHRSupdateIMU(gx, gy, gz, ax, ay, az);

	Imu_Pitch = -Rad2Ang * Pitch_a_Pi;
	Imu_Roll  =  Rad2Ang * Roll_a_Pi;
	Imu_Yaw   =  Rad2Ang * Yaw_a_Pi;

	/*Yaw总角度：按360°/圈累计（跨±180时圈数加/减）*/
	if (Imu_Yaw - Yaw_AngleLast > 180.0f)
		Yaw_RoundCount--;
	else if (Imu_Yaw - Yaw_AngleLast < -180.0f)
		Yaw_RoundCount++;
	Imu_Yaw_Total = 360.0f * (float)Yaw_RoundCount + Imu_Yaw;
	Yaw_AngleLast = Imu_Yaw;
}
