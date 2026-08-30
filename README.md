# ICM42688-Attitude-Solve

ICM42688 姿态解算（STM32-江协代码风格）

基于 STM32F103C8T6 标准库（江协工程风格），复刻江南大学方案的姿态解算：用 SPI1 读取 ICM-42688-P，在独立的 200Hz 定时中断里用 Mahony 四元数 + 加速度融合出三轴欧拉角。

## 解算思路

- **量程与频率**：陀螺 ±2000dps / ODR 200Hz；加速度 ±16g / ODR 200Hz；`PWR_MGMT0=0x0F` 陀螺+加速度都开。
- **零偏标定**：上电静止采样取平均，作为陀螺零偏，读数时减掉，消除主要漂移源。
- **滤波**：对各轴的角速度和加速度分别做 Biquad 二阶低通（角速度 30/20Hz，加速度 10Hz）。
- **姿态融合**：Mahony AHRS（IMU 版，无磁力计），用陀螺（高频）+ 加速度（低频）估计姿态，得出四元数再转欧拉角 `Pitch / Roll / Yaw`。
- **Yaw**：`atan2` 得到 ±180°；并另按 **360°/圈** 累计总角度（顺时针加、逆时针减）。
- **注意**：ICM-42688-P 无磁力计，Yaw 会缓慢漂移；Pitch/Roll 有加速度补偿，短期较稳。

## 目录结构

- `姿态解算/driver/` —— SPI1 底层 + ICM-42688-P 驱动
  - `SPI.c / SPI.h`：SPI1（PA4=CS, PA5=SCK, PA6=MISO, PA7=MOSI，Mode 0，18MHz）
  - `ICM42688.c / ICM42688.h`：配置与读取（陀螺+加计、量程、零偏）
- `姿态解算/solve/` —— 姿态解算算法
  - `imu_fusion.c / imu_fusion.h`：Mahony + Biquad + 欧拉角（`Imu_Fusion_Init` / `Imu_Fusion_Update`）

## 使用流程

```c
ICM_Init();          // 初始化 ICMM（陀螺+加计，200Hz ODR）
ICM_CalibGyroBias(); // 上电静止标定（约0.2s，期间保持静止）
Imu_Fusion_Init();   // 解算初始化
Timer_200Hz_Init();  // 启动200Hz定时中断（TIM3 5ms）
```

姿态解算在 `TIM3_IRQHandler` 里按 200Hz 执行：读陀螺+加计 → `Imu_Fusion_Update()` → 更新 `Imu_Pitch / Imu_Roll / Imu_Yaw / Imu_Yaw_Total`。主循环只负责显示。
