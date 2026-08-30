# ICM42688-Attitude-Solve

ICM42688 姿态解算（STM32-江协代码风格）

基于“四元数，欧拉角解算”的 ICM-42688-P 六轴姿态解算工程，STM32F103C8T6 标准库，**江协科技代码风格**，适合**初学者 / 电赛群体**。克隆后**直接编译烧录**即可跑通，且**可移植**到其他工程。

> 解算思路借鉴自江南大学方案：Mahony 四元数 + 加速度计补偿，融合出三轴欧拉角。

## 特性

- **直接编译烧录**：Keil 打开 `Project.uvprojx` → 编译 → 下载即可；上电约 0.2s 内保持静止（零偏标定）。
- **江协风格**：OLED、Delay、Timer 均为江协科技模板，结构统一、易读，适合入门。
- **硬件接线**：ICM-42688-P 走 SPI1（PA4=CS, PA5=SCK, PA6=MISO, PA7=MOSI，Mode 0，18MHz）；OLED 0.96" I2C（PB8=SCL, PB9=SDA）。
- **可移植**：解算在 `Fusion/imu_fusion.c`，只依赖输入（陀螺 dps、加计 g），可拷到任意 MCU 复用。

## 解算思路

1. **零偏标定**：上电静止采样取平均作为陀螺零偏，读数时减掉，消除主要漂移源。
2. **量程**：陀螺 ±2000dps / ODR 200Hz；加计 ±16g / ODR 200Hz；`PWR_MGMT0=0x0F` 双轴开启。
3. **滤波**：各轴角速度（30/20Hz）与加速度（10Hz）分别做 Biquad 二阶低通。
4. **四元数**：Mahony AHRS（IMU 版，无磁力计）用陀螺（高频）+ 加速度（低频）估计姿态，得到四元数 q0~q3。
5. **三轴欧拉角**：由四元数旋转矩阵反解出 Pitch / Roll / Yaw（`Imu_Pitch`、`Imu_Roll`、`Imu_Yaw`，单位度）。
6. **Yaw**：`atan2` 得 ±180°；另按 360°/圈累计总角度（`Imu_Yaw_Total`，顺时针加、逆时针减）。
7. **注意**：芯片无磁力计，Yaw 会缓慢漂移；Pitch/Roll 有加速度补偿，短期较稳。

## 目录

- `Start/` `Library/` `System/` `User/`：江协工程骨架 + STM32 标准库
- `Hardware/`：`OLED.c`（江协）、`SPI.c/h`、`ICM42688.c/h` 驱动
- `Fusion/`：`imu_fusion.c/h`（Mahony + Biquad + 欧拉角）

## 使用

```c
ICM_Init();          // 初始化 ICM-42688（陀螺+加计，200Hz ODR）
ICM_CalibGyroBias(); // 上电静止标定（约0.2s）
Imu_Fusion_Init();   // 解算初始化
Timer_200Hz_Init();  // 启动200Hz定时中断（TIM3, 5ms）
```

姿态解算在 `TIM3_IRQHandler` 里按 200Hz 执行：读陀螺+加计 → `Imu_Fusion_Update()` → 更新 `Imu_Pitch / Imu_Roll / Imu_Yaw`；主循环只负责显示。
