#include "icm20608.h"
#include "scheduler.h"
#include "uart.h"
#include <math.h>

/* 任务栈大小（单位：字）：本模块自定义，不依赖调度器
   ICM_Task 栈深最大：I2C 读取链 + Madgwick 浮点 + 1Hz printf(%f)。
   512 字 = 2 KB，已由栈高水位实测校准。 */
#define ICM_STACK_SIZE    512

/* bias 用 float 存储，支持 IIR 平滑更新 */
static float gyro_bias_x = 0.0f, gyro_bias_y = 0.0f, gyro_bias_z = 0.0f;

//=============================================================================
// 传感器驱动层 — ICM-20608-G I2C 读写
//=============================================================================

static uint8_t ICM_WriteReg(uint8_t reg, uint8_t data)
{
    return HAL_I2C_Mem_Write(&hi2c1, ICM_I2C_ADDR << 1, reg,
                             I2C_MEMADD_SIZE_8BIT, &data, 1, 100) == HAL_OK;
}

static uint8_t ICM_ReadReg(uint8_t reg, uint8_t *data)
{
    return HAL_I2C_Mem_Read(&hi2c1, ICM_I2C_ADDR << 1, reg,
                            I2C_MEMADD_SIZE_8BIT, data, 1, 100) == HAL_OK;
}

static uint8_t ICM_ReadBurst(uint8_t reg, uint8_t *buf, uint8_t len)
{
    return HAL_I2C_Mem_Read(&hi2c1, ICM_I2C_ADDR << 1, reg,
                            I2C_MEMADD_SIZE_8BIT, buf, len, 100) == HAL_OK;
}

/* 上电静止校准陀螺零偏：连续采样 300 次取平均，结果存为 float */
static void ICM_CalibrateGyro(void)
{
    int32_t sum_x = 0, sum_y = 0, sum_z = 0;

    printf("Gyro cal...\r\n");
    for (int i = 0; i < 300; i++)
    {
        uint8_t buf[6];
        if (ICM_ReadBurst(ICM_GYRO_XOUT_H, buf, 6))
        {
            sum_x += (int16_t)(buf[0] << 8 | buf[1]);
            sum_y += (int16_t)(buf[2] << 8 | buf[3]);
            sum_z += (int16_t)(buf[4] << 8 | buf[5]);
        }
        HAL_Delay(10);
    }

    gyro_bias_x = (float)sum_x / 300.0f;
    gyro_bias_y = (float)sum_y / 300.0f;
    gyro_bias_z = (float)sum_z / 300.0f;
    printf("Gyro bias: %.2f %.2f %.2f\r\n",
           gyro_bias_x, gyro_bias_y, gyro_bias_z);
}

uint8_t ICM_Init(void)
{
    uint8_t id;

    HAL_Delay(100);
    ICM_WriteReg(ICM_PWR_MGMT_1, 0x80);    /* reset device                */
    HAL_Delay(50);
    ICM_WriteReg(ICM_PWR_MGMT_1, 0x01);    /* select PLL with gyro ref    */
    HAL_Delay(50);

    ICM_ReadReg(ICM_WHO_AM_I, &id);
    if (id != 0xAF)
    {
        printf("ICM-20608 not found (ID=0x%02X)\r\n", id);
        return 0;
    }
    printf("ICM-20608 ready (ID=0x%02X)\r\n", id);

    ICM_WriteReg(ICM_SMPLRT_DIV,    0x00); /* 1kHz output rate            */
    ICM_WriteReg(ICM_CONFIG,        0x04); /* gyro DLPF ~20Hz             */
    ICM_WriteReg(ICM_GYRO_CONFIG,   0x18); /* gyro ±2000°/s               */
    ICM_WriteReg(ICM_ACCEL_CONFIG,  0x18); /* accel ±16g                  */
    ICM_WriteReg(ICM_ACCEL_CONFIG2, 0x04); /* accel DLPF ~21Hz            */
    ICM_WriteReg(ICM_PWR_MGMT_1,   0x00); /* wake up, all axes enabled   */

    HAL_Delay(100);
    ICM_CalibrateGyro();

    /* 自注册任务：须在 scheduler_init() 之后、scheduler_start() 之前调用。
       放在校准完成之后 —— 校准是阻塞式的（MSP 上跑），任务创建后
       不会在调度器启动前被调度。 */
    scheduler_task_create(ICM_Task, "ICM",
                          TASK_PRIO_NORMAL, ICM_STACK_SIZE);
    return 1;
}

//=============================================================================
// 姿态解算层 — Madgwick AHRS (IMU 6-axis)
//=============================================================================

#define GYRO_RAW2RAD      0.00106465f    /* ±2000°/s: raw → rad/s             */
#define ACCEL_RAW2G       (1.0f/2048.0f) /* ±16g:     raw → g                 */
#define SAMPLE_PERIOD     0.01f          /* ICM_Task 周期 10ms                 */

#define BETA_NORMAL       0.05f          /* 正常增益                           */
#define BETA_DYNAMIC      0.01f          /* 有线加速度干扰时降低加速度计权重   */

/*
 * 静止检测 & 在线 bias 估计参数
 *   GYRO_STATIC_THR : bias 校正后陀螺幅值阈值（LSB），约 1.5°/s
 *   ACCEL_STATIC_THR: 合加速度偏离 1g 的允许量（g）
 *   STATIC_CONFIRM  : 连续静止帧数（×10ms = 200ms），防止短暂停顿误触发
 *   BIAS_ALPHA      : IIR 系数，时间常数 ≈ 1/0.001 × 10ms = 10s
 *   GYRO_DEADZONE   : 死区（LSB），校正后绝对值低于此值的读数清零
 */
#define GYRO_STATIC_THR   25.0f
#define ACCEL_STATIC_THR  0.15f
#define STATIC_CONFIRM    20
#define BIAS_ALPHA        0.001f
#define GYRO_DEADZONE     2.0f

static float mw_q0 = 1.0f, mw_q1 = 0.0f, mw_q2 = 0.0f, mw_q3 = 0.0f;
static uint8_t static_count = 0;

static float invSqrt(float x)
{
    float halfx = 0.5f * x;
    long i = *(long *)&x;
    i = 0x5F3759DF - (i >> 1);
    x = *(float *)&i;
    x = x * (1.5f - halfx * x * x);
    return x;
}

static void Madgwick_UpdateIMU(float gx, float gy, float gz,
                               float ax, float ay, float az,
                               float beta)
{
    float recipNorm;
    float s0, s1, s2, s3;
    float qDot1, qDot2, qDot3, qDot4;
    float _2q0, _2q1, _2q2, _2q3, _4q0, _4q1, _4q2, _8q1, _8q2;
    float q0q0, q1q1, q2q2, q3q3;

    qDot1 = 0.5f * (-mw_q1 * gx - mw_q2 * gy - mw_q3 * gz);
    qDot2 = 0.5f * ( mw_q0 * gx + mw_q2 * gz - mw_q3 * gy);
    qDot3 = 0.5f * ( mw_q0 * gy - mw_q1 * gz + mw_q3 * gx);
    qDot4 = 0.5f * ( mw_q0 * gz + mw_q1 * gy - mw_q2 * gx);

    if (!(ax == 0.0f && ay == 0.0f && az == 0.0f))
    {
        recipNorm = invSqrt(ax*ax + ay*ay + az*az);
        ax *= recipNorm; ay *= recipNorm; az *= recipNorm;

        _2q0 = 2.0f*mw_q0; _2q1 = 2.0f*mw_q1;
        _2q2 = 2.0f*mw_q2; _2q3 = 2.0f*mw_q3;
        _4q0 = 4.0f*mw_q0; _4q1 = 4.0f*mw_q1; _4q2 = 4.0f*mw_q2;
        _8q1 = 8.0f*mw_q1; _8q2 = 8.0f*mw_q2;
        q0q0 = mw_q0*mw_q0; q1q1 = mw_q1*mw_q1;
        q2q2 = mw_q2*mw_q2; q3q3 = mw_q3*mw_q3;

        s0 = _4q0*q2q2 + _2q2*ax + _4q0*q1q1 - _2q1*ay;
        s1 = _4q1*q3q3 - _2q3*ax + 4.0f*q0q0*mw_q1 - _2q0*ay
           - _4q1 + _8q1*q1q1 + _8q1*q2q2 + _4q1*az;
        s2 = 4.0f*q0q0*mw_q2 + _2q0*ax + _4q2*q3q3 - _2q3*ay
           - _4q2 + _8q2*q1q1 + _8q2*q2q2 + _4q2*az;
        s3 = 4.0f*q1q1*mw_q3 - _2q1*ax + 4.0f*q2q2*mw_q3 - _2q2*ay;

        recipNorm = invSqrt(s0*s0 + s1*s1 + s2*s2 + s3*s3);
        s0 *= recipNorm; s1 *= recipNorm; s2 *= recipNorm; s3 *= recipNorm;

        qDot1 -= beta * s0;
        qDot2 -= beta * s1;
        qDot3 -= beta * s2;
        qDot4 -= beta * s3;
    }

    mw_q0 += qDot1 * SAMPLE_PERIOD;
    mw_q1 += qDot2 * SAMPLE_PERIOD;
    mw_q2 += qDot3 * SAMPLE_PERIOD;
    mw_q3 += qDot4 * SAMPLE_PERIOD;

    recipNorm = invSqrt(mw_q0*mw_q0 + mw_q1*mw_q1
                      + mw_q2*mw_q2 + mw_q3*mw_q3);
    mw_q0 *= recipNorm; mw_q1 *= recipNorm;
    mw_q2 *= recipNorm; mw_q3 *= recipNorm;
}

//=============================================================================
// 应用层 — 数据采集 + 在线 bias 估计 + 姿态解算 + 周期输出
//=============================================================================

static icm20608_data_t icm_data;

void ICM_Task(void)
{
    uint32_t print_tick = 0;

    while (1)
    {
        uint8_t buf[14];

        if (!ICM_ReadBurst(ICM_ACCEL_XOUT_H, buf, 14))
        {
            scheduler_delay(10);
            continue;
        }

        /*--- 1. 读取原始数据 ---*/
        int16_t ax_raw  = (int16_t)(buf[0]  << 8 | buf[1]);
        int16_t ay_raw  = (int16_t)(buf[2]  << 8 | buf[3]);
        int16_t az_raw  = (int16_t)(buf[4]  << 8 | buf[5]);
        int16_t tmp_raw = (int16_t)(buf[6]  << 8 | buf[7]);
        int16_t gx_raw  = (int16_t)(buf[8]  << 8 | buf[9]);
        int16_t gy_raw  = (int16_t)(buf[10] << 8 | buf[11]);
        int16_t gz_raw  = (int16_t)(buf[12] << 8 | buf[13]);

        /*--- 2. 陀螺 bias 校正 ---*/
        float gx_cor = (float)gx_raw - gyro_bias_x;
        float gy_cor = (float)gy_raw - gyro_bias_y;
        float gz_cor = (float)gz_raw - gyro_bias_z;

        /*--- 3. 陀螺死区：残余噪声清零，阻断随机漂移积分 ---*/
        if (fabsf(gx_cor) < GYRO_DEADZONE) gx_cor = 0.0f;
        if (fabsf(gy_cor) < GYRO_DEADZONE) gy_cor = 0.0f;
        if (fabsf(gz_cor) < GYRO_DEADZONE) gz_cor = 0.0f;

        /*--- 4. 静止检测 ---*/
        float gyro_sq = gx_cor*gx_cor + gy_cor*gy_cor + gz_cor*gz_cor;
        float accel_g = sqrtf((float)ax_raw*ax_raw +
                              (float)ay_raw*ay_raw +
                              (float)az_raw*az_raw) * ACCEL_RAW2G;

        uint8_t is_static = (gyro_sq < GYRO_STATIC_THR * GYRO_STATIC_THR) &&
                            (fabsf(accel_g - 1.0f) < ACCEL_STATIC_THR);

        if (is_static) { if (static_count < 255) static_count++; }
        else           { static_count = 0; }

        /*--- 5. 在线 bias IIR 估计（仅静止稳定后更新）---*/
        if (static_count >= STATIC_CONFIRM)
        {
            gyro_bias_x += BIAS_ALPHA * ((float)gx_raw - gyro_bias_x);
            gyro_bias_y += BIAS_ALPHA * ((float)gy_raw - gyro_bias_y);
            gyro_bias_z += BIAS_ALPHA * ((float)gz_raw - gyro_bias_z);
        }

        /*--- 6. 自适应 beta ---*/
        float beta = (fabsf(accel_g - 1.0f) < 0.2f) ? BETA_NORMAL : BETA_DYNAMIC;

        /*--- 7. Madgwick 姿态解算 ---*/
        Madgwick_UpdateIMU(gx_cor * GYRO_RAW2RAD,
                           gy_cor * GYRO_RAW2RAD,
                           gz_cor * GYRO_RAW2RAD,
                           (float)ax_raw,
                           (float)ay_raw,
                           (float)az_raw,
                           beta);

        /*--- 8. 存储供外部读取 ---*/
        icm_data.accel_x = ax_raw;
        icm_data.accel_y = ay_raw;
        icm_data.accel_z = az_raw;
        icm_data.temp    = tmp_raw;
        icm_data.gyro_x  = (int16_t)gx_cor;
        icm_data.gyro_y  = (int16_t)gy_cor;
        icm_data.gyro_z  = (int16_t)gz_cor;

        /*--- 9. 1Hz 打印姿态 ---*/
        if (HAL_GetTick() - print_tick >= 1000)
        {
            print_tick = HAL_GetTick();

            float roll  = atan2f(2.0f*(mw_q0*mw_q1 + mw_q2*mw_q3),
                                 1.0f - 2.0f*(mw_q1*mw_q1 + mw_q2*mw_q2)) * 57.29578f;
            float pitch = asinf (2.0f*(mw_q0*mw_q2 - mw_q3*mw_q1))        * 57.29578f;
            float yaw   = atan2f(2.0f*(mw_q1*mw_q2 + mw_q0*mw_q3),
                                 1.0f - 2.0f*(mw_q2*mw_q2 + mw_q3*mw_q3)) * 57.29578f;
            /* 任务里一律走异步通道：投递给 Uart_Send_Task 统一发送，
               避免本任务在 printf 上阻塞约 3.5 ms（占 10 ms 周期的 1/3）。
               ICM_Init() 中的校准打印仍在裸机阶段，保留 printf。 */
            Uart_Printf("R:%6.1f P:%6.1f Y:%6.1f  [%s]\r\n",
                        roll, pitch, yaw,
                        (static_count >= STATIC_CONFIRM) ? "ST" : "DY");
        }

        scheduler_delay(10);
    }
}
