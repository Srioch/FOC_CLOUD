#include "kalman_filter.h"

/*================================================================*/
// 卡尔曼滤波核心公式参考:
// 1. 预测状态:   x_hat- = A * x_hat_prev + B * u
// 2. 预测协方差: P- = A * P_prev * A^T + Q
// 3. 卡尔曼增益: K = P- * H^T * inv(H * P- * H^T + R)
// 4. 更新状态:   x_hat = x_hat- + K * (z - H * x_hat-)
// 5. 更新协方差: P = (I - K * H) * P-
/*================================================================*/

void kalman_filter_Init(kalman_filter_t *kf)
{
    // 使用合理默认值（可运行时覆盖）
    kf->Q_angle = Q_ANGLE_DEFAULT;
    kf->Q_bias  = Q_BIAS_DEFAULT;
    kf->R_measure = R_MEASURE_DEFAULT;
    kf->angle = 0.0f;
    kf->bias  = 0.0f;
    kf->rate  = 0.0f;

    // 初始化协方差矩阵 P+
    kf->P[0][0] = 0.0f;
    kf->P[0][1] = 0.0f;
    kf->P[1][0] = 0.0f;
    kf->P[1][1] = 0.0f;
}

/**
 * @brief 卡尔曼滤波更新函数
 * @param kf 结构体指针
 * @param newAngle 加速度计计算出的角度 (观测值 z)
 * @param newRate  陀螺仪测得的角速度 (控制量 u)
 * @param dt       采样周期 (单位: 秒)
 */
float kalman_filter_Update(kalman_filter_t *kf, float newAngle, float newRate, float dt)
{
    /*========================= 第一步：预测 (Prediction) ========================*/
    // 物理意义：根据上一时刻的状态和陀螺仪读数，推算当前时刻的状态
    // 公式：x(k|k-1) = A * x(k-1|k-1) + B * u
    
    // 真实角速度 = 陀螺仪读数 - 估计的偏差
    kf->rate = newRate - kf->bias; 
    
    // 当前角度 = 上次角度 + 真实角速度 * 时间
    kf->angle += dt * kf->rate;

    // 预测误差协方差矩阵 P 的更新
    // 公式：P(k|k-1) = A * P(k-1|k-1) * A^T + Q
    // 对应矩阵 A = [1, -dt; 0, 1]
    
    // 预测误差协方差更新，注意将过程噪声按采样周期 dt 缩放
    kf->P[0][0] += dt * (dt * kf->P[1][1] - kf->P[0][1] - kf->P[1][0]) + kf->Q_angle * dt;
    kf->P[0][1] -= dt * kf->P[1][1];
    kf->P[1][0] -= dt * kf->P[1][1];
    kf->P[1][1] += kf->Q_bias * dt;//P11 = P11 + Q_bias  这里*dt表示离散时间积分,与采样周期成正比 

    /*========================= 第二步：更新 (Correction) ========================*/
    // 物理意义：利用加速度计的观测值，修正预测值
    
    // 1. 计算测量残差 (Innovation) 的协方差 S
    // S = H * P * H^T + R  (因为 H=[1, 0]，所以 HPH^T 就是 P[0][0])
    float S = kf->P[0][0] + kf->R_measure;

    // 2. 计算卡尔曼增益 K
    // K = P * H^T * S^-1
    float K[2];
    K[0] = kf->P[0][0] / S; // 角度增益
    K[1] = kf->P[1][0] / S; // 偏差增益

    // 3. 计算测量残差 y (观测值 - 预测值)
    // newAngle 是加速度计算出来的角度，kf->angle 是上面预测出来的角度
    float y = newAngle - kf->angle;

    // 4. 更新后验状态估计 (修正预测值)
    // x(k|k) = x(k|k-1) + K * y
    kf->angle += K[0] * y; 
    kf->bias  += K[1] * y;

    // 5. 更新后验误差协方差矩阵 P
    // 公式：P(k|k) = (I - K * H) * P(k|k-1)
    // 注意：这里必须使用临时变量 Ptemp，因为计算 P[1][0] 需要用到旧的 P[0][0]
    float P00_temp = kf->P[0][0];
    float P01_temp = kf->P[0][1];
    // float P10_temp = kf->P[1][0]; // 
    // float P11_temp = kf->P[1][1]; 

    // 矩阵乘法展开结果：
    kf->P[0][0] -= K[0] * P00_temp;
    kf->P[0][1] -= K[0] * P01_temp;
    kf->P[1][0] -= K[1] * P00_temp;
    kf->P[1][1] -= K[1] * P01_temp;

    return kf->angle;
}
