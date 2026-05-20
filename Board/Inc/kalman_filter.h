#ifndef __KALMAN_FILTER_H__
#define __KALMAN_FILTER_H__
/*===================inlucde==================*/
#include <stdint.h>
#include <stddef.h>
#include <stdbool.h>
/*===================define===================*/
#define KALMAN_FILTER_INIT_DEFAULT \
{\
    .Q_angle = 0.001f,\
    .Q_bias = 0.001f,\
    .R_measure = 0.0f,\
    .angle = 0.0f,\
    .bias = 0.0f,\
    .rate = 0.0f,\
    .P = {{0.0f, 0.0f}, {0.0f, 0.0f}}\
}
// 推荐默认值：可根据采样率和噪声调整
#define Q_ANGLE_DEFAULT 0.003f
#define Q_BIAS_DEFAULT 0.003f
#define R_MEASURE_DEFAULT 0.05f

/*===================typedef==================*/
typedef struct kalman_filter
{
    /* data */
    float Q_angle;//过程噪声协方差
    float Q_bias;
    float R_measure;//测量噪声协方差
    float angle;//角度
    float bias;//偏差
    float rate;//角速度
    float P[2][2];//误差协方差矩阵

} kalman_filter_t;



void kalman_filter_Init(kalman_filter_t *kf);

float kalman_filter_Update(kalman_filter_t *kf, float newAngle, float newRate, float dt);

/*========================== 位置→速度 Kalman ==========================*/
// 状态: [position, speed]^T
// 观测: position only (H = [1, 0])
// 速度通过常速度模型递推 + 位置观测修正，天然滤波
typedef struct kalman_filter_pos_speed
{
    float Q_position;  // 位置过程噪声
    float Q_speed;     // 速度过程噪声 (越大则KF对加速度响应越快，但噪声也越大)
    float R_measure;   // 位置测量噪声 (越大则滤波越平滑，但滞后越大)
    float position;    // 估计位置 (rad)
    float speed;       // 估计速度 (rad/s)
    float P[2][2];     // 误差协方差矩阵
} kalman_filter_pos_speed_t;

#define KALMAN_FILTER_POS_SPEED_INIT_DEFAULT \
{\
    .Q_position = 0.001f,\
    .Q_speed = 0.1f,\
    .R_measure = 0.01f,\
    .position = 0.0f,\
    .speed = 0.0f,\
    .P = {{0.01f, 0.0f}, {0.0f, 0.1f}}\
}

void kalman_filter_pos_speed_Init(kalman_filter_pos_speed_t *kf);
float kalman_filter_pos_speed_Update(kalman_filter_pos_speed_t *kf, float measured_position, float dt);


















#endif /* KALMAN_FILTER_H */





/*===================end=====================*/

/*===========================================*/
//Xt = Xt-1 + K(Zt - HXt-1)   最优估计 = 上次估计 + 卡尔曼增益 * (本次测量 - 上次估计的测量值)
//K = Pt-HT(HPt-HT + R)^-1   卡尔曼增益 = 误差协方差 * H转置 * (H * 误差协方差 * H转置 + 测量噪声协方差)的逆
//Pt = (I - KH)Pt-1 + Q   误差协方差更新 = (单位矩阵 - 卡尔曼增益 * H) * 上次误差协方差 + 过程噪声协方差
//Pt = AP-1A转置 + Q  误差协方差预测 = 状态转移矩阵 * 上次误差协方差 * 状态转移矩阵转置 + 过程噪声协方差
//Zt = 观测矩阵Xt + vt  观测值 = 观测矩阵 * 状态值 + 观测噪声
//H = 观测矩阵
//Q = 过程噪声协方差
//R = 测量噪声协方差
//I = 单位矩阵
/*===========================================*/
