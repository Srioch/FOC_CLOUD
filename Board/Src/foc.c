#include "foc.h"
#include "math.h"

// 统一使用弧度制：机械角度和电角度均为弧度
float GetelectricalAngle(float mechanicalAngle)
{ return mechanicalAngle * FOC_POLE_PAIRS; }

float nomalizeAngle(float angle)
{
    float two_pi = 2.0f * M_PI;
    float a = fmodf(angle, two_pi);
    return a < 0 ? a + two_pi : a;
}

static float normalizeAngleSigned(float angle)
{
    float two_pi = 2.0f * M_PI;
    float a = fmodf(angle, two_pi);

    if (a > M_PI)
    {
        a -= two_pi;
    }
    else if (a < -M_PI)
    {
        a += two_pi;
    }

    return a;
}

void setPWM(float Ua, float Ub, float Uc)
{
    Ua = _COMSTRAIN(Ua, 0.0f, VOLTAGE_LIMIT);
    Ub = _COMSTRAIN(Ub, 0.0f, VOLTAGE_LIMIT);
    Uc = _COMSTRAIN(Uc, 0.0f, VOLTAGE_LIMIT);

    uint32_t top = __HAL_TIM_GET_AUTORELOAD(&htim2); // ARR
    uint16_t pwm1 = _COMSTRAIN((uint16_t)((Ua / VOLTAGE_LIMIT) * (float)top), 0.0f, (uint16_t)top);
    uint16_t pwm2 = _COMSTRAIN((uint16_t)((Ub / VOLTAGE_LIMIT) * (float)top), 0.0f, (uint16_t)top);
    uint16_t pwm3 = _COMSTRAIN((uint16_t)((Uc / VOLTAGE_LIMIT) * (float)top), 0.0f, (uint16_t)top);

    __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, pwm1);
    __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_2, pwm2);
    __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_3, pwm3);
}

//svpwm
void setPhaseVoltage(float Uq, float Ud, float el_angle)// el_angle 单位为弧度
{
    el_angle = nomalizeAngle(el_angle + ANGLE_DEADZONE);

    // 使用 Clarke/逆Park 简化为直接三相电压分配（Ud 被忽略，Uq 为主）
    float Ualpha = -Uq * sinf(el_angle);
    float Ubeta  =  Uq * cosf(el_angle);

    // 三相电压需要围绕母线中点偏置，避免负半轴被裁剪为 0
    float v_center = VOLTAGE_LIMIT * 0.5f;
    float Ua = Ualpha + v_center;
    float Ub = -0.5f * Ualpha + (sqrtf(3.0f) / 2.0f) * Ubeta + v_center;
    float Uc = -0.5f * Ualpha - (sqrtf(3.0f) / 2.0f) * Ubeta + v_center;

    setPWM(Ua, Ub, Uc);
}

float velocityToVoltage(float velocity)
{
    // 简单线性映射：假设 velocity 单位为 RPM，且 100 RPM 对应最大电压
    float voltage = (velocity / 100.0f) * VOLTAGE_LIMIT; // 如需改为不同标定，请调整 100.0f
    return _COMSTRAIN(voltage, 0.0f, VOLTAGE_LIMIT);
}

// 开环速度控制：target_rpm 单位为 RPM，dt 单位为 秒
void openLoopSpeedControl(float target_rpm, float dt)
{
    static float mech_angle = 0.0f; // 机械角度，弧度

    // 将转速 (RPM) 转为 机械角速度 (rad/s)
    float rev_per_sec = target_rpm / 60.0f;
    float mech_omega = rev_per_sec * 2.0f * M_PI; // rad/s

    // 累计机械角度
    mech_angle += mech_omega * dt;
    mech_angle = nomalizeAngle(mech_angle);

    // 计算电角度
    float electrical = GetelectricalAngle(mech_angle);

    // 将速度映射为 q 轴电压并输出三相
    float Uq = velocityToVoltage(target_rpm);
    setPhaseVoltage(Uq, 0.0f, electrical);
}

float angleControl(float target_angle, float current_angle, float Kp)
{
    
    float error = normalizeAngleSigned(target_angle - current_angle);
    float uq = _COMSTRAIN((error * Kp), -VOLTAGE_LIMIT, VOLTAGE_LIMIT);

 
    setPhaseVoltage(uq, 0.0f, GetelectricalAngle(current_angle));

    return uq;

}

float torqueControl(float Uq,float eleangle)
{
    Uq = _COMSTRAIN(Uq,-VOLTAGE_LIMIT, VOLTAGE_LIMIT);
    float Ud = 0.0f;
    setPhaseVoltage(Uq, Ud, eleangle);
    return Uq;
}