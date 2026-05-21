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

void setPWM(float Ua, float Ub, float Uc, uint8_t motor_id)
{
    Ua = _COMSTRAIN(Ua, 0.0f, VOLTAGE_LIMIT);
    Ub = _COMSTRAIN(Ub, 0.0f, VOLTAGE_LIMIT);
    Uc = _COMSTRAIN(Uc, 0.0f, VOLTAGE_LIMIT);

    uint32_t top = __HAL_TIM_GET_AUTORELOAD(&htim2); // ARR
    uint16_t pwm1 = _COMSTRAIN((uint16_t)((Ua / VOLTAGE_LIMIT) * (float)top), 0.0f, (uint16_t)top);
    uint16_t pwm2 = _COMSTRAIN((uint16_t)((Ub / VOLTAGE_LIMIT) * (float)top), 0.0f, (uint16_t)top);
    uint16_t pwm3 = _COMSTRAIN((uint16_t)((Uc / VOLTAGE_LIMIT) * (float)top), 0.0f, (uint16_t)top);

    if(motor_id == 1U)
    {
        __HAL_TIM_SET_COMPARE(&CLOUD_UP_TIM, CLOUD_UP_CHANNEL_1, pwm1);
        __HAL_TIM_SET_COMPARE(&CLOUD_UP_TIM, CLOUD_UP_CHANNEL_2, pwm2);
        __HAL_TIM_SET_COMPARE(&CLOUD_UP_TIM, CLOUD_UP_CHANNEL_3, pwm3);
    }
    else if(motor_id == 2U)
    {
        __HAL_TIM_SET_COMPARE(&CLOUD_DOWN_TIM, CLOUD_DOWN_CHANNEL_1, pwm1);
        __HAL_TIM_SET_COMPARE(&CLOUD_DOWN_TIM, CLOUD_DOWN_CHANNEL_2, pwm2);
        __HAL_TIM_SET_COMPARE(&CLOUD_DOWN_TIM, CLOUD_DOWN_CHANNEL_3, pwm3);
    }
}

void disablePWM(uint8_t motor_id)
{
    if(motor_id == 1U)
    {
        __HAL_TIM_SET_COMPARE(&CLOUD_UP_TIM, CLOUD_UP_CHANNEL_1, 0U);
        __HAL_TIM_SET_COMPARE(&CLOUD_UP_TIM, CLOUD_UP_CHANNEL_2, 0U);
        __HAL_TIM_SET_COMPARE(&CLOUD_UP_TIM, CLOUD_UP_CHANNEL_3, 0U);
    }
    else if(motor_id == 2U)
    {
        __HAL_TIM_SET_COMPARE(&CLOUD_DOWN_TIM, CLOUD_DOWN_CHANNEL_1, 0U);
        __HAL_TIM_SET_COMPARE(&CLOUD_DOWN_TIM, CLOUD_DOWN_CHANNEL_2, 0U);
        __HAL_TIM_SET_COMPARE(&CLOUD_DOWN_TIM, CLOUD_DOWN_CHANNEL_3, 0U);
    }
}

void disableAllPWM(void)
{
    disablePWM(MOTOR_UP);
    disablePWM(MOTOR_DOWN);
}

//svpwm
void setPhaseVoltage(float Uq, float Ud, float el_angle, uint8_t motor_id)// el_angle 单位为弧度
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

    setPWM(Ua, Ub, Uc, motor_id);
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
    setPhaseVoltage(Uq, 0.0f, electrical, 1U);
}

float angleControl(float target_angle, float current_angle, float Kp)
{
    
    float error = normalizeAngleSigned(target_angle - current_angle);
    float uq = _COMSTRAIN((error * Kp), -VOLTAGE_LIMIT, VOLTAGE_LIMIT);

 
    setPhaseVoltage(uq, 0.0f, GetelectricalAngle(current_angle), 1U);

    return uq;

}

float torqueControl(float Uq,float eleangle,uint8_t motor_id)
{
    Uq = _COMSTRAIN(Uq,-VOLTAGE_LIMIT, VOLTAGE_LIMIT);
    float Ud = 0.0f;
    setPhaseVoltage(Uq, Ud, eleangle, motor_id);
    return Uq;
}

void SpeedLoop_Update(Encoder_t *enc, kalman_filter_pos_speed_t *kf, float dt_s,float speed_ref)
  {
      static uint8_t speed_loop_div = 0U;
      const uint8_t speed_loop_div_n = 5U;

      if (++speed_loop_div >= speed_loop_div_n)
      {
          speed_loop_div = 0U;

          Encoder_Update(enc, dt_s);
          float speed_raw = Encoder_GetMechanicalVelocity(enc);

          float speed_filt = kalman_filter_pos_speed_Update(kf,
              ((float)Encoder_GetTotalCount(enc) * 2.0f * M_PI) / (float)enc->cpr, dt_s);

          speed_filt = 0.7f * kf->speed + 0.3f * speed_raw;

          float uq = PID_Update(&pid_speed, speed_ref, speed_filt);
          torqueControl(-uq, Encoder_GetElectricalAngle(enc), MOTOR_UP);
      }
  }

float Cloud_Control(uint16_t x,uint16_t y,uint8_t find)
{
    float temp = 0.0f;
    if(find)
    {
        Moto_Control(x, &pid_cloud_x, DEFAULT_CONTORL_X, &encoder_up, MOTOR_UP);
        temp = Moto_Control(y, &pid_cloud_y, DEFAULT_CONTORL_Y, &encoder_down, MOTOR_DOWN);
        return temp;
    }
    else return 0;
}

float Moto_Control(uint16_t measure,PID_t *pid,uint16_t target,Encoder_t *enc,uint8_t motor_id)
{
    uint16_t error = target - measure;
    
    uint16_t speed_ref = _COMSTRAIN((uint16_t)(error * DEFAULT_KP), 0U, 100U); // 简单比例控制，输出范围为 0-100
    float uq = _COMSTRAIN(PID_Update(pid, (float)target, (float)measure), -VOLTAGE_LIMIT, VOLTAGE_LIMIT);
    return torqueControl(uq, Encoder_GetElectricalAngle(enc), motor_id); // 输出到电机，假设 motor_id 为 1U

}


void VisionControl_Init(VisionControl_t *vc, PID_t *pid_speed, kalman_filter_pos_speed_t *kf_speed, Encoder_t *enc, uint8_t motor_id)
{
    vc->pid_speed = *pid_speed;
    vc->kf_speed = *kf_speed;
    vc->enc = enc;
    vc->motor_id = motor_id;
    vc->speed_ref = 0.0f;
    vc->uq = 0.0f;
}


void VisionOuterLoop(VisionControl_t *vc,float measure,float center,uint8_t valid,float dt)
{
    float err;
    
    if(!valid)
    {
        vc->speed_ref = 0.0f;
        PID_Reset(&vc->pid_vis);
        return;
    }

    err = center - measure;

    if(fabsf(err) < 4.0f)
        err = 0;

    vc->speed_ref = PID_Update(&vc->pid_vis, 0.0f, -err);
    vc->speed_ref = _COMSTRAIN(vc->speed_ref, -20.0f, 20.0f);
}

float MotorSpeedLoop_Update(VisionControl_t *vc, float dt_s)
  {
      float speed_meas;

      Encoder_Update(vc->enc, dt_s);
      speed_meas = Encoder_GetMechanicalVelocity(vc->enc);

      vc->uq = PID_Update(&vc->pid_speed, vc->speed_ref, speed_meas);
      return torqueControl(-vc->uq, Encoder_GetElectricalAngle(vc->enc), vc->motor_id);
  }
