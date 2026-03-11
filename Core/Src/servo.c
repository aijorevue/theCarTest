#include "servo.h"
#include "tim.h"  // 必须包含 tim.h，因为我们要用到 &htim4

/**
 * @brief 初始化舵机，启动 PWM
 */
void Servo_Init(void) {
    HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_3);
    HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_4);
}

/**
 * @brief 设置舵机角度
 * @param channel TIM_CHANNEL_3 或 TIM_CHANNEL_4
 * @param angle 0.0 到 180.0
 */
void Servo_SetAngle(uint32_t channel, float angle) {
    if (angle < 0) angle = 0;
    if (angle > 180) angle = 180;

    // 线性映射：0-180度 映射到 500-2500 PWM值
    uint16_t pwm_value = (uint16_t)(SERVO_MIN_PWM + (angle / 180.0) * (SERVO_MAX_PWM - SERVO_MIN_PWM));
    
    __HAL_TIM_SET_COMPARE(&htim4, channel, pwm_value);
}