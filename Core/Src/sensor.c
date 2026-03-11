#include "sensor.h"


//左避障
uint8_t Sensor_Read_Avoid_L(void)
{
    return HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_13);
}

//右避障
uint8_t Sensor_Read_Avoid_R(void)
{
    return HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_14);
}
//读取灰度
uint8_t Sensor_Read_Tracking(void)
{
    uint8_t status = 0;
    
    // 依次读取5个引脚，拼成一个 5位的二进制数
    // PA8 是最左边，PA12 是最右边 
    if (HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_8) == GPIO_PIN_SET)  status |= (1 << 3); // L2
    if (HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_9) == GPIO_PIN_SET)  status |= (1 << 2); // L1
    //if (HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_10) == GPIO_PIN_SET) status |= (1 << 2); // Center
    if (HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_11) == GPIO_PIN_SET) status |= (1 << 1); // R1
    if (HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_12) == GPIO_PIN_SET) status |= (1 << 0); // R2
    
    return status;
}