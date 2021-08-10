/*******************************************************************************
 *  电机驱动源文件
 *  直流有刷电机
 *  驱动器：DRV8801
 *******************************************************************************/ 
#ifndef DRV_MOTOR_H
#define DRV_MOTOR_H

#include "drv_motor.h"
#include "stm32f4xx_hal.h"
#include "data_typedef.h"
#include "tim.h"
#include "adc.h"

#define PWM_PULSE_MID         2500      /* PWM占空比范围[0..4999] 中值2500 */
#define MOTOR_ENCODER_ROUND		2048.0f   /* 电机一圈编码器计数值 */
#define MOTOR_CIRCLEANGLE		  360.0f    /* 电机一圈角度值 */

/************************************************************************/
#ifdef __cplusplus
extern "C"
{
#endif
  
typedef enum 
{
	MOTOR1 = 0,
	MOTOR2,
  MOTOR3
}motorID_t;
  
/* 电机参数信息结构体 */
typedef struct
{
	int16_t	speed_rpm;       /* 转子转速RPM */
  int16_t real_current;    /* 实际转矩电流A */
  
  uint8_t  dir;             /* 编码器旋转方向 1:正转 0:反转 */
  int16_t  circle_count;    /* 编码器圈数计数 */
  uint16_t encode;          /* 编码器单圈计数 [0...65534] */
 	int64_t  total_encode;    /* 电机编码值总计数 */
  
  float    motorAngle;      /* 电机多圈角度值 */
  float    offset_angle;    /* 初始时记忆的电机角度值 0.01度/LSB */
}motor_t;

void drv_Motor_PWM_Encoder_Start_Init(void);
void drv_motor_all_enable(void);
void drv_motor_all_disable(void);

void drv_motor_set_pwm(motorID_t MOTOR, uint16_t pwm_val);
uint8_t drv_motor_get_iq_value(motorID_t MOTOR, uint16_t* motor_iq);

motor_t* get_motor_axis_data_pointer(void);


#ifdef __cplusplus
}
#endif

#endif

