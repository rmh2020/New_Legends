/**
  ****************************(C) COPYRIGHT 2019 DJI****************************
  * @file       servo_task.c/h
  * @brief      
  * @note       
  * @history
  *  Version    Date            Author          Modification
  *  V1.0.0     Oct-21-2019     RM              1. done
  *
  @verbatim
  ==============================================================================

  ==============================================================================
  @endverbatim
  ****************************(C) COPYRIGHT 2019 DJI****************************
  */

#include "servo_task.h"
#include "main.h"
#include "cmsis_os.h"
#include "bsp_servo_pwm.h"
#include "remote_control.h"
#include "shoot.h"



#define SERVO_MIN_PWM   500
#define SERVO_MAX_PWM   2500

#define PWM_DETAL_VALUE 10

//弹仓舵机开关
#define SERVO_MAGAZINE_KEY KEY_PRESSED_OFFSET_R


//0号舵机为弹仓舵机 1号舵机为限位舵机
//舵机运动初始位置
const uint16_t servo_open_pwm[4] = {SERVO_MIN_PWM, SERVO_MIN_PWM, SERVO_MIN_PWM, SERVO_MIN_PWM};
//舵机运动最终位置
const uint16_t servo_close_pwm[4] = {SERVO_MAX_PWM, SERVO_MAX_PWM, SERVO_MAX_PWM, SERVO_MAX_PWM};
//舵机发送的控制值
uint16_t servo_pwm[4] = {SERVO_MIN_PWM, SERVO_MIN_PWM, SERVO_MIN_PWM, SERVO_MIN_PWM};
//弹仓按键计算 保证双击情况下打开弹仓
uint8_t magazine_key_num = 0;

const RC_ctrl_t *servo_rc;
const RC_ctrl_t *last_servo_rc;

#define    IF_KEY_PRESSED_MAGAZINE       ( (servo_rc->key.v & KEY_PRESSED_OFFSET_R)    != 0 )
#define    LAST_IF_KEY_PRESSED_MAGAZINE       ( (last_servo_rc->key.v & KEY_PRESSED_OFFSET_R)    != 0 )
#define    IF_KEY_SINGAL_PRESSED_MAGAZINE       ( IF_KEY_PRESSED_MAGAZINE && !LAST_IF_KEY_PRESSED_MAGAZINE )


/**
  * @brief          舵机任务
  * @param[in]      pvParameters: NULL
  * @retval         none
  */
void servo_task(void const * argument)
{
    servo_rc = get_remote_control_point();
    last_servo_rc = get_last_remote_control_point();

    while(1)
    {       
    
        if(IF_KEY_SINGAL_PRESSED_MAGAZINE)
             magazine_key_num++;

        //按下R键 打开或弹仓
        if(shoot_control.magazine_status == FALSE)
        {   
            if(magazine_key_num == 2)
            {
                //打开弹仓
            servo_pwm[0] = servo_open_pwm[0];
            shoot_control.magazine_status = TRUE;
            magazine_key_num = 0;
            }
        }
        else if(shoot_control.magazine_status == TRUE)
        {
            if(magazine_key_num == 2)
            {
                //关闭弹仓
            servo_pwm[0] = servo_close_pwm[0];
            shoot_control.magazine_status = FALSE;
            magazine_key_num = 0;
            }
            
        }

        
        //限位开关
        if((shoot_control.speed_set != 0) && shoot_control.limit_switch_status == FALSE)
        {
            //当拨盘电机开始旋转时 限位开关打开
            servo_pwm[1] = servo_open_pwm[1];
            shoot_control.limit_switch_status = TRUE; 

        }
        else if((shoot_control.speed_set == 0) && shoot_control.limit_switch_status == TRUE)
        {
            //当拨盘电机停止旋转时 限位开关关闭
            servo_pwm[1] = servo_close_pwm[1];
            shoot_control.limit_switch_status = FALSE;
        }
            

        for(uint8_t i = 0; i < 4; i++)
        {
           //限制pwm
            if(servo_pwm[i] < SERVO_MIN_PWM)
            {
                servo_pwm[i] = SERVO_MIN_PWM;
            }
            else if(servo_pwm[i] > SERVO_MAX_PWM)
            {
                servo_pwm[i] = SERVO_MAX_PWM;
            }

            servo_pwm_set(servo_pwm[i], i);
        }

       
        osDelay(50);
    }
}


