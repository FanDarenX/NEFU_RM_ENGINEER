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

#define SERVO_MIN_PWM   1100
#define SERVO_MAX_PWM   2100

#define PWM_DETAL_VALUE 10

#define SERVO1_ADD_PWM_KEY  KEY_PRESSED_OFFSET_Z
#define SERVO2_ADD_PWM_KEY  KEY_PRESSED_OFFSET_X
#define SERVO3_ADD_PWM_KEY  KEY_PRESSED_OFFSET_B
#define SERVO4_ADD_PWM_KEY  KEY_PRESSED_OFFSET_CTRL

//#define SERVO_MINUS_PWM_KEY KEY_PRESSED_OFFSET_SHIFT

const RC_ctrl_t *servo_rc;
const static uint16_t servo_key[4] = {SERVO1_ADD_PWM_KEY, SERVO2_ADD_PWM_KEY, SERVO3_ADD_PWM_KEY, SERVO4_ADD_PWM_KEY};
uint16_t servo_pwm[4] = {SERVO_MIN_PWM, SERVO_MIN_PWM, SERVO_MIN_PWM, SERVO_MIN_PWM};
/**
  * @brief          servo_task
  * @param[in]      pvParameters: NULL
  * @retval         none
  */
uint16_t pwm3;
/**
  * @brief          ¶æ»úÈÎÎñ
  * @param[in]      pvParameters: NULL
  * @retval         none
  */
void servo_task(void const * argument)
{
    servo_rc = get_remote_control_point();

    while(1)
    {


        if(  (servo_rc->key.v & servo_key[3])||((servo_rc->rc.ch[2]>330)&&(servo_rc->rc.s[0]==2)))
        {
            servo_pwm[3] = SERVO_MIN_PWM;
        }
        else if(servo_rc->key.v & servo_key[2]||((servo_rc->rc.ch[2]<-330)&&(servo_rc->rc.s[0]==2)))
        {
            servo_pwm[3] =SERVO_MAX_PWM ;
        }

        servo_pwm_set(servo_pwm[3], 3);
        osDelay(10);
    }
}


