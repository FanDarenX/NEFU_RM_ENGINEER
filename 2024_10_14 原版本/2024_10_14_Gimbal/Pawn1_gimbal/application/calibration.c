#include "calibration.h"
#include "gimbal_task.h"
#include "math.h"
#include "FreeRTOS.h"
#include "task.h"
#include "timers.h"
#include "stack_macros.h"
#include "bsp_flash.h"
#define ABS(x) (((x) > 0) ? (x) : (-(x)))

uint8_t gyro_off_read_data[GYRO_DATA_LENGHT]={0};
uint8_t gyro_off_write_data[GYRO_DATA_LENGHT]={0};
cailbration_flag_t cailbration_flag;

void Calibration_Init(cailbration_flag_t *cailbration_flag)
{
	cailbration_flag->gravity_flag=0;
	cailbration_flag->gyro_flag=0;
}

uint8_t Debug_switch(cailbration_flag_t *cailbration_flag)
{
  uint8_t flag;
	if(cailbration_flag->gravity_flag!=0)     return flag=1;
	else if(cailbration_flag->gyro_flag!=0)   return flag=2;
	
	else                                      return flag=0;
}
/**
 * @brief 重力前馈系数测定（debug功能中使用）
 *
 */
void Gravity_compensation(gimbal_control_t *gimbal)
{
	if(cailbration_flag.gravity_flag==1)
	{
		cailbration_flag.gravity_flag=2;
		gimbal->gimbal_pitch_motor.absolute_angle_set1 =gimbal->gimbal_pitch_motor.absolute_angle;
		gimbal->gimbal_pitch_motor.raw_cmd_current = -5000.0f;
	}
	
	if(ABS(gimbal->gimbal_pitch_motor.absolute_angle_set1-gimbal->gimbal_pitch_motor.absolute_angle)>0.0174532925f*2)//当前角度与目标角度差一度以上
	{
		if(gimbal->gimbal_pitch_motor.absolute_angle_set1 < gimbal->gimbal_pitch_motor.absolute_angle)//目标角度高
		{
			if(gimbal->gimbal_pitch_motor.absolute_angle_set1-gimbal->gimbal_pitch_motor.absolute_angle < -0.0872664626f)//设定值比当前小五度（高五度）
			{
				gimbal->gimbal_pitch_motor.raw_cmd_current -= 0.5f;
			}
			else
			{
				gimbal->gimbal_pitch_motor.raw_cmd_current -= 0.1f;
			}
		}
		else if(gimbal->gimbal_pitch_motor.absolute_angle_set1 > gimbal->gimbal_pitch_motor.absolute_angle)//目标角度低
		{
			if(gimbal->gimbal_pitch_motor.absolute_angle_set1-gimbal->gimbal_pitch_motor.absolute_angle > 0.0872664626f)//设定值比当前大五度（低五度）
			{
				gimbal->gimbal_pitch_motor.raw_cmd_current += 2.0f;
			}
			else
			{
				gimbal->gimbal_pitch_motor.raw_cmd_current += 0.5f;
			}
		}
			gimbal->gimbal_pitch_motor.gravity_recordtime = 0;
		  gimbal->gimbal_pitch_motor.given_current = gimbal->gimbal_pitch_motor.raw_cmd_current;
	}
	else
	{
	   gimbal->gimbal_pitch_motor.gravity_recordtime+=0.002f;
	}
	
	if(gimbal->gimbal_pitch_motor.gravity_recordtime>1.0f)
	{
		gimbal->gimbal_pitch_motor.given_current = gimbal->gimbal_pitch_motor.raw_cmd_current;
		if(gimbal->gimbal_pitch_motor.absolute_angle_set1>0.0174532925f*2)
		{
			gimbal->gimbal_pitch_motor.absolute_angle_set1=gimbal->gimbal_pitch_motor.absolute_angle_set1-0.0174532925f*2;
		}
		else
		{
			for(uint8_t i = 0;i<5;i++)
			{
				gimbal->gimbal_pitch_motor.calibration_t[i] = gimbal->gimbal_pitch_motor.given_current/cos(ABS(gimbal->gimbal_pitch_motor.absolute_angle));
				gimbal->gimbal_pitch_motor.calibration += 0.2*(gimbal->gimbal_pitch_motor.calibration_t[i]);
				cailbration_flag.gravity_flag = 0;
			}
		}
	}
	

}

/**
 * @brief yaw轴零漂测定（debug功能中使用）
 *
 */
fp32   angle_start=0,angle_temp[5],angle_num[5];
int16_t temp;
void Dritt_compensation(gimbal_control_t *gimbal)
{
	static uint16_t num=0;
		gimbal->gimbal_yaw_motor.absolute_angle_offest=0;
	
		if(num<2500&&cailbration_flag.gyro_flag==1)num++;//刚开始2.5秒不进行计算
		else if((num>=2500&&cailbration_flag.gyro_flag==1))
		{
			cailbration_flag.gyro_flag=2;num=0;
			angle_start = gimbal->gimbal_yaw_motor.absolute_angle;//记录当前角度
			gimbal->gimbal_yaw_motor.absolute_angle_set1=angle_start;
		}
		gimbal->gimbal_yaw_motor.given_current=0;

		gimbal->gimbal_pitch_motor.given_current= gimbal_PI_LQR(LQR_PI_pitch[3],LQR_PI_pitch[4],LQR_PI_pitch[0],LQR_PI_pitch[1],LQR_PI_pitch[2],//PI_LQR Serise
																																			(fp32) gimbal->gimbal_pitch_motor.absolute_angle,
																																			(fp32) gimbal->gimbal_pitch_motor.motor_gyro,
																																      (fp32) gimbal->gimbal_pitch_motor.absolute_current,
  																																		(fp32) 0);
	
	if(cailbration_flag.gyro_flag==2)
	{
		num++;
//		if(num==500)			angle_temp[0]= gimbal->gimbal_yaw_motor.absolute_angle;
//		else if(num==1000)angle_temp[1]= gimbal->gimbal_yaw_motor.absolute_angle;
//		else if(num==2500)angle_temp[2]= gimbal->gimbal_yaw_motor.absolute_angle;
//		else if(num==4000)angle_temp[3]= gimbal->gimbal_yaw_motor.absolute_angle;
		if(num==10000)angle_temp[4]= gimbal->gimbal_yaw_motor.absolute_angle;
		else if(num==11000)cailbration_flag.gyro_flag=3;
	}
	else if(cailbration_flag.gyro_flag==3)
	{
		num=0;
//		angle_num[0]=(angle_temp[0]-angle_start)/500;
//		angle_num[1]=(angle_temp[1]-angle_start)/1000;
//		angle_num[2]=(angle_temp[2]-angle_start)/2500;
//		angle_num[3]=(angle_temp[3]-angle_start)/4000;
		angle_num[4]=(angle_temp[4]-angle_start);
//		gimbal->gimbal_yaw_motor.absolute_angle_offest =angle_num[4]/5000;
		temp =(int16_t)(angle_num[4]*50000);//10次方
		
		gyro_off_write_data[0]=temp>>8;
		gyro_off_write_data[1]=temp;

    //擦除flash页
    flash_erase_address(ADDR_FLASH_SECTOR_10, 1);
		vTaskDelay(5);
    //往flash写数据
    flash_write_single_address(ADDR_FLASH_SECTOR_10, (uint32_t *)gyro_off_write_data, (GYRO_DATA_LENGHT + 3) / 4);
				vTaskDelay(5);
    //在写数据之后,从flash读取数据
    flash_read(ADDR_FLASH_SECTOR_10, (uint32_t *)gyro_off_read_data, (GYRO_DATA_LENGHT + 3) / 4);
		
		gimbal->gimbal_yaw_motor.absolute_angle_offest=
		((fp32)(gyro_off_read_data[0]<<8 | gyro_off_read_data[1])/100000000);
		cailbration_flag.gyro_flag=0;	
	}
	
}