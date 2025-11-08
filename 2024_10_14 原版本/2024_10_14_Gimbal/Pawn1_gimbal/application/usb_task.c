#include "usb_task.h"

#include "cmsis_os.h"

#include "usb_device.h"
#include "usbd_cdc_if.h"
#include <stdio.h>
#include <stdarg.h>
#include "string.h"
#include "ANO_DT.h"
#include "decet_task.h"
#include "shoot.h"
#include "bsp_usart.h"
#include "ins_task.h"
#include "referee.h"
#include "gimbal_task.h"
void usb_printf(const char *fmt,...);

static uint8_t usb_buf[256];
static const char status[2][7] = {"OK", "ERROR!"};
uint8_t test_usb_send[10] = {110,111,112,113,114,115,116,117,118,119};
uint8_t test_usb_recieve[10] = {0};
void USB_Send_Buff(Command command,uint8_t *data_input, uint8_t data_len)
{
  uint8_t  Buffer[PC_BUF_LEN];
	Buffer[0] = 0x55;
	Buffer[1] = command;
	Buffer[2] = data_len;
	Buffer[3] = 0xff - data_len;
	memcpy(Buffer + HEAD_LEN, data_input, data_len);
	ISO14443AAppendCRCA(Buffer, data_len + HEAD_LEN);
	CDC_Transmit_FS(Buffer,data_len+ HEAD_LEN + 2);
}

void usb_task(void const * argument)
{
    MX_USB_DEVICE_Init();
	  vTaskDelay(500);
    while(1)
    {
			int16_t refree_temp=0;
			char char_temp;
			PC_send_data[0]=((int16_t)(INS_angle[0]*1000) >> 8);//pitch角度
		  PC_send_data[1]=(int16_t)(INS_angle[0]*1000);
		  PC_send_data[2]=((int16_t)(INS_angle[2]*1000) >> 8);//yaw角度
		  PC_send_data[3]=(int16_t)(INS_angle[2]*1000);
			PC_send_data[4]=((int16_t)(gimbal_control.gimbal_pitch_motor.motor_gyro*1000) >> 8);//pitch角速度
		  PC_send_data[5]=(int16_t)(gimbal_control.gimbal_pitch_motor.motor_gyro*1000);
		  PC_send_data[6]=((int16_t)(gimbal_control.gimbal_yaw_motor.motor_gyro*1000) >> 8);//yaw角度
		  PC_send_data[7]=(int16_t)(gimbal_control.gimbal_yaw_motor.motor_gyro*1000);
			if(RM_Referee.robot_state.robot_id <=5)
			{ refree_temp =0; }//红色0+
			else
			{ refree_temp =10000; }//蓝色50+
//			refree_temp+=shoot_control.bullet_speed*10;
			refree_temp+=25*10;
		  PC_send_data[8]=(int16_t)refree_temp>>8;
			PC_send_data[9]=(int16_t)refree_temp;
			if(Pawn_mode==0)char_temp =      Remote_mode;
			else if(Pawn_mode==1)char_temp = Kill_mode;
			else if(Pawn_mode==2)char_temp = SmallBuff_mode;
			else if(Pawn_mode==3)char_temp = BIGBuff_mode;
			else if(Pawn_mode==4)char_temp = Huang_mode;
		//	Waveform_Write();
			USB_Send_Buff(char_temp,PC_send_data,sizeof(PC_send_data));
			Time.USB_Time = Time.main_Time;
			Time.INS_USB_delat_Time = Time.USB_Time -  Time.INS_Time ;
			decet_flag.usb_send_count++;
      vTaskDelay(5);
    }
}

void usb_printf(const char *fmt,...)
{
	
    static va_list ap;
    uint16_t len = 0;
    va_start(ap, fmt);
    len = vsprintf((char *)usb_buf, fmt, ap);
    va_end(ap);
    CDC_Transmit_FS(usb_buf, len);
}
