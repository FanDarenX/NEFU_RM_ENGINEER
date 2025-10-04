#include "read.h"

#include "main.h"
#include "string.h"
#include "stdio.h"

// UART8句柄
extern UART_HandleTypeDef huart8;

// 全局变量

uint8_t uart_rx_buf[RX_BUF_SIZE];
uint8_t uart_rx_flag = 0;
uint16_t servo_angle = 0;
uint16_t rx_index = 0; // 当前写入位置

typedef struct
{
    uint16_t angle;
    uint8_t updated; // 角度更新标志
} ServoInfo;

ServoInfo servos[MAX_SERVO_NUM]; // ID0~3的舵机数据

// 初始化卸力
void servo_init(void)
{
    // 发送卸力指令（ID=0）
    uint8_t cmd[] = "#255PULK!";
    HAL_UART_Transmit(&huart8, cmd, strlen((char *)cmd), 100);
    HAL_Delay(100);
}

// 发送角度读取请求
void request_angle(void)
{
    uint8_t cmd[] = "#000PRAD!";
    HAL_UART_Transmit(&huart8, cmd, strlen((char *)cmd), 100);
}

// 请求指定ID舵机角度
void request_angle_id(uint8_t id)
{
    if (id >= MAX_SERVO_NUM)
        return;

    char cmd[10];
    snprintf(cmd, sizeof(cmd), "#%03dPRAD!", id); // 格式化指令
    HAL_UART_Transmit(&huart8, (uint8_t *)cmd, strlen(cmd), 100);
}

void parse_angle(void)
{
    static uint8_t frame[ANGLE_RESPONSE_LEN];
    static uint8_t frame_pos = 0;

    for (uint16_t i = 0; i < RX_BUF_SIZE; i++)
    {
        uint8_t byte = uart_rx_buf[i];

        // 帧头检测
        if (byte == '#')
        {
            frame_pos = 0;
            frame[frame_pos++] = byte;
            continue;
        }

        // 帧数据收集
        if (frame_pos > 0 && frame_pos < ANGLE_RESPONSE_LEN)
        {
            frame[frame_pos++] = byte;
        }
        // 完整帧检测（注意：去除了I后帧尾是'!')
        if (byte == '!' && frame_pos == ANGLE_RESPONSE_LEN)
        {
            // 验证格式 "#000P1500!"
            if (frame[4] == 'P')
            {
                uint8_t id = (frame[1] - '0') * 100 + (frame[2] - '0') * 10 + (frame[3] - '0');
                // 提取角度（500-2500）
                // 仅处理ID0~3
                if (id < MAX_SERVO_NUM)
                {
                    servos[id].angle = (frame[5] - '0') * 1000 +
                                       (frame[6] - '0') * 100 +
                                       (frame[7] - '0') * 10 +
                                       (frame[8] - '0');
                    servos[id].updated = 1;
					
                }
            }
            frame_pos = 0;
        }
    }
    uart_rx_flag = 0;
}

// 获取指定ID的角度（非阻塞式）
uint8_t get_servo_angle(uint8_t id, uint16_t *angle)
{
    if (id >= MAX_SERVO_NUM || angle == NULL)
        return 0;

    if (servos[id].updated)
    {
		if(servos[id].angle>500 && servos[id].angle< 2500)
		{
			*angle = servos[id].angle;
		}
        servos[id].updated = 0;
        return 1; // 成功获取
    }
    return 0; // 数据未更新
}

// UART接收回调函数
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance == UART8)
    {
        uart_rx_buf[rx_index] = huart->Instance->DR;
        rx_index = (rx_index + 1) % RX_BUF_SIZE;
        uart_rx_flag = 1;
    }
}
