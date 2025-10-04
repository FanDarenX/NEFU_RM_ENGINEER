#include "send.h"





uint8_t seq_num = 0;
uint8_t send_buf[39] = {0};




#pragma pack(push, 1)
typedef struct
{
    uint8_t header; // 例如0xAA
    uint16_t angles[4];
    uint8_t checksum; // 校验和
} AnglePacket;
#pragma pack(pop)

void Send_Angle_Binary(void)
{
    AnglePacket packet = {
        .header = 0xAA,
        .angles = {angle[0], angle[1], angle[2], angle[3]},
        .checksum = 0};

    // 计算校验和（简单示例）
    for (size_t i = 0; i < sizeof(packet) - 1; i++)
    {
        packet.checksum += ((uint8_t *)&packet)[i];
    }

    HAL_UART_Transmit_DMA(&huart6, (uint8_t *)&packet, sizeof(packet));
}

// DMA发送完成回调
void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance == USART6)
    {
    }
}

/**
 * @brief 发送4个舵机角度数据（使用0x0302命令码）
 * @param angles 4个舵机角度值数组（范围500-2500）
 * @param timestamp 时间戳（可选）
 */
void Send_Servo_Angles(uint16_t angles[4])
{
    /* 协议帧结构说明（总长度39字节）：
     * [0] 0xA5       帧头
     * [1] data_lenL  数据长度低8位（角度数据+时间戳=10字节）
     * [2] data_lenH  数据长度高8位
     * [3] seq        包序号（自动递增）
     * [4] crc8       帧头CRC8校验（覆盖0-4字节）
     * [5] cmd_idL    命令码低8位（0x02）
     * [6] cmd_idH    命令码高8位（0x03）
     * [7-36] data    数据区（实际使用10字节，剩余补零）
     * [37] crc16L    CRC16低8位（覆盖0-36字节）
     * [38] crc16H    CRC16高8位
     */
	int self_defining_data_ID = 0x0302; // 客户端自定义数据ID
    
    int Data_Length = 30;

    // 1. 填充帧头
    send_buf[0] = 0xA5;                        // 帧头
    send_buf[1] = (uint8_t)(Data_Length);      // 数据长度低8位（4角度*2 + 时间戳2）
    send_buf[2] = (uint8_t)(Data_Length >> 8); // 数据长度高8位
    send_buf[3] = seq_num++;                   // 包序号递增

    // 2. 计算帧头CRC8（覆盖前5字节）
    Append_CRC8_Check_Sum(send_buf, 5);

    // 3. 填充命令码（0x0302）
    send_buf[5] =(uint8_t)(self_defining_data_ID); // 命令码低字节
    send_buf[6] = (uint8_t)(self_defining_data_ID >> 8); // 命令码高字节

    // 4. 填充角度数据（小端序）
    send_buf[7] = (uint8_t)angles[0];  // 舵机0低8位
    send_buf[8] = (uint8_t)(angles[0] >> 8);    // 舵机0高8位
    send_buf[9] = (uint8_t)angles[1];  // 舵机1低8位
    send_buf[10] = (uint8_t)(angles[1] >> 8);   // 舵机1高8位
    send_buf[11] = (uint8_t)angles[2]; // 舵机2低8位
    send_buf[12] = (uint8_t)(angles[2] >> 8);   // 舵机2高8位
    send_buf[13] = (uint8_t)angles[3]; // 舵机3低8位
    send_buf[14] = (uint8_t)(angles[3] >> 8);   // 舵机3高8位

    //    // 5. 填充时间戳（可选）
    //    send_buf[15] = timestamp & 0xFF;     // 时间戳低8位
    //    send_buf[16] = timestamp >> 8;       // 时间戳高8位
    // 17-36保留为0（根据协议要求填充剩余空间）

    // 6. 计算整帧CRC16（覆盖0-36字节，结果放在37-38）
    Append_CRC16_Check_Sum(send_buf, 39);

    // 7. 通过USART6发送（DMA模式）
    HAL_UART_Transmit_DMA(&huart6, send_buf, sizeof(send_buf));
}
