#include "send.h"





uint8_t seq_num = 0;
uint8_t send_buf[39] = {0};




#pragma pack(push, 1)
typedef struct
{
    uint8_t header; // ����0xAA
    uint16_t angles[4];
    uint8_t checksum; // У���
} AnglePacket;
#pragma pack(pop)

void Send_Angle_Binary(void)
{
    AnglePacket packet = {
        .header = 0xAA,
        .angles = {angle[0], angle[1], angle[2], angle[3]},
        .checksum = 0};

    // ����У��ͣ���ʾ����
    for (size_t i = 0; i < sizeof(packet) - 1; i++)
    {
        packet.checksum += ((uint8_t *)&packet)[i];
    }

    HAL_UART_Transmit_DMA(&huart6, (uint8_t *)&packet, sizeof(packet));
}

// DMA������ɻص�
void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance == USART6)
    {
    }
}

/**
 * @brief ����4������Ƕ����ݣ�ʹ��0x0302�����룩
 * @param angles 4������Ƕ�ֵ���飨��Χ500-2500��
 * @param timestamp ʱ�������ѡ��
 */
void Send_Servo_Angles(uint16_t angles[4])
{
    /* Э��֡�ṹ˵�����ܳ���39�ֽڣ���
     * [0] 0xA5       ֡ͷ
     * [1] data_lenL  ���ݳ��ȵ�8λ���Ƕ�����+ʱ���=10�ֽڣ�
     * [2] data_lenH  ���ݳ��ȸ�8λ
     * [3] seq        ����ţ��Զ�������
     * [4] crc8       ֡ͷCRC8У�飨����0-4�ֽڣ�
     * [5] cmd_idL    �������8λ��0x02��
     * [6] cmd_idH    �������8λ��0x03��
     * [7-36] data    ��������ʵ��ʹ��10�ֽڣ�ʣ�ಹ�㣩
     * [37] crc16L    CRC16��8λ������0-36�ֽڣ�
     * [38] crc16H    CRC16��8λ
     */
	int self_defining_data_ID = 0x0302; // �ͻ����Զ�������ID
    
    int Data_Length = 30;

    // 1. ���֡ͷ
    send_buf[0] = 0xA5;                        // ֡ͷ
    send_buf[1] = (uint8_t)(Data_Length);      // ���ݳ��ȵ�8λ��4�Ƕ�*2 + ʱ���2��
    send_buf[2] = (uint8_t)(Data_Length >> 8); // ���ݳ��ȸ�8λ
    send_buf[3] = seq_num++;                   // ����ŵ���

    // 2. ����֡ͷCRC8������ǰ5�ֽڣ�
    Append_CRC8_Check_Sum(send_buf, 5);

    // 3. ��������루0x0302��
    send_buf[5] =(uint8_t)(self_defining_data_ID); // ��������ֽ�
    send_buf[6] = (uint8_t)(self_defining_data_ID >> 8); // ��������ֽ�

    // 4. ���Ƕ����ݣ�С����
    send_buf[7] = (uint8_t)angles[0];  // ���0��8λ
    send_buf[8] = (uint8_t)(angles[0] >> 8);    // ���0��8λ
    send_buf[9] = (uint8_t)angles[1];  // ���1��8λ
    send_buf[10] = (uint8_t)(angles[1] >> 8);   // ���1��8λ
    send_buf[11] = (uint8_t)angles[2]; // ���2��8λ
    send_buf[12] = (uint8_t)(angles[2] >> 8);   // ���2��8λ
    send_buf[13] = (uint8_t)angles[3]; // ���3��8λ
    send_buf[14] = (uint8_t)(angles[3] >> 8);   // ���3��8λ

    //    // 5. ���ʱ�������ѡ��
    //    send_buf[15] = timestamp & 0xFF;     // ʱ�����8λ
    //    send_buf[16] = timestamp >> 8;       // ʱ�����8λ
    // 17-36����Ϊ0������Э��Ҫ�����ʣ��ռ䣩

    // 6. ������֡CRC16������0-36�ֽڣ��������37-38��
    Append_CRC16_Check_Sum(send_buf, 39);

    // 7. ͨ��USART6���ͣ�DMAģʽ��
    HAL_UART_Transmit_DMA(&huart6, send_buf, sizeof(send_buf));
}
