#include "bsp_usart.h"
#include "main.h"
#include "string.h"
#include "decet_task.h"
#include "ins_task.h"
#include "motor_control.h"
#include "A1_motor.h"

_PCDATA Pc_Data= {0};

uint32_t PcClear_Timer=0;
bool_t PC_update_flag = 0;
bool_t PC_flag = 0;
extern UART_HandleTypeDef huart1;
extern DMA_HandleTypeDef hdma_usart1_tx;
extern UART_HandleTypeDef huart6;
extern DMA_HandleTypeDef hdma_usart6_rx;
extern DMA_HandleTypeDef hdma_usart6_tx;
extern DMA_HandleTypeDef hdma_usart1_rx;
void UART_Send_Buff(UART_HandleTypeDef *huart,uint16_t command,uint8_t *data_input);
uint8_t PC_temp[78];
uint8_t PC_send_data[10];
uint8_t usart_state = 0;
uint8_t UI_state = 0;
// Go1 电机数据 反馈数据为 16 字节
uint8_t usart1_buf[2][16];
uint8_t usart6_buf[2][16];
// A1 电机数据 反馈数据为 78 字节
uint8_t usart1_A1_buf [2][78];
uint8_t usart6_A1_buf [2][78];

extern A1_MOTOR_recv A1_PC_Buf1;
extern A1_MOTOR_recv A1_PC_Buf6;
extern A1_MOTOR_send A1_send_data[4];
extern A1_MOTOR_send A1_Stand_send_data[4]; 
extern A1_MOTOR_recv A1_get_data[4];
extern uint8_t A1_extract_data(A1_MOTOR_recv* motor_r);



HAL_StatusTypeDef Bsp_UART_Receive_DMA(UART_HandleTypeDef *huart, uint8_t *pData, uint16_t Size)
{
    uint32_t *tmp;
    uint32_t tmp1 = 0;

    tmp1 = huart->gState;
    if((tmp1 == HAL_UART_STATE_READY) || (tmp1 == HAL_UART_STATE_BUSY_TX))
    {
        if((pData == NULL ) || (Size == 0))
        {
            return HAL_ERROR;
        }


        /* Process Locked */
        __HAL_LOCK(huart);

        huart->pRxBuffPtr = pData;
        huart->RxXferSize = Size;

        huart->ErrorCode = HAL_UART_ERROR_NONE;
        /* Check if a transmit process is ongoing or not */
        if(huart->gState == HAL_UART_STATE_BUSY_TX)
        {
            huart->gState = HAL_UART_STATE_BUSY_TX_RX;
        }
        else
        {
            huart->gState = HAL_UART_STATE_BUSY_RX;
        }

        /* Enable the DMA Stream */
        tmp = (uint32_t*)&pData;
        HAL_DMA_Start(huart->hdmarx, (uint32_t)&huart->Instance->DR, *(uint32_t*)tmp, Size);

        /* Enable the DMA transfer for the receiver request by setting the DMAR bit
        in the UART CR3 register */
        huart->Instance->CR3 |= USART_CR3_DMAR;

        /* Process Unlocked */
        __HAL_UNLOCK(huart);

        return HAL_OK;
    }
    else
    {
        return HAL_BUSY;
    }
}

HAL_StatusTypeDef Bsp_UART_Receive_IT(UART_HandleTypeDef *huart, uint8_t *pData, uint16_t Size)
{
    Bsp_UART_Receive_DMA(huart,pData,Size);//利用DMA接受数据
    __HAL_UART_ENABLE_IT(huart, UART_IT_IDLE);//开启串口空闲中断
    return HAL_OK;
}
void Bsp_UsartIdleHanlder(UART_HandleTypeDef *huart,uint16_t Size)
{   //清除DMA标志位
    uint32_t DMA_FLAGS;//根据串口的不同来选择清除不同的DMA标志位
//  uint32_t tmp;

    if(__HAL_UART_GET_FLAG(huart, UART_FLAG_IDLE))
    {
        //关闭dma
        __HAL_DMA_DISABLE(huart->hdmarx);

        //及时更新
        UART_IdleRxCallback(huart);

        DMA_FLAGS = __HAL_DMA_GET_TC_FLAG_INDEX(huart->hdmarx);
        __HAL_DMA_CLEAR_FLAG(huart->hdmarx,DMA_FLAGS);

        //while(DMA_GetCmdStatus(DMA2_Stream1) != DISABLE);

        __HAL_DMA_SET_COUNTER(huart->hdmarx,Size);

        __HAL_DMA_ENABLE(huart->hdmarx);

        /*清除IDLE标志位*/
        __HAL_UART_CLEAR_IDLEFLAG(huart);

    }
}

uint8_t ISO14443ACheckLen(uint8_t* Buffer)
{
    if((Buffer[2]+Buffer[3])==0xff)
        return 1;
    else
        return 0;
}
//CRC16打包
void ISO14443AAppendCRCA(void* Buffer, uint16_t ByteCount) {
    uint16_t Checksum = 0x6363;
    uint8_t* DataPtr = (uint8_t*) Buffer;

    while(ByteCount--) {
        uint8_t Byte = *DataPtr++;

        Byte ^= (uint8_t) (Checksum & 0x00FF);
        Byte ^= Byte << 4;

        Checksum = (Checksum >> 8) ^ ( (uint16_t) Byte << 8 ) ^
                   ( (uint16_t) Byte << 3 ) ^ ( (uint16_t) Byte >> 4 );
    }

    *DataPtr++ = (Checksum >> 0) & 0x00FF;
    *DataPtr = (Checksum >> 8) & 0x00FF;
}
//CRC16解包
uint8_t ISO14443ACheckCRCA(void* Buffer, uint16_t ByteCount)
{
    uint16_t Checksum = 0x6363;
    uint8_t* DataPtr = (uint8_t*) Buffer;

    while(ByteCount--) {
        uint8_t Byte = *DataPtr++;

        Byte ^= (uint8_t) (Checksum & 0x00FF);
        Byte ^= Byte << 4;

        Checksum = (Checksum >> 8) ^ ( (uint16_t) Byte << 8 ) ^
                   ( (uint16_t) Byte << 3 ) ^ ( (uint16_t) Byte >> 4 );
    }

    return (DataPtr[0] == ((Checksum >> 0) & 0xFF)) && (DataPtr[1] == ((Checksum >> 8) & 0xFF));
}



void UART_IdleRxCallback(UART_HandleTypeDef *huart)//串口接收中断
{   //串口
    if(huart->Instance == USART1)
    {
        if(A1_MOTOR == 1)
        {
            // A1
            if(usart1_A1_buf[1][0]==0xfe&&usart1_A1_buf[1][1]==0xee&&usart1_A1_buf[1][2]==0x00)
            {
                memcpy(&A1_get_data[3].motor_recv_data, &usart1_A1_buf[1],78);
                A1_extract_data(&A1_get_data[3]);
                decet_flag.motor_count[3]++;

            }
            if(usart1_A1_buf[0][0]==0xfe&&usart1_A1_buf[0][1]==0xee&&usart1_A1_buf[0][2]==0x01)
            {
                memcpy(&A1_get_data[2].motor_recv_data, &usart1_A1_buf[0],78);
                A1_extract_data(&A1_get_data[2]);
                decet_flag.motor_count[2]++;

            }
        }
        else
        {
            // GO1
            if(usart1_buf[0][0]==0xfd&&usart1_buf[0][1]==0xEE&&usart1_buf[0][2]==0x12)
            {
                memcpy(&Utree_get_data[2].motor_recv_data, &usart1_buf[0],16);
                extract_data(&Utree_get_data[2]);
                decet_flag.motor_count[2]++;
            }
            if(usart1_buf[1][0]==0xfd&&usart1_buf[1][1]==0xEE&&usart1_buf[1][2]==0x13)
            {
                memcpy(&Utree_get_data[3].motor_recv_data,&usart1_buf[1], 16);
                extract_data(&Utree_get_data[3]);
                decet_flag.motor_count[3]++;
            }
        }
    }

    else if(huart->Instance == USART6)
    {

        if(A1_MOTOR ==1)
        {


            if(usart6_A1_buf[0][0]==0xfe&&usart6_A1_buf[0][1]==0xEE&&usart6_A1_buf[0][2]==0x00)
            {
                memcpy(&A1_get_data[0].motor_recv_data, &usart6_A1_buf[0],78);
                A1_extract_data(&A1_get_data[0]);
                decet_flag.motor_count[0]++;

            }
            if(usart6_A1_buf[1][0]==0xfe&&usart6_A1_buf[1][1]==0xEE&&usart6_A1_buf[1][2]==0x01)
            {
                memcpy(&A1_get_data[1].motor_recv_data, &usart6_A1_buf[1],78);
                A1_extract_data(&A1_get_data[1]);
                decet_flag.motor_count[1]++;
            }

        }
        else
        {
            if(usart6_buf[0][0]==0xfd&&usart6_buf[0][1]==0xEE&&usart6_buf[0][2]==0x10)
            {
                memcpy(&Utree_get_data[0].motor_recv_data, &usart6_buf[0],  16);
                extract_data(&Utree_get_data[0]);
                decet_flag.motor_count[0]++;
            }

            if(usart6_buf[1][0]==0xfd&&usart6_buf[1][1]==0xEE&&usart6_buf[1][2]==0x11)
            {
                memcpy(&Utree_get_data[1].motor_recv_data, &usart6_buf[1],  16);
                extract_data(&Utree_get_data[1]);
                decet_flag.motor_count[1]++;
            }
        }
    }
    decet_flag.PC_count++;
}

uint8_t  Buffer[PC_BUF_LEN];
void UART_Send_Buff(UART_HandleTypeDef *huart,uint16_t command,uint8_t *data_input)
{

}

void CAN_TTL_9025(int8_t ID,int16_t current)
{
    uint8_t data[8];
    data[0] = 0xA1;
    data[1] = 0;
    data[2] = 0;
    data[3] = 0;
    data[4] = current;
    data[5] = (current>>8);
    data[6] = 0;
    data[7] = 0;
    ID=(ID+1)*32;
    UART_Send_Buff(&huart1,ID,data);
}

void UI_Send_Buff(UART_HandleTypeDef *huart,uint8_t *data_input, uint8_t data_len)
{
//  uint8_t  Buffer[200];

//	UI_state = HAL_UART_Transmit(huart, Buffer, data_len,5);
//	//HAL_UART_Transmit_DMA(huart, Buffer, data_len + HEAD_LEN + 2);
//	if(UI_state != HAL_OK) //刚上电会发送失败，重新初始化串口6
//	MX_USART6_UART_Init();
}

void usart1_tx_dma_init(void)
{

    //enable the DMA transfer for the receiver and tramsmit request
    //使能DMA串口接收和发送
    SET_BIT(huart1.Instance->CR3, USART_CR3_DMAR);
    SET_BIT(huart1.Instance->CR3, USART_CR3_DMAT);

    //disable DMA
    //失效DMA
    __HAL_DMA_DISABLE(&hdma_usart1_tx);

    while(hdma_usart1_tx.Instance->CR & DMA_SxCR_EN)
    {
        __HAL_DMA_DISABLE(&hdma_usart1_tx);
    }

    hdma_usart1_tx.Instance->PAR = (uint32_t) & (USART1->DR);
    hdma_usart1_tx.Instance->M0AR = (uint32_t)(NULL);
    hdma_usart1_tx.Instance->NDTR = 0;


}
void usart1_tx_dma_enable(uint8_t *data, uint16_t len)
{
    //disable DMA
    //失效DMA
    __HAL_DMA_DISABLE(&hdma_usart1_tx);

    while(hdma_usart1_tx.Instance->CR & DMA_SxCR_EN)
    {
        __HAL_DMA_DISABLE(&hdma_usart1_tx);
    }

    __HAL_DMA_CLEAR_FLAG(&hdma_usart1_tx, DMA_HISR_TCIF7);

    hdma_usart1_tx.Instance->M0AR = (uint32_t)(data);
    __HAL_DMA_SET_COUNTER(&hdma_usart1_tx, len);

    __HAL_DMA_ENABLE(&hdma_usart1_tx);
}

void USART1_IRQHandler(void)
{
    /* USER CODE BEGIN USART1_IRQn 0 */
    if(A1_MOTOR == 1)
    {
        Bsp_UsartIdleHanlder(&huart1,78);
    }
    else
    {
        Bsp_UsartIdleHanlder(&huart1,16);
    }
    //  Bsp_UsartIdleHanlder(&huart1,16);
    /* USER CODE END USART1_IRQn 0 */
    HAL_UART_IRQHandler(&huart1);
    /* USER CODE BEGIN USART1_IRQn 1 */

    /* USER CODE END USART1_IRQn 1 */
}



void USART6_IRQHandler(void)
{
    /* USER CODE BEGIN USART6_IRQn 0 */
    if(A1_MOTOR == 1)
    {
        Bsp_UsartIdleHanlder(&huart6,78);
    }
    else
    {
        Bsp_UsartIdleHanlder(&huart6,16);
    }
    //Bsp_UsartIdleHanlder(&huart6,16);
    /* USER CODE END USART6_IRQn 0 */
    HAL_UART_IRQHandler(&huart6);
    /* USER CODE BEGIN USART6_IRQn 1 */

    /* USER CODE END USART6_IRQn 1 */
}
void usart6_init(uint8_t *rx1_buf, uint8_t *rx2_buf, uint16_t dma_buf_num)
{
    //enable the DMA transfer for the receiver and tramsmit request
    //使能DMA串口接收和发送
    SET_BIT(huart6.Instance->CR3, USART_CR3_DMAR);
    SET_BIT(huart6.Instance->CR3, USART_CR3_DMAT);

    //enalbe idle interrupt
    //使能空闲中断
    __HAL_UART_ENABLE_IT(&huart6, UART_IT_IDLE);
    //disable DMA
    //失效DMA
    __HAL_DMA_DISABLE(&hdma_usart6_rx);

    while(hdma_usart6_rx.Instance->CR & DMA_SxCR_EN)
    {
        __HAL_DMA_DISABLE(&hdma_usart6_rx);
    }

    __HAL_DMA_CLEAR_FLAG(&hdma_usart6_rx, DMA_LISR_TCIF1);

    hdma_usart6_rx.Instance->PAR = (uint32_t) & (USART6->DR);
    //memory buffer 1
    //内存缓冲区1
    hdma_usart6_rx.Instance->M0AR = (uint32_t)(rx1_buf);
    //memory buffer 2
    //内存缓冲区2
    hdma_usart6_rx.Instance->M1AR = (uint32_t)(rx2_buf);
    //data length
    //数据长度
    __HAL_DMA_SET_COUNTER(&hdma_usart6_rx, dma_buf_num);

    //enable double memory buffer
    //使能双缓冲区
    SET_BIT(hdma_usart6_rx.Instance->CR, DMA_SxCR_DBM);

    //enable DMA
    //使能DMA
    __HAL_DMA_ENABLE(&hdma_usart6_rx);
    //disable DMA
    //失效DMA
    __HAL_DMA_DISABLE(&hdma_usart6_tx);

    while(hdma_usart6_tx.Instance->CR & DMA_SxCR_EN)
    {
        __HAL_DMA_DISABLE(&hdma_usart6_tx);
    }

    hdma_usart6_tx.Instance->PAR = (uint32_t) & (USART6->DR);
}
void usart1_init(uint8_t *rx1_buf, uint8_t *rx2_buf, uint16_t dma_buf_num)
{
    //enable the DMA transfer for the receiver and tramsmit request
    //使能DMA串口接收和发送
    SET_BIT(huart1.Instance->CR3, USART_CR3_DMAR);
    SET_BIT(huart1.Instance->CR3, USART_CR3_DMAT);

    //enalbe idle interrupt
    //使能空闲中断
    __HAL_UART_ENABLE_IT(&huart1, UART_IT_IDLE);
    //disable DMA
    //失效DMA
    __HAL_DMA_DISABLE(&hdma_usart1_rx);

    while(hdma_usart1_rx.Instance->CR & DMA_SxCR_EN)
    {
        __HAL_DMA_DISABLE(&hdma_usart1_rx);
    }

    __HAL_DMA_CLEAR_FLAG(&hdma_usart1_rx, DMA_LISR_TCIF1);

    hdma_usart1_rx.Instance->PAR = (uint32_t) & (USART1->DR);
    //memory buffer 1
    //内存缓冲区1
    hdma_usart1_rx.Instance->M0AR = (uint32_t)(rx1_buf);
    //memory buffer 2
    //内存缓冲区2
    hdma_usart1_rx.Instance->M1AR = (uint32_t)(rx2_buf);
    //data length
    //数据长度
    __HAL_DMA_SET_COUNTER(&hdma_usart1_rx, dma_buf_num);

    //enable double memory buffer
    //使能双缓冲区
    SET_BIT(hdma_usart1_rx.Instance->CR, DMA_SxCR_DBM);

    //enable DMA
    //使能DMA
    __HAL_DMA_ENABLE(&hdma_usart1_rx);
    //disable DMA
    //失效DMA
    __HAL_DMA_DISABLE(&hdma_usart1_tx);

    while(hdma_usart1_tx.Instance->CR & DMA_SxCR_EN)
    {
        __HAL_DMA_DISABLE(&hdma_usart1_tx);
    }

    hdma_usart1_tx.Instance->PAR = (uint32_t) & (USART1->DR);
}



void usart6_tx_dma_enable(uint8_t *data, uint16_t len)
{
    //disable DMA
    //失效DMA
    __HAL_DMA_DISABLE(&hdma_usart6_tx);

    while(hdma_usart6_tx.Instance->CR & DMA_SxCR_EN)
    {
        __HAL_DMA_DISABLE(&hdma_usart6_tx);
    }

    __HAL_DMA_CLEAR_FLAG(&hdma_usart6_tx, DMA_HISR_TCIF6);

    hdma_usart6_tx.Instance->M0AR = (uint32_t)(data);
    __HAL_DMA_SET_COUNTER(&hdma_usart6_tx, len);

    __HAL_DMA_ENABLE(&hdma_usart6_tx);
}


