#include "bsp_usart.h"
#include "main.h"
#include "string.h"
#include "decet_task.h"
#include "ins_task.h"
uint8_t PC_Buf[PC_BUF_LEN];
_PCDATA Pc_Data={0};

uint32_t PcClear_Timer=0;
bool_t PC_update_flag = 0;
bool_t PC_flag = 0;
extern UART_HandleTypeDef huart1;
extern DMA_HandleTypeDef hdma_usart1_tx;
extern UART_HandleTypeDef huart6;
extern DMA_HandleTypeDef hdma_usart6_rx;
extern DMA_HandleTypeDef hdma_usart6_tx;
void UART_Send_Buff(UART_HandleTypeDef *huart,Command command,uint8_t *data_input, uint8_t data_len);
uint8_t PC_temp[50];
//uint8_t PC_send_data[6]; 
uint8_t PC_send_data[10]; 
uint8_t usart_state = 0;
uint8_t UI_state = 0;
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
{//清除DMA标志位
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
fp32 WT_gyro[3] = {0.0f, 0.0f, 0.0f};
fp32 WT_angle[3] = {0.0f, 0.0f, 0.0f};
fp32 distance=0;
void UART_IdleRxCallback(UART_HandleTypeDef *huart)//串口接收中断
{//串口
	static fp32 last_yaw=0;
	uint8_t gyro_temp=0;
	uint8_t angle_temp=0;
	if(huart->Instance == USART1)
	{
//				gyro_temp=PC_Buf[20]+PC_Buf[11]+PC_Buf[12]+PC_Buf[13]+PC_Buf[14]+
//										 PC_Buf[15]+PC_Buf[16]+PC_Buf[17]+PC_Buf[18]+PC_Buf[19];
//				angle_temp=PC_Buf[22]+PC_Buf[23]+PC_Buf[24]+PC_Buf[25]+PC_Buf[26]+
//										 PC_Buf[27]+PC_Buf[28]+PC_Buf[29]+PC_Buf[30]+PC_Buf[31];
//			if(PC_Buf[21]==gyro_temp)//此时看到了目标
//	  		{
//			   for(int i=11;i<22;i++)
//				 {PC_temp[i]=PC_Buf[i];}//缓冲PC数据
//				 WT_gyro[0]=((fp32)((PC_temp)[14] << 8 | (PC_temp)[13])/32768* 4.214587f);
//				 WT_gyro[1]=((fp32)((PC_temp)[16] << 8 | (PC_temp)[15])/32768* 4.214587f);
//				 WT_gyro[2]=((fp32)((PC_temp)[18] << 8 | (PC_temp)[17])/32768* 4.214587f);
//				 if(WT_gyro[0]>4.214587f)WT_gyro[0]=WT_gyro[0]-2*4.214587f;
//				 if(WT_gyro[1]>4.214587f)WT_gyro[1]=WT_gyro[1]-2*4.214587f;
//				 if(WT_gyro[2]>4.214587f)WT_gyro[2]=WT_gyro[2]-2*4.214587f;
//  			 }
//			if(PC_Buf[32]==angle_temp)//此时看到了目标
//				{
//				for(int i=22;i<33;i++)
//				 {PC_temp[i]=PC_Buf[i];}//缓冲PC数据
//				 WT_angle[0]=((fp32)((PC_temp)[25] << 8 | (PC_temp)[24])/32768*PI);
//				 WT_angle[1]=((fp32)((PC_temp)[27] << 8 | (PC_temp)[26])/32768*PI);
//				 WT_angle[2]=((fp32)((PC_temp)[29] << 8 | (PC_temp)[28])/32768*PI);
//				 if(WT_angle[0]>PI)WT_angle[0]=WT_angle[0]-2*PI;
//				 if(WT_angle[1]>PI)WT_angle[1]=WT_angle[1]-2*PI;
//				 if(WT_angle[2]>PI)WT_angle[2]=WT_angle[2]-2*PI;
				 distance=(PC_Buf[3]-0x30)*100+(PC_Buf[4]-0x30)*10+(PC_Buf[5]-0x30)
				 +(PC_Buf[7]-0x30)*0.1f+(PC_Buf[8]-0x30)*0.01f+(PC_Buf[9]-0x30)*0.001f;

				 decet_flag.PC_count++;
			 //}
		 }
	
}


void UART_Send_Buff(UART_HandleTypeDef *huart,Command command,uint8_t *data_input, uint8_t data_len)
{
  uint8_t  Buffer[PC_BUF_LEN];
	Buffer[0] = 0x55;
	Buffer[1] = command;
	Buffer[2] = data_len;
	Buffer[3] = 0xff - data_len;
	memcpy(Buffer + HEAD_LEN, data_input, data_len);
	ISO14443AAppendCRCA(Buffer, data_len + HEAD_LEN);
	usart_state = HAL_UART_Transmit(huart, Buffer, data_len + HEAD_LEN + 2,5);
	//HAL_UART_Transmit_DMA(huart, Buffer, data_len + HEAD_LEN + 2);
	if(usart_state != HAL_OK) //刚上电会发送失败，重新初始化串口1
	MX_USART1_UART_Init();
}

void UI_Send_Buff(UART_HandleTypeDef *huart,uint8_t *data_input, uint8_t data_len)
{
  uint8_t  Buffer[200];

	UI_state = HAL_UART_Transmit(huart, Buffer, data_len,5);
	//HAL_UART_Transmit_DMA(huart, Buffer, data_len + HEAD_LEN + 2);
	if(UI_state != HAL_OK) //刚上电会发送失败，重新初始化串口6
	MX_USART6_UART_Init();
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
  Bsp_UsartIdleHanlder(&huart1,PC_BUF_LEN);
  /* USER CODE END USART1_IRQn 0 */
  HAL_UART_IRQHandler(&huart1);
  /* USER CODE BEGIN USART1_IRQn 1 */

  /* USER CODE END USART1_IRQn 1 */
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


