#include "can_bsp.h"



static canFrame Can1_Rx = { 0 };
static canFrame Can2_Rx = { 0 };

/**
************************************************************************
* @brief:      	can_filter_init(void)
* @param:       void
* @retval:     	void
* @details:    	CAN过滤器初始化
************************************************************************
**/
static void can_filter_init(void)
{
	FDCAN_FilterTypeDef fdcan_filter;

	fdcan_filter.IdType = FDCAN_STANDARD_ID;                       //标准ID
	fdcan_filter.FilterIndex = 0;                                  //滤波器索引
	fdcan_filter.FilterConfig = FDCAN_FILTER_TO_RXFIFO0;           //过滤器0关联到FIFO0
	
	fdcan_filter.FilterType = FDCAN_FILTER_RANGE;                 //接收为ID1~ID2
	fdcan_filter.FilterID1 = 0x000;                               //32位ID 接收ID起始
	fdcan_filter.FilterID2 = 0x7FF;                               //接收ID范围末尾
	
	
	HAL_FDCAN_ConfigFilter(&hfdcan1,&fdcan_filter);
	HAL_FDCAN_ConfigGlobalFilter(&hfdcan1, FDCAN_REJECT, FDCAN_REJECT,
			FDCAN_FILTER_REMOTE, FDCAN_FILTER_REMOTE);
	HAL_FDCAN_ConfigFifoWatermark(&hfdcan1, FDCAN_CFG_RX_FIFO0, 2);
	
	
	HAL_FDCAN_ConfigFilter(&hfdcan2,&fdcan_filter);
	HAL_FDCAN_ConfigGlobalFilter(&hfdcan2, FDCAN_REJECT, FDCAN_REJECT,
			FDCAN_FILTER_REMOTE, FDCAN_FILTER_REMOTE);
	HAL_FDCAN_ConfigFifoWatermark(&hfdcan2, FDCAN_CFG_RX_FIFO0, 2);
}



void can_bsp_init(void)
{
	can_filter_init();
	HAL_FDCAN_Start(&hfdcan1);                               //开启FDCAN
	HAL_FDCAN_ActivateNotification(&hfdcan1, FDCAN_IT_RX_FIFO0_NEW_MESSAGE, 0);
	HAL_FDCAN_Start(&hfdcan2);                               //开启FDCAN
	HAL_FDCAN_ActivateNotification(&hfdcan2, FDCAN_IT_RX_FIFO0_NEW_MESSAGE, 0);
}



HAL_StatusTypeDef fdcanx_send_frame(FDCAN_HandleTypeDef *hfdcan, canFrame* send_frame)
{
	FDCAN_TxHeaderTypeDef TxHeader;

	TxHeader.Identifier = send_frame->frame_id;
	TxHeader.IdType = FDCAN_STANDARD_ID;							// 标准ID
	TxHeader.TxFrameType = FDCAN_DATA_FRAME;						// 数据帧 
	TxHeader.DataLength = send_frame->data_len;								// 发送数据长度
	TxHeader.ErrorStateIndicator = FDCAN_ESI_ACTIVE;				// 设置错误状态指示
	TxHeader.BitRateSwitch = FDCAN_BRS_OFF;						// 不开启可变波特率
	TxHeader.FDFormat = FDCAN_CLASSIC_CAN;						// 普通CAN格式
	TxHeader.TxEventFifoControl = FDCAN_NO_TX_EVENTS;				// 用于发送事件FIFO控制, 不存储
	TxHeader.MessageMarker = 0x00; 			// 用于复制到TX EVENT FIFO的消息Maker来识别消息状态，范围0到0xFF

	return HAL_FDCAN_AddMessageToTxFifoQ(hfdcan, &TxHeader, send_frame->frame_data);
}




/**
************************************************************************
* @brief:      	HAL_FDCAN_RxFifo0Callback(FDCAN_HandleTypeDef *hfdcan, uint32_t RxFifo0ITs)
* @param:       hfdcan；FDCAN句柄
* @param:       RxFifo0ITs：中断标志位
* @retval:     	void
* @details:    	HAL库的FDCAN中断回调函数
************************************************************************
**/
void HAL_FDCAN_RxFifo0Callback(FDCAN_HandleTypeDef *hfdcan, uint32_t RxFifo0ITs)
{
  if((RxFifo0ITs & FDCAN_IT_RX_FIFO0_NEW_MESSAGE) != RESET)
  {
		if(hfdcan == &hfdcan1)
		{
			fdcan1_rx_callback();
		}
		if(hfdcan == &hfdcan2)
		{
			fdcan2_rx_callback();
		}
  }
}




int8_t fdcanx_receive_frame(FDCAN_HandleTypeDef *hfdcan, canFrame* frame_loc, uint32_t FIFOx)
{
	FDCAN_RxHeaderTypeDef fdcan_RxHeader;
	memset(frame_loc->frame_data, 0, 8);		// 将data原本的值全部清空
	
	if( HAL_FDCAN_GetRxMessage(hfdcan,FIFOx,
		&fdcan_RxHeader, frame_loc->frame_data) != HAL_OK) {
		return 0;//接收数据
	}
	frame_loc->data_len = fdcan_RxHeader.DataLength;
	frame_loc->frame_id = fdcan_RxHeader.Identifier;
	
	return fdcan_RxHeader.DataLength;
}





/**
************************************************************************
* @brief:      	fdcan_rx_callback(void)
* @param:       void
* @retval:     	void
* @details:    	供用户调用的接收弱函数
************************************************************************
**/

__weak void fdcan1_rx_callback(void)
{
	fdcanx_receive_frame(&hfdcan1, &Can1_Rx, FDCAN_RX_FIFO0);
	fdcanx_send_frame(&hfdcan1, &Can1_Rx);
}

__weak void fdcan2_rx_callback(void)
{
	fdcanx_receive_frame(&hfdcan2, &Can2_Rx, FDCAN_RX_FIFO0);
	fdcanx_send_frame(&hfdcan2, &Can2_Rx);
}

