#ifndef __CAN_BSP_H__
#define __CAN_BSP_H__
#include "main.h"
#include "fdcan.h"
#include "string.h"


typedef struct canFrame{
    uint16_t frame_id;
    uint8_t frame_data[8];
    uint8_t data_len;
    
}canFrame;


void can_bsp_init(void);

void fdcan2_rx_callback(void);
void fdcan1_rx_callback(void);

HAL_StatusTypeDef fdcanx_send_frame(FDCAN_HandleTypeDef *hfdcan, canFrame* send_frame);
int8_t fdcanx_receive_frame(FDCAN_HandleTypeDef *hfdcan, canFrame* frame_loc, uint32_t FIFOx);


#endif /* __CAN_BSP_H_ */


