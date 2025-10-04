#ifndef BSP_CAN_H
#define BSP_CAN_H

#include "can.h"
#include "struct_typedef.h"

typedef CAN_HandleTypeDef hcan_t;


void CAN1_FILTER_CONFIG(CAN_HandleTypeDef* hcan);
void CAN2_FILTER_CONFIG(CAN_HandleTypeDef* hcan);

void can_filter_init(void);
void bsp_can_init(void);

uint8_t canx_send_data(hcan_t *hcan, uint16_t id, uint8_t *data, uint32_t len);
uint8_t canx_receive(hcan_t *hcan, uint16_t *recid, uint8_t *buf);
#endif
