/*
 * can_gateway.c
 *
 *  Created on: Nov 21, 2025
 *      Author: sydor
 */


#include "can_gateway.h"

#define ID_ALLOW1 0x040
#define ID_ALLOW2 0x120
#define ID_REWRITE 0x121
#define RATE_LIMIT_MS 100   // 10 Гц

static CAN_HandleTypeDef *canA;
static CAN_HandleTypeDef *canB;

static uint32_t last_forward_time[2048] = {0};

void CAN_GW_Init(CAN_HandleTypeDef *hcanA, CAN_HandleTypeDef *hcanB)
{
    canA = hcanA;
    canB = hcanB;

    HAL_CAN_ActivateNotification(canA, CAN_IT_RX_FIFO0_MSG_PENDING);
    HAL_CAN_ActivateNotification(canB, CAN_IT_RX_FIFO0_MSG_PENDING);
}

void CAN_GW_ProcessA(void)
{
    CAN_RxHeaderTypeDef rx;
    uint8_t data[8];

    if (HAL_CAN_GetRxMessage(canA, CAN_RX_FIFO0, &rx, data) != HAL_OK)
        return;

    uint32_t now = HAL_GetTick();

    // allowlist
    if (rx.StdId != ID_ALLOW1 && rx.StdId != ID_ALLOW2)
        return;

    // rate limit
    if (now - last_forward_time[rx.StdId] < RATE_LIMIT_MS)
        return;

    last_forward_time[rx.StdId] = now;

    // rewrite ID
    if (rx.StdId == ID_ALLOW2)
        rx.StdId = ID_REWRITE;

    HAL_CAN_AddTxMessage(canB, &rx, data, NULL);
}
