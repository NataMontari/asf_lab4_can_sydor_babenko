/*
 * can_filters.c
 *
 *  Created on: Nov 18, 2025
 *      Author: Cerberus
 */


#include "can_filters.h"
#include "main.h"

extern CAN_HandleTypeDef hcan;

/*
 *  - Filter 0: CMD group 0x040..0x04F  -> FIFO0 (CMD/ACK)
 *  - Filter 1: HB  group 0x080..0x08F  -> FIFO1 (HB/diagnostics)
 *
 * Для 11-bit STD IDs в регістри ставимо (ID << 5), (MASK << 5) згідно RM/HAL.
 * Ми використовуємо режим маски (IDMASK) та 32-bit scale.
 */
void CAN_Config_Filter(void)
{
    HAL_StatusTypeDef st;
    CAN_FilterTypeDef sFilterConfig;

    // --- Filter 0: Commands group -> FIFO0 ---
    sFilterConfig.FilterBank = 0;
    sFilterConfig.FilterMode = CAN_FILTERMODE_IDMASK;
    sFilterConfig.FilterScale = CAN_FILTERSCALE_32BIT;

    // IDs і маски (11-bit) потрібно зсунути <<5 при записі в регістри
    sFilterConfig.FilterIdHigh = (uint16_t)((CAN_ID_CMD << 5) & 0xFFFF);
    sFilterConfig.FilterIdLow  = 0x0000;
    sFilterConfig.FilterMaskIdHigh = (uint16_t)((0x7F0 << 5) & 0xFFFF); // маска для 0x040..0x04F
    sFilterConfig.FilterMaskIdLow  = 0x0000;

    sFilterConfig.FilterFIFOAssignment = CAN_FILTER_FIFO0;
    sFilterConfig.FilterActivation = ENABLE;
#ifdef CAN_HAS_SLAVE_START_BANK
    sFilterConfig.SlaveStartFilterBank = 14;
#endif

    st = HAL_CAN_ConfigFilter(&hcan, &sFilterConfig);
    if (st != HAL_OK) {
        Error_Handler();
    }

    // --- Filter 1: Heartbeats group -> FIFO1 ---
    sFilterConfig.FilterBank = 1;
    sFilterConfig.FilterMode = CAN_FILTERMODE_IDMASK;
    sFilterConfig.FilterScale = CAN_FILTERSCALE_32BIT;
    sFilterConfig.FilterIdHigh = (uint16_t)(((0x080) << 5) & 0xFFFF);
    sFilterConfig.FilterIdLow = 0x0000;
    sFilterConfig.FilterMaskIdHigh = (uint16_t)(((0x7F0) << 5) & 0xFFFF);
    sFilterConfig.FilterMaskIdLow = 0x0000;
    sFilterConfig.FilterFIFOAssignment = CAN_FILTER_FIFO1;
    sFilterConfig.FilterActivation = ENABLE;

    st = HAL_CAN_ConfigFilter(&hcan, &sFilterConfig);
    if (st != HAL_OK) {
        Error_Handler();
    }

}
