/*
 * can_gateway.h
 *
 *  Created on: Nov 21, 2025
 *      Author: sydor
 */

#ifndef INC_CAN_GATEWAY_H_
#define INC_CAN_GATEWAY_H_

#include "stm32f1xx_hal.h"

void CAN_GW_Init(CAN_HandleTypeDef *hcanA, CAN_HandleTypeDef *hcanB);
void CAN_GW_ProcessA(void);   // A → B
void CAN_GW_ProcessB(void);   // B → A (якщо треба)



#endif /* INC_CAN_GATEWAY_H_ */
