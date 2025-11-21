/*
 * can_network.h
 *
 *  Created on: Nov 17, 2025
 *      Author: Cerberus
 */

#ifndef CAN_NETWORK_H
#define CAN_NETWORK_H

#include "stm32f1xx_hal.h"
#include <stdint.h>
#include <stdbool.h>

/* ============================================
 * CAN ID MAP (11-bit Standard IDs)
 * ============================================ */
#define CAN_ID_CMD          0x040  // Команди (найвищий пріоритет)
#define CAN_ID_ACK_ERR      0x041  // ACK/ERR відповіді
#define CAN_ID_HEARTBEAT_A  0x081  // Node A HB
#define CAN_ID_HEARTBEAT_B  0x082  // Node B HB
#define CAN_ID_HEARTBEAT_C  0x083  // Node C HB
#define CAN_ID_TLM_FAST     0x120  // Телеметрія швидка (Node A)
#define CAN_ID_TLM_SLOW     0x180  // Телеметрія повільна (Node B)

/* ============================================
 * Node IDs
 * ============================================ */
typedef enum {
    NODE_A = 0x01,  // GPS/TLM_fast
    NODE_B = 0x02,  // ENV/TLM_slow
    NODE_C = 0x03   // CTRL/CMD
} NodeID_t;

/* ============================================
 * Command IDs (для CAN_ID_CMD)
 * ============================================ */
typedef enum {
    CMD_LED_ON      = 0x10,
    CMD_LED_OFF     = 0x11,
    CMD_SET_MODE    = 0x20,
    CMD_REQUEST_TLM = 0x30,
    CMD_RESET       = 0xFF
} CommandID_t;

/* ============================================
 * CAN Message Structures
 * ============================================ */

// Структура команди (8 байт)
typedef struct __attribute__((packed)) {
    uint8_t cmd_id;      // CommandID_t
    uint8_t arg0;
    uint8_t arg1;
    uint8_t arg2;
    uint8_t token;       // Простий токен для захисту
    uint8_t seq;         // Sequence number
    uint8_t crc8;        // CRC для перевірки
    uint8_t version;     // Версія протоколу
} CANCommand_t;

// Структура ACK/ERR (8 байт)
typedef struct __attribute__((packed)) {
    uint8_t cmd_id;      // Повторюємо CommandID
    uint8_t seq;         // Sequence number
    uint8_t result;      // 0=OK, 1=Error
    uint8_t detail;      // Деталі помилки
    uint8_t reserved[4];
} CANACK_t;

// Структура Heartbeat (8 байт)
typedef struct __attribute__((packed)) {
    uint8_t node_id;     // NodeID_t
    uint8_t status;      // 0=OK, 1=Warning, 2=Error
    uint16_t uptime_sec; // Час роботи (секунди)
    uint16_t can_tx_cnt; // Лічильник відправлених
    uint16_t can_rx_cnt; // Лічильник отриманих
} CANHeartbeat_t;

// Структура TLM Fast (Node A) - 8 байт
// GPS: lat(24bit)*1e5 | lon(24bit)*1e5 | sats(8b) | fix/hdop(8b)
typedef struct __attribute__((packed)) {
    int32_t lat_e5;      // Широта * 100000 (3 байти використовуються)
    int32_t lon_e5;      // Довгота * 100000 (3 байти)
    uint8_t satellites;
    uint8_t fix_quality;
} CANTelemetryFast_t;

// Структура TLM Slow (Node B) - 8 байт
// ENV: T(Q9.7) | P(Q13.3) | RH(%*2) | lux(16b)
typedef struct __attribute__((packed)) {
    int16_t temp_q97;    // Температура Q9.7 (°C * 128)
    uint16_t press_q133; // Тиск Q13.3 (hPa * 8)
    uint8_t humidity_x2; // Вологість * 2 (%)
    uint8_t reserved;
    uint16_t lux;        // Освітленість (люкси)
} CANTelemetrySlow_t;

/* ============================================
 * Utility Functions
 * ============================================ */

// CRC8 для перевірки цілісності
static inline uint8_t CAN_CalcCRC8(uint8_t *data, uint8_t len) {
    uint8_t crc = 0xFF;
    for (uint8_t i = 0; i < len; i++) {
        crc ^= data[i];
        for (uint8_t j = 0; j < 8; j++) {
            if (crc & 0x80)
                crc = (crc << 1) ^ 0x07;
            else
                crc <<= 1;
        }
    }
    return crc;
}

// Перевірка токену
#define VALID_TOKEN  0xA5
static inline bool CAN_ValidateToken(uint8_t token) {
    return (token == VALID_TOKEN);
}

#define PROTOCOL_VERSION  0x01

#endif /* INC_CAN_NETWORK_H_ */
