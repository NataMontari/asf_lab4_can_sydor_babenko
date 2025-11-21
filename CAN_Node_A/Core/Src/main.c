  /*****************************************************************************
  * NODE A: Відправляє швидку телеметрію (GPS) кожні 100 мс (10 Hz)
  *         та Heartbeat кожну секунду (1 Hz)
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include <string.h>
#include <stdio.h>

/* Private variables ---------------------------------------------------------*/
extern CAN_HandleTypeDef hcan;
extern TIM_HandleTypeDef htim2;

/* USER CODE BEGIN PV */
// CAN Message IDs
#define CAN_ID_TLM_FAST    0x120  // Telemetry Fast (GPS) - пріоритет високий
#define CAN_ID_HEARTBEAT   0x080  // Heartbeat Base ID
#define CAN_ID_CMD         0x040  // Commands (приймання)
#define CAN_ID_ACK         0x041  // ACK/ERR (приймання)

#define NODE_ID            0x01   // Node A ID

// Timing counters
volatile uint32_t tick_counter = 0;
volatile uint8_t flag_100ms = 0;  // 10 Hz для TLM_fast
volatile uint8_t flag_1s = 0;     // 1 Hz для Heartbeat

// GPS симуляція
typedef struct {
    int32_t latitude;   // *1e5 (наприклад, 50.45 → 5045000)
    int32_t longitude;  // *1e5
    uint8_t satellites;
    uint8_t fix_quality; // 0=no fix, 1=GPS, 2=DGPS
} GPS_Data_t;

GPS_Data_t gps_data = {
    .latitude = 4976000,   // 49.76° (Львів приблизно)
    .longitude = 2401000,  // 24.01°
    .satellites = 8,
    .fix_quality = 1
};

// Heartbeat data
typedef struct {
    uint8_t node_id;
    uint8_t status;     // 0=OK, 1=Warning, 2=Error
    uint16_t uptime_s;  // Uptime у секундах
    uint16_t can_tx_count;
    uint16_t can_rx_count;
} Heartbeat_t;

Heartbeat_t hb_data = {0};

// CAN Statistics
uint32_t can_tx_count = 0;
uint32_t can_rx_count = 0;
uint32_t can_errors = 0;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_CAN_Init(void);
static void MX_TIM2_Init(void);
static void MX_USART1_UART_Init(void);

/* USER CODE BEGIN PFP */
void CAN_Config_Filter(void);
void CAN_Send_TLM_Fast(void);
void CAN_Send_Heartbeat(void);
void CAN_Process_Received(CAN_RxHeaderTypeDef *rxHeader, uint8_t *rxData);
uint8_t CRC8_Calculate(uint8_t *data, uint8_t len);
void LED_Toggle(void);
void LED_Blink_Fast(uint8_t count);

#ifdef USE_UART_DEBUG
void Debug_Printf(const char *fmt, ...);
#endif
/* USER CODE END PFP */

/**
  * @brief  The application entry point.
  */
int main(void)
{
  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/
  HAL_Init();
  SystemClock_Config();

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_CAN_Init();
  MX_TIM2_Init();
  #ifdef USE_UART_DEBUG
  MX_USART1_UART_Init();
  #endif

  /* USER CODE BEGIN 2 */

  // CAN Filter Configuration
  CAN_Config_Filter();

  // Start CAN
  if (HAL_CAN_Start(&hcan) != HAL_OK) {
      Error_Handler();
  }

  /* Активуємо повідомлення для FIFO0 (CMD/ACK) і FIFO1 (HB/діагностика) */
  if (HAL_CAN_ActivateNotification(&hcan,
          CAN_IT_RX_FIFO0_MSG_PENDING | CAN_IT_RX_FIFO1_MSG_PENDING) != HAL_OK) {
      Error_Handler();
  }


  // Start Timer for periodic tasks
  HAL_TIM_Base_Start_IT(&htim2);

  // Initialize Heartbeat data
  hb_data.node_id = NODE_ID;
  hb_data.status = 0; // OK
  hb_data.uptime_s = 0;

  // LED startup blink
  LED_Blink_Fast(3);

  #ifdef USE_UART_DEBUG
  Debug_Printf("\r\n=== CAN Node A Started ===\r\n");
  Debug_Printf("ID: 0x%03X (TLM_fast)\r\n", CAN_ID_TLM_FAST);
  #endif

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */

    // Обробка періодичних завдань по флагам з таймера

    if (flag_100ms) {
        flag_100ms = 0;

        // Відправка швидкої телеметрії (GPS) @ 10 Hz
        CAN_Send_TLM_Fast();

        // Симуляція зміни GPS даних (у реальності - читання з модуля)
        gps_data.latitude += (rand() % 10 - 5);   // ±5
        gps_data.longitude += (rand() % 10 - 5);

        LED_Toggle(); // Індикація активності
    }

    if (flag_1s) {
        flag_1s = 0;

        // Відправка Heartbeat @ 1 Hz
        CAN_Send_Heartbeat();

        hb_data.uptime_s++;

        #ifdef USE_UART_DEBUG
        Debug_Printf("Uptime: %u s | TX: %lu | RX: %lu | ERR: %lu\r\n",
                     hb_data.uptime_s, can_tx_count, can_rx_count, can_errors);
        #endif
    }

    // Можна додати sleep mode для економії енергії, поки коментуємо
    // HAL_PWR_EnterSLEEPMode(PWR_MAINREGULATOR_ON, PWR_SLEEPENTRY_WFI);
  }
  /* USER CODE END 3 */
}

/* USER CODE BEGIN 4 */

//void CAN_Config_Filter(void)
//{
//    CAN_FilterTypeDef canFilter;
//
//    // Filter 0: приймаємо команди (0x040) та ACK (0x041)
//    canFilter.FilterBank = 0;
//    canFilter.FilterMode = CAN_FILTERMODE_IDLIST; // List mode
//    canFilter.FilterScale = CAN_FILTERSCALE_32BIT;
//    canFilter.FilterIdHigh = (CAN_ID_CMD << 5);   // 0x040
//    canFilter.FilterIdLow = (CAN_ID_ACK << 5);    // 0x041
//    canFilter.FilterMaskIdHigh = 0x0000;
//    canFilter.FilterMaskIdLow = 0x0000;
//    canFilter.FilterFIFOAssignment = CAN_RX_FIFO0;
//    canFilter.FilterActivation = ENABLE;
//    canFilter.SlaveStartFilterBank = 14;
//
//    if (HAL_CAN_ConfigFilter(&hcan, &canFilter) != HAL_OK) {
//        Error_Handler();
//    }
//}

void CAN_Send_TLM_Fast(void)
{
    CAN_TxHeaderTypeDef txHeader;
    uint8_t txData[8];
    uint32_t txMailbox;

    txHeader.StdId = CAN_ID_TLM_FAST;
    txHeader.ExtId = 0;
    txHeader.RTR = CAN_RTR_DATA;
    txHeader.IDE = CAN_ID_STD;
    txHeader.DLC = 8;
    txHeader.TransmitGlobalTime = DISABLE;

    // Пакування даних: lat(3B) | lon(3B) | sats(1B) | fix(1B)
    txData[0] = (gps_data.latitude >> 16) & 0xFF;
    txData[1] = (gps_data.latitude >> 8) & 0xFF;
    txData[2] = gps_data.latitude & 0xFF;

    txData[3] = (gps_data.longitude >> 16) & 0xFF;
    txData[4] = (gps_data.longitude >> 8) & 0xFF;
    txData[5] = gps_data.longitude & 0xFF;

    txData[6] = gps_data.satellites;
    txData[7] = gps_data.fix_quality;

    // Відправка
    if (HAL_CAN_AddTxMessage(&hcan, &txHeader, txData, &txMailbox) == HAL_OK) {
        can_tx_count++;
    } else {
        can_errors++;
    }
}

void CAN_Send_Heartbeat(void)
{
    CAN_TxHeaderTypeDef txHeader;
    uint8_t txData[8];
    uint32_t txMailbox;

    txHeader.StdId = CAN_ID_HEARTBEAT + NODE_ID;
    txHeader.ExtId = 0;
    txHeader.RTR = CAN_RTR_DATA;
    txHeader.IDE = CAN_ID_STD;
    txHeader.DLC = 8;
    txHeader.TransmitGlobalTime = DISABLE;

    hb_data.can_tx_count = (uint16_t)can_tx_count;
    hb_data.can_rx_count = (uint16_t)can_rx_count;

    txData[0] = hb_data.node_id;
    txData[1] = hb_data.status;
    txData[2] = (hb_data.uptime_s >> 8) & 0xFF;
    txData[3] = hb_data.uptime_s & 0xFF;
    txData[4] = (hb_data.can_tx_count >> 8) & 0xFF;
    txData[5] = hb_data.can_tx_count & 0xFF;
    txData[6] = (hb_data.can_rx_count >> 8) & 0xFF;
    txData[7] = hb_data.can_rx_count & 0xFF;

    if (HAL_CAN_AddTxMessage(&hcan, &txHeader, txData, &txMailbox) == HAL_OK) {
        can_tx_count++;
    }
}

void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan_ptr)
{
    CAN_RxHeaderTypeDef rxHeader;
    uint8_t rxData[8];

    if (HAL_CAN_GetRxMessage(hcan_ptr, CAN_RX_FIFO0, &rxHeader, rxData) == HAL_OK) {
        can_rx_count++;
        CAN_Process_Received(&rxHeader, rxData);
    }
}


void CAN_Process_Received(CAN_RxHeaderTypeDef *rxHeader, uint8_t *rxData)
{
    uint32_t msgID = rxHeader->StdId;

    if (msgID == CAN_ID_CMD) {
        // Прийом команди (0x040)
        // Format: CmdID | Arg0 | Arg1 | Arg2 | Token | Seq | CRC8 | Ver

        uint8_t cmdID = rxData[0];
        uint8_t token = rxData[4];
        uint8_t seq = rxData[5];
        uint8_t crc_rx = rxData[6];

        // Перевірка CRC
        uint8_t crc_calc = CRC8_Calculate(rxData, 6);

        if (crc_calc == crc_rx) {
            // CRC OK - обробка команди
            #ifdef USE_UART_DEBUG
            Debug_Printf("CMD RX: ID=0x%02X, Seq=%u, Token=0x%02X [OK]\r\n",
                         cmdID, seq, token);
            #endif

            switch(cmdID) {
                case 0x01: // Reset counters
                    can_tx_count = 0;
                    can_rx_count = 0;
                    can_errors = 0;
                    LED_Blink_Fast(2);
                    break;

                case 0x02: // Change GPS simulation speed
                    // Arg0 = speed multiplier
                    break;

                default:
                    break;
            }
        } else {
            // CRC Error
            can_errors++;
            #ifdef USE_UART_DEBUG
            Debug_Printf("CMD RX: CRC ERROR (calc=0x%02X, rx=0x%02X)\r\n",
                         crc_calc, crc_rx);
            #endif
        }
    }
}


uint8_t CRC8_Calculate(uint8_t *data, uint8_t len)
{
    uint8_t crc = 0x00;

    for (uint8_t i = 0; i < len; i++) {
        crc ^= data[i];
        for (uint8_t j = 0; j < 8; j++) {
            if (crc & 0x80) {
                crc = (crc << 1) ^ 0x07;
            } else {
                crc <<= 1;
            }
        }
    }

    return crc;
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
    if (htim->Instance == TIM2) {
        tick_counter++;

        // 100 ms flag (10 Hz)
        if (tick_counter % 1 == 0) { // TIM2 налаштований на 100 мс
            flag_100ms = 1;
        }

        // 1 second flag
        if (tick_counter % 10 == 0) {
            flag_1s = 1;
        }
    }
}

// Може і не треба
void LED_Toggle(void)
{
    HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_13);
}


void LED_Blink_Fast(uint8_t count)
{
    for (uint8_t i = 0; i < count; i++) {
        HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_RESET);
        HAL_Delay(50);
        HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_SET);
        HAL_Delay(50);
    }
}

#ifdef USE_UART_DEBUG
/**
  * @brief  Debug printf через UART
  */
void Debug_Printf(const char *fmt, ...)
{
    char buffer[128];
    va_list args;
    va_start(args, fmt);
    vsnprintf(buffer, sizeof(buffer), fmt, args);
    va_end(args);

    HAL_UART_Transmit(&huart1, (uint8_t*)buffer, strlen(buffer), HAL_MAX_DELAY);
}
#endif

// ================================================================
//                     1 — REPLAY ATTACK
//   Повторна відправка тієї ж команди з тими ж аргументами.
//   Token/Seq/CRC8 повторяться, бо генеруються у CAN_Send_Command.
// ================================================================
void Attack_Replay(uint8_t cmdID, uint8_t arg0, uint8_t arg1, uint8_t arg2)
{
    CAN_Send_Command(cmdID, arg0, arg1, arg2);
    HAL_Delay(5);
    CAN_Send_Command(cmdID, arg0, arg1, arg2);   // повтор
}



// ================================================================
//                    2 — SPOOFING ATTACK
//       2.1. Підробка Heartbeat від іншого вузла
// ================================================================
void Attack_Spoof_Heartbeat(uint8_t fake_node_id)
{
    uint16_t fake_id = 0x080 + fake_node_id;  // ID Heartbeat чужого вузла

    CAN_TxHeaderTypeDef h;
    h.StdId = fake_id;
    h.IDE   = CAN_ID_STD;
    h.RTR   = CAN_RTR_DATA;
    h.DLC   = 1;

    uint8_t payload = 0x55;

    uint32_t mb;
    HAL_CAN_AddTxMessage(&hcan, &h, &payload, &mb);
}



// ================================================================
//                   2.2. Підробка керуючої команди
// ================================================================
void Attack_Spoof_Command(void)
{
    uint8_t fake_cmd  = 0xAB;  // фальшива команда
    uint8_t a0 = 0x11;
    uint8_t a1 = 0x22;
    uint8_t a2 = 0x33;

    CAN_Send_Command(fake_cmd, a0, a1, a2);
}



// ================================================================
//                  3 — TAMPERING / MODIFY ATTACK
//    Зловмисник підміняє один аргумент на шкідливий (0xFF).
// ================================================================
void Attack_Modify_Command(uint8_t cmdID, uint8_t real_a0, uint8_t real_a1)
{
    uint8_t malicious_a2 = 0xFF;   // підміна

    CAN_Send_Command(cmdID, real_a0, real_a1, malicious_a2);
}



// ================================================================
//                  4 — FLOODING (DoS)
//    Засипаємо шину кадрами, блокуючи нормальний трафік
// ================================================================
void Attack_Flood(void)
{
    for (int i = 0; i < 200; i++)
    {
        CAN_Send_Heartbeat();
        HAL_Delay(1);
    }
}



// ================================================================
//                5 — ПІДРОБКА ACK (маскування помилки)
// ================================================================
void Attack_Fake_ACK(uint8_t seq)
{
    CAN_Send_ACK(seq, 0x00); // ACK без помилок
}



// ================================================================
//                  6 — SUPPRESSION ATTACK
//    Глушіння: відправляємо високопріоритетні команди без пауз
// ================================================================
void Attack_Suppress_Node(void)
{
    for (int i = 0; i < 50; i++)
    {
        CAN_Send_Command(0xFF, 0x00, 0x00, 0x00);
    }
}



// ================================================================
//                  Attack Mode Handler
//     Можна залишити як є або коментувати окремі атаки
// ================================================================
void Attack_Mode_Handler(void)
{
    HAL_Delay(3000); // Чекаємо поки вузли піднімуться

    // 1 — Replay
    Attack_Replay(0x01, 0x10, 0x20, 0x30);
    HAL_Delay(300);

    // 2 — Spoof heartbeat
    Attack_Spoof_Heartbeat(2);
    HAL_Delay(300);

    // 3 — Tampering
    Attack_Modify_Command(0x02, 0x44, 0x55);
    HAL_Delay(300);

    // 4 — Flood
    Attack_Flood();
    HAL_Delay(300);

    // 5 — Fake ACK
    Attack_Fake_ACK(5);
    HAL_Delay(300);

    // 6 — Suppression
    Attack_Suppress_Node();
}
/* USER CODE END 4 */

/**
  * @brief System Clock Configuration
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
}

static void MX_CAN_Init(void)
{
  hcan.Instance = CAN1;
  hcan.Init.Prescaler = 9;  // 500 kbit/s @ 36 MHz APB1
  hcan.Init.Mode = CAN_MODE_NORMAL;
  hcan.Init.SyncJumpWidth = CAN_SJW_1TQ;
  hcan.Init.TimeSeg1 = CAN_BS1_6TQ;
  hcan.Init.TimeSeg2 = CAN_BS2_1TQ;
  hcan.Init.TimeTriggeredMode = DISABLE;
  hcan.Init.AutoBusOff = ENABLE;
  hcan.Init.AutoWakeUp = ENABLE;
  hcan.Init.AutoRetransmission = ENABLE;
  hcan.Init.ReceiveFifoLocked = DISABLE;
  hcan.Init.TransmitFifoPriority = DISABLE;

  if (HAL_CAN_Init(&hcan) != HAL_OK)
  {
    Error_Handler();
  }
}

static void MX_TIM2_Init(void)
{
  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 7199;      // 72 MHz / 7200 = 10 kHz
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 999;          // 10 kHz / 1000 = 10 Hz (100 ms)
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;

  if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }

  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }

  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
}


static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_SET);

  GPIO_InitStruct.Pin = GPIO_PIN_13;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);
}

void Error_Handler(void)
{
  __disable_irq();
  while (1)
  {
    HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_13);
    HAL_Delay(100);
  }
}

#ifdef  USE_FULL_ASSERT
void assert_failed(uint8_t *file, uint32_t line)
{
}
#endif
