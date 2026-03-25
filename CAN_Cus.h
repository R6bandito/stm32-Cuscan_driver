#ifndef __CAN_CUS_H__
#define __CAN_CUS_H__


/* ******************************************* */
  /* 
    用户必须在使用本库前包含正确的 STM32 头文件.
    - 对于 F1 系列： #include "stm32f1xx_hal.h".
    - 对于 F4 系列： #include "stm32f4xx_hal.h".
    - 已经给出了 F1 与 F4 系列的实现,若芯片属于其它型号,请自行包含其相关的HAL库文件.
  */
  #include "stm32f1xx_hal.h"    
      #define STM32F1xx

  // #include "stm32f4xx_hal.h"
      // #define STM32F4xx

  // ....
  // #define STM32ADVANCE             // 注意：若芯片为其它(高于F4系列)时，请在自行添加相关hal头文件后，启用该宏并注释掉
                                          // #include "stm32f1xx_hal.h" 与 #define STM32F1xx

/* ******************************************* */

#include <stdbool.h>
#include <stdlib.h>
#include <string.h>


/* *************** Define ****************** */
  #define MAX_SUPPORT_CANDEV             (3)
  #define CAN1_INDEX                     (0)
  #define CAN2_INDEX                     (1)
  #define CAN3_INDEX                     (2)

/* ***************************************** */


/* ******************************************* */
typedef enum 
{
  MODE_NORMAL,
  MODE_LOOPBACK,
  MODE_SILENT,
  MODE_SILENT_LPBACK

} Cus_CAN_Mode_t;



typedef enum
{
  CAN_BAUDRATE_125K,
  CAN_BAUDRATE_250K,
  CAN_BAUDRATE_500K,
  CAN_BAUDRATE_1M

} Cus_CAN_Baudrate_t;


typedef enum
{
  Cus_CAN_FILTERMODE_IDMASK,
  Cus_CAN_FILTERMODE_IDLIST

} Cus_CANFilter_Mode_t;


typedef enum
{
  Cus_CAN_SCALE_16BIT,
  Cus_CAN_SCALE_32BIT

} Cus_CANFilter_Scale_t;


typedef enum
{
  Cus_CAN_FIFOASSIGNMENT_FIFO0,
  Cus_CAN_FIFOASSIGNMENT_FIFO1

} Cus_CANFIFOASS_t;


typedef enum
{
  Cus_CAN_Enable,
  Cus_CAN_Disable

} Cus_CAN_Enb_t;


typedef enum 
{
  Cus_CAN_SJW_1Tq,
  Cus_CAN_SJW_2Tq,
  Cus_CAN_SJW_3Tq,
  Cus_CAN_SJW_4Tq

} Cus_CAN_SJW_t;


typedef struct 
{
  GPIO_TypeDef *CAN_GPIOPort_x;
  uint16_t CAN_GPIO_RX; 
  uint16_t CAN_GPIO_TX;
  uint8_t Alternate;   /* 仅 F4 等系列使用，F1 忽略 */

} Cus_CAN_GPIO_t;


typedef struct CANInitConfig_t CANInitConfig_t;
struct CANInitConfig_t
{
  CAN_TypeDef *Instance;
  Cus_CAN_Mode_t Mode;
  Cus_CAN_Baudrate_t baudrate;
  Cus_CAN_GPIO_t CAN_gpio;
  Cus_CAN_SJW_t SJW;
  bool is_AutoRestransmission;
  bool is_AutoWakeUP;
  bool is_ReceiveFifoLocked;
  bool is_TimeTriggeredMode;
  bool is_TransmitFifoPriority;
  bool is_AutoBusOff;
  bool is_DynamicAlloc;

  HAL_StatusTypeDef (*Cus_CAN_Init)( const CANInitConfig_t *pConf_Structure );
  void (*Self_Release)( CANInitConfig_t **pConf );
};


typedef struct CANFilterConfig_t CANFilterConfig_t;
struct CANFilterConfig_t
{
  Cus_CANFilter_Mode_t Mode;
  Cus_CANFilter_Scale_t Scale;
  Cus_CANFIFOASS_t FIFOAssignment;
  uint8_t FilterBank;
  uint32_t  IdHigh;              
  uint32_t  IdLow;               
  uint32_t  MaskIdHigh;          
  uint32_t  MaskIdLow;       
  Cus_CAN_Enb_t is_Activation;
  bool is_DynamicAlloc;

  HAL_StatusTypeDef (*Cus_CAN_FilterInit)( const CANFilterConfig_t *pFilterConf, CAN_TypeDef *instance );
  void (*Self_Release)( CANFilterConfig_t **pConf );
};


typedef struct Cus_CAN_Device Cus_CAN_Device_t;
struct Cus_CAN_Device
{
  CAN_TypeDef *Instance;
  CAN_HandleTypeDef *canHandle;
  HAL_StatusTypeDef (*Send)(Cus_CAN_Device_t *pDev, CAN_TxHeaderTypeDef Txheader, uint8_t *Send_Buf);

};

/* ******************************************* */



/* ----------------------------------------------------------------- */
uint8_t Factory_CANInitConfig_t( CANInitConfig_t **pOutConfig );
uint8_t Factory_CANFilterConfig_t( CANFilterConfig_t **pOutConfig );
CAN_HandleTypeDef *Cus_CAN_getHandle( CAN_TypeDef *instance );
const Cus_CAN_Device_t *Cus_CAN_getControlBlock( CAN_TypeDef *instance );
HAL_StatusTypeDef Cus_CAN_Start( CAN_TypeDef *instance );


/* ----------------------------------------------------------------- */


/* ----------------------------------------------------------------- */
__weak void Cus_FilterConfigFailed_Hook( CAN_HandleTypeDef *hcan, const CANFilterConfig_t *pFilterConf, HAL_StatusTypeDef hal_status );
__weak void Cus_CANInitFailed_Hook( CAN_HandleTypeDef *hcan, const CANInitConfig_t *pInitConf, HAL_StatusTypeDef hal_status );
__weak void Cus_CANStartFailed_Hook( CAN_HandleTypeDef *hcan, HAL_StatusTypeDef hal_status );
__weak void Cus_CANSendFailed_Hook( Cus_CAN_Device_t *pDev, HAL_StatusTypeDef hal_status );



/* ----------------------------------------------------------------- */



#endif // __CAN_CUS_H__
