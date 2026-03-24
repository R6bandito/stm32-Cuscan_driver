#include "CAN_Cus.h"


/* ------------------------ g_Ver & Static Ver -------------------------------- */
CAN_HandleTypeDef canHandle[MAX_SUPPORT_CANDEV];

/* ---------------------------------------------------------------------------- */


/* ----------------------------------------------------------------- */
uint8_t Factory_CANInitConfig_t( CANInitConfig_t **pOutConfig );
uint8_t Factory_CANFilterConfig_t( CANFilterConfig_t **pOutConfig );
CAN_HandleTypeDef *Cus_CAN_getHandle( CAN_TypeDef *instance );
static HAL_StatusTypeDef cus_canInit( const CANInitConfig_t * pConf_Structure );
static HAL_StatusTypeDef cus_canfilterInit( const CANFilterConfig_t * pConf_Structure );
static void __release( CANInitConfig_t **pConf );
static void __release_filter( CANFilterConfig_t **pFilter );
static HAL_StatusTypeDef CAN_GetTimingFromBaudrate(Cus_CAN_Baudrate_t Baudrate, uint32_t *prescaler, uint32_t *pbs1, uint32_t *pbs2);

/* ----------------------------------------------------------------- */


uint8_t Factory_CANInitConfig_t( CANInitConfig_t **pOutConfig )
{
  #warning "The param pOutConfig Must be global ver.Otherwise you will lose the memory that created by DynamicAlloc."
  /* 注意: 传入的 pOutConfig 参数必须是对应调用层的全局变量.若为局部变量，则会丢失通过动态分配的内存从而造成内存泄漏！ */

  if ( !pOutConfig )  return 0xFF;

  CANInitConfig_t *pReturn = (CANInitConfig_t *)malloc(sizeof(CANInitConfig_t));
  if ( pReturn == NULL )  return 0xFF;
  *pOutConfig = pReturn;

  memset(*pOutConfig, 0, sizeof(CANInitConfig_t));
  (**pOutConfig).Cus_CAN_Init = cus_canInit;
  (**pOutConfig).Self_Release = __release;
  (**pOutConfig).is_DynamicAlloc = true;
  return 0;
}


uint8_t Factory_CANFilterConfig_t( CANFilterConfig_t **pOutConfig )
{
  if ( !pOutConfig )  return 0xFF;

  CANFilterConfig_t *pReturn = (CANFilterConfig_t *)malloc(sizeof(CANFilterConfig_t));
  if ( pReturn == NULL )  return 0xFF;
  *pOutConfig = pReturn;

  memset(*pOutConfig, 0, sizeof(CANFilterConfig_t));
  (*pOutConfig)->Cus_CAN_FilterInit = cus_canfilterInit;
  (*pOutConfig)->Self_Release = __release_filter;
  (*pOutConfig)->is_DynamicAlloc = true;
  return 0;
}


static void __release( CANInitConfig_t **pConf )
{
  if ( !pConf || !(*pConf) ) return;
  if ( (*pConf)->is_DynamicAlloc != true ) return;
  free(*pConf);
  *pConf = NULL;
}


static void __release_filter( CANFilterConfig_t **pFilter )
{
  if ( !pFilter || !(*pFilter) )  return;
  if ( (*pFilter)->is_DynamicAlloc != true )  return;
  free(*pFilter);
  *pFilter = NULL;
}



static HAL_StatusTypeDef CAN_GetTimingFromBaudrate(Cus_CAN_Baudrate_t Baudrate, uint32_t *prescaler, uint32_t *pbs1, uint32_t *pbs2)
{
  if ( !prescaler || !pbs1 || !pbs2 )   return HAL_ERROR;

  #warning "Please make sure that your CAN_Controler is belonged to APB1 BUS. If not you should Switch the Defines button and that will switch to APB3"
  #if 1
    uint32_t pclk_clock = HAL_RCC_GetPCLK1Freq();
  #else 
    uint32_t pclk_clock = HAL_RCC_GetPCLK3Freq();
  #endif 

  uint32_t baudrate;
  switch(Baudrate)
  {
    case CAN_BAUDRATE_125K:  baudrate = (125000UL); break;
    case CAN_BAUDRATE_250K:  baudrate = (250000UL); break;
    case CAN_BAUDRATE_500K:  baudrate = (500000UL); break;
    case CAN_BAUDRATE_1M:    baudrate = (1000000UL); break;
  }

  uint32_t target_bit_time_ns = (1000000000UL) / baudrate;
  uint32_t best_error = UINT32_MAX;
  uint32_t best_prescaler = 0, best_pbs1 = 0, best_pbs2 = 0;

  for( uint16_t prescaler = 1; prescaler < 256; prescaler++ )
  {
    uint32_t can_clock = pclk_clock / prescaler;        // APB BUS分频后，CAN外设的实际工作频率.
    uint32_t tq_ns = (1000000000UL) / can_clock;        // 1个tq的时间 换算为ns 避免浮点运算.

    uint32_t min_tq = 8, max_tq = 25;
    uint32_t tq_num = target_bit_time_ns / tq_ns;
    if ( tq_num < min_tq || tq_num > max_tq )  continue;  // 确保一个位中的tq数符合CAN规范(8~25).

    /* 尝试分配 BS1 和 BS2，使采样点约 75% */
    uint32_t bs1 = tq_num * ( 3 / 4 );
    uint32_t bs2 = tq_num - bs1 - 1;                  // Ps: PTS传播段已被硬件整合进 PBS1 段中.
    if ( bs1 < 1 || bs1 > 16 )   continue;
    if ( bs2 < 1 || bs2 > 8 )   continue;

    uint32_t actual_bit_time_ns = tq_ns * ( 1 + bs1 + bs2 );
    uint32_t error = ( actual_bit_time_ns > target_bit_time_ns ) ? (actual_bit_time_ns - target_bit_time_ns) : (target_bit_time_ns - actual_bit_time_ns);
    if ( error < best_error )
    {
      best_error = error;
      best_prescaler = prescaler;
      best_pbs1 = bs1;
      best_pbs2 = bs2;

      if ( error == 0 )   break;              // 精准匹配. 已是最优解，直接退出循环.
    }
  }

  if ( best_prescaler == 0 )  return HAL_ERROR;

  *prescaler = best_prescaler;
  *pbs1 = best_pbs1;
  *pbs2 = best_pbs2;

  return HAL_OK;
}



static HAL_StatusTypeDef cus_canInit( const CANInitConfig_t * pConf_Structure )
{
  if ( !pConf_Structure )   return HAL_ERROR;

  if ( pConf_Structure->Instance == CAN1 )  __HAL_RCC_CAN1_CLK_ENABLE();
  #if defined(CAN2)
    if ( pConf_Structure->Instance == CAN2 )  __HAL_RCC_CAN2_CLK_ENABLE();
  #endif // CAN2
  #if defined(CAN3)
    if ( pConf_Structure->Instance == CAN3 )  __HAL_RCC_CAN3_CLK_ENABLE();
  #endif // CAN3

  GPIO_TypeDef *port = pConf_Structure->CAN_gpio.CAN_GPIOPort_x;
  if ( port == GPIOA )  __HAL_RCC_GPIOA_CLK_ENABLE();
  else if ( port == GPIOB )  __HAL_RCC_GPIOB_CLK_ENABLE();
  else if ( port == GPIOC )  __HAL_RCC_GPIOC_CLK_ENABLE();
  else if ( port == GPIOD )  __HAL_RCC_GPIOD_CLK_ENABLE();
  /* 可根据需要添加其它端口. */
  /* else if ( port == GPIOE )  __HAL_RCC_GPIOE_CLK_ENABLE(); */ 

  GPIO_InitTypeDef gpio = { 0 };
  gpio.Pin = pConf_Structure->CAN_gpio.CAN_GPIO_TX;
  gpio.Mode = GPIO_MODE_AF_PP;
  gpio.Pull = GPIO_PULLUP;
  gpio.Speed = GPIO_SPEED_FREQ_HIGH;
  #if defined(STM32F4xx) || defined(STM32ADVANCE)
    gpio.Alternate = pConf_Structure->CAN_gpio.Alternate;
  #endif // STM32F4xx

  HAL_GPIO_Init(port, &gpio);

  gpio.Pin = pConf_Structure->CAN_gpio.CAN_GPIO_RX;
  gpio.Mode = GPIO_MODE_INPUT;
  gpio.Pull = GPIO_NOPULL;
  #if defined(STM32F4xx) || defined(STM32ADVANCE)
    gpio.Alternate = pConf_Structure->CAN_gpio.Alternate;
  #endif // STM32F4xx
  HAL_GPIO_Init(port, &gpio);

  CAN_HandleTypeDef *hcan;
  if ( pConf_Structure->Instance == CAN1 )
  {
    hcan = &canHandle[0];
  }
  
  #if defined(CAN2)
    if ( pConf_Structure->Instance == CAN2 )
    {
      hcan = &canHandle[1];
    }
  #endif // CAN2

  #if defined(CAN3)
    if ( pConf_Structure->Instance == CAN3 )
    {
      hcan = &canHandle[2];
    }
  #endif // CAN3

  hcan->Instance = pConf_Structure->Instance;
  uint32_t prescaler, pbs1, pbs2;
  if ( CAN_GetTimingFromBaudrate(pConf_Structure->baudrate, &prescaler, &pbs1, &pbs2) != HAL_OK )
  {
    return HAL_ERROR;
  }

  uint32_t SyncJumpWidth = 0;
  switch ( pConf_Structure->SJW )
  {
    case Cus_CAN_SJW_1Tq: SyncJumpWidth = CAN_SJW_1TQ;  break;
    case Cus_CAN_SJW_2Tq: SyncJumpWidth = CAN_SJW_2TQ;  break;
    case Cus_CAN_SJW_3Tq: SyncJumpWidth = CAN_SJW_3TQ;  break;
    case Cus_CAN_SJW_4Tq: SyncJumpWidth = CAN_SJW_4TQ;  break;

    default:  SyncJumpWidth = CAN_SJW_1TQ;
  }

  uint32_t can_Mode = 0;
  switch ( pConf_Structure->Mode )
  {
    case MODE_NORMAL:           can_Mode =  CAN_MODE_NORMAL;   break;
    case MODE_LOOPBACK:         can_Mode =  CAN_MODE_LOOPBACK;   break;
    case MODE_SILENT:           can_Mode =  CAN_MODE_SILENT;   break;
    case MODE_SILENT_LPBACK:    can_Mode =  CAN_MODE_SILENT_LOOPBACK;   break;
  
    default:  can_Mode = CAN_MODE_NORMAL;
  }

  hcan->Init.AutoBusOff = pConf_Structure->is_AutoBusOff ? ENABLE : DISABLE;
  hcan->Init.AutoRetransmission = pConf_Structure->is_AutoRestransmission ? ENABLE : DISABLE;
  hcan->Init.AutoWakeUp = pConf_Structure->is_AutoWakeUP ? ENABLE : DISABLE;
  hcan->Init.ReceiveFifoLocked = pConf_Structure->is_ReceiveFifoLocked ? ENABLE : DISABLE;
  hcan->Init.TimeTriggeredMode = pConf_Structure->is_TimeTriggeredMode ? ENABLE : DISABLE;
  hcan->Init.TransmitFifoPriority = pConf_Structure->is_TransmitFifoPriority ? ENABLE : DISABLE;
  hcan->Init.Prescaler = prescaler;
  hcan->Init.TimeSeg1 = pbs1;
  hcan->Init.TimeSeg2 = pbs2;
  hcan->Init.SyncJumpWidth = SyncJumpWidth;
  hcan->Init.Mode = can_Mode;

  if ( HAL_CAN_Init(hcan) != HAL_OK )    return HAL_ERROR;

  return HAL_OK; 
}



CAN_HandleTypeDef *Cus_CAN_getHandle( CAN_TypeDef *instance )
{
  if ( !instance )   return NULL;
  if ( instance == CAN1 )   return &canHandle[0];

  #if defined(CAN2)
    if ( instance == CAN2 )   return &canHandle[1];
  #endif // CAN2

  #if defined(CAN3)
    if ( instance == CAN3 )   return &canHandle[2];
  #endif // CAN3

  return NULL;
}
