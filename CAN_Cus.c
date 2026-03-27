#include "CAN_Cus.h"


/* ------------------------ g_Ver & Static Ver -------------------------------- */
static CAN_HandleTypeDef canHandle[MAX_SUPPORT_CANDEV] = { 0 };
static Cus_CAN_Device_t *CanDevice[MAX_SUPPORT_CANDEV];
/* ---------------------------------------------------------------------------- */


/* ----------------------------------------------------------------- */
uint8_t Factory_CANInitConfig_t( CANInitConfig_t **pOutConfig );
uint8_t Factory_CANFilterConfig_t( CANFilterConfig_t **pOutConfig );
HAL_StatusTypeDef Cus_CAN_Start( CAN_TypeDef *instance );
CAN_HandleTypeDef *Cus_CAN_getHandle( CAN_TypeDef *instance );
HAL_StatusTypeDef Cus_CAN_getRateInfo( CAN_TypeDef *instance, Cus_CAN_RateInfo_t *pOutInfo );
const Cus_CAN_Device_t *Cus_CAN_getControlBlock( CAN_TypeDef *instance );


void Cus_CAN_Filter_SetStdList32(  CANFilterConfig_t *pFilter, uint8_t Filter_RTR, uint16_t id1, uint16_t id2 );
void Cus_CAN_Filter_SetStdList16( CANFilterConfig_t *pFilter, uint8_t Filter_RTR, uint16_t id1, uint16_t id2, uint16_t id3, uint16_t id4 );
void Cus_CAN_Filter_SetExtList32( CANFilterConfig_t *pFilter, uint8_t Filter_RTR, uint32_t id1, uint32_t id2 );
void Cus_CAN_Filter_SetStdMask32( CANFilterConfig_t *pFilter, uint8_t Filter_RTR, uint16_t id1, uint16_t id1_mask );


static HAL_StatusTypeDef Cus_CAN_Send( Cus_CAN_Device_t *pDev, CAN_TxHeaderTypeDef Txheader, uint8_t *Send_Buf );
static HAL_StatusTypeDef Cus_CAN_Recv( const Cus_CAN_Device_t *pDev, CAN_RxHeaderTypeDef *pHeader, uint8_t *Recv_Buf, uint8_t RxFifoIndex );
static HAL_StatusTypeDef cus_canInit( const CANInitConfig_t *pConf_Structure );
static uint8_t createCANTCB( Cus_CAN_Device_t **pDevice, const CANInitConfig_t *pInit );
static HAL_StatusTypeDef cus_canfilterInit( const CANFilterConfig_t *pConf_Structure, CAN_TypeDef *instance );
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

  /* 注意: 传入的 pOutConfig 参数必须是对应调用层的全局变量.若为局部变量，则会丢失通过动态分配的内存从而造成内存泄漏！ */
  CANFilterConfig_t *pReturn = (CANFilterConfig_t *)malloc(sizeof(CANFilterConfig_t));
  if ( pReturn == NULL )  return 0xFF;
  *pOutConfig = pReturn;

  memset(*pOutConfig, 0, sizeof(CANFilterConfig_t));
  (*pOutConfig)->Cus_CAN_FilterInit = cus_canfilterInit;
  (*pOutConfig)->Self_Release = __release_filter;
  (*pOutConfig)->is_DynamicAlloc = true;
  return 0;
}




/* -------------------------------------- CAN_TCB Relevant ----------------------------------------------------- */
static uint8_t createCANTCB( Cus_CAN_Device_t ** pDevice, const CANInitConfig_t *pInit )
{
//  if ( !pDevice || !(*pDevice) )    return 0xFF;
  if ( !pInit )   return 0xFF;

  Cus_CAN_Device_t *pTCB = (Cus_CAN_Device_t *)malloc(sizeof(Cus_CAN_Device_t));
  if ( pTCB == NULL )   return 0xFF;
  *pDevice = pTCB;

  memset(*pDevice, 0, sizeof(Cus_CAN_Device_t));
  (*pDevice)->Instance = pInit->Instance;
  (*pDevice)->canHandle = Cus_CAN_getHandle(pInit->Instance);
  (*pDevice)->Send = Cus_CAN_Send;
  (*pDevice)->Receive = Cus_CAN_Recv;

  // 控制TCB不允许人为清理. 不注册清理函数.
  return 0;
}

static void __canTCB_release( Cus_CAN_Device_t ** pDevice )
{
  if ( !pDevice || !(*pDevice) )   return;
  free(*pDevice);
  *pDevice = NULL;
}


const Cus_CAN_Device_t *Cus_CAN_getControlBlock( CAN_TypeDef *instance )
{
  if ( !instance )  return NULL;

  int idx = -1;
  if ( instance == CAN1 )  idx = CAN1_INDEX;
  #if defined(CAN2)
    if ( instance == CAN2 ) idx = CAN2_INDEX;
  #endif // CAN2
  #if defined(CAN3)
    if ( instance == CAN3 ) idx = CAN3_INDEX;
  #endif // CAN3

  if ( idx < 0 || idx > MAX_SUPPORT_CANDEV )  return NULL;

  return (const Cus_CAN_Device_t *)CanDevice[idx];
}
/* --------------------------------------------------------------------------------------------------- */




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

  uint32_t pclk_clock = HAL_RCC_GetPCLK1Freq();

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
    uint32_t bs1 = ( tq_num *  3 ) / 4 ;
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
    hcan = &canHandle[CAN1_INDEX];
  }
  
  #if defined(CAN2)
    if ( pConf_Structure->Instance == CAN2 )
    {
      hcan = &canHandle[CAN2_INDEX];
    }
  #endif // CAN2

  #if defined(CAN3)
    if ( pConf_Structure->Instance == CAN3 )
    {
      hcan = &canHandle[CAN3_INDEX];
    }
  #endif // CAN3

  hcan->Instance = pConf_Structure->Instance;
  uint32_t prescaler, pbs1, pbs2;
  if ( CAN_GetTimingFromBaudrate(pConf_Structure->baudrate, &prescaler, &pbs1, &pbs2) != HAL_OK )
  {
    return HAL_ERROR;
  }

  prescaler = prescaler;
  pbs1 = ((pbs1 - 1) << 16);
  pbs2 = ((pbs2 - 1) << 20);

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

  HAL_StatusTypeDef hReturn = HAL_CAN_Init(hcan);
  if ( hReturn != HAL_OK )
  {
    Cus_CANInitFailed_Hook(hcan, (const CANInitConfig_t *)pConf_Structure, hReturn);

    return HAL_ERROR;
  }    

  if ( pConf_Structure->Instance == CAN1 ) 
  {
    if ( createCANTCB( &CanDevice[CAN1_INDEX], pConf_Structure ) != 0 )    return HAL_ERROR;
  }
  #if defined(CAN2)
    if ( pConf_Structure->Instance == CAN2 )
    {
      if ( createCANTCB( &CanDevice[CAN2_INDEX], pConf_Structure ) != 0 )    return HAL_ERROR;
    }
  #endif // CAN2
  #if defined(CAN3)
    if ( pConf_Structure->Instance == CAN3 )
    {
      if ( createCANTCB( &CanDevice[CAN3_INDEX], pConf_Structure ) != 0 )    return HAL_ERROR;
    }
  #endif // CAN3

  return HAL_OK; 
}




/* -------------------------------------- CAN_get API Relevant ----------------------------------------------------- */
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


HAL_StatusTypeDef Cus_CAN_getRateInfo( CAN_TypeDef *instance, Cus_CAN_RateInfo_t *pOutInfo )
{
  if ( !instance || !pOutInfo )   return HAL_ERROR;

  CAN_HandleTypeDef *hcan = Cus_CAN_getHandle(instance);
  if ( hcan == NULL )   return HAL_ERROR;

  uint32_t BTR_cpy = (uint32_t)hcan->Instance->BTR;
  uint32_t pres = ( BTR_cpy & 0x3FF ) + 1;

  uint32_t temp_bs1 = (( BTR_cpy >> 16 ) & 0xF) + 1;
  uint32_t temp_bs2 = (( BTR_cpy >> 20 ) & 0x7) + 1;
  // 有效性检查（根据 STM32 参考手册）
  if (pres < 1 || pres > 1024 ||
    temp_bs1 < 1 || temp_bs1 > 16 ||
    temp_bs2 < 1 || temp_bs2 > 8) 
  {
    return HAL_ERROR;
  }

  pOutInfo->prescaler = pres;
  pOutInfo->bs1 = temp_bs1;
  pOutInfo->bs2 = temp_bs2;

  pOutInfo->can_clock = HAL_RCC_GetPCLK1Freq();

  float tq = (float)pOutInfo->prescaler / (float)pOutInfo->can_clock;
  float bit_time = (float)(1 + pOutInfo->bs1 + pOutInfo->bs2) * tq;
  pOutInfo->real_baudrate = 1.0f / bit_time;

  return HAL_OK;
}
/* -------------------------------------- CAN_get API Relevant ----------------------------------------------------- */




static HAL_StatusTypeDef cus_canfilterInit( const CANFilterConfig_t * pConf_Structure, CAN_TypeDef *instance )
{
  if ( !pConf_Structure || !instance )   return HAL_ERROR;

  CAN_FilterTypeDef temp_filter = { 0 };
  CAN_HandleTypeDef *hcan;
  hcan = Cus_CAN_getHandle(instance);

  uint32_t filterMode = 0;
  switch(pConf_Structure->Mode)
  {
    case Cus_CAN_FILTERMODE_IDMASK: filterMode = CAN_FILTERMODE_IDMASK; break;
    case Cus_CAN_FILTERMODE_IDLIST: filterMode = CAN_FILTERMODE_IDLIST; break;
    default: filterMode = CAN_FILTERMODE_IDLIST;
  }

  uint32_t fifo_assignment = 0;
  switch (pConf_Structure->FIFOAssignment)
  {
    case Cus_CAN_FIFOASSIGNMENT_FIFO0:  fifo_assignment = CAN_FILTER_FIFO0; break;
    case Cus_CAN_FIFOASSIGNMENT_FIFO1:  fifo_assignment = CAN_FILTER_FIFO1; break;
    default: fifo_assignment = CAN_FILTER_FIFO0; 
  }

  uint32_t filter_scale = 0;
  switch (pConf_Structure->Scale)
  {
    case Cus_CAN_SCALE_16BIT: filter_scale = CAN_FILTERSCALE_16BIT; break;
    case Cus_CAN_SCALE_32BIT: filter_scale = CAN_FILTERSCALE_32BIT; break;
    default:  filter_scale = CAN_FILTERSCALE_16BIT;
  }

  uint32_t filter_activation = 0;
  switch (pConf_Structure->is_Activation)
  {
    case Cus_FILTER_Enable: filter_activation = CAN_FILTER_ENABLE; break;
    case Cus_FILTER_Disable: filter_activation = CAN_FILTER_DISABLE; break;
    default:  filter_scale = CAN_FILTERSCALE_16BIT;
  }  

  temp_filter.FilterFIFOAssignment = fifo_assignment;
  temp_filter.FilterMode = filterMode;
  temp_filter.FilterBank = pConf_Structure->FilterBank;
  temp_filter.FilterScale = filter_scale;

  /* 滤波器ID配置部分将 直接使用由用户填充好的 ID 和掩码 */
  /* 所以调用该方法时, 请确保上层已经正确填充 Id */
  temp_filter.FilterIdHigh = pConf_Structure->IdHigh;
  temp_filter.FilterIdLow = pConf_Structure->IdLow;
  temp_filter.FilterMaskIdHigh = pConf_Structure->MaskIdHigh;
  temp_filter.FilterMaskIdLow = pConf_Structure->MaskIdLow;

  temp_filter.FilterActivation = filter_activation;

  HAL_StatusTypeDef hReturn = HAL_CAN_ConfigFilter(hcan, (const CAN_FilterTypeDef *)&temp_filter);
  if ( hReturn != HAL_OK )
  {
    Cus_FilterConfigFailed_Hook( hcan, (const CANFilterConfig_t *)pConf_Structure, hReturn );

    return HAL_ERROR;
  }

  return HAL_OK; 
}


HAL_StatusTypeDef Cus_CAN_Start( CAN_TypeDef *instance )
{
  if ( !instance )  return HAL_ERROR;

  CAN_HandleTypeDef *pcan = Cus_CAN_getHandle(instance);
  if ( pcan == NULL )   return HAL_ERROR;

  HAL_StatusTypeDef hReturn = HAL_CAN_Start( pcan );
  if ( hReturn != HAL_OK )
  { 
    Cus_CANStartFailed_Hook(pcan, hReturn);

    return HAL_ERROR;
  }

  return HAL_OK;
}



/* ------------------------------------------------------------------------- */
static HAL_StatusTypeDef Cus_CAN_Send( Cus_CAN_Device_t *pDev, CAN_TxHeaderTypeDef Txheader, uint8_t *Send_Buf )
{
  if ( !pDev || !Send_Buf || !pDev->canHandle || !pDev->Instance )    return HAL_ERROR;

  if ( Txheader.DLC > 8 )   Txheader.DLC = 8;       // 长度限制为8.

  uint32_t TxMailBox;
  HAL_StatusTypeDef hReturn = HAL_CAN_AddTxMessage(pDev->canHandle, &Txheader, Send_Buf, &TxMailBox);
  if ( hReturn != HAL_OK )
  {
    return HAL_ERROR;
  }

  uint32_t start_tick = HAL_GetTick();  
  uint32_t txok_mask;

  switch (TxMailBox)
  {
    case 1: txok_mask = CAN_TSR_TXOK0; break;
    case 2: txok_mask = CAN_TSR_TXOK1; break;
    case 4: txok_mask = CAN_TSR_TXOK2; break;
    default: return HAL_ERROR;
  }

  // 等待发送完成标志位置起.
  while( (pDev->canHandle->Instance->TSR & txok_mask) == 0 )
  {
    if ( (HAL_GetTick() - start_tick) >= 100 )   // 100ms 超时.
    {
			Cus_CANSendFailed_Hook(pDev, hReturn);

      return HAL_TIMEOUT;
    }
  }

  return HAL_OK;
}


static HAL_StatusTypeDef Cus_CAN_Recv( const Cus_CAN_Device_t *pDev, CAN_RxHeaderTypeDef *pHeader, uint8_t *Recv_Buf, uint8_t RxFifoIndex )
{
  if ( !pDev || !pHeader || !Recv_Buf )   return HAL_ERROR;

  if ( RxFifoIndex == 0 )
  {
    return HAL_CAN_GetRxMessage(pDev->canHandle, CAN_RX_FIFO0, pHeader, Recv_Buf);
  }
  else if ( RxFifoIndex == 1 )
  {
    return HAL_CAN_GetRxMessage(pDev->canHandle, CAN_RX_FIFO1, pHeader, Recv_Buf);
  }
  else return HAL_ERROR;
}

/* ------------------------------------------------------------------------- */




void Cus_CAN_Filter_SetStdList32(  CANFilterConfig_t *pFilter, uint8_t Filter_RTR, uint16_t id1, uint16_t id2 )
{
  if ( id1 > 0x7FF || id2 > 0x7FF || !pFilter )  return;

  uint32_t ide_bit = 0;

  uint8_t RTR_bit_1 = (Filter_RTR & 0x01UL);
  RTR_bit_1 = (RTR_bit_1 >> 0) & 0x01UL;
  uint32_t id1_regVal = ((uint32_t)id1 << 21) | (RTR_bit_1 << 1) | (ide_bit << 2);

  uint8_t RTR_bit_2 = (Filter_RTR & (0x01UL << 1));
  RTR_bit_2 = (RTR_bit_2 >> 1) & 0x01UL;
  uint32_t id2_regVal = ((uint32_t)id2 << 21) | (RTR_bit_2 << 1) | (ide_bit << 2);

  pFilter->IdHigh = (id1_regVal >> 16) & 0xFFFF;      // 高16位放在IDHigh.
  pFilter->IdLow = id1_regVal & 0xFFFF;               // 低16位放在IDLow.
  pFilter->MaskIdHigh = (id2_regVal >> 16) & 0xFFFF;
  pFilter->MaskIdLow = id2_regVal & 0xFFFF;
}


void Cus_CAN_Filter_SetStdList16( CANFilterConfig_t *pFilter, uint8_t Filter_RTR, uint16_t id1, uint16_t id2, uint16_t id3, uint16_t id4 )
{
  if ( !pFilter || id1 > 0x7FF || id2 > 0x7FF || id3 > 0x7FF || id4 > 0x7FF )   return;

  uint32_t ide_bit = 0;
  uint8_t RTR_bit_1 = (Filter_RTR & 0x01UL);    // 取第一位.
  RTR_bit_1 = RTR_bit_1 >> 0 & 0x01UL;
  uint32_t id1_regVal = ((uint32_t)id1 << 5 ) | (RTR_bit_1 << 4) | (ide_bit << 3);

  uint8_t RTR_bit_2 = (Filter_RTR & (0x01UL << 1)); 
  RTR_bit_2 = RTR_bit_2 >> 1 & 0x01UL;
  uint32_t id2_regVal = ((uint32_t)id2 << 5 ) | (RTR_bit_2 << 4) | (ide_bit << 3);

  uint8_t RTR_bit_3 = (Filter_RTR & (0x01UL << 2));
  RTR_bit_3 = RTR_bit_3 >> 2 & 0x01UL;
  uint32_t id3_regVal = ((uint32_t)id3 << 5 ) | (RTR_bit_3 << 4) | (ide_bit << 3);

  uint8_t RTR_bit_4 = (Filter_RTR & (0x01UL << 3));
  RTR_bit_4 = RTR_bit_4 >> 3 & 0x01UL;
  uint32_t id4_regVal = ((uint32_t)id4 << 5 ) | (RTR_bit_4 << 4) | (ide_bit << 3);

  pFilter->IdHigh = id1_regVal;
  pFilter->IdLow = id2_regVal;
  pFilter->MaskIdHigh = id3_regVal;
  pFilter->MaskIdLow = id4_regVal;
}


void Cus_CAN_Filter_SetExtList32( CANFilterConfig_t *pFilter, uint8_t Filter_RTR, uint32_t id1, uint32_t id2 )
{
  if ( !pFilter || id1 > 0x1FFFFFFF || id2 > 0x1FFFFFFF )  return;

  uint32_t ide = 1;
  uint8_t RTR_bit_1 = (Filter_RTR & 0x01UL);
  RTR_bit_1 = (RTR_bit_1 >> 0) & 0x01UL;
  uint32_t id1_regVal = (id1 << 3) | (RTR_bit_1 << 1) | (ide << 2);

  uint8_t RTR_bit_2 = (Filter_RTR & (0x01UL << 1));
  RTR_bit_2 = (RTR_bit_2 >> 1) & 0x01UL;
  uint32_t id2_regVal = (id2 << 3) | (RTR_bit_2 << 1) | (ide << 2);

  pFilter->IdHigh = (id1_regVal >> 16) & 0xFFFF;
  pFilter->IdLow = (id1_regVal) & 0xFFFF;
  pFilter->MaskIdHigh = (id2_regVal >> 16) & 0xFFFF;
  pFilter->MaskIdLow = (id2_regVal) & 0xFFFF;
}


void Cus_CAN_Filter_SetStdMask32( CANFilterConfig_t *pFilter, uint8_t Filter_RTR, uint16_t id1, uint16_t id1_mask )
{
  if ( !pFilter || id1 > 0x7FF || id1_mask > 0x7FF )  return;

  uint32_t RTR_bit;
  uint32_t RTR_Force_Match;
  uint32_t ide_bit = 0;
  if ( Filter_RTR == CAN_FILTER_MASK_DATA )   
  {
    RTR_Force_Match = 1;                  // 不匹配遥控帧. 强制RTR位为0,只匹配数据帧.
    RTR_bit = 0;
  }
  else if ( Filter_RTR == CAN_FILTER_MASK_REMOTE )
  {
    RTR_Force_Match = 1;                    // 只匹配遥控帧. 强制RTR为1.
    RTR_bit = 1;
  }
  else if ( Filter_RTR == (CAN_FILTER_MASK_REMOTE | CAN_FILTER_MASK_DATA) )
  {
    RTR_Force_Match = 0;                    // 遥控帧与数据帧均匹配.
    RTR_bit = 0;
  }
  else 
  {
    // 无效模式，默认只收数据帧
    RTR_Force_Match = 1;
    RTR_bit = 0;
  }

  uint32_t id_regVal = ((uint32_t)id1 << 21) | (RTR_bit << 1) | (ide_bit << 2);
  uint32_t mask_regVal = ((uint32_t)id1_mask << 21) | (RTR_Force_Match << 1) | (ide_bit << 2);

  pFilter->IdHigh = (id_regVal >> 16) & 0xFFFF;
  pFilter->IdLow = (id_regVal) & 0xFFFF;
  pFilter->MaskIdHigh = (mask_regVal >> 16) & 0xFFFF;
  pFilter->MaskIdLow = (mask_regVal) & 0xFFFF;
}



/*
  Default Fault Hook Defines.
  错误处理相关Hook函数默认定义.
*/
/* ------------------------------------------------------------------------- */
__weak void Cus_FilterConfigFailed_Hook( CAN_HandleTypeDef *hcan, const CANFilterConfig_t *pFilterConf, HAL_StatusTypeDef hal_status )
{
  UNUSED(hcan);
  UNUSED(pFilterConf);
  UNUSED(hal_status);
}


__weak void Cus_CANInitFailed_Hook( CAN_HandleTypeDef *hcan, const CANInitConfig_t *pInitConf, HAL_StatusTypeDef hal_status )
{
  UNUSED(hcan);
  UNUSED(pInitConf);
  UNUSED(hal_status);
}


__weak void Cus_CANStartFailed_Hook( CAN_HandleTypeDef *hcan, HAL_StatusTypeDef hal_status )
{
  UNUSED(hcan);
  UNUSED(hal_status);
}


__weak void Cus_CANSendFailed_Hook( Cus_CAN_Device_t *pDev, HAL_StatusTypeDef hal_status )
{
  UNUSED(pDev);
  UNUSED(hal_status);
}



/**
 * @brief 快速配置 CAN（默认普通模式，500kbps）
 * @note 弱函数，用户可覆盖。可自定义快速启动配置。
 */
__attribute__((used)) __weak HAL_StatusTypeDef Cus_CAN_QuickConfig( CAN_TypeDef *instance, const Cus_CAN_GPIO_t *g_gpio )
{
  if ( !instance || !g_gpio )   return HAL_ERROR;

  CAN_HandleTypeDef *hcan = Cus_CAN_getHandle(instance);
  CANInitConfig_t *pInit;
  if ( Factory_CANInitConfig_t(&pInit) != 0 )   return HAL_ERROR;

  pInit->baudrate = CAN_BAUDRATE_500K;
  pInit->Instance = instance;
  pInit->Mode = MODE_LOOPBACK;
  pInit->is_AutoBusOff = false;
  pInit->is_AutoRestransmission = true;
  pInit->is_AutoWakeUP = false;
  pInit->is_ReceiveFifoLocked = false;
  pInit->is_TimeTriggeredMode = false;
  pInit->is_TransmitFifoPriority = false;
  pInit->SJW = Cus_CAN_SJW_1Tq;

  memcpy(&pInit->CAN_gpio, g_gpio, sizeof(Cus_CAN_GPIO_t));

  HAL_StatusTypeDef hReturn = pInit->Cus_CAN_Init((const CANInitConfig_t *)pInit);

  pInit->Self_Release(&pInit);      // 释放配置结构体. 不再需要.

  return hReturn;
}



/**
 * @brief 快速配置 过滤器（默认全通 32位宽，接收所有帧）
 * @note 弱函数，用户可覆盖。可自定义快速启动配置。
 */
__attribute__((used)) __weak HAL_StatusTypeDef Cus_Filter_QuickConfig( CAN_TypeDef *instance )
{
  if ( !instance )    return HAL_ERROR;

  CAN_HandleTypeDef *hcan = Cus_CAN_getHandle(instance);
  if ( hcan == NULL )   return HAL_ERROR;

  CANFilterConfig_t *pFilter;
  if ( Factory_CANFilterConfig_t(&pFilter) != 0 )   return HAL_ERROR;

  pFilter->FIFOAssignment = Cus_CAN_FIFOASSIGNMENT_FIFO0;
  pFilter->FilterBank = 0;
  pFilter->IdHigh = 0x0;
  pFilter->IdLow = 0x0;
  pFilter->MaskIdHigh = 0x0;
  pFilter->MaskIdLow = 0x0;
  pFilter->Mode = Cus_CAN_FILTERMODE_IDMASK;
  pFilter->Scale = Cus_CAN_SCALE_32BIT;
  pFilter->is_Activation = Cus_FILTER_Enable;

  HAL_StatusTypeDef hReturn = pFilter->Cus_CAN_FilterInit((const CANFilterConfig_t *)pFilter, instance);

  pFilter->Self_Release(&pFilter);

  return hReturn;
}



/**
 * @brief 一键配置CAN启动
 * @note 弱函数，用户可覆盖。可自定义快速启动配置。
 */
__attribute__((used)) __weak HAL_StatusTypeDef Cus_CAN_QuickSetup( CAN_TypeDef *instance, const Cus_CAN_GPIO_t *g_gpio )
{
  if ( !instance || !g_gpio )   return HAL_ERROR;
  
  HAL_StatusTypeDef hReturn;
  hReturn = Cus_CAN_QuickConfig(instance, g_gpio);
  if ( hReturn != HAL_OK )  return HAL_ERROR;

  hReturn = Cus_Filter_QuickConfig(instance);
  if ( hReturn != HAL_OK )  return HAL_ERROR;

  hReturn = Cus_CAN_Start(instance);

  return hReturn;
}


/* ------------------------------------------------------------------------- */
