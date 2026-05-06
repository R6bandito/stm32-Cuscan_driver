#include "can_test.h"


static CANInitConfig_t *pConf_dynamic = NULL;
static CANFilterConfig_t *pFilter_dynamic = NULL;


/* ---------------------------- 测试辅助API ---------------------------------- */
static BasicTestResult_t TestEnv_Init( void ) 
{
  Cus_CAN_Device_t *pDev;
  pDev = Cus_CAN_getControlBlock(CAN1);
  if (!pDev) {
      printf("  TestEnv_Init: CAN1 device not initialized\r\n");
      return TEST_FAIL;
  }
  return TEST_PASS;
}


#define BASIC_ASSERT(cond) \
    do \
    { \
        if (!(cond)) { \
            printf("  FAIL at %s, line %d\r\n", __FUNCTION__, __LINE__); \
            return TEST_FAIL; \
        } \
    } while(0)
/* ----------------------------------------------------------------------------- */


/* ----------------------------------------------------------------------------- */
#if (!CAN_CFG_ALLOC_DYNAMIC) 
  BasicTestResult_t BasicTest_InitAndGetDev_Static( void )
  {
    printf("\n======== BasicTest_InitAndGetDev_Static Test Start ========\n");

    uint8_t caninitconf_buffer[CANINITCONF_BUFFER_SIZE] = { 0 };
    uint8_t caninitfilter_buffer[CANINITFILTER_BUFFER_SIZE] = { 0 };
    Cus_CAN_GPIO_t gpio = { .Alternate = 0, 
                            .CAN_GPIO_RX = GPIO_PIN_11,
                            .CAN_GPIO_TX = GPIO_PIN_12,
                            .CAN_GPIOPort_x = GPIOA};

    int8_t ret = Factory_CANInitConfig_t_Static(caninitconf_buffer, sizeof(caninitconf_buffer));
    if ( ret < 0 )  
    {
      printf("\nBasicTest_InitAndGetDev_Static Test Faild.(called Factory_CANInitConfig_t_Static error.)\n");
      for( ; ; );
    }

    CANInitConfig_t *pConf = (CANInitConfig_t *)caninitconf_buffer;

    pConf->CAN_gpio = gpio;
    pConf->baudrate = CAN_BAUDRATE_500K;
    pConf->Instance = CAN1;
    pConf->is_AutoBusOff = false;
    pConf->is_AutoRestransmission = false;
    pConf->is_AutoWakeUP = false;
    pConf->is_ReceiveFifoLocked = false;
    pConf->is_TimeTriggeredMode = false;
    pConf->is_TransmitFifoPriority = false;
    pConf->Mode = MODE_LOOPBACK;      // 回环自测模式.
    pConf->SJW = Cus_CAN_SJW_1Tq;
    if ( pConf->Cus_CAN_Init(pConf) != HAL_OK )
    {
      printf("\nBasicTest_InitAndGetDev_Static Test Faild.(called pConf->Cus_CAN_Init error.)\n");
      for( ; ; );
    }

    int8_t fret = Factory_CANFilterConfig_t_Static(caninitfilter_buffer, sizeof(caninitfilter_buffer));
    if ( fret < 0 )
    {
      printf("\nBasicTest_InitAndGetDev_Static Test Faild.(called Factory_CANFilterConfig_t_Static error.)\n");
      for( ; ; );
    }

    CANFilterConfig_t *pFilter = (CANFilterConfig_t *)caninitfilter_buffer;

    pFilter->FIFOAssignment = Cus_CAN_FIFOASSIGNMENT_FIFO0;
    pFilter->FilterBank = 0;
    pFilter->is_Activation = Cus_FILTER_Enable;
    pFilter->Mode = Cus_CAN_FILTERMODE_IDMASK;
    pFilter->Scale = Cus_CAN_SCALE_32BIT;
    // Cus_CAN_Filter_SetExtMask32(pFilter, CAN_FILTER_RTR_NONE, 0, 0);
    Cus_CAN_Filter_SetStdMask32(pFilter, CAN_FILTER_RTR_NONE, 0, 0);

    if ( pFilter->Cus_CAN_FilterInit(pFilter, CAN1) != HAL_OK )
    {
      printf("\nBasicTest_InitAndGetDev_Static Test Faild.(called pFilter->Cus_CAN_FilterInit error.)\n");
      for( ; ; );
    }

    if (Cus_CAN_Start(CAN1) != HAL_OK )
    {
      printf("\nBasicTest_InitAndGetDev_Static Test Faild.(called Cus_CAN_Start error.)\n");
      for( ; ; );
    }

    pConf->Self_Release(&pConf);      // 注意. static 分配的配置结构体其释放函数为空占位. 此处仅作演示调用说明该问题. 

    pFilter->Self_Release(&pFilter);

    printf("  pConf: %x\tpFilter: %x\n", pConf, pFilter);

    CAN_HandleTypeDef *hcan = Cus_CAN_getHandle(CAN1);
    BASIC_ASSERT(hcan != NULL);

    int8_t index = Cus_CAN_getIndex(CAN1);
    BASIC_ASSERT(index == CAN1_INDEX);

    Cus_CAN_Device_t *dev = Cus_CAN_getControlBlock(CAN1);
    BASIC_ASSERT(dev != NULL);
    BASIC_ASSERT(dev->Instance == CAN1);
    BASIC_ASSERT(dev->canHandle == hcan);

    printf("======== BasicTest_InitAndGetDev_Static Test PASS! =========\n\n");
    return TEST_PASS;
  }
#endif
/* ----------------------------------------------------------------------------- */


/* ----------------------------------------------------------------------------- */
BasicTestResult_t BasicTest_GetRateInfo( void )
{
  printf("\n======== BasicTest_GetRateInfo Test Start ========\n");

  // 检查运行环境. (确保 Cus 库已正确加载运行并初始化)
  if (TestEnv_Init() != TEST_PASS) return TEST_FAIL;    

  Cus_CAN_RateInfo_t rate_info = { 0 };
  HAL_StatusTypeDef ret = Cus_CAN_getRateInfo(CAN1, &rate_info);
  if ( ret != HAL_OK )
  {
    printf("\nBasicTest_GetRateInfo Test Faild.(called Cus_CAN_getRateInfo error.)\n");
    for( ; ; );
  }

  printf("  Prescaler: %lu, BS1: %lu, BS2: %lu, Real Baudrate: %.2f\r\n",
    (unsigned long)rate_info.prescaler, (unsigned long)rate_info.bs1,
    (unsigned long)rate_info.bs2, rate_info.real_baudrate);
  BASIC_ASSERT(rate_info.real_baudrate > 100000 && rate_info.real_baudrate < 1100000);

  printf("======== BasicTest_GetRateInfo Test PASS! =========\n\n");
  return TEST_PASS;
}
/* ----------------------------------------------------------------------------- */


/* ----------------------------------------------------------------------------- */
#if (CAN_CFG_ALLOC_DYNAMIC) 
  BasicTestResult_t BasicTest_InitAndGetDev_Dynamic( void )
  {
    printf("\n======== BasicTest_InitAndGetDev_Dynamic Test Start ========\n");

    uint8_t confret = Factory_CANInitConfig_t(&pConf_dynamic);
    if ( confret )
    {
      printf("\nBasicTest_InitAndGetDev_Dynamic Test Faild.(called Factory_CANInitConfig_t error.)\n");
      for( ; ; );
    }

    uint8_t filterret = Factory_CANFilterConfig_t(&pFilter_dynamic);
    if ( filterret )
    {
      printf("\nBasicTest_InitAndGetDev_Dynamic Test Faild.(called Factory_CANFilterConfig_t error.)\n");
      for( ; ; );
    }

    Cus_CAN_GPIO_t gpio = { .Alternate = 0, 
      .CAN_GPIO_RX = GPIO_PIN_11,
      .CAN_GPIO_TX = GPIO_PIN_12,
      .CAN_GPIOPort_x = GPIOA};

    pConf_dynamic->CAN_gpio = gpio;
    pConf_dynamic->baudrate = CAN_BAUDRATE_500K;
    pConf_dynamic->Instance = CAN1;
    pConf_dynamic->is_AutoBusOff = false;
    pConf_dynamic->is_AutoRestransmission = false;
    pConf_dynamic->is_AutoWakeUP = false;
    pConf_dynamic->is_ReceiveFifoLocked = false;
    pConf_dynamic->is_TimeTriggeredMode = false;
    pConf_dynamic->is_TransmitFifoPriority = false;
    pConf_dynamic->Mode = MODE_LOOPBACK;      // 回环自测模式.
    pConf_dynamic->SJW = Cus_CAN_SJW_1Tq;

    if ( pConf_dynamic->Cus_CAN_Init(pConf_dynamic) != HAL_OK )
    {
      printf("\nBasicTest_InitAndGetDev_Dynamic Test Faild.(called pConf_dynamic->Cus_CAN_Init error.)\n");
      for( ; ; );
    }

    pFilter_dynamic->FIFOAssignment = Cus_CAN_FIFOASSIGNMENT_FIFO0;
    pFilter_dynamic->FilterBank = 0;
    pFilter_dynamic->is_Activation = Cus_FILTER_Enable;
    pFilter_dynamic->Mode = Cus_CAN_FILTERMODE_IDMASK;
    pFilter_dynamic->Scale = Cus_CAN_SCALE_16BIT;
    Cus_CAN_Filter_SetStdMask16(pFilter_dynamic, CAN_FILTER_RTR_NONE, 0, 0, 0, 0);    // 全通过滤器配置.

    if ( pFilter_dynamic->Cus_CAN_FilterInit(pFilter_dynamic, CAN1) != HAL_OK )
    {
      printf("\nBasicTest_InitAndGetDev_Dynamic Test Faild.(called pFilter_dynamic->Cus_CAN_FilterInit error.)\n");
      for( ; ; );
    }

    if ( Cus_CAN_Start(CAN1) != HAL_OK )
    {
      printf("\nBasicTest_InitAndGetDev_Dynamic Test Faild.(called Cus_CAN_Start error.)\n");
      for( ; ; );
    }

    printf("  Before Release: pConf: %x\tpFilter: %x\n", pConf_dynamic, pFilter_dynamic);
    pConf_dynamic->Self_Release(&pConf_dynamic);
    pFilter_dynamic->Self_Release(&pFilter_dynamic);
    printf("  After Release: pConf: %x\tpFilter: %x\n", pConf_dynamic, pFilter_dynamic);

    CAN_HandleTypeDef *hcan = Cus_CAN_getHandle(CAN1);
    BASIC_ASSERT(hcan != NULL);

    int8_t index = Cus_CAN_getIndex(CAN1);
    BASIC_ASSERT(index == CAN1_INDEX);

    Cus_CAN_Device_t *dev = Cus_CAN_getControlBlock(CAN1);
    BASIC_ASSERT(dev != NULL);
    BASIC_ASSERT(dev->Instance == CAN1);
    BASIC_ASSERT(dev->canHandle == hcan);

    printf("======== BasicTest_InitAndGetDev_Dynamic Test PASS! =========\n\n");
    return TEST_PASS;
  }
#endif 
/* ----------------------------------------------------------------------------- */


/* ----------------------------------------------------------------------------- */
BasicTestResult_t BasicTest_BlockingSendRecv_SingleFrame( void )
{
  printf("\n======== BasicTest_BlockingSendRecv_SingleFrame Test Start ========\n");
  if ( TestEnv_Init() != TEST_PASS ) return TEST_FAIL;

  CAN_TxHeaderTypeDef txHeader = {
    .StdId = 0x123,
    .DLC = 4,
    .ExtId = 0,
    .IDE = CAN_ID_STD,
    .RTR = CAN_RTR_DATA
  };

  uint8_t data[8] = { 0xDE, 0xAD, 0xBE, 0xEF };

  Cus_CAN_Device_t *pDev = Cus_CAN_getControlBlock(CAN1);
  if ( !pDev )
  {
    printf("\nBasicTest_BlockingSendRecv_SingleFrame Test Faild.(called Cus_CAN_getControlBlock error.)\n");
    for( ; ; );
  }

  // 阻塞发送.
  HAL_StatusTypeDef hReturn = pDev->Send(pDev, txHeader, data);
  BASIC_ASSERT(hReturn == HAL_OK);

  // 轮询接收.
  CAN_RxHeaderTypeDef rxHeader;
  uint8_t rxData[8] = { 0 };

  HAL_Delay(10);
  hReturn = pDev->Receive(pDev, &rxHeader, rxData, CAN_RX_FIFO0);
  BASIC_ASSERT(hReturn == HAL_OK);
  BASIC_ASSERT(rxHeader.StdId == 0x123);
  BASIC_ASSERT(rxHeader.DLC == 4);
  BASIC_ASSERT(memcmp(rxData, data, 4) == 0);

  printf("  StdId=%X\tDLC=%d\t", rxHeader.StdId, rxHeader.DLC);
  for( uint8_t i = 0; i < rxHeader.DLC; i++ )
  {
    printf("Data=%X ", rxData[i]);
    if ( i == rxHeader.DLC - 1 )
    {
      printf("\n");
    }
  }

  printf("======== BasicTest_BlockingSendRecv_SingleFrame Test PASS! =========\n\n");
  return TEST_PASS;
}
/* ----------------------------------------------------------------------------- */


/* ----------------------------------------------------------------------------- */
BasicTestResult_t BasicTest_BlockingSendRecv_MultiFrame( void )
{
  printf("\n======== BasicTest_BlockingSendRecv_MultiFrame Test Start ========\n");
  if ( TestEnv_Init() != TEST_PASS )  return TEST_FAIL;

  Cus_CAN_Device_t *pDev = Cus_CAN_getControlBlock(CAN1);

  // 初始化并发送3帧报文. (刚好塞满 FIFO0)
  for (uint16_t id = 0x200; id < 0x203; id++) 
  {
    CAN_TxHeaderTypeDef txH = {
        .StdId = id, .IDE = CAN_ID_STD, .RTR = CAN_RTR_DATA, .DLC = 2,
        .TransmitGlobalTime = DISABLE
    };

    uint8_t data[8] = {id & 0xFF, (id >> 8) & 0xFF};
    BASIC_ASSERT(pDev->Send(pDev, txH, data) == HAL_OK);
    HAL_Delay(2);
  }

  // 接收所有报文.
  int cnt = 0;
  for( uint8_t i = 0; i < 3; i++ )
  {
    CAN_RxHeaderTypeDef rxH;
    uint8_t rxD[8];

    if ( pDev->Receive(pDev, &rxH, rxD, CAN_RX_FIFO0) == HAL_OK )
    {
      cnt++;
      printf("  StdId=%X\tDLC=%d\t", rxH.StdId, rxH.DLC);
      for( uint8_t i = 0; i < rxH.DLC; i++ )
      {
        printf("Data=%X ", rxD[i]);
        if ( i == rxH.DLC - 1 )
        {
          printf("\n");
        }
      }
    }

    HAL_Delay(1);
  }
  BASIC_ASSERT(cnt >= 3);

  printf("======== BasicTest_BlockingSendRecv_MultiFrame Test PASS! =========\n\n");
  return TEST_PASS;
}
/* ----------------------------------------------------------------------------- */


/* ----------------------------------------------------------------------------- */
BasicTestResult_t BasicTest_BlockingSendRecv_DiffDLC_ID( void )
{
  printf("\n======== BasicTest_BlockingSendRecv_DiffDLC_ID Test Start ========\n");
  if ( TestEnv_Init() != TEST_PASS )    return TEST_FAIL;

  Cus_CAN_Device_t *pDev = Cus_CAN_getControlBlock(CAN1);

  // 发送扩展帧 DLC=8
  CAN_TxHeaderTypeDef txH = {
    .StdId = 0, .ExtId = 0x1FFFFFFF, .IDE = CAN_ID_EXT,
    .RTR = CAN_RTR_DATA, .DLC = 8, .TransmitGlobalTime = DISABLE
  };

  uint8_t txD[8] = { 1,2,3,4,5,6,7,8 };
  BASIC_ASSERT(pDev->Send(pDev, txH, txD) == HAL_OK);
  HAL_Delay(5);

  // 接收拓展帧.
  CAN_RxHeaderTypeDef rxH;
  uint8_t rxD[8];
  BASIC_ASSERT(pDev->Receive(pDev, &rxH, rxD, CAN_RX_FIFO0) == HAL_OK);

  printf("  EXTId=%X\tDLC=%d\t", rxH.ExtId, rxH.DLC);
  for( uint8_t i = 0; i < rxH.DLC; i++ )
  {
    printf("Data=%X ", rxD[i]);
    if ( i == rxH.DLC - 1 )
    {
      printf("\n");
    }
  }
  BASIC_ASSERT(memcmp(rxD, txD, 8) == 0);

  printf("======== BasicTest_BlockingSendRecv_DiffDLC_ID Test PASS! =========\n\n");
  return TEST_PASS;
}
/* ----------------------------------------------------------------------------- */


/* ----------------------------------------------------------------------------- */
BasicTestResult_t BasicTest_RegisterRxBuffer( void )
{
  printf("\n======== BasicTest_RegisterRxBuffer Test Start ========\n");
  if ( TestEnv_Init() != TEST_PASS )    return TEST_FAIL;

  Cus_CAN_Device_t *pDev = Cus_CAN_getControlBlock(CAN1);

  static Cus_CAN_RxMsg_t buf[8];
  BASIC_ASSERT(pDev->registerRxBuffer(pDev, buf, sizeof(buf)) == 0);

  // 重复注册应覆盖
  static Cus_CAN_RxMsg_t buf2[10];
  BASIC_ASSERT(pDev->registerRxBuffer(pDev, buf2, sizeof(buf2)) == 0);

  printf("======== BasicTest_BlockingSendRecv_DiffDLC_ID Test PASS! =========\n\n");
  return TEST_PASS;
}
/* ----------------------------------------------------------------------------- */


/* ----------------------------------------------------------------------------- */
BasicTestResult_t BasicTest_InterruptReceive( void )
{
  printf("\n======== BasicTest_InterruptReceive Test Start ========\n");
  if ( TestEnv_Init() != TEST_PASS )    return TEST_FAIL;

  Cus_CAN_Device_t *pDev = Cus_CAN_getControlBlock(CAN1);

  // 注册缓冲区.
  static Cus_CAN_RxMsg_t rxBuf[10];
  BASIC_ASSERT(pDev->registerRxBuffer(pDev, rxBuf, sizeof(rxBuf)) == 0);

  // 开中断.
  BASIC_ASSERT(pDev->EnableInterrupt(pDev, CAN_IT_RX_FIFO0_MSG_PENDING) == HAL_OK);

  // 阻塞发送 8 帧.
  CAN_TxHeaderTypeDef txH = {
    .StdId = 0x333, .IDE = CAN_ID_STD, .RTR = CAN_RTR_DATA, .DLC = 1, .TransmitGlobalTime = DISABLE
  };
  uint8_t data[8] = { 0xA1 };
  for ( int i = 0; i < 8; i++ ) 
  {
    data[0] = 0xA1 + i;
    BASIC_ASSERT(pDev->Send(pDev, txH, data) == HAL_OK);
    HAL_Delay(5);
  }

  HAL_Delay(10);    // 留出一定处理时间.


  CAN_RxHeaderTypeDef rxH;
  uint8_t rxD[8];

  // 从环形缓冲区取报文.
  for( uint8_t i = 0; i < 8; i++ )
  {
    BASIC_ASSERT(pDev->Receive_IT(pDev, &rxH, rxD) == HAL_OK);

    printf("  StdId=%X\tDLC=%d\t", rxH.StdId, rxH.DLC);
    for( uint8_t i = 0; i < rxH.DLC; i++ )
    {
      printf("Data=%X ", rxD[i]);
      if ( i == rxH.DLC - 1 )
      {
        printf("\n");
      }
    }

    HAL_Delay(2);
  }

  // 再次读应该失败（缓冲区空）
  BASIC_ASSERT(pDev->Receive_IT(pDev, &rxH, rxD) == HAL_ERROR);

  printf("======== BasicTest_InterruptReceive Test PASS! =========\n\n");
  return TEST_PASS;
}
/* ----------------------------------------------------------------------------- */


/* ----------------------------------------------------------------------------- */
#if (USE_SEND_ASYNC)
  BasicTestResult_t BasicTest_AsyncSend_SingleFrame( void )
  {
    printf("\n======== BasicTest_AsyncSend_SingleFrame Test Start ========\n");
    if ( TestEnv_Init() != TEST_PASS )    return TEST_FAIL;

    Cus_CAN_Device_t *pDev = Cus_CAN_getControlBlock(CAN1);

    // 配置接收缓冲区.
    static Cus_CAN_RxMsg_t rxBuf[10];
    BASIC_ASSERT(pDev->registerRxBuffer(pDev, rxBuf, sizeof(rxBuf)) == 0);

    // 使能中断.
    BASIC_ASSERT(pDev->EnableInterrupt(pDev, CAN_IT_RX_FIFO0_MSG_PENDING) == HAL_OK);
    BASIC_ASSERT(pDev->EnableInterrupt(pDev, CAN_IT_TX_MAILBOX_EMPTY) == HAL_OK);

    CAN_TxHeaderTypeDef txHeader = { .StdId = 0x321, .DLC = 3, .IDE = CAN_ID_STD, .RTR = CAN_RTR_DATA };
    uint8_t txData[8] = {0x11, 0x22, 0x33};

    // 异步发送一帧.
    BASIC_ASSERT(pDev->Send_IT(pDev, txHeader, txData) == HAL_OK);

    HAL_Delay(5);

    CAN_RxHeaderTypeDef rxHeader;
    uint8_t rxData[8];

    // 从缓冲区接收一帧.
    BASIC_ASSERT(pDev->Receive_IT(pDev, &rxHeader, rxData) == HAL_OK);

    printf("  StdId=%X\tDLC=%d\t", rxHeader.StdId, rxHeader.DLC);
    for( uint8_t i = 0; i < rxHeader.DLC; i++ )
    {
      printf("Data=%X ", rxData[i]);
      if ( i == rxHeader.DLC - 1 )
      {
        printf("\n");
      }
    }

    // 再次读取应该失败.
    BASIC_ASSERT(pDev->Receive_IT(pDev, &rxHeader, rxData) == HAL_ERROR);

    printf("======== BasicTest_AsyncSend_SingleFrame Test PASS! =========\n\n");
    return TEST_PASS;
  }


  BasicTestResult_t BasicTest_AsyncSend_HighFreq( void )
  {
    printf("\n======== BasicTest_AsyncSend_HighFreq Test Start ========\n");
    if ( TestEnv_Init() != TEST_PASS )    return TEST_FAIL;

    Cus_CAN_Device_t *pDev = Cus_CAN_getControlBlock(CAN1);

    // 配置接收缓冲区.
    static Cus_CAN_RxMsg_t rxBuf[64];
    BASIC_ASSERT(pDev->registerRxBuffer(pDev, rxBuf, sizeof(rxBuf)) == 0);

    // 使能中断.
    BASIC_ASSERT(pDev->EnableInterrupt(pDev, CAN_IT_RX_FIFO0_MSG_PENDING) == HAL_OK);
    BASIC_ASSERT(pDev->EnableInterrupt(pDev, CAN_IT_TX_MAILBOX_EMPTY) == HAL_OK);

    CAN_TxHeaderTypeDef header = { .StdId = 0x200, .DLC = 1, .IDE = CAN_ID_STD, .RTR = CAN_RTR_DATA };

    uint8_t data[8] = {0};
    int fail_count = 0;

    // 连续发送 50 帧，间隔 1ms，确保队列能正常消化.
    for (int i = 0; i < 50; i++) 
    {
      data[0] = i;
      HAL_StatusTypeDef ret = pDev->Send_IT(pDev, header, data);
      if ( ret != HAL_OK ) 
      {
        fail_count++;
      }

      HAL_Delay(1);
    }

    printf("  Fail count: %d (should be 0)\r\n", fail_count);

    // 将50帧数据全部接收.
    CAN_RxHeaderTypeDef rxheader;
    uint8_t rxdata[8];

    for( uint8_t i = 0; i < 50; i++ )
    {
      BASIC_ASSERT(pDev->Receive_IT(pDev, &rxheader, rxdata) == HAL_OK);

      printf("  StdId=%X\tDLC=%d\t", rxheader.StdId, rxheader.DLC);
      for( uint8_t i = 0; i < rxheader.DLC; i++ )
      {
        printf("Data=%X ", rxdata[i]);
        if ( i == rxheader.DLC - 1 )
        {
          printf("\n");
        }
      }

      HAL_Delay(1);
    }

    BASIC_ASSERT(fail_count == 0);

    // 再次获取应该失败.
    BASIC_ASSERT(pDev->Receive_IT(pDev, &rxheader, rxdata) == HAL_ERROR);

    printf("======== BasicTest_AsyncSend_HighFreq Test PASS! =========\n\n");
  }
#endif 
/* ----------------------------------------------------------------------------- */
