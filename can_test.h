#ifndef __CAN_TEST_H__
#define __CAN_TEST_H__


#include "CAN_Cus.h"
#include <stdio.h>


typedef enum {
  TEST_PASS = 0,
  TEST_FAIL
} BasicTestResult_t;


/* ************************************************************* */
  #define CANINITCONF_BUFFER_SIZE         (sizeof(CANInitConfig_t) + 2)
  #define CANINITFILTER_BUFFER_SIZE       (sizeof(CANFilterConfig_t) + 2)


/* ************************************************************* */


/* ************************************************************* */
BasicTestResult_t BasicTest_GetRateInfo( void );
BasicTestResult_t BasicTest_BlockingSendRecv_SingleFrame( void );   // 阻塞发送单帧 + 轮询接收(CAN回环模式).
BasicTestResult_t BasicTest_BlockingSendRecv_MultiFrame( void );    // 阻塞发送多帧 + 轮询接收(CAN回环模式).
BasicTestResult_t BasicTest_BlockingSendRecv_DiffDLC_ID( void );    // 变化DLC + EXT_ID测试.
BasicTestResult_t BasicTest_RegisterRxBuffer( void );               // 注册接收缓冲区.
BasicTestResult_t BasicTest_InterruptReceive( void );               // 中断接收测试.

#if (USE_SEND_ASYNC)
  BasicTestResult_t BasicTest_AsyncSend_SingleFrame( void );          // 异步发送单帧.
#endif 

#if (CAN_CFG_ALLOC_DYNAMIC) 
  BasicTestResult_t BasicTest_InitAndGetDev_Dynamic( void );
#endif

#if (!CAN_CFG_ALLOC_DYNAMIC)
  BasicTestResult_t BasicTest_InitAndGetDev_Static( void );
#endif 
/* ************************************************************* */


#endif // __CAN_TEST_H__
