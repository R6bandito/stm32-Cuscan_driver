# Cus_CAN – 通用 STM32 CAN 驱动库

[TOC]

------

## 仓库概述

​	`Cus_CAN` 是一个高度可配置、面向设备的 **STM32 CAN 外设驱动库**，基于 HAL 库实现。提供了简洁的 API、环形缓冲区中断接收、异步发送队列、灵活的过滤器配置以及静态/动态内存分配支持，帮助快速搭建稳定、高效的 CAN 通信应用。

------

## 项目特性

- **多实例管理支持**（未经实测）：该库可同时支持多个CAN实例（同时支持 CAN1、CAN2、CAN3，取决于STM32型号）。由于测试设备型号局限性，因此该点未经测试（理论设计上是支持的）。
- **设备对象模型**：通过 `Cus_CAN_Device_t`结构体封装了发送（阻塞/异步），开关中断，轮询接收/缓冲区接收，设备实例等操作与信息，一个CAN实例对应一个操作对象(pDev)。正常使用时通过操作对象实例 pDev 来完成发送接收等动作。
- **环形接收缓冲区**：采用环形缓冲区方式，通过CAN接收中断自动缓存接收帧。并提供对应API从缓冲区中消费帧。
- **异步发送队列**：通过CAN TX中断推动队列发送状态机实现异步发送。队列深度可配置，自动发送排队帧。
- **快捷滤波器ID配置**：提供多个便捷API一键配置标准/扩展帧的列表模式或掩码模式。
- **波特率自动计算**：提供多个预设选项（125K/250K/500k/1MB），根据用户选择自动计算 PBS1 PBS2 Prescaler 参数。
- **内存分配策略**：支持动态分配与静态分配一键切换。
- **Hook钩子函数**：错误处理、接收缓冲区满、中断关闭等回调，便于自定义处理。
- **一键启动**：强力推荐，一行 API 开启整个CAN通信，测试时能够快速搭建测试环境。
- **CANTP协议栈对接**：内部提供了对 `Cus_CANTP` 的对接方法，可自动对接传输层协议。

------

## 文件结构

```c
Cus_CAN/
├── example				   // 内封测试用例
├── CAN_Cus.h               // 驱动主头文件（类型定义、API 声明、配置宏）
├── CAN_Cus.c               // 驱动核心实现（初始化、收发、过滤器、异步队列等）
├── Cus_CAN_IT.h            // 中断相关头文件
└── Cus_CAN_IT.c            // 中断处理函数、回调及弱函数 Hook 默认实现
```

------

## 快速开始

- 将以下所有文件加入到你的工程中

  > ├── CAN_Cus.h              
  > ├── CAN_Cus.c               
  > ├── Cus_CAN_IT.h           
  > └── Cus_CAN_IT.c           

包含头文件（注：无需手动包含`Cus_CAN_IT.h`，只需确保`Cus_CAN_IT.c/h`能被正常编译及链接即可）

```c
#include "CAN_Cus.h"
```

- **GPIO 配置**
  Cus_CAN库提供了 GPIO 配置结构体（`Cus_CAN_GPIO_t`），用于底层bxCAN的引脚配置。请根据芯片实际情况填写该配置结构体（注：底层依然使用HAL，因此用于填充的参数必须是由ST官方给出或等价于ST官方实现的外设地址映射宏）。

  ```c
    const Cus_CAN_GPIO_t can1_gpio = { .Alternate = 0,	// 仅 F4/F7 等需要，F1 忽略(填充0即可)
                                       .CAN_GPIO_RX = GPIO_PIN_11,
                                       .CAN_GPIO_TX = GPIO_PIN_12, 
                                       .CAN_GPIOPort_x = GPIOA };
  ```

- **一键启动（推荐）**
  提供了一键启动API：`Cus_CAN_QuickSetup()`（详见API Reference），调用后**一键完成 bxCAN外设初始化 + 过滤器初始化 + CAN启动**。用于快速搭建和验证通信环境。

  ```c
    if ( Cus_CAN_QuickSetup(CAN1, &can1_gpio) != HAL_OK )
    {
      Error_Handler();
    }
  ```

- **自行配置（若使用一键启动则无需该配置）**
  若需要精细化配置外设及过滤器，请按如下步骤进行：

  1. 创建配置结构体指针（动态分配模式）：若配置结构体内存分配模式为动态分配，则需要创建类型为`CANInitConfig_t *`（用于bxCAN工作参数配置）/ `CANFilterConfig_t *`（用于过滤器配置）两个全局指针变量。**注意：创建的指针变量生命周期在整个配置流程中必须有效！由于采用动态分配，若配置流程结束之前所创建指针变量已被系统回收，则会导致配置失败及内存泄漏**！

     ```c
     // 全局可见.
     static CANInitConfig_t *pInit;
     static  CANFilterConfig_t *pFilter;
     ```

  2. 调用“工厂”加工初始化相关变量。

     ```c
       Factory_CANInitConfig_t(&pInit);
       Factory_CANFilterConfig_t(&pFilter);
     ```

     以上执行后，内部便会将所需的堆空间通过malloc进行获取后，存入`pInit/pFilter`中，并挂载对应的初始化回调，释放回调及动态分配标志位 (**在释放配置结构体资源之前，不要改变 `pInit` / `pFilter` 变量本身的存储值（即不要重新赋值），否则原始动态内存地址会丢失，导致无法正确释放，造成内存泄漏。若需要备份指针，可复制到其他变量中，但释放时必须传入原始地址**)。

  3. 按需填充相关参数。(注：在填充 `CANInitConfig_t`\ `CANFilterConfig_t` 结构体时，请使用**本驱动库自定义的枚举类型**(详见`CAN_Cus.h`)，**不要**混用 STM32 HAL 库的原生枚举常量。)

     ```c
       pInit->baudrate = CAN_BAUDRATE_500K;	// 驱动库枚举型
       pInit->CAN_gpio = can1_gpio;
       pInit->Instance = CAN1;				// 驱动库枚举型
       pInit->Mode = MODE_LOOPBACK;			// 驱动库枚举型
       pInit->SJW = Cus_CAN_SJW_1Tq;			// 驱动库枚举型
     
       pInit->is_AutoBusOff = false;		// bool常量
       pInit->is_AutoRestransmission = false;
       pInit->is_AutoWakeUP = false;
       pInit->is_ReceiveFifoLocked = false;
       pInit->is_TimeTriggeredMode = false;
       pInit->is_TransmitFifoPriority = false;
     
       pInit->Cus_CAN_Init(pInit);
     
       pInit->Self_Release(&pInit);	// 初始化后若不再需要，可调用该API进行空间释放.
     ```

     过滤器的配置同上

     ```c
       pFilter->FIFOAssignment = Cus_CAN_FIFOASSIGNMENT_FIFO0;
       pFilter->FilterBank = 0;
       pFilter->Mode = Cus_CAN_FILTERMODE_IDMASK;
       pFilter->Scale = Cus_CAN_SCALE_32BIT;
       pFilter->is_Activation = Cus_FILTER_Enable;
     
       Cus_CAN_Filter_SetStdMask32(pFilter, CAN_FILTER_MASK_DATA, 0, 0);   // 数据帧全通过滤.
     
       pFilter->Cus_CAN_FilterInit(pFilter, CAN1);
     
       pFilter->Self_Release(&pFilter);
     ```

     注意：`pFilter/pInit -> is_DynamicAlloc`成员是由工厂进行初始化和设置的，请不要手动修改！否则可能会导致动态分配的内存释放失败从而导致内存泄漏（静态模式下该成员无用）。对于`IdHigh/IdLow/MaskIdHigh/MaskIdLow`等成员，用于对过滤器的报文ID过滤进行配置，但是需要按照手册根据模式手动处理偏移，不推荐直接配置。请使用提供的`Cus_CAN_Filter_SetStdMask32()`等辅助API进行直接配置（无需自行处理偏移）。

- **启动CAN外设（若使用一键启动则无需该配置）**
  通过`Cus_CAN_Start()`启动CAN外设。

  ```c
  Cus_CAN_Start(CAN1);
  ```

- **设备控制块**
  初始化完毕后，驱动库内部会维护对应CAN实例的设备控制块，通过`Cus_CAN_getControlBlock()`来获取对应的设备控制块.

  ```c
  Cus_CAN_Device_t *pDev = Cus_CAN_getControlBlock(CAN1);
  ```

  成功获取设备控制块后，即可通过其成员回调函数进行发送/接收等操作。

  ```c
    uint8_t txD[] = { 1, 2, 3 };
  
    CAN_TxHeaderTypeDef txHeader;
    txHeader.DLC = 3;
    txHeader.IDE = CAN_ID_STD;
    txHeader.RTR = 0;
    txHeader.StdId = 0x12;
  
    pDev->Send(pDev, txHeader, txD);	// 阻塞式发送. 直到帧被成功发送至总线才返回
  ```

  附注：若配置结构体采用**静态分配模式**
  则类似于 `FreeRTOS` 实现方式，用户需自行提供一块空间用于存储配置。

  ```c
  uint8_t init_buf[sizeof(CANInitConfig_t)];					// 由用户分配的RAM空间
  Factory_CANInitConfig_t_Static(init_buf, sizeof(init_buf));	  // 静态专属工厂
  CANInitConfig_t *pInit = (CANInitConfig_t*)init_buf;		 // 类型转换.
  
  // ... 填充参数 (同上述所示)
  
  pInit->Cus_CAN_Init(pInit);
  
  // 无需调用 Self_Release.
  // 静态模式下也拥有 Self_Release 这个成员，这个设计主要是为了考虑在某些情况下与动态模式进行兼容. 在实现上是空实现，仅作占位.
  ```

------

