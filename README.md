# Cus_CAN – 通用 STM32 CAN 驱动库

- [Cus_CAN – 通用 STM32 CAN 驱动库](#cus-can------stm32-can----)
  * [仓库概述](#----)
  * [项目特性](#----)
  * [文件结构](#----)
  * [快速开始](#----)
  * [核心配置宏](#-----)
    + [功能配置](#----)
    + [宏定义](#---)
  * [API Reference](#api-reference)

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
    if ( Cus_CAN_QuickSetup(CAN1, &can1_gpio, MODE_LOOPBACK) != HAL_OK )
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

  成功获取设备控制块后，即可通过其成员回调函数进行发送/接收等操作。更具体和细致化的操作将在下文相关部分继续讲解。

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

## 核心配置宏

### 功能配置

|             宏名称             |                             描述                             | 默认值 |
| :----------------------------: | :----------------------------------------------------------: | :----: |
|  USE_DEFAULT_RxFIFO_FULL_HOOK  | 选择是否开启由本库默认提供的缓冲区溢出备份机制.0=不使用，1=使用.关闭该宏后将不启用相关API以及不编译相关代码。关闭状态下用户可自行重写Hook来自定义溢出后处理机制。 |   0    |
|    BACKUP_BUFFER_LIMIT_NUM     | 这是一个子级宏，在USE_DEFAULT_RxFIFO_FULL_HOOK该宏开启的情况下有效，用于限制用户注册的备份缓冲区数目。设置该数目会影响对应处理机制的响应速度(更多的遍历)，以及增加RAM的占用，建议保持默认值。 |   2    |
|     CAN_CFG_ALLOC_DYNAMIC      | bxCAN硬件配置相关结构体是否采用动态内存分配. 0=静态分配，1=动态分配.一般情况下建议保持默认值. |   0    |
|     CAN_TCB_ALLOC_DYNAMIC      | CusCAN库内部靠维护实例控制块进行工作.该选项用于决定任务控制块是否通过动态方式进行分配. 0=静态分配，1=动态分配. 建议保持默认值. |   0    |
|         USE_SEND_ASYNC         | 是否启用 异步发送模式 (非阻塞). 0=不启用，1=启用. 默认为0(使用阻塞发送模式，提交发送请求后，成功收到TXOK方才返回). 根据实际需求进行修改. |   0    |
|  SEND_ASYNC_NodePOLL_DYNAMIC   | 这是一个子级宏，在USE_SEND_ASYNC开启的情况下有效. 用于选择发送队列的节点来源于动态分配还是静态分配. 0=静态分配，1=动态分配(当前版本待实现). 建议使用默认值并在上层采用失败重试机制. |   0    |
|         USE_CUS_CANTP          | 是否启用配套的Cus_CANTP库. 本库内部封装了一个发送API供Cus_CANTP使用，并且配套的Cus_CAN_IT.c机制中将会开启CANTP通路.  0=不启用，1=启用. 根据实际需求修改(本库不包含CANTP核心代码，详见 Cus_CANTP 库) |   0    |
|            USE_RTOS            | 是否启用 RTOS 适配模式. 若运行在 RTOS 环境中请将其开启，并按照文件中指导进行对应的移植操作. 0=裸机环境，1=RTOS环境. |   0    |
|       CUS_CAN_RTOS_CMSIS       | 这是一个子级宏，仅在USE_RTOS启用的情况下有效。该宏用于表示用户所使用的RTOS是否经过CMSIS-RTOS抽象层的处理。若为CMSIS-RTOS则自动在当前路径中包含相关头文件，并对应启动备份缓冲区机制的RTOS形式(如果启用USE_DEFAULT_RxFIFO_FULL_HOOK)。0=用户自实现, 1=CMSIS-RTOS2 内置。注意：若设置为0且启用了USE_DEFAULT_RxFIFO_FULL_HOOK，则用户需要手动在上层重写__weak void Cus_CAN_BackupNotifyFromISR( void )实现自己的通知与处理逻辑，否则缓冲区满后将静默返回，可能丢失整个缓冲区数据！ |   1    |
| Cus_SYSCALL_INTERRUPT_PRIORITY | 这是一个子级宏，在USE_RTOS开启的情况下有效.用于获取当前操作系统所管理的中断最大优先级，为库中临界区管理部分使用. 请根据使用的操作系统按照.h文件中的示例来正确定义设置. |   5    |

------

### 宏定义

|               宏名称               |                             描述                             | 默认值 |
| :--------------------------------: | :----------------------------------------------------------: | :----: |
|         MAX_SUPPORT_CANDEV         | 最大支持 CAN 实例数（CAN1~CAN3）。该值反映的就是该库中CAN实例对象的数目(一个CAN实例对应一个操作对象)。默认为3，若所用控制器并不支持多个CAN(例：STM32F1大部分常规系列)，可以自行减小该值来节省RAM。不建议增大该值（也没有必要增大）。 |   3    |
| CAN1_INDEX\|CAN2_INDEX\|CAN3_INDEX |        CAN 实例在内部数组中的索引。通常情况无需改动。        | 0/1/2  |
|         MAX_SUPPORT_RXFIFO         | 每个 CAN 实例的 FIFO 数量。请根据底层CAN硬件的具体配置来设置。例如：STM32F103C8T6 的bxCAN硬件有两个FIFO(FIFO0/FIFO1)，则该值设置为2。在资源十分有限的设备上，若你确保在整个通信过程中只使用一个FIFO，也可以将其设置为1以此来节省RAM开销。 |   2    |
|     FIFO0_IDX_0 \| FIFO_IDX_1      | FIFO0/FIFO1 在内部数组中的索引。可按需进行修改。例如：仅使用FIFO1，则可以删除FIFO0_IDX_0，并将FIFO_IDX_1定义为0。后续在对相关API的调用中，FIFO参数传参时传入该宏即可。该宏主要为了API调用传参的可读性而设计。 |  0/1   |
|        CUS_CAN_IRQ_PRIO_RX0        |   FIFO0 接收中断优先级。用于EnableIT时自动配置NVIC。下同。   |   6    |
|        CUS_CAN_IRQ_PRIO_RX1        |                    FIFO1 接收中断优先级。                    |   6    |
|        CUS_CAN_IRQ_PRIO_TX         |                     发送完成中断优先级。                     |   6    |
|        CUS_CAN_IRQ_PRIO_SCE        |                   错误/状态变化中断优先级                    |   6    |
|         TX_NODE_POLL_SIZE          | 该宏为一个子级宏，仅在USE_SEND_ASYNC有效且SEND_ASYNC_NodePOLL_DYNAMIC(不使用动态分配)无效的情况下有效。反映异步发送队列节点池大小(也可理解为队列长度大小)。在当前默认配置下，异步发送队列最多只允许同时挂起8帧发送请求。超出将返回错误并由上层进行对应处理。 |   8    |
|              CAN_FREE              | 标记异步队列节点空闲状态（是否可用）。由于设计原因后续将考虑移除相关定义。不要修改。 |   -1   |
|        CAN_FILTER_RTR_NONE         |                列表模式：所有 ID 仅收数据帧。                |   0    |
|     CAN_FILTER_RTR_ID1` ~ `ID4     |                列表模式：对指定 ID 收远程帧。                | 1<<0~3 |
|         CAN_FILTER_RTR_ALL         |                 列表模式：全部 ID 收远程帧。                 | 组合值 |
|        CAN_FILTER_MASK_DATA        |                    掩码模式：仅收数据帧。                    |  0xAA  |
|       CAN_FILTER_MASK_REMOTE       |                    掩码模式：仅收远程帧。                    |  0xBB  |

------

## API Reference

--------------------  初始化与生命周期管理 --------------------

- **快速启动CAN通信**

```c
__weak HAL_StatusTypeDef Cus_CAN_QuickSetup( CAN_TypeDef *instance, const Cus_CAN_GPIO_t *g_gpio, Cus_CAN_Mode_t mode );
```

**参数**：

- `CAN_TypeDef *instance`：所要快速启动的CAN实例。例如 CAN1。
- `const Cus_CAN_GPIO_t *g_gpio`：驱动库中定义的 GPIO 专用配置类型（Cus_CAN_GPIO_t）配置变量指针。
- `Cus_CAN_Mode_t mode`：启动模式（MODE_NORMAL/MODE_LOOPBACK/MODE_SILENT/MODE_SILENT_LPBACK）。

**返回值**：

- `HAL_ERROR`：传入参数非法/底层控制器配置或滤波器配置失败/CAN启动失败。
- `HAL_OK`：快速启动成功。CAN控制器目前正在运行。

**描述**：

​	该API用于一键启动CAN通信（初始化CAN + 全通过滤器 + CAN启动），多用于快速组建通信环境以及通信测试。内部会自动使用一组通用的默认配置对用户所传入的CAN实例进行功能配置。默认配置如下：

```
Baudrate = 500K
AutoBusOff = false
AutoRestransmission = false
AutoWakeUP = false
ReceiveFifoLocked = false
TimeTriggeredMode = false
TransmitFifoPriority = false
SJW = 1Tq

FIFO = FIFO0
FilterBank = 0
ID(High/Low/MaskHigh/MaskLow) = 0
Mode = IDMask
Scale = 32Bit
```

**注意事项**：

​	该API定义为了 __weak 形式，用户可自行覆盖并根据需要自行实现QuickSetup方法，无需修改库源码。

------

- **快速配置 bxCAN**

```c
__weak HAL_StatusTypeDef Cus_CAN_QuickConfig( CAN_TypeDef *instance, const Cus_CAN_GPIO_t *g_gpio, Cus_CAN_Mode_t mode );
```

**参数**：

- `CAN_TypeDef *instance`：所要快速启动的CAN实例。例如 CAN1。
- `const Cus_CAN_GPIO_t *g_gpio`：驱动库中定义的 GPIO 专用配置类型（Cus_CAN_GPIO_t）配置变量指针。
- `Cus_CAN_Mode_t mode`：启动模式（MODE_NORMAL/MODE_LOOPBACK/MODE_SILENT/MODE_SILENT_LPBACK）。

**返回值**：

- `HAL_ERROR`：传入参数非法/Factory工厂返回错误。
- `HAL_OK`：快速配置成功。

**描述**：

​	该API会采用默认配置（详见上文）对bxCAN进行配置，但是不会初始化过滤器以及启动CAN设备。QuickSetup的实现基础之一。

**注意事项**：

​	该API被定义为 __weak ，用户可使用自己的配置逻辑覆盖该方法。用自行逻辑覆盖后，调用QuickSetup 内部会按"你的 QuickConfig → 全通过滤 → Start"执行。

------

- **快速配置过滤器**

```c
__weak HAL_StatusTypeDef Cus_Filter_QuickConfig( CAN_TypeDef *instance );
```

**参数**：

- `CAN_TypeDef *instance`：需要配置的CAN实例。

**返回值**：

- `HAL_ERROR`：传入参数非法/Factory工厂返回错误/过滤器初始化失败。
- `HAL_OK`：过滤器快速配置成功。

**描述**：

​	快速配置全通过滤器，接收所有帧。

**注意事项**：

​	与上述 QuickConfig 一样，被定义为 __weak ，用户可使用自己逻辑进行覆盖，不影响QuickSetup调用。

也可以将三个快速配置API全部采用自行逻辑覆盖，从而全面自定义和接管整个快速初始化流程。

------

- **启动CAN设备**

```c
HAL_StatusTypeDef Cus_CAN_Start( CAN_TypeDef *instance );
```

**参数**：

- `CAN_TypeDef *instance`：所要启动的CAN实例。

**返回值**：

- `HAL_ERROR`：传入参数非法/实例无效（传入的CAN实例未经CusCAN库初始化，不属于CusCAN库管辖，不允许调用库API）
- `HAL_OK`：启动成功。

**描述**：

​	启动所传入实例对应的CAN外设。

**注意事项**：无

------

- **关闭设备（释放实例资源）**

```c
void Cus_CAN_DeviceClose( Cus_CAN_Device_t **pDev );
```

**参数**：

- `Cus_CAN_Device_t **pDev`：所要关闭的CAN设备（二级指针）。

**返回值**：无

**描述**：

​	该API用于关闭已经由CusCAN库初始化过的CAN实例，调用该API后所传入的实例控制块将被释放并回收资源。

**注意事项**：

​	该API可重入，线程安全。取决于TCB块的内存分配策略，有两种回收方式：

- 动态分配：底层调用free释放由malloc分配的空间。并将传入的 pDev 置为 NULL。
- 静态分配：清空对应内存区的字段。

------

- **获取实例控制块**

```c
Cus_CAN_Device_t *Cus_CAN_getControlBlock( CAN_TypeDef *instance );
```

**参数**：

- `CAN_TypeDef *instance`：对应控制块的CAN实例。需要获取哪一个控制块就传入对应CAN实例。

**返回值**：

- `=NULL`：传入实例无效/传入实例与配置不符（例如MAX_SUPPORT_CANDEV = 1，传入CAN2_INDEX）/传入实例未经CusCAN库初始化。
- `!=NULL`：成功获取对应CAN实例的设备指针对象，可通过该指针进行发送/接收等操作。

**描述**：

​	返回经由CusCAN初始化的对应CAN实例的指针，用于操控实例进行发送/接收/开关中断/缓冲区注册等操作。

注意事项：

1.返回的虽是非Const类型指针，但是随意修改该指针指向的任何成员都有可能导致状态异常，进而可能造成后续工作状态异常。因此不要手动修改任何成员。
2.传入的CAN实例必须由CusCAN库进行初始化，否则在CusCAN库中无该设备对应的信息。这种情况将返回NULL。

--------------------  初始化与生命周期管理 --------------------

------

---------------------------  发送 & 接收 ----------------------------

- **阻塞式发送**

```c
HAL_StatusTypeDef (*Send)( Cus_CAN_Device_t *pDev, CAN_TxHeaderTypeDef Txheader, uint8_t *Send_Buf );

// 该函数为设备指针内部的回调函数. pDev -> Send( .... )
```

**参数**：

- `Cus_CAN_Device_t *pDev`：所用设备控制块指针。控制块内部蕴含了Instance与Handle等信息，因此调用时需要传入自身。
- `CAN_TxHeaderTypeDef Txheader`：报文发送头。来源于HAL库，标识该帧报文的详细信息。
- `uint8_t *Send_Buf`：所要发送的数据缓冲区。

**返回值**：

- `HAL_ERROR`：传入参数非法/**使能了发送完成中断**/发送请求提交失败/发送邮箱检测错误(通常不会有该情况)。
- `HAL_TIMEOUT`：报文请求提交后未能在规定时间内检测到TXOK发送完成标志位。
- `HAL_OK`：该次发送完成。

**描述**：

​	该API为单帧报文发送基础方法，采用**阻塞式发送**机制，直到检测到发送完成标志位置位方返回HAL_OK。

**注意事项**：

1.**该方法仅限于未开启Tx中断时调用**，若开启了Tx中断再调用该方法则会提前返回HAL_ERROR。防止Tx提前清标志而导致该方法中轮询不到标志位，进而错误触发超时。

2.该方法内部时基依赖于 HAL 库中的 uwTick。因此若运行在 RTOS 环境下，**务必保证 uwTick 能正常被更新**，否则将会锁死。

------

- **异步发送（非阻塞）**

```c
HAL_StatusTypeDef (*Send_IT)( Cus_CAN_Device_t *pDev, CAN_TxHeaderTypeDef Txheader, uint8_t *Send_Buf );

// 该函数为设备指针内部的回调函数. pDev -> Send_IT( .... )
```

**参数**：

- `Cus_CAN_Device_t *pDev`：所用设备控制块指针。控制块内部蕴含了Instance与Handle等信息，因此调用时需要传入自身。
- `CAN_TxHeaderTypeDef Txheader`：报文发送头。来源于HAL库，标识该帧报文的详细信息。
- `uint8_t *Send_Buf`：所要发送的数据缓冲区。

**返回值**：

- `HAL_ERROR`：参数非法/发送请求提交失败（例如发送邮箱已满）。
- `HAL_BUSY`：节点分配失败，队列内存池中无多余空闲节点。
- `HAL_OK`：成功提交发送请求/成功将待发送报文挂入发送队列。

**描述**：

​	该 API 为非阻塞式报文发送方法，无论是否成功将立即返回，不会进行阻塞。**内部每个CAN实例分别维护一个发送队列，多个CAN实例共享一个节点池**。请求发起且硬件正忙时将会从空闲节点池中取出一个节点，填充后排入发送队列等待发送。发送队列的状态推进依靠发送中断完成。

**注意事项**：

1.函数内部已有临界段保护，上层无需额外保护。
2.该方法使用的前提是在头文件中开启 `USE_SEND_ASYNC`。
3.该方法需要开启发送中断，否则将无法推进状态机。
4.节点池大小由宏 `TX_NODE_POLL_SIZE` 定义，默认 8。当发送请求速率超过硬件发送能力时，队列会满，返回 `HAL_BUSY`，用户应实现重试或丢帧策略。

------

- **轮询接收**

```c
HAL_StatusTypeDef (*Receive)( const Cus_CAN_Device_t *pDev, CAN_RxHeaderTypeDef *pHeader, uint8_t *Recv_Buf, uint8_t FIFO_idx );

// 该函数为设备指针内部的回调函数. pDev -> Receive( ..... )
```

**参数**：

- `const Cus_CAN_Device_t *pDev`：所用设备控制块指针。控制块内部蕴含了Instance与Handle等信息，因此调用时需要传入自身。
- `CAN_RxHeaderTypeDef *pHeader`：报文信息头(接收)。接收到的报文元信息将会放入该变量中。
- `uint8_t *Recv_Buf`：数据接收缓冲区。
- `uint8_t FIFO_idx`：要轮询读取的 FIFO。

**返回值**：

- `HAL_ERROR`：传入参数非法/ 底层 `HAL_CAN_GetRxMessage` 返回错误。 
- `HAL_BUSY`：检测到接收中断开启，禁用该API，返回BUSY，以防止中断与轮询冲突。
- `HAL_OK`：`HAL_CAN_GetRxMessage`返回OK（接收到报文）。

**描述**：

​	该方法用于手动轮询接收CAN报文。当关闭所有高级功能且不使用Tx与Rx中断时，该库将最简且退化为对HAL的基础封装。

**注意事项**：

1.传入的FIFO_idx参数为CusCAN驱动库中定义的宏参数（例如 `FIFO_IDX_0` 或 `FIFO_IDX_1`），不应传入HAL库定义的FIFO宏，否则提前返回HAL_ERROR。
2.使用该函数的前提是不能使能Rx中断，否则提前返回HAL_BUSY。

------

- **缓冲区接收**

```c
HAL_StatusTypeDef (*Receive_IT)( Cus_CAN_Device_t *pDev, CAN_RxHeaderTypeDef *pHeader, uint8_t *Recv_Buf, uint8_t FIFO_Idx );

// 该函数为设备指针内部的回调函数. pDev -> Receive_IT( .... )
```

**参数**：

- `Cus_CAN_Device_t *pDev`：所用设备控制块指针。控制块内部蕴含了Instance与Handle等信息，因此调用时需要传入自身。
- `CAN_RxHeaderTypeDef *pHeader`：报文信息头(接收)。接收到的报文元信息将会放入该变量中。
- `uint8_t *Recv_Buf`：数据接收缓冲区。
- `uint8_t FIFO_Idx`：要读取的 FIFO。

**返回值**：

- `HAL_ERROR`：传入参数非法/未开对应中断/缓冲区未注册。
- `HAL_BUSY`：资源当前不可用（缓冲区为空）。
- `HAL_OK`：成功从缓冲区中取出一帧数据。

**描述**：

​	该API用于从内部维护的环形缓冲区中取出一帧CAN报文。具体设计信息详见下文。

**注意事项**：

1.**使用该API需要开启Rx中断**，否则提前返回HAL_ERROR。
2.用户应该在开始CAN通信之前注册缓冲区。调用该API时，若缓冲区未注册则提前返回HAL_ERROR。
3.内部已有临界段保护，无需上层额外保护。

---------------------------  发送 & 接收 ----------------------------

------

---------------------------  缓冲区管理相关 ----------------------------

- **注册接收缓冲区**

```c
uint8_t (*registerRxBuffer)( Cus_CAN_Device_t *pDev, void *pBuffer, uint32_t size, uint8_t FIFO_idx );
// 该函数为设备指针内部的回调函数. pDev -> registerRxBuffer( .... )
```

**参数**：

- `Cus_CAN_Device_t *pDev`：所用设备控制块指针。控制块内部蕴含了Instance与Handle等信息，因此调用时需要传入自身。
- `void *pBuffer`：需要注册给接收方法使用的缓冲区。（通用指针形式）
- `uint32_t size`：缓冲区大小。
- `uint8_t FIFO_idx`：注册的FIFO。缓冲区注册粒度是针对于一个CAN实例的单个FIFO进行的。若传入为 `FIFO_IDX_0`则为FIFO0注册缓冲区，只存入来自FIFO0的数据。而存入FIFO1的数据由于未注册缓冲区将会返回ERROR。可多次调用该API为该实例的多个FIFO分别创建缓冲区。

**返回值**：

- `0xFF`：传入参数无效（pBuffer为NULL/size > UINT32_MAX/ pDev无效）/ 传入的缓冲区大小小于一个消息元素(`Cus_CAN_RxMsg_t`)
  意味着连一条信息都无法存入，返回错误。
- `0`：缓冲区成功注册。

**描述**：

​	该API用于将用户提供的缓冲区注册作为接收环形缓冲区。后续的一系列相关操作（环形缓冲区写入/读出/判满）等均以该缓冲区及其相关元数据为基础。

**注意事项**：

​	1.若使用`Receive_IT`方法，**则必须在收报文之前调用该API为接收机制注册数据缓冲区**！否则数据到来触发中断后无缓冲区可用，会丢掉报文并静默返回。

​	2.接收缓冲区的大小应能够容纳至少一条消息结构(`Cus_CAN_RxMsg_t`)。虽然缓冲区的传入采用`void *`类型通用结构，但是此处建议按照以下格式定义缓冲区后直接注册缓冲区，而不是依赖API内部进行换算（不直观，用户无法准确获知当前传入的缓冲区具体能够存入几条消息）。

```c
/* 推荐形式  */
static Cus_CAN_RxMsg_t RecvRingBuf[16];	 // 创建一个最大能容纳15条消息的缓冲区.
pDev->registerRxBuffer(pDev, (void *)RecvRingBuf, sizeof(RecvRingBuf), FIFO_IDX_0);	// 为FIFO0注册缓冲区.

/* 不推荐形式 */
static uint8_t RingBuf[128];	// 究竟能存入多少条信息？内部会换算，但是用户层无法感知，进而对性能无法准确把握.
pDev->registerRxBuffer(pDev, (void *)RingBuf, sizeof(RingBuf), FIFO_IDX_0);


/* 附: */
/* 环形缓冲区元素：一条完整的 CAN 消息 */
typedef struct 
{
  CAN_RxHeaderTypeDef RxHeader;
  uint8_t RxData[8];
    
} Cus_CAN_RxMsg_t;
```

​	3.该API内部无临界区保护，不可重入。应该在初始化时刻进行注册，且不建议在运行时刻（无论 裸机/RTOS 环境）重新注册先前已被注册过的缓冲区，由于ISR抢占或多线程抢占可能会导致竞争进而出现残缺状态引发潜在问题。

------

- **缓冲区当前时刻报文数目**

```c
int16_t Cus_CAN_GetRxBufferPendingCount( Cus_CAN_Device_t *pDev, uint8_t FIFO_idx );
```

**参数**：

- `Cus_CAN_Device_t *pDev`：所用设备控制块指针。控制块内部蕴含了Instance与Handle等信息，因此调用时需要传入自身。
- `uint8_t FIFO_idx`：所要查询的FIFO编号。（`FIFO_IDX_0`/`FIFO_IDX_1`/你自定义的编号宏）

**返回值**：

- `-1`：传入的pDev无效。
- `-2`：所选择的FIFO未注册缓冲区。
- `>0`：缓冲区中当前报文数目。

**描述**：

​	从指定CAN实例的指定FIFO中读出缓冲区中当前（API调用时刻）剩余的报文数目。

**注意事项**：

​	1.所选择的FIFO必须已注册缓冲区，读没有缓冲区的FIFO没有意义，返回错误。

​	2.该API内部有临界段保护，可重入。但是由于读出的数目为瞬时值（仅在当前时刻有效，下一时刻很可能由于另一帧报文的存入或取出而发生改变）因此不建议将该值用于后续逻辑判断。该值推荐的应用方向有：

--- 手动关闭接收中断时，在Hook回调中指示缓冲区仍有多少数据待处理（允许用户关闭中断后对剩余数据快速处理）

--- Burst大流量冲击统计Pending最大值，用于指示在突发流量冲击时缓冲区溢出风险。

------

- **备份缓冲区注册**

注意：该API仅在开启 `USE_DEFAULT_RxFIFO_FULL_HOOK = 1` 后可用。

```c
uint8_t (*registerBackUPBuffer)( Cus_CAN_Device_t *pDev, void *pBuffer, uint32_t size );
// 该函数为设备指针内部的回调函数. pDev -> registerBackUPBuffer( .... )
```

**参数**：

- `Cus_CAN_Device_t *pDev`：所用设备控制块指针。控制块内部蕴含了Instance与Handle等信息，因此调用时需要传入自身。
- `void *pBuffer`：备份缓冲区。
- `uint32_t size`：备份缓冲区大小。

**返回值**：

- `0xFF`：pDev无效/pBuffer为NULL/缓冲区大小无效(size > UINT32_MAX)/缓冲区注册数量超限。
- `0`：成功注册备份缓冲区。

**描述**：

​	该API用于在开启由库默认提供的备份功能(`USE_DEFAULT_RxFIFO_FULL_HOOK`)后，注册备用缓冲区。该缓冲区在溢出发生之前将不被使用，当溢出发生后，注册的备份缓冲区将作为替补缓冲区被切换为接收缓冲区进行使用。

**注意事项**：

​	1.该API内部操纵全局数据，且没有临界段保护。如需使用备份机制，应在初始化阶段调用该API注册备份用缓冲区，而不应在任务等有重入风险的地方调用该API。

​	2.备份缓冲区的注册上限取决于 `BACKUP_BUFFER_LIMIT_NUM`。

---------------------------  缓冲区管理相关 ----------------------------

------

---------------------------  中断相关  ----------------------------

- **查询指定中断是否启用**

```c
bool (*CheckInterrupt)( const Cus_CAN_Device_t *pDev, uint32_t interrupt_mask );
// 该函数为设备指针内部的回调函数. pDev -> CheckInterrupt( .... )
```

**参数**：

- `const Cus_CAN_Device_t *pDev`：所用设备控制块指针。控制块内部蕴含了Instance与Handle等信息，因此调用时需要传入自身。
- `uint32_t interrupt_mask`：中断掩码。用于指定要查询的中断源。该掩码与 HAL 库定义的 CAN 中断标识兼容(例：`CAN_IT_RX_FIFO0_MSG_PENDING`/`CAN_IT_TX_MAILBOX_EMPTY`)。

**返回值**：

- `false`：存在至少一个指定的中断源未使能，或设备无效。
- `true`：指定的所有中断源均已使能。

**描述**：

​	根据传入的 `interrupt_mask` 参数（支持通过按位 | 进行组合），查询指定一个或多个中断源是否均已开启。

**注意事项**：

​	1.该函数不检查 NVIC 层中断是否开启，仅反映 CAN 控制器内部中断使能状态。

------

- **使能中断**

```c
HAL_StatusTypeDef (*EnableInterrupt)( Cus_CAN_Device_t *pDev, uint32_t interrupt_mask );
// 该函数为设备指针内部的回调函数. pDev -> EnableInterrupt( .... )
```

**参数**：

- `Cus_CAN_Device_t *pDev`：所用设备控制块指针。控制块内部蕴含了Instance与Handle等信息。
- `uint32_t interrupt_mask`：中断掩码。用于指定要查询的中断源。该掩码与 HAL 库定义的 CAN 中断标识兼容。支持按位或 I  组合多个中断源。

**返回值**：

- `HAL_ERROR`：传入pDev无效/中断使能失败（`HAL_CAN_ActivateNotification`断言失败）。
- `HAL_TIMEOUT`：使能中断必须在CAN控制器正常启动后进行，API中会对底层CAN控制器是否启动进行检查，若未启动则会调用`Cus_CAN_Start`启动控制器，若启动失败则调用`Cus_CANStartFailed_Hook`后返回 HAL_TIMEOUT状态。
- `HAL_OK`：成功开启对应中断。

**描述**：

​	根据传入的 `interrupt_mask` 参数（支持通过按位 | 进行组合），开启指定一个或多个中断源。

**注意事项**：

​	1.该API内部在检测到未启动CAN后，会自动调用`Cus_CAN_Start()`。但是请按照标准流程在初始化时于外部显示启动CAN，而不是依赖该API内部启动。

​	2.该API内部在开启中断源后会调用`Cus_CAN_NVIC_Config()`自动配置NVIC，后续无需重复调用相关API开启NVIC。具体配置的优先级由 .h中 `CUS_CAN_IRQ_PRIO_RX0/CUS_CAN_IRQ_PRIO_RX1`等决定，可自行修改。

​	3.若开启了 `USE_SEND_ASYNC`。则开启中断时，如果开启的中断为Tx中断，API内部会检查当前发送队列是否有报文挂起，若有报文挂起，则会调用一次 `Cus_CAN_ProcessTxQueue()`推动队列发送状态机继续发送。

------

- **失能中断**

```c
HAL_StatusTypeDef (*DisableInterrupt)( Cus_CAN_Device_t *pDev, uint32_t interrupt_mask );
// 该函数为设备指针内部的回调函数. pDev -> DisableInterrupt( .... )
```

**参数**：

- `Cus_CAN_Device_t *pDev`：所用设备控制块指针。控制块内部蕴含了Instance与Handle等信息。
- `uint32_t interrupt_mask`：中断掩码。用于指定要关闭的中断源。该掩码与 HAL 库定义的 CAN 中断标识兼容。支持按位或 I  组合多个中断源。

**返回值**：

- `HAL_ERROR`：传入参数无效/底层HAL关闭通知无效(`HAL_CAN_DeactivateNotification`断言失败)。
- `HAL_OK`：目标中断调用该API前已被关闭，空调用/成功失能对应中断。

**描述**：

​	根据传入的 `interrupt_mask` 参数（支持通过按位 | 进行组合），失能指定一个或多个中断源。

**注意事项**：

​	1.重复调用该API失能同一中断不会返回错误。后续调用均属于空调用。

​	2.当挂起的中断为RX接收中断时(FIFO0/FIFO1)，内部会检查对应的FIFO缓冲区是否仍有剩余数据，**若有剩余数据则调用`Cus_CAN_OnDisableRxIT_NonEmpty`回调来通知用户进行处理**。该回调默认空实现，若无需接收通知，也可以选择不实现该回调，而由自身进行对应处理。

---------------------------  中断相关  ----------------------------

------

---------------------------  过滤器ID快捷配置  ----------------------------

**Ps:以下函数用于填充 `CANFilterConfig_t` 结构体的 `IdHigh`/`IdLow`/`MaskIdHigh`/`MaskIdLow` 字段，简化 ID 和掩码的偏移计算。调用前需自行设置 `FilterBank`、`Mode`、`Scale`、`FIFOAssignment`、`is_Activation` 等成员。**

- **过滤器32位标准列表模式**

```c
void Cus_CAN_Filter_SetStdList32( CANFilterConfig_t *pFilter, uint8_t Filter_RTR, uint16_t id1, uint16_t id2 );
```

**参数**：

- `CANFilterConfig_t *pFilter`：指向过滤器配置结构体的指针。
-  `uint8_t Filter_RTR`：RTR 控制位掩码，低 2 位分别对应 `id1` 和 `id2`（bit0=id1，bit1=id2）。0=数据帧，1=远程帧。可使用 `CAN_FILTER_RTR_ID1` / `CAN_FILTER_RTR_ID2` 组合。表示该id对应的报文帧允许的通过类型。
- `uint16_t id1/uint16_t id2`：两个标准ID。（标准模式下 ID范围 0~0x7FF）

**返回值**： 无

**描述**：配置 32 位列表模式过滤器，接收两个标准 ID（11 位）。

**注意事项**：

​	1.对于Std**标准模式，传入的ID必须严格小于7FF**(11位)，否则空调用。

​	2.此类方法仅填充 ID 字段，不修改 `Mode`、`Scale` 等。过滤器模式需预先设为 `Cus_CAN_FILTERMODE_IDLIST/Cus_CAN_FILTERMODE_IDMASK`，尺度为 `Cus_CAN_SCALE_32BIT/Cus_CAN_SCALE_16BIT`。下同，一定要注意！

------

- **过滤器16位标准列表模式**

```c
void Cus_CAN_Filter_SetStdList16( CANFilterConfig_t *pFilter, uint8_t Filter_RTR, uint16_t id1, uint16_t id2, uint16_t id3, uint16_t id4 );
```

该方法及其以下相关方法参数都是与上文相同的，此处为了篇幅考虑不做赘述参数。

**描述**：

​	配置 16 位列表模式过滤器，接收四个标准 ID（11 位）。

**返回值**： 无

**注意事项**：

​	1.`Filter_RTR`低 4 位分别对应 `id1`~`id4`（bit0=id1,…,bit3=id4）。0=数据帧，1=远程帧。

​	2.过滤器模式需设为 `Cus_CAN_FILTERMODE_IDLIST`，尺度为 `Cus_CAN_SCALE_16BIT`。

------

- **过滤器32位拓展列表模式**

```c
void Cus_CAN_Filter_SetExtList32( CANFilterConfig_t *pFilter, uint8_t Filter_RTR, uint32_t id1, uint32_t id2 )
```

**描述**：

​	配置 32 位列表模式过滤器，接收两个扩展 ID（29 位）。

**返回值**：无

**注意事项**：

​	1.可接受ID范围( 0~0x1FFFFFFF)。

​	2.自动强制匹配扩展帧（IDE=1）。过滤器模式需为 `Cus_CAN_FILTERMODE_IDLIST`，尺度 `Cus_CAN_SCALE_32BIT`。

------

