#include "system.h"
#include "sht40_port.h"
#include "sht40_handler.h"
#include "../bsp/bsp_MLX_90614/bsp_MLX_90614_port.h"
#include "../bsp/usart_wifi_esp/usart_wifi_esp.h"
#include "FreeRTOS.h"
#include "queue.h"
#include "semphr.h"
#include"Middle_ring_buffer.h"
extern QueueHandle_t xUART_Queue;        // 消息队列句柄
extern SemaphoreHandle_t xDMA_Sem;      // DMA发送完成信号量
extern SemaphoreHandle_t xBufferDataSemaphore;  // 缓冲区数据信号量
extern QueueHandle_t sensor_data_queue; // 传感器数据队列句柄
extern RingBuffer_t g_ring_buffer; //全局环形缓冲区实例
EventGroupHandle_t xEventGroup; // 事件组句柄
//app_task 任务负责从队列中接收传感器数据，并通过USART发送给ESP8266。

#define ARR  (20000 - 1) // 定时器自动重装载值，决定定时周期
#define PSC  (7200 - 1)  // 定时器预分频值，决定定时器计数频率


#if 1
/**
 * @brief 单元测试 1：验证 I2C 控制器硬件状态
 */
void UT_Hardware_Verify(void) {
    // 1. 检查 I2C1 是否使能 (PE位)
    if (I2C1->CTLR1 & I2C_CTLR1_PE) {
        printf("[UT] I2C1 Peripheral: ENABLED\n");
    } else {
        printf("[UT] I2C1 Peripheral: ERROR (Check Clock/Init)\n");
    }

    // 2. 检查总线 Busy 位
    // 如果没有上拉电阻或 SDA/SCL 被短路接地，这里会显示 BUSY
    if (I2C_GetFlagStatus(I2C1, I2C_FLAG_BUSY)) {
        printf("[UT] Bus Status: BUSY (Check Pull-up Resistors)\n");
    } else {
        printf("[UT] Bus Status: IDLE (Ready)\n");
    }
}

/**
 * @brief 单元测试 2：验证 SHT40 是否在位
 * @return 0: 成功, 1: 失败
 */
uint8_t UT_SHT40_Ping(void) {
    uint32_t timeout = 10000;

    // 1. 产生起始信号
    I2C_GenerateSTART(I2C1, ENABLE);
    while(!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_MODE_SELECT)) {
        if(--timeout == 0) return 1;
    }

    // 2. 发送写地址 (0x44 << 1 = 0x88) [cite: 409, 415]
    I2C_Send7bitAddress(I2C1, 0x88, I2C_Direction_Transmitter);
    
    timeout = 10000;
    // 等待地址被从机应答 (ACK) [cite: 417, 445]
    while(!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED)) {
        if(--timeout == 0) {
            I2C_GenerateSTOP(I2C1, ENABLE);
            printf("[UT] SHT40 Ping: FAILED (No ACK)\n");
            return 1;
        }
    }

    // 3. 产生停止信号
    I2C_GenerateSTOP(I2C1, ENABLE);
    printf("[UT] SHT40 Ping: SUCCESS (ACK Received)\n");
    return 0;
}
#endif
//串口初始化函数
usart_wifi_esp_Status_t USART_WIFI_ESP_Init(void)
{
    
    USART_InitTypeDef USART_InitStructure={0};
    GPIO_InitTypeDef GPIO_InitStructure={0};
    
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART2, ENABLE);
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);
    //RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO, ENABLE);

    GPIO_InitStructure.GPIO_Pin = DATA_USART_TX_PIN;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP; // 复用推挽
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(DATA_USART_PORT, &GPIO_InitStructure);

    GPIO_InitStructure.GPIO_Pin = DATA_USART_RX_PIN;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING; // 浮空输入
    GPIO_Init(DATA_USART_PORT, &GPIO_InitStructure);

    USART_InitStructure.USART_BaudRate = USART_WIFI_ESP_BAUDRATE;//115200
    USART_InitStructure.USART_WordLength = USART_WordLength_8b;
    USART_InitStructure.USART_StopBits = USART_StopBits_1;
    USART_InitStructure.USART_Parity = USART_Parity_No;
    USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
    USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
    USART_Init(USART2, &USART_InitStructure);

    USART_Cmd(USART2, ENABLE);
    return USART_WIFI_ESP_OK;
}
//DMA初始化函数
DMA_Status_t USART_DMA_Init(void)
{
    DMA_InitTypeDef DMA_InitStructure={0};
    NVIC_InitTypeDef NVIC_InitStructure={0};

    RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMA1, ENABLE);

    // 配置 DMA1 Channel7 用于 USART2 TX
    DMA_InitStructure.DMA_PeripheralBaseAddr = (uint32_t)&USART2->DATAR; // USART2 数据寄存器地址
    DMA_InitStructure.DMA_MemoryBaseAddr = 0; // 发送缓冲区地址，后续发送时设置
    DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralDST; // 外设为目的地
    DMA_InitStructure.DMA_BufferSize = 0; // 发送数据长度，后续发送时设置
    DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable; // 外设地址不变
    DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable; // 内存地址递增
    DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte; // 外设数据大小为字节
    DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte; // 内存数据大小为字节
    DMA_InitStructure.DMA_Mode = DMA_Mode_Normal; // 普通模式   
    DMA_InitStructure.DMA_Priority = DMA_Priority_High; // 高优先级
    DMA_InitStructure.DMA_M2M = DMA_M2M_Disable; // 非内存到内存传输
    DMA_Init(DMA1_Channel7, &DMA_InitStructure);

    DMA_ITConfig(DMA1_Channel7, DMA_IT_TC, ENABLE); // 传输完成中断

    // 配置 DMA1 Channel7 中断
    NVIC_InitStructure.NVIC_IRQChannel = DMA1_Channel7_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 7;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);
    return DMA_OK;

}

//定时器初始化函数//周期为2S
SensorTimStatus_t Sensor_TIM_Init(void)
{
    TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure={0};
    NVIC_InitTypeDef NVIC_InitStructure={0};

    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, ENABLE);

    TIM_TimeBaseStructure.TIM_Prescaler = PSC; // 72MHz / 7200 = 10kHz
    TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
    TIM_TimeBaseStructure.TIM_Period = ARR; // 10kHz / 20000 = 0.5Hz (2秒周期)
    TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1;
    TIM_TimeBaseStructure.TIM_RepetitionCounter = 0;
    TIM_TimeBaseInit(SENSOR_TIM, &TIM_TimeBaseStructure);
    TIM_ITConfig(SENSOR_TIM, TIM_IT_Update, ENABLE);
    
    // 配置 TIM2 中断
    NVIC_InitStructure.NVIC_IRQChannel = TIM2_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 7;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);

    TIM_Cmd(SENSOR_TIM, ENABLE);

    return SENSOR_TIM_OK;
}





//外设初始化函数，负责调用各个外设的初始化函数，并进行必要的错误检查
uint8_t  g_peripheral_init()
{

    printf("[System] Initializing peripherals*****************************\n");
    // 初始化USART与ESP8266通信
    usart_wifi_esp_Status_t usart_ret;
    usart_ret = USART_WIFI_ESP_Init();
    if(usart_ret != USART_WIFI_ESP_OK) {
         printf("[Error] USART_WIFI_ESP Init Failed\n");
         return 0xFF;
    }
    
    // 初始化DMA
    DMA_Status_t dma_ret;
    dma_ret = USART_DMA_Init();
    if(dma_ret != DMA_OK) {
         printf("[Error] DMA Init Failed\n");
         return 0xFF;
    }
    

    // 初始化传感器采集定时器
    SensorTimStatus_t tim_ret;
    tim_ret = Sensor_TIM_Init();
    if(tim_ret != SENSOR_TIM_OK) {
         printf("[Error] Sensor TIM Init Failed\n");
         return 0xFF;
    }       
   

#if 0// SHT40 Handler 初始化
sht40_handler_start();
#endif 

#if 0
    MLX90614_Handler_start();
#endif

#if 1   // MAX30102 Handler 初始化
printf("[System] starting MAX30102...\n");
bsp_max30102_port_init();
bsp_max30102_handler_init();
#endif

#if 0   //电池检测模块初始化
bsp_battery_port_init();
bsp_battery_task_init();
#endif

    //信号量用于DMA传输完成通知
    xDMA_Sem = xSemaphoreCreateBinary();
    if (xDMA_Sem == NULL) {
    printf("[Error] Failed to create DMA semaphore\n");
    return 0xFF;
    }
 
    xSemaphoreGive(xDMA_Sem); // 初始状态为可用
    
    //信号量用于缓冲区数据通知
    xBufferDataSemaphore = xSemaphoreCreateBinary();
    if (xBufferDataSemaphore == NULL) {
        printf("[Error] Failed to create buffer data semaphore\n");
        return 0xFF;
    }
    //ring_buffer初始化
    RingBuffer_Init(&g_ring_buffer);
    printf("Ring Buffer Init SUCCESS\n");
   
    //事件组初始化
    xEventGroup = xEventGroupCreate();
    if (xEventGroup == NULL) {
        printf("[Error] Failed to create event group\n");
        return 0xFF;
    }
   

    return 0x32;
}   