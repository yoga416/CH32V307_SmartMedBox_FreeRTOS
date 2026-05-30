#include "system.h"
#include "debug.h"
#include "app_task.h"
#include "Middle_ring_buffer.h"

/* --- 引入所有底层外设与传感器的头文件 --- */
#include "../bsp/usart_wifi_esp/usart_wifi_esp.h"
#include "sht40_port.h"
#include "sht40_handler.h"
#include "../bsp/bsp_MLX_90614/bsp_MLX_90614_port.h"
#include "../bsp/bsp_MLX_90614/bsp_MLX_90614_handler.h"
#include "../bsp/bsp_max_30102/bsp_max_30102_port.h"
#include "../bsp/bsp_max_30102/bsp_max_30102_handler.h"
#include "../bsp/bsp_battery_check/bsp_battery_port.h"
#include "../bsp/bsp_battery_check/bsp_battery_driver.h"
#include "bsp_tianwen_reg.h"
// 如果有电池 handler 头文件请取消注释
/////////////////////////////////////////
#include "../bsp_rtc/bsp_rtc.h"      // 解决 BSP_RTC_Init 报错
#include "lcd.h"                     // 解决 LCD_Init, WHITE, lcddev 报错
#include "touch.h"                   // 解决触控相关报错
#include "ctpiic.h"                  // 解决 IIC 报错
#include "lvgl.h"                    // 解决 lv_init 报错
#include "lv_port_disp.h"            // 解决 lv_port_disp_init 报错
#include "lv_port_indev.h"           // 解决 lv_port_indev_init 报错
#include "gui_guider.h"              // 解决 setup_ui 报错
#include "custom.h"                  // LVGL 自定义 UI 头文件
#include "../bsp/bsp_lcd_lvgl/General_File/system.h"  // system_Init()
#include "spi_w25q.h"                // W25Q_Init, W25Q_ReadData
/* 全局变量定义 */
lv_ui guider_ui;
// ========================================================
/* --- 外部变量引入 --- */
extern QueueHandle_t xUART_Queue;        // 消息队列句柄
extern SemaphoreHandle_t xDMA_Sem;       // DMA发送完成信号量
extern SemaphoreHandle_t xBufferDataSemaphore;  // 缓冲区数据信号量
extern QueueHandle_t sensor_data_queue;  // 传感器数据队列句柄
extern RingBuffer_t g_ring_buffer;       // 全局环形缓冲区实例
extern QueueHandle_t CmdQueue;          // 系统命令队列句柄
extern QueueHandle_t sendtianwenQueue;       // 系统命令队列句柄
extern volatile uint8_t a7670c_ready; // 在A7670C初始化完成后置1
extern QueueHandle_t sensor_data_lcd_queue; // 传感器数据队列
extern SemaphoreHandle_t xSem_face_recog; // 人脸识别信号量

//事件定义
QueueHandle_t xAppEventQueue = NULL;
/* --- 全局句柄定义 --- */
EventGroupHandle_t xEventGroup = NULL; // 事件组句柄
 SystemCommand_t received_cmd;
/* --- 定时器参数宏 --- */
#define ARR  (20000 - 1) // 定时器自动重装载值，决定定时周期
#define PSC  (7200 - 1)  // 定时器预分频值，决定定时器计数频率


/* =========================================================================
 * 内部私有初始化函数
 * ========================================================================= */

// 1. WiFi串口初始化
static usart_wifi_esp_Status_t USART_WIFI_ESP_Init(void)
{
    USART_InitTypeDef USART_InitStructure={0};
    GPIO_InitTypeDef GPIO_InitStructure={0};
    
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART2, ENABLE);
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOD | RCC_APB2Periph_AFIO, ENABLE);
    GPIO_PinRemapConfig(GPIO_Remap_USART2, ENABLE);

    GPIO_InitStructure.GPIO_Pin = DATA_USART_TX_PIN;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP; 
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(DATA_USART_PORT, &GPIO_InitStructure);

    GPIO_InitStructure.GPIO_Pin = DATA_USART_RX_PIN;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING; 
    GPIO_Init(DATA_USART_PORT, &GPIO_InitStructure);

    USART_InitStructure.USART_BaudRate = USART_WIFI_ESP_BAUDRATE;
    USART_InitStructure.USART_WordLength = USART_WordLength_8b;
    USART_InitStructure.USART_StopBits = USART_StopBits_1;
    USART_InitStructure.USART_Parity = USART_Parity_No;
    USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
    USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
    USART_Init(USART2, &USART_InitStructure);

    // --- 新增：配置 USART2 中断优先级并开启接收中断 ---
    NVIC_InitTypeDef NVIC_InitStructure = {0};
    NVIC_InitStructure.NVIC_IRQChannel = USART2_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 6; // 比天问任务低一点，或者根据系统需求调整
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);

    USART_ITConfig(USART2, USART_IT_RXNE, ENABLE);

    USART_Cmd(USART2, ENABLE);
    return USART_WIFI_ESP_OK;
}// 天问串口初始化 (USART3完全重映射: TX=PD8, RX=PD9)
static void  USART_TIANWEN_Init(void)
{
    USART_InitTypeDef USART_InitStructure={0};
    GPIO_InitTypeDef GPIO_InitStructure={0};
    NVIC_InitTypeDef NVIC_InitStructure={0}; 
    
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART3, ENABLE);
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOD | RCC_APB2Periph_AFIO, ENABLE);
    GPIO_PinRemapConfig(GPIO_FullRemap_USART3, ENABLE);

    // 配置 TX (PD8)
    GPIO_InitStructure.GPIO_Pin = TW_USART_TX_PIN; 
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP; 
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(TW_USART_PORT, &GPIO_InitStructure);

    // 配置 RX (PD9)
    GPIO_InitStructure.GPIO_Pin = TW_USART_RX_PIN; 
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING; 
    GPIO_Init(TW_USART_PORT, &GPIO_InitStructure);

    // USART3 参数配置
    USART_InitStructure.USART_BaudRate = 115200; 
    USART_InitStructure.USART_WordLength = USART_WordLength_8b;
    USART_InitStructure.USART_StopBits = USART_StopBits_1;
    USART_InitStructure.USART_Parity = USART_Parity_No;
    USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
    USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
    USART_Init(USART3, &USART_InitStructure);

    // --- 新增：配置 USART3 中断优先级 ---
    // 警告：在 FreeRTOS 中，调用 FreeRTOS API 的中断，其抢占优先级必须大于或等于 configLIBRARY_MAX_SYSCALL_INTERRUPT_PRIORITY (通常设为 5)
    // 这里的优先级数字越大，优先级越低。这里设为 5 比较安全。
    NVIC_InitStructure.NVIC_IRQChannel = USART3_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 5; 
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);

    // --- 新增：开启接收中断 (RXNE) ---
    USART_ITConfig(USART3, USART_IT_RXNE, ENABLE);

    USART_Cmd(USART3, ENABLE);
}
// 2. USART的DMA初始化
static uint8_t USART_DMA_Init(void)
{
    DMA_InitTypeDef DMA_InitStructure={0};
    NVIC_InitTypeDef NVIC_InitStructure={0};

    RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMA1, ENABLE);

    DMA_InitStructure.DMA_PeripheralBaseAddr = (uint32_t)&USART2->DATAR;
    DMA_InitStructure.DMA_MemoryBaseAddr = 0; 
    DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralDST; 
    DMA_InitStructure.DMA_BufferSize = 0; 
    DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable; 
    DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable; 
    DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte; 
    DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte; 
    DMA_InitStructure.DMA_Mode = DMA_Mode_Normal;    
    DMA_InitStructure.DMA_Priority = DMA_Priority_High; 
    DMA_InitStructure.DMA_M2M = DMA_M2M_Disable; 
    DMA_Init(DMA1_Channel7, &DMA_InitStructure);

    DMA_ITConfig(DMA1_Channel7, DMA_IT_TC, ENABLE); 

    NVIC_InitStructure.NVIC_IRQChannel = DMA1_Channel7_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 7;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);
    
    return 0;
}

// 3. 传感器定时器初始化 (周期为2S)
static SensorTimStatus_t Sensor_TIM_Init(void)
{
    TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure={0};
    NVIC_InitTypeDef NVIC_InitStructure={0};

    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, ENABLE);

    TIM_TimeBaseStructure.TIM_Prescaler = PSC; 
    TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
    TIM_TimeBaseStructure.TIM_Period = ARR; 
    TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1;
    TIM_TimeBaseStructure.TIM_RepetitionCounter = 0;
    TIM_TimeBaseInit(SENSOR_TIM, &TIM_TimeBaseStructure);
    TIM_ITConfig(SENSOR_TIM, TIM_IT_Update, ENABLE);
    
    NVIC_InitStructure.NVIC_IRQChannel = TIM2_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 7;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);

    TIM_Cmd(SENSOR_TIM, ENABLE);
    return SENSOR_TIM_OK;
}


/* =========================================================================
 * 全局外设初始化统一入口
 * ========================================================================= */
uint8_t g_peripheral_init(void)
{

    /* 1. 基础通信外设初始化 */
    if(USART_WIFI_ESP_Init() != USART_WIFI_ESP_OK) {
         printf("[Error] USART_WIFI_ESP Init Failed\n");
         return SYSTEM_INIT_FAIL;
    }
    if(USART_DMA_Init() != 0) {
         printf("[Error] DMA Init Failed\n");
         return SYSTEM_INIT_FAIL;
    }
    if(Sensor_TIM_Init() != SENSOR_TIM_OK) {
         printf("[Error] Sensor TIM Init Failed\n");
         return SYSTEM_INIT_FAIL;
    }

    //天问初始化
    USART_TIANWEN_Init();

     /* 2. I2C总线初始化 */
    /* 3. 操作系统同步原语初始化 */
    xDMA_Sem = xSemaphoreCreateBinary();
    if(xDMA_Sem != NULL) xSemaphoreGive(xDMA_Sem);
    
    xBufferDataSemaphore = xSemaphoreCreateBinary();
    xEventGroup = xEventGroupCreate();
    RingBuffer_Init(&g_ring_buffer);
        if(xEventGroup == NULL) {
            printf("[Error] Event Group Create Failed\n");
        }

    // WiFi 接收队列创建
    xUART_Queue = xQueueCreate(128, sizeof(uint8_t));
    if (xUART_Queue == NULL) {
        printf("[Error] Failed to create xUART_Queue!\n");
    }

    // 系统命令队列创建
    CmdQueue = xQueueCreate(10, sizeof(SystemCommand_t));
    if (CmdQueue == NULL) {
        printf("[Error] Failed to create CmdQueue!\n");
    }
    // 天问通信命令队列创建
    sendtianwenQueue = xQueueCreate(10, sizeof(Tianwen_Packet_t));
    if (sendtianwenQueue == NULL) {
        printf("[Error] Failed to create sendtianwenQueue!\n");
    }
    
      xSysCmdQueue = xQueueCreate(10, sizeof(SystemCommand_t));
    if (xSysCmdQueue == NULL) {
        printf("[Error] Failed to create SysCmdQueue!\n");
        vTaskDelete(NULL);
    }
    //传感器数据采集完成发送到lcd显示的队列
    sensor_data_lcd_queue = xQueueCreate(10, sizeof(Tianwen_Packet_t));
    if (sensor_data_lcd_queue == NULL) {
        printf("[Error] Failed to create sensor_data_lcd_queue!\n");
        return SYSTEM_INIT_FAIL;
    }

     // 人脸识别信号量创建
    xSem_face_recog = xSemaphoreCreateBinary();

     /* 4. 传感器和其他外设初始化 */
    // 创建应用层统一事件队列，深度设为 10 足够了
    xAppEventQueue = xQueueCreate(10, sizeof(AppMsg_t));
    if (xAppEventQueue == NULL) {
        printf("[Error] Failed to create xAppEventQueue!\n");
        // 最好在这里加个错误处理
    }
    // 创建 LVGL GUI 互斥锁（必须在 LCD 任务之前创建）
    xGuiMutex = xSemaphoreCreateMutex();
    if (xGuiMutex == NULL) {
        printf("[Error] Create GUI mutex failed\n");
        while (1) {}
    }
    

    FR_Init(); // 初始化人脸识别模块
 
     /* 4. 传感器和其他外设初始化 */
    // SHT40 温湿度
    sht40_handler_start();  
    
    // MLX90614 红外测温
    MLX90614_Handler_start();
    
    // MAX30102 引脚和中断初始化
    bsp_max30102_port_init();

    // MAX30102 Handler 初始化（创建任务等）
    bsp_max30102_handler_init();
    
    // 电池电量检测
    // 注意：如果有 handler 启动函数也可以加在这里，目前只看到你写了这两个
    bsp_battery_port_init();
    // bsp_battery_task_init(); 
    
    // //rtc初始化
    BSP_RTC_Init();

    printf("\r\n========================================\r\n");
    printf("CH32V307 LVGL + GUI Guider Test\r\n");
    printf("========================================\r\n");
    
    // 初始化系统相关外设（如LED等）
    system_Init();
    
    // ========== 初始化并测试 LCD 显示屏 ==========
    LCD_Init();          // 初始化 LCD 显示屏
    /* [DEBUG] 强制打开背光，排除软件背光控制问题 */
    //GPIO_SetBits(GPIOA, GPIO_Pin_1);
    //LCD_LED_ON;

    LCD_Clear(WHITE);    // 用白色清屏，检查 LCD 显示是否正常

    // 设置显示字符串的颜色参数并在 LCD 上显示测试信息
    POINT_COLOR = BLACK;
    BACK_COLOR = WHITE;
    LCD_ShowString(50, 50, (u8*)"WANG YUE TING IS HERE!", BLUE, WHITE, 16);
    Delay_Ms(2000);
    
    // 初始化触摸屏的 I2C 接口
    CTP_IIC_Init();
    
    if(FT6336_Init() == 0) {
        //printf("FT6336 OK!\r\n");
    } else {
        printf("FT6336 Failed!\r\n");
    }
    
    /* 初始化 SPI2 接口和 W25Q 外部闪存 */
    W25Q_Init();

    // /* ========== 烧录字体到 W25Q 闪存（首次使用时取消注释） ========== */
    // printf("Erasing W25Q for font...\r\n");
    // // 1. 擦除整个芯片（所有扇区）
    // W25Q_ChipErase();
    // // 2. 将中文字体（宋体）烧录到地址 0x000000
    // W25Q_WriteBuffer(0x000000, (uint8_t*)han_bitmap, han_bitmap_size);
    // // 3. 将 12px 英文字体烧录到地址 0x020000
    // W25Q_WriteBuffer(0x020000, (uint8_t*)m12_bitmap, m12_bitmap_size);
    // // 4. 将 20px 英文字体烧录到地址 0x030000
    // W25Q_WriteBuffer(0x030000, (uint8_t*)m20_bitmap, m20_bitmap_size);
    // printf("All fonts burned!\r\n");
    // while(1); // 烧录完成后停止，观察结果
    // /* ========== 烧录结束 ========== */
    
    printf("[LCD] Starting LVGL Initialization in Task...\r\n");

    lv_init();
    lv_port_disp_init();  // 初始化显示接口
    lv_port_indev_init(); // 初始化输入设备接口
    printf("[LCD] Free Heap after LVGL: %u B\n", (unsigned int)xPortGetFreeHeapSize());
    // 加载 UI
    setup_ui(&guider_ui);
    printf("[LCD] UI Setup Done\r\n");
    
    return SYSTEM_INIT_OK;
}