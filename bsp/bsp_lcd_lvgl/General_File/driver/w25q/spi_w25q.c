#include "spi_w25q.h"

/* SPI2 收发一个字节 */
static uint8_t SPI2_ReadWriteByte(uint8_t data)
{
    while (SPI_I2S_GetFlagStatus(SPI2, SPI_I2S_FLAG_TXE) == RESET);
    SPI_I2S_SendData(SPI2, data);
    while (SPI_I2S_GetFlagStatus(SPI2, SPI_I2S_FLAG_RXNE) == RESET);
    return SPI_I2S_ReceiveData(SPI2);
}

/* 初始化 SPI2 + CS 引脚 */
void W25Q_Init(void)
{
    GPIO_InitTypeDef GPIO_InitStructure = {0};
    SPI_InitTypeDef  SPI_InitStructure  = {0};

    /* 时钟：SPI2 在 APB1，GPIOB 和 GPIOD 在 APB2 */
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_SPI2, ENABLE);
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB | RCC_APB2Periph_GPIOD, ENABLE);

    /* CS → PD1 推挽输出 */
    GPIO_InitStructure.GPIO_Pin   = W25Q_CS_PIN;
    GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_Out_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(W25Q_CS_PORT, &GPIO_InitStructure);
    W25Q_CS_HIGH();

    /* SCK → PB13 */
    GPIO_InitStructure.GPIO_Pin  = GPIO_Pin_13;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
    GPIO_Init(GPIOB, &GPIO_InitStructure);

    /* MOSI → PB15 */
    GPIO_InitStructure.GPIO_Pin  = GPIO_Pin_15;
    GPIO_Init(GPIOB, &GPIO_InitStructure);

    /* MISO → PB14 浮空输入 */
    GPIO_InitStructure.GPIO_Pin  = GPIO_Pin_14;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
    GPIO_Init(GPIOB, &GPIO_InitStructure);

    /* SPI2 配置 */
    SPI_InitStructure.SPI_Direction      = SPI_Direction_2Lines_FullDuplex;
    SPI_InitStructure.SPI_Mode           = SPI_Mode_Master;
    SPI_InitStructure.SPI_DataSize       = SPI_DataSize_8b;
    SPI_InitStructure.SPI_CPOL           = SPI_CPOL_Low;    // W25Q 模式 0: CPOL=0, CPHA=0
    SPI_InitStructure.SPI_CPHA           = SPI_CPHA_1Edge;
    SPI_InitStructure.SPI_NSS            = SPI_NSS_Soft;
    SPI_InitStructure.SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_4;
    SPI_InitStructure.SPI_FirstBit       = SPI_FirstBit_MSB;
    SPI_InitStructure.SPI_CRCPolynomial  = 7;
    SPI_Init(SPI2, &SPI_InitStructure);

    SPI_Cmd(SPI2, ENABLE);
}

/* 读芯片 ID */
uint16_t W25Q_ReadID(void)
{
    uint16_t id = 0;
    W25Q_CS_LOW();
    SPI2_ReadWriteByte(W25Q_CMD_READ_ID);
    id  = SPI2_ReadWriteByte(0xFF) << 8;
    id |= SPI2_ReadWriteByte(0xFF);
    W25Q_CS_HIGH();
    return id;
}

/* 写使能 */
static void W25Q_WriteEnable(void)
{
    W25Q_CS_LOW();
    SPI2_ReadWriteByte(W25Q_CMD_WRITE_ENABLE);
    W25Q_CS_HIGH();
}

/* 等待 BUSY 位清除 */
static void W25Q_WaitBusy(void)
{
    uint8_t sr;
    W25Q_CS_LOW();
    SPI2_ReadWriteByte(W25Q_CMD_READ_STATUS);
    do {
        sr = SPI2_ReadWriteByte(0xFF);
    } while (sr & 0x01);
    W25Q_CS_HIGH();
}

/* 扇区擦除 (4KB) */
void W25Q_SectorErase(uint32_t addr)
{
    W25Q_WriteEnable();
    W25Q_CS_LOW();
    SPI2_ReadWriteByte(W25Q_CMD_SECTOR_ERASE);
    SPI2_ReadWriteByte((addr >> 16) & 0xFF);
    SPI2_ReadWriteByte((addr >> 8)  & 0xFF);
    SPI2_ReadWriteByte( addr        & 0xFF);
    W25Q_CS_HIGH();
    W25Q_WaitBusy();
}

/* 整片擦除 */
void W25Q_ChipErase(void)
{
    W25Q_WriteEnable();
    W25Q_CS_LOW();
    SPI2_ReadWriteByte(W25Q_CMD_CHIP_ERASE);
    W25Q_CS_HIGH();
    W25Q_WaitBusy();  // 整片擦除耗时较长
}

/* 连续读取数据 */
void W25Q_ReadData(uint32_t addr, uint8_t *buf, uint32_t len)
{
    W25Q_CS_LOW();
    SPI2_ReadWriteByte(W25Q_CMD_READ_DATA);
    SPI2_ReadWriteByte((addr >> 16) & 0xFF);
    SPI2_ReadWriteByte((addr >> 8)  & 0xFF);
    SPI2_ReadWriteByte( addr        & 0xFF);
    for (uint32_t i = 0; i < len; i++) {
        buf[i] = SPI2_ReadWriteByte(0xFF);
    }
    W25Q_CS_HIGH();
}

/* 页编程（单次 ≤256 字节，不能跨页） */
void W25Q_WritePage(uint32_t addr, uint8_t *buf, uint16_t len)
{
    if (len == 0) return;
    W25Q_WriteEnable();
    W25Q_CS_LOW();
    SPI2_ReadWriteByte(W25Q_CMD_PAGE_PROGRAM);
    SPI2_ReadWriteByte((addr >> 16) & 0xFF);
    SPI2_ReadWriteByte((addr >> 8)  & 0xFF);
    SPI2_ReadWriteByte( addr        & 0xFF);
    for (uint16_t i = 0; i < len; i++) {
        SPI2_ReadWriteByte(buf[i]);
    }
    W25Q_CS_HIGH();
    W25Q_WaitBusy();
}

/* 任意长度写入（自动处理跨页） */
void W25Q_WriteBuffer(uint32_t addr, uint8_t *buf, uint32_t size)
{
    uint32_t remaining = size;
    uint32_t offset    = 0;

    while (remaining > 0) {
        uint32_t page_end = (addr & 0xFFFFFF00) + 256;     // 当前页尾地址
        uint32_t chunk    = page_end - addr;                // 本页剩余空间
        if (chunk > remaining) chunk = remaining;

        W25Q_WritePage(addr, buf + offset, (uint16_t)chunk);

        addr      += chunk;
        offset    += chunk;
        remaining -= chunk;
    }
}