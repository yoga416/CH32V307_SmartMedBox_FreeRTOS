#ifndef __SPI_W25Q_H
#define __SPI_W25Q_H

#include "debug.h"

/* CS 引脚定义 - PD1 */
#define W25Q_CS_PORT        GPIOD
#define W25Q_CS_PIN         GPIO_Pin_1
#define W25Q_CS_LOW()       GPIO_ResetBits(W25Q_CS_PORT, W25Q_CS_PIN)
#define W25Q_CS_HIGH()      GPIO_SetBits(W25Q_CS_PORT, W25Q_CS_PIN)

/* W25Q 指令 */
#define W25Q_CMD_READ_ID        0x9F
#define W25Q_CMD_READ_STATUS    0x05
#define W25Q_CMD_WRITE_ENABLE   0x06
#define W25Q_CMD_READ_DATA      0x03
#define W25Q_CMD_PAGE_PROGRAM   0x02
#define W25Q_CMD_SECTOR_ERASE   0x20
#define W25Q_CMD_CHIP_ERASE     0xC7

void W25Q_Init(void);
uint16_t W25Q_ReadID(void);
void W25Q_ReadData(uint32_t addr, uint8_t *buf, uint32_t len);
void W25Q_WritePage(uint32_t addr, uint8_t *buf, uint16_t len);
void W25Q_SectorErase(uint32_t addr);
void W25Q_ChipErase(void);
void W25Q_WriteBuffer(uint32_t addr, uint8_t *buf, uint32_t size);

#endif