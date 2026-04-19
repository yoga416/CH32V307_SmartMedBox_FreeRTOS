#ifndef _DMA_H
#define _DMA_H

//DEFINE DMA相关的函数和数据结构
#include "ch32v30x.h"

typedef enum {
    DMA_OK = 0,
    DMA_ERROR = -1,
    DMA_BUSY = 1,
} DMA_Status_t;


#endif // !_DMA_H
