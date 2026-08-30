#ifndef __SPI_H
#define __SPI_H

#include "stm32f10x.h"

/*功能声明*********************/

/*初始化方函数*/
void SPI1_Init(void);

/*单字节收发函数*/
uint8_t SPI_ReadWriteByte(uint8_t TxData);

/*片选控制函数*/
void SPI_CS_Low(void);
void SPI_CS_High(void);

/*********************功能声明*/

#endif
