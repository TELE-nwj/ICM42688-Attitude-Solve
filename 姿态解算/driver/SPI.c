#include "SPI.h"

/*参数宏定义*********************/

/*片选引脚*/
#define SPI_CS_GPIO			GPIOA
#define SPI_CS_GPIO_PIN		GPIO_Pin_4

/*********************参数宏定义*/

/**
  * 函    数：SPI1初始化
  * 参    数：无
  * 返 回 值：无
  * 说    明：GPIO与SPI外设配置，工作于主模式，SPI Mode 0（CPOL=0，CPHA=0）
  *           时钟为APB2=72MHz，分频4，即SCLK=18MHz（低于ICM-42688-P的24MHz上限）
  */
void SPI1_Init(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;
	SPI_InitTypeDef SPI_InitStructure;

	/*开启GPIOA和SPI1的时钟*/
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA | RCC_APB2Periph_SPI1, ENABLE);

	/*PA4：CS片选，推挽输出*/
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_4;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOA, &GPIO_InitStructure);

	/*PA5：SCK，复用推挽输出*/
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_5;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOA, &GPIO_InitStructure);

	/*PA6：MISO，浮空输入*/
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOA, &GPIO_InitStructure);

	/*PA7：MOSI，复用推挽输出*/
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_7;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOA, &GPIO_InitStructure);

	/*SPI1初始化*/
	SPI_InitStructure.SPI_Direction = SPI_Direction_2Lines_FullDuplex;		//全双工
	SPI_InitStructure.SPI_Mode = SPI_Mode_Master;							//主模式
	SPI_InitStructure.SPI_DataSize = SPI_DataSize_8b;						//8位数据
	SPI_InitStructure.SPI_CPOL = SPI_CPOL_Low;								//空闲为低电平（Mode 0）
	SPI_InitStructure.SPI_CPHA = SPI_CPHA_1Edge;							//第一个沿采样（Mode 0）
	SPI_InitStructure.SPI_NSS = SPI_NSS_Soft;								//软件NSS
	SPI_InitStructure.SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_4;		//分频4，72/4=18MHz
	SPI_InitStructure.SPI_FirstBit = SPI_FirstBit_MSB;						//MSB先发
	SPI_InitStructure.SPI_CRCPolynomial = 7;								//CRC多项式，未使用
	SPI_Init(SPI1, &SPI_InitStructure);

	/*使能SPI1*/
	SPI_Cmd(SPI1, ENABLE);

	/*片选默认拉高*/
	SPI_CS_High();
}

/**
  * 函    数：SPI单字节收发
  * 参    数：TxData：待发送的字节
  * 返 回 值：接收到的字节
  * 说    明：发送一个字节，同时返回MISO上接收到的字节
  */
uint8_t SPI_ReadWriteByte(uint8_t TxData)
{
	/*等待发送缓冲区为空*/
	while (SPI_I2S_GetFlagStatus(SPI1, SPI_I2S_FLAG_TXE) == RESET);

	/*发送一个字节*/
	SPI_I2S_SendData(SPI1, TxData);

	/*等待接收缓冲区非空*/
	while (SPI_I2S_GetFlagStatus(SPI1, SPI_I2S_FLAG_RXNE) == RESET);

	/*返回接收数据*/
	return SPI_I2S_ReceiveData(SPI1);
}

/**
  * 函    数：片选拉低（选中从机）
  * 参    数：无
  * 返 回 值：无
  */
void SPI_CS_Low(void)
{
	GPIO_ResetBits(SPI_CS_GPIO, SPI_CS_GPIO_PIN);
}

/**
  * 函    数：片选拉高（释放从机）
  * 参    数：无
  * 返 回 值：无
  */
void SPI_CS_High(void)
{
	GPIO_SetBits(SPI_CS_GPIO, SPI_CS_GPIO_PIN);
}
