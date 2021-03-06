#ifndef __SPI_H
#define __SPI_H
#include "sys.h"
//////////////////////////////////////////////////////////////////////////////////	 
//本程序只供学习使用，未经作者许可，不得用于其它任何用途
//Mini STM32开发板
//SPI 驱动函数	   
//正点原子@ALIENTEK
//技术论坛:www.openedv.com
//修改日期:2010/6/13 
//版本：V1.0
//版权所有，盗版必究。
//Copyright(C) 正点原子 2009-2019
//All rights reserved
////////////////////////////////////////////////////////////////////////////////// 	  

void SPI3_Init(void);			 //初始化SPI3口
void SPI3_SetSpeed(u8 SpeedSet); //设置SPI3速度   
u8 SPI3_ReadWriteByte(u8 TxData);//SPI3总线读写一个字节
void SPI3_WriteByte(u8 TxData);

void SPI3_DR_DMA_Configuration(u32 cmar,u16 cndtr);//SPI2 DMA配置	
void SPI3_DR_DMA_Enable(u16 cndtr);
void SPI3_DR_DMA_Disable(void);
u16  SPI3_DMA_GetRXCounter(u16 cndtr);
			  	    													  
void SPI2_Init(void);			 //初始化SPI2口
void SPI2_SetSpeed(u8 SpeedSet); //设置SPI2速度   
u8 SPI2_ReadWriteByte(u8 TxData);//SPI2总线读写一个字节 
void SPI2_WriteByte(u8 TxData);

void SPI1_DR_DMA_Configuration(u32 cmar,u16 cndtr);//SPI2 DMA配置	
void SPI1_DR_DMA_Enable(u16 cndtr);
void SPI1_DR_DMA_Disable(void);
u16  SPI1_DMA_GetRXCounter(u16 cndtr);

void SPI1_Init(void);			 //初始化SPI口
void SPI1_SetSpeed(u8 SpeedSet); //设置SPI速度   
u8 SPI1_ReadWriteByte(u8 TxData);//SPI总线读写一个字节
void SPI1_WriteByte(u8 TxData);//SPI总线写一个字节		 
#endif

