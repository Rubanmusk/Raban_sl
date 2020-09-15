#ifndef __SPI_H
#define __SPI_H
#include "sys.h"
//////////////////////////////////////////////////////////////////////////////////	 
//������ֻ��ѧϰʹ�ã�δ��������ɣ��������������κ���;
//Mini STM32������
//SPI ��������	   
//����ԭ��@ALIENTEK
//������̳:www.openedv.com
//�޸�����:2010/6/13 
//�汾��V1.0
//��Ȩ���У�����ؾ���
//Copyright(C) ����ԭ�� 2009-2019
//All rights reserved
////////////////////////////////////////////////////////////////////////////////// 	  

void SPI3_Init(void);			 //��ʼ��SPI3��
void SPI3_SetSpeed(u8 SpeedSet); //����SPI3�ٶ�   
u8 SPI3_ReadWriteByte(u8 TxData);//SPI3���߶�дһ���ֽ�
void SPI3_WriteByte(u8 TxData);

void SPI3_DR_DMA_Configuration(u32 cmar,u16 cndtr);//SPI2 DMA����	
void SPI3_DR_DMA_Enable(u16 cndtr);
void SPI3_DR_DMA_Disable(void);
u16  SPI3_DMA_GetRXCounter(u16 cndtr);
			  	    													  
void SPI2_Init(void);			 //��ʼ��SPI2��
void SPI2_SetSpeed(u8 SpeedSet); //����SPI2�ٶ�   
u8 SPI2_ReadWriteByte(u8 TxData);//SPI2���߶�дһ���ֽ� 
void SPI2_WriteByte(u8 TxData);

void SPI1_DR_DMA_Configuration(u32 cmar,u16 cndtr);//SPI2 DMA����	
void SPI1_DR_DMA_Enable(u16 cndtr);
void SPI1_DR_DMA_Disable(void);
u16  SPI1_DMA_GetRXCounter(u16 cndtr);

void SPI1_Init(void);			 //��ʼ��SPI��
void SPI1_SetSpeed(u8 SpeedSet); //����SPI�ٶ�   
u8 SPI1_ReadWriteByte(u8 TxData);//SPI���߶�дһ���ֽ�
void SPI1_WriteByte(u8 TxData);//SPI����дһ���ֽ�		 
#endif

