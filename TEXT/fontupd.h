#ifndef __FONTUPD_H__
#define __FONTUPD_H__	 
#include <stm32f10x.h>
//////////////////////////////////////////////////////////////////////////////////	 
//������ֻ��ѧϰʹ�ã�δ��������ɣ��������������κ���;
//ALIENTEK MiniSTM32������
//�ֿ���� ��������	   
//����ԭ��@ALIENTEK
//������̳:www.openedv.com
//�޸�����:2014/3/14
//�汾��V1.0
//��Ȩ���У�����ؾ���
//Copyright(C) ������������ӿƼ����޹�˾ 2009-2019
//All rights reserved									  
//////////////////////////////////////////////////////////////////////////////////
//�ⲿflash��ַ˵��(8M)
//0--5M theme����
//5M--6M marlin�������
//6M--6M+500K Icon ͼ��
//6M+500K---8M ����
//������Ϣ�����ַ,ռ33���ֽ�,��1���ֽ����ڱ���ֿ��Ƿ����.����ÿ8���ֽ�һ��,�ֱ𱣴���ʼ��ַ���ļ���С														   
#define FONTINFOADDR  	(u32)(1024*6+100)*1024 //6803456 	//(1024*6+100)*1024;//Ĭ����6M + 500K�ĵ�ַ//0x67D000
#define FONTINFOADDR2  	(u32)(1024*6+500)*1024 
#define ICON_START_ADDR (u32)6291456 //6*1024*1024;//ͼ��洢�׵�ַ��ͼ����������Ϣ��
#define THEME_ADDR  	(u32)0 	//0;//0��ʼ��ַ
//#define THEME_ADDR  	(u32)4096 	//0;//0��ʼ��ַ
//�ֿ���Ϣ�ṹ�嶨��
//���������ֿ������Ϣ����ַ����С�� ��33���ֽ�
__packed typedef struct 
{
	u8 fontok;				//�ֿ���ڱ�־��0XAA���ֿ��������������ֿⲻ����
	u32 ugbkaddr; 			//unigbk�ĵ�ַ
	u32 ugbksize;			//unigbk�Ĵ�С	 
	u32 f12addr;			//gbk12��ַ	
	u32 gbk12size;			//gbk12�Ĵ�С	 
	u32 f16addr;			//gbk16��ַ
	u32 gbk16size;			//gbk16�Ĵ�С	 
	u32 icon_codeaddr;			//icon_code��ַ//20150909
	u32 icon_codesize;			//icon_code�Ĵ�С//20150909
}_font_info;																   

extern _font_info ftinfo;	//�ֿ���Ϣ�ṹ��
__packed typedef struct 
{
	u8 themeok;				//�ֿ���ڱ�־��0XBB���ֿ��������������ֿⲻ����
	u32 themeaddr;			//icon_code��ַ//20150909
	u32 themesize;			//icon_code�Ĵ�С//20150909
}_theme_info;		
extern _theme_info themeinfo;	//������Ϣ�ṹ��

u32 fupd_prog(u16 x,u16 y,u8 size,u32 fsize,u32 pos);//��ʾ���½���
u8 updata_fontx(u16 x,u16 y,u8 size,u8 *fxpath,u8 fx);//����ָ���ֿ�
u8 update_font(u16 x,u16 y,u8 size,u8 src);//����ȫ���ֿ�
u8 font_init(void);//��ʼ���ֿ�
u8 update_theme(u16 x,u16 y,u8 size);//��������
u8 theme_init(void);
#endif




















