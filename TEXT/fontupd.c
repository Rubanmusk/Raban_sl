#include "fontupd.h"
#include "ff.h"	  
#include "flash.h"   
#include "lcd.h"  
#include "malloc.h"
//////////////////////////////////////////////////////////////////////////////////	 
//������ֻ��ѧϰʹ�ã�δ��������ɣ��������������κ���;
//ALIENTEKս��STM32������
//�ֿ⡢��ɫͼ��    ���� ��������	   
//����ԭ��@ALIENTEK
//������̳:www.openedv.com
//�޸�����:20150909
//�汾��V1.0
/*�汾��V1.1
				�޸�˵������fontupd�ļ���Ȼ��Ϊ������£��������Ǹ����ļ������������������ļ���Ҳ���������ļ���
				��1.1�汾������ͼ��icon_code.bin�ļ��ĸ��£�20150909��
*/
//��Ȩ���У�����ؾ���
//Copyright(C) ������������ӿƼ����޹�˾ 2009-2019
//All rights reserved									  
//////////////////////////////////////////////////////////////////////////////////
						   
//�ֿ⡢��ɫͼ�� ��Ϣ�ṹ��. 
//���������ֿ������Ϣ����ַ����С��
_font_info ftinfo;
_theme_info themeinfo;
//��sd���е�·��
const u8 *GBK16_SDPATH="0:/SYSTEM/FONT/GBK16.FON";		//GBK16�Ĵ��λ��
const u8 *GBK12_SDPATH="0:/SYSTEM/FONT/GBK12.FON";		//GBK12�Ĵ��λ��
const u8 *UNIGBK_SDPATH="0:/SYSTEM/FONT/UNIGBK.BIN";	//UNIGBK.BIN�Ĵ��λ��

const u8 *ICON_CODE_SDPATH="0:/SYSTEM/FONT/ICON_CODE.BIN";	//icon_code.bin�Ĵ��λ��//20150909
const u8 *THEME_CODE_SDPATH="0:/SYSTEM/THEME/THEME.BIN";	//icon_code.bin�Ĵ��λ��//
//��25Qxx�е�·��
const u8 *GBK16_25QPATH="1:/SYSTEM/FONT/GBK16.FON";		//GBK16�Ĵ��λ��
const u8 *GBK12_25QPATH="1:/SYSTEM/FONT/GBK12.FON";		//GBK12�Ĵ��λ��
const u8 *UNIGBK_25QPATH="1:/SYSTEM/FONT/UNIGBK.BIN";	//UNIGBK.BIN�Ĵ��λ��

//��ʾ��ǰ������½���
//x,y:����
//size:�����С
//fsize:�����ļ���С
//pos:��ǰ�ļ�ָ��λ��
u32 fupd_prog(u16 x,u16 y,u8 size,u32 fsize,u32 pos)
{
	float prog;
	u8 t=0XFF;
	prog=(float)pos/fsize;
	prog*=100;
	if(t!=prog)
	{
		LCD_ShowString(x+3*size/2,y,240,320,size,"%");		
		t=prog;
		if(t>100)t=100;
		LCD_ShowNum(x,y,t,3,size);//��ʾ��ֵ
	}
	return 0;					    
} 
//����ĳһ��
//x,y:����
//size:�����С
//fxpath:·��
//fx:���µ����� 0,ungbk;1,gbk12;2,gbk16;
//����ֵ:0,�ɹ�;����,ʧ��.
u8 updata_fontx(u16 x,u16 y,u8 size,u8 *fxpath,u8 fx)//5,  0,  12,  "1:/SYSTEM/FONT/UNIGBK.BIN";   ��0
{
	u32 flashaddr=0;								    
	FIL * fftemp;
	u8 *tempbuf;
 	u8 res;	
	u16 bread;
	u32 offx=0;
	u8 rval=0;	  
	//�������ڴ�
	fftemp=(FIL*)mymalloc(SRAMIN,sizeof(FIL));	//�����ڴ�	  ΪĿ���ļ��ṹ������ڴ�
	if(fftemp==NULL)rval=1;
	tempbuf=mymalloc(SRAMIN,4096);	//����4096���ֽڿռ�   ΪҪд��Flash�е��ֽ�BUF����4096�ֽڵ��ڴ�
	if(tempbuf==NULL)rval=1;
	
 	res=f_open(fftemp,(const TCHAR*)fxpath,FA_READ); //��SD���д��ļ�  ��ȡ�ļ��ĸ����������Ϣ��������С��
 	if(res)rval=2;//���ļ�ʧ��  
 	if(rval==0)	 
	{
		if(fx==0)		//����UNIGBK.BIN
		{
			ftinfo.ugbkaddr=FONTINFOADDR+sizeof(ftinfo);//��Ϣͷ֮�󣬽���UNIGBKת�����
  			ftinfo.ugbksize=fftemp->fsize;				//UNIGBK��С
 			flashaddr=ftinfo.ugbkaddr;
		}else if(fx==1)	//GBK12
		{				  
			ftinfo.f12addr=ftinfo.ugbkaddr+ftinfo.ugbksize;		//UNIGBK֮�󣬽���GBK12�ֿ�
			ftinfo.gbk12size=fftemp->fsize;						//GBK12�ֿ��С
			flashaddr=ftinfo.f12addr;							//GBK12����ʼ��ַ
		}else	if(fx==2)		//GBK16
		{
			ftinfo.f16addr=ftinfo.f12addr+ftinfo.gbk12size;		//GBK12֮�󣬽���GBK16�ֿ�
			ftinfo.gbk16size=fftemp->fsize;						//GBK16�ֿ��С
			flashaddr=ftinfo.f16addr;							//GBK16����ʼ��ַ
		}else	if(fx==3)		//ICON_CODE                                                                             //20150909
		{
			ftinfo.icon_codeaddr=ICON_START_ADDR;//GBK16֮�󣬽���ICON_CODE�ֿ�
			ftinfo.icon_codesize=fftemp->fsize;						//ICON_CODE�ֿ��С
			flashaddr=ftinfo.icon_codeaddr;							//ICON_CODE����ʼ��ַ
		}	   
		while(res==FR_OK)//��ѭ��ִ��
		{
	 		res=f_read(fftemp,tempbuf,4096,(UINT *)&bread);		//��SD���ļ��ж�ȡ���ݣ���ȡ �Ѿ���ȡ���ֽ���bread
			if(res!=FR_OK)break;								//ִ�д���
			SPI_Flash_Write(tempbuf,offx+flashaddr,4096);		//��0+FONTINFOADDR+sizeof(ftinfo) ��ʼд��4096������
	  		offx+=bread;
			fupd_prog(x,y,size,fftemp->fsize,offx);	 			//������ʾ
			if(bread!=4096)break;								//������.
	 	}
		f_close(fftemp);
	}
	myfree(SRAMIN,fftemp);	 //�ͷ��ڴ�
	myfree(SRAMIN,tempbuf);	 //�ͷ��ڴ�
	return res;
}
//��������
//x,y:��ʾ��Ϣ����ʾ��ַ
//size:�����С
//��ʾ��Ϣ�����С											  
//����ֵ:0,���³ɹ�;
//		 ����,�������.	  
u8 update_theme(u16 x,u16 y,u8 size)//5,0,12,0
{	
	
	u32 flashaddr=0;								    
	FIL * fftemp;
	u8 *tempbuf;	
	u16 bread;
	u32 offx=0;
	u8 rval=0;	
	
	u8 *theme_code_path;
	u8 res;		  

	theme_code_path=(u8*)THEME_CODE_SDPATH;

 	res=0XFF;
//	themeinfo.themeok=0XFF;
//  SPI_Flash_Write((u8*)&themeinfo,THEME_ADDR,sizeof(themeinfo));	//�ٴ�д��0xFF�� ��ʹ֮ǰ��0xAAҲ���¶�ȫ�����¸����ֿ�  ���֮ǰ�ֿ�ɹ��ı�־.��ֹ���µ�һ������,���µ��ֿⲿ�����ݶ�ʧ.
// 	SPI_Flash_Read((u8*)&themeinfo,THEME_ADDR,sizeof(themeinfo));	//���¶���ftinfo�ṹ������

	LCD_ShowString(x,y,240,320,size,"Updating THEME.BIN"); //20150909	
	//�������ڴ�
	fftemp=(FIL*)mymalloc(SRAMIN,sizeof(FIL));	//�����ڴ�	  ΪĿ���ļ��ṹ������ڴ�
	if(fftemp==NULL)rval=1;
	tempbuf=mymalloc(SRAMIN,4096);	//����4096���ֽڿռ�   ΪҪд��Flash�е��ֽ�BUF����4096�ֽڵ��ڴ�
	if(tempbuf==NULL)rval=1;
	
 	res=f_open(fftemp,(const TCHAR*)theme_code_path,FA_READ); //��SD���д������ļ�
	
 	if(res)rval=2;//���ļ�ʧ��  
 	if(rval==0)	 
	{
			themeinfo.themeok=0XFF;
			SPI_Flash_Write((u8*)&themeinfo,THEME_ADDR,sizeof(themeinfo));	//�ٴ�д��0xFF�� ��ʹ֮ǰ��0xAAҲ���¶�ȫ�����¸����ֿ�  ���֮ǰ�ֿ�ɹ��ı�־.��ֹ���µ�һ������,���µ��ֿⲿ�����ݶ�ʧ.
			SPI_Flash_Read((u8*)&themeinfo,THEME_ADDR,sizeof(themeinfo));	//���¶���ftinfo�ṹ������
		
		
//		LCD_ShowString(x,y+12,240,320,size,"1"); //20150909	
			themeinfo.themeaddr=THEME_ADDR+sizeof(themeinfo);//��Ϣͷ֮��
  		themeinfo.themesize=fftemp->fsize;				//UNIGBK��С
 			flashaddr=themeinfo.themeaddr;

 
		while(res==FR_OK)//��ѭ��ִ��
		{
	 		res=f_read(fftemp,tempbuf,4096,(UINT *)&bread);		//��SD���ļ��ж�ȡ���ݣ���ȡ �Ѿ���ȡ���ֽ���bread
			if(res!=FR_OK)break;								//ִ�д���
			SPI_Flash_Write(tempbuf,offx+flashaddr,4096);		//��0+FONTINFOADDR+sizeof(ftinfo) ��ʼд��4096������
	  		offx+=bread;
			fupd_prog(x+20*size/2,y,size,fftemp->fsize,offx);	 			//������ʾ
			if(bread!=4096)break;								//������.
	 	}
		f_close(fftemp);
	}
	myfree(SRAMIN,fftemp);	 //�ͷ��ڴ�
	myfree(SRAMIN,tempbuf);	 //�ͷ��ڴ�

	if(res)return 1;
	
	//ȫ�����º���
	themeinfo.themeok=0XBB;
  SPI_Flash_Write((u8*)&themeinfo,THEME_ADDR,sizeof(themeinfo));	//����������Ϣ
	return 0;//�޴���.		 
}
//���������ļ�,UNIGBK,GBK12,GBK16һ�����
//x,y:��ʾ��Ϣ����ʾ��ַ
//size:�����С
//��ʾ��Ϣ�����С
//src:0,��SD������.
//	  1,��25QXX����											  
//����ֵ:0,���³ɹ�;
//		 ����,�������.	  
u8 update_font(u16 x,u16 y,u8 size,u8 src)//5,0,12,0
{	
	u8 *gbk16_path;
	u8 *gbk12_path;
	u8 *unigbk_path;
	u8 *icon_code_path;
	u8 res;		  
	if(src)//��25qxx����
	{
		unigbk_path=(u8*)UNIGBK_25QPATH;
		gbk12_path=(u8*)GBK12_25QPATH;
		gbk16_path=(u8*)GBK16_25QPATH;
		gbk16_path=(u8*)GBK16_25QPATH;
	}else//��sd������
	{
		unigbk_path=(u8*)UNIGBK_SDPATH;
		gbk12_path=(u8*)GBK12_SDPATH;
		gbk16_path=(u8*)GBK16_SDPATH;
		icon_code_path=(u8*)ICON_CODE_SDPATH;
	}
 	res=0XFF;
	ftinfo.fontok=0XFF;
  	SPI_Flash_Write((u8*)&ftinfo,FONTINFOADDR,sizeof(ftinfo));	//�ٴ�д��0xFF�� ��ʹ֮ǰ��0xAAҲ���¶�ȫ�����¸����ֿ�  ���֮ǰ�ֿ�ɹ��ı�־.��ֹ���µ�һ������,���µ��ֿⲿ�����ݶ�ʧ.
 	SPI_Flash_Read((u8*)&ftinfo,FONTINFOADDR,sizeof(ftinfo));	//���¶���ftinfo�ṹ������
	
 	LCD_ShowString(x,y,240,320,size,"Updating UNIGBK.BIN  ");
	res=updata_fontx(x+20*size/2,y,size,unigbk_path,0);			//����UNIGBK.BIN
	if(res)return 1;
	
 	LCD_ShowString(x,y,240,320,size,"Updating GBK12.BIN");
	res=updata_fontx(x+20*size/2,y,size,gbk12_path,1);			//����GBK12.FON
	if(res)return 2;
	
	LCD_ShowString(x,y,240,320,size,"Updating GBK16.BIN  ");
	res=updata_fontx(x+20*size/2,y,size,gbk16_path,2);			//����GBK16.FON
	if(res)return 3;
	
	LCD_ShowString(x,y,240,320,size,"Updating ICON_CODE.BIN  "); //20150909
	res=updata_fontx(x+20*size/2,y,size,icon_code_path,3);			//����ICON_CODE.FON
	if(res)return 4;
	
	//ȫ�����º���
	ftinfo.fontok=0XAA;
 	SPI_Flash_Write((u8*)&ftinfo,FONTINFOADDR,sizeof(ftinfo));	//�����ֿ���Ϣ
  SPI_Flash_Write((u8*)&ftinfo,FONTINFOADDR2,sizeof(ftinfo));	//�����ֿ���Ϣ
		
	return 0;//�޴���.		 
}
//��ʼ������
//����ֵ:0,�ֿ����.
//		 ����,�ֿⶪʧ
u8 font_init(void)
{			  												 
//	SPI_Flash_Init();
//	FONTINFOADDR=(1024*6+500)*1024;			//W25Q64,6M�Ժ�	 
	ftinfo.ugbkaddr=FONTINFOADDR+33;		//UNICODEGBK �����׵�ַ�̶���ַ//20150909   25--->33
	SPI_Flash_Read((u8*)&ftinfo,FONTINFOADDR,sizeof(ftinfo));//����ftinfo�ṹ������   ftinfo��Ȼ�ǽṹ�壬�����������Ԫ�ص�ַҲ�ǰ���ַ˳��洢�ģ�����25���ֽ�
	if(ftinfo.fontok!=0XAA)return 1;		//�ֿ����.
	return 0;		    
}

//��ʼ������
//����ֵ:0,����OK
//		 ����,���ⶪʧ
u8 theme_init(void)
{			  												 
	themeinfo.themeaddr=THEME_ADDR+9;		//UNICODEGBK �����׵�ַ�̶���
	SPI_Flash_Read((u8*)&themeinfo,THEME_ADDR,sizeof(themeinfo));//����ftinfo�ṹ������   ftinfo��Ȼ�ǽṹ�壬�����������Ԫ�ص�ַҲ�ǰ���ַ˳��洢�ģ�����9���ֽ�
	if(themeinfo.themeok!=0XBB)return 1;		//�ֿ����.
	return 0;		    
}





















































