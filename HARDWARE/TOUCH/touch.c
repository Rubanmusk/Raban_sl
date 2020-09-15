#include "touch.h" 
#include "lcd.h"
#include "delay.h"
#include "stdlib.h"
#include "math.h"
//#include "24cxx.h"
#include "spi.h"
#include "flash.h"
//////////////////////////////////////////////////////////////////////////////////	 
//������ֻ��ѧϰʹ�ã�δ��������ɣ��������������κ���;
//ALIENTEKս��STM32������
//������������֧��ADS7843/7846/UH7843/7846/XPT2046/TSC2046/OTT2001A�ȣ� ����	   
//����ԭ��@ALIENTEK
//������̳:www.openedv.com
//�޸�����:2014/2/16
//�汾��V2.0
//��Ȩ���У�����ؾ���
//Copyright(C) ������������ӿƼ����޹�˾ 2009-2019
//All rights reserved									  
//********************************************************************************
//V2.0�޸�˵��
//���ӶԵ��ݴ�������֧��(��Ҫ���:ctiic.c��ott2001a.c�����ļ�)
//////////////////////////////////////////////////////////////////////////////////
///�жϴ������Ƿ�У׼��mini��

_m_tp_dev tp_dev=
{
	TP_Init, //��ʼ��������������
	TP_Scan, //ɨ�败����.0,��Ļɨ��;1,��������;	 
	TP_Adjust, //������У׼ 	   
	0,					//��ǰ����
	0, //�����������5������,����������x[0],y[0]����:�˴�ɨ��ʱ,����������,��
								//x[4],y[4]�洢��һ�ΰ���ʱ������. 
	0,					//�ʵ�״̬ 
								//b7:����1/�ɿ�0; 
	                            //b6:0,û�а�������;1,�а�������. 
								//b5:����
								//b4~b0:���ݴ��������µĵ���(0,��ʾδ����,1��ʾ����)
	/////////////////////������У׼����(����������ҪУ׼)//////////////////////								
	0,	// 	float xfac;
	0,	// 	float yfac;
	0,	// 	short xoff;
	0,	// 	short yoff;	
	//�����Ĳ���,��������������������ȫ�ߵ�ʱ��Ҫ�õ�.
	//b0:0,����(�ʺ�����ΪX����,����ΪY�����TP)
	//   1,����(�ʺ�����ΪY����,����ΪX�����TP) 
	//b1~6:����.
	//b7:0,������
	//   1,������ 	
	0,
};					
//Ĭ��Ϊtouchtype=0������.
u8 CMD_RDX=0XD0;
u8 CMD_RDY=0X90;
 	 			    					   
//SPIд����
//������ICд��1byte����    
//num:Ҫд������� 
u8 TP_Write_Byte(u8 num)    
{ 
   SPI_FLASH_CS=1;//mini
 	return SPI1_ReadWriteByte(num);//SPI3���߶�дһ���ֽ�
} 
//SPI������ 
//�Ӵ�����IC��ȡadcֵ
//CMD:ָ��
//����ֵ:����������	   
u16 TP_Read_AD(u8 CMD)	  
{ 	 	  
	u16 Num=0; 	
	SPI_FLASH_CS=1;//mini
	T_CS=0; 		//ѡ�д�����IC
	TP_Write_Byte(CMD);//����������
	//delay_us(6);//ADS7846��ת��ʱ���Ϊ6us
  Num=TP_Write_Byte(0X00);

	Num<<=8; 
	Num+=TP_Write_Byte(0X00);
	Num>>=3;   	//ֻ�и�[15,3]λ��Ч.  
	Num&=0XFFF; 	
	T_CS=1;		//�ͷ�Ƭѡ		
	return Num;   
}
//��ȡһ������ֵ(x����y)
//������ȡREAD_TIMES������,����Щ������������,
//Ȼ��ȥ����ͺ����LOST_VAL����,ȡƽ��ֵ 
//xy:ָ�CMD_RDX/CMD_RDY��
//����ֵ:����������
#define READ_TIMES 7 	//��ȡ����
#define LOST_VAL 1	  	//����ֵ

u16 TP_Read_XOY(u8 xy)
{
	u16 i, j;
	u16 buf[READ_TIMES];
	u16 sum=0;
	u16 temp;
	for(i=0;i<READ_TIMES;i++)buf[i]=TP_Read_AD(xy);		 		    
	for(i=0;i<READ_TIMES-1; i++)//����
	{
		for(j=i+1;j<READ_TIMES;j++)
		{
			if(buf[i]>buf[j])//��������
			{
				temp=buf[i];
				buf[i]=buf[j];
				buf[j]=temp;
			}
		}
	}	  
	sum=0;
	for(i=LOST_VAL;i<READ_TIMES-LOST_VAL;i++)sum+=buf[i];
	temp=sum/(READ_TIMES-2*LOST_VAL);
	return temp;   
} 
//��ȡx,y����
//��Сֵ��������100.
//x,y:��ȡ��������ֵ
//����ֵ:0,ʧ��;1,�ɹ���
u8 TP_Read_XY(u16 *x,u16 *y)
{
	u16 xtemp,ytemp;			 	 		  
	xtemp=TP_Read_XOY(CMD_RDX);
	ytemp=TP_Read_XOY(CMD_RDY);	  												   
	//if(xtemp<100||ytemp<100)return 0;//����ʧ��
	*x=xtemp;
	*y=ytemp;
	return 1;//�����ɹ�
}
//����2�ζ�ȡ������IC,�������ε�ƫ��ܳ���
//ERR_RANGE,��������,����Ϊ������ȷ,�����������.	   
//�ú����ܴ�����׼ȷ��
//x,y:��ȡ��������ֵ
//����ֵ:0,ʧ��;1,�ɹ���
#define ERR_RANGE 30 //��Χ 
u8 TP_Read_XY2(u16 *x,u16 *y) 
{
	u16 x1,y1;
 	u16 x2,y2;
 	u8 flag;    
    flag=TP_Read_XY(&x1,&y1);   
    if(flag==0)return(0);
    flag=TP_Read_XY(&x2,&y2);	   
    if(flag==0)return(0);   
    if(((x2<=x1&&x1<x2+ERR_RANGE)||(x1<=x2&&x2<x1+ERR_RANGE))//ǰ�����β�����+-50��
    &&((y2<=y1&&y1<y2+ERR_RANGE)||(y1<=y2&&y2<y1+ERR_RANGE)))
    {
        *x=(x1+x2)/2;
        *y=(y1+y2)/2;
        return 1;
    }else return 0;	  
}  
//////////////////////////////////////////////////////////////////////////////////		  
//��LCD�����йصĺ���  
//��һ��������
//����У׼�õ�
//x,y:����
//color:��ɫ
void TP_Drow_Touch_Point(u16 x,u16 y,u16 color)
{
	POINT_COLOR=color;
	LCD_DrawLine(x-12,y,x+13,y);//����
	LCD_DrawLine(x,y-12,x,y+13);//����
	LCD_DrawPoint(x+1,y+1);
	LCD_DrawPoint(x-1,y+1);
	LCD_DrawPoint(x+1,y-1);
	LCD_DrawPoint(x-1,y-1);
	Draw_Circle(x,y,6);//������Ȧ
}	  
//��һ�����(2*2�ĵ�)		   
//x,y:����
//color:��ɫ
void TP_Draw_Big_Point(u16 x,u16 y,u16 color)
{	    
	POINT_COLOR=color;
	LCD_DrawPoint(x,y);//���ĵ� 
	LCD_DrawPoint(x+1,y);
	LCD_DrawPoint(x,y+1);
	LCD_DrawPoint(x+1,y+1);	 	  	
}						  
//////////////////////////////////////////////////////////////////////////////////		  
//��������ɨ��
//tp:0,��Ļ����;1,��������(У׼�����ⳡ����)
//����ֵ:��ǰ����״̬.
//0,�����޴���;1,�����д���
u8 TP_Scan(u8 tp)
{	
 SPI1_SetSpeed(SPI_BaudRatePrescaler_256);//mini	
	delay_us(50);
	if(PEN==0)//�а�������
	{
		if(tp)
			TP_Read_XY2(&tp_dev.x[0],&tp_dev.y[0]);//��ȡ��������
		else if(TP_Read_XY2(&tp_dev.x[0],&tp_dev.y[0]))//��ȡ��Ļ����
		{
	 		tp_dev.x[0]=tp_dev.xfac*tp_dev.x[0]+tp_dev.xoff;//�����ת��Ϊ��Ļ����
			tp_dev.y[0]=tp_dev.yfac*tp_dev.y[0]+tp_dev.yoff;  
	 	} 
		if((tp_dev.sta&TP_PRES_DOWN)==0)//֮ǰû�б�����
		{		 
			tp_dev.sta=TP_PRES_DOWN|TP_CATH_PRES;//��������  
			tp_dev.x[4]=tp_dev.x[0];//��¼��һ�ΰ���ʱ������
			tp_dev.y[4]=tp_dev.y[0];  	   			 
		}			   
	}else
	{
		if(tp_dev.sta&TP_PRES_DOWN)//֮ǰ�Ǳ����µ�
		{
			tp_dev.sta&=~(1<<7);//��ǰ����ɿ�	
		}else//֮ǰ��û�б�����
		{
			tp_dev.x[4]=0;
			tp_dev.y[4]=0;
			tp_dev.x[0]=0xffff;
			tp_dev.y[0]=0xffff;
		}	    
	}
	SPI1_SetSpeed(SPI_BaudRatePrescaler_2);//mini	
	return tp_dev.sta&TP_PRES_DOWN;//���ص�ǰ�Ĵ���״̬
}	  
//////////////////////////////////////////////////////////////////////////	 
//������EEPROM����ĵ�ַ�����ַ,ռ��13���ֽ�(RANGE:SAVE_ADDR_BASE~SAVE_ADDR_BASE+12)
//#define SAVE_ADDR_BASE 40
//����У׼����	6*4=24byte									    
void TP_Save_Adjdata(void)
{
	s32 temp;	
  //SPI1_SetSpeed(SPI_BaudRatePrescaler_2);//mini		
	//����У�����!		   							  
	temp=tp_dev.xfac*100000000;//����xУ������  
  SPI_Flash_Write((u8*)&temp,SAVE_ADDR_BASE,4);	
  //AT24CXX_WriteLenByte(SAVE_ADDR_BASE,temp,4);   
	temp=tp_dev.yfac*100000000;//����yУ������ 
	SPI_Flash_Write((u8*)&temp,SAVE_ADDR_BASE+4,4);	
  //AT24CXX_WriteLenByte(SAVE_ADDR_BASE+4,temp,4);
	//����xƫ����
	SPI_Flash_Write((u8*)&tp_dev.xoff,SAVE_ADDR_BASE+8,2);	
  //AT24CXX_WriteLenByte(SAVE_ADDR_BASE+8,tp_dev.xoff,2);		    
	//����yƫ����
	SPI_Flash_Write((u8*)&tp_dev.yoff,SAVE_ADDR_BASE+10,2);
  //AT24CXX_WriteLenByte(SAVE_ADDR_BASE+10,tp_dev.yoff,2);	
	//���津������
	SPI_Flash_Write((u8*)&tp_dev.touchtype,SAVE_ADDR_BASE+12,1);
  //AT24CXX_WriteOneByte(SAVE_ADDR_BASE+12,tp_dev.touchtype);	
	temp=0X0A;//���У׼����
	SPI_Flash_Write((u8*)&temp,SAVE_ADDR_BASE+13,1);
	//AT24CXX_WriteOneByte(SAVE_ADDR_BASE+13,temp);
}

//MINI�津����flash����spi1
//�Ѷ��������ݴ浽TP_DATAINFO[6]
//void TP_ReadDataInfo(void)
//{
//	SPI_Flash_Read((u8*)&TP_DATAINFO[0],SAVE_ADDR_BASE,4);	//����xУ������     
//	SPI_Flash_Read((u8*)&TP_DATAINFO[1],SAVE_ADDR_BASE+4,4);//����yУ������       
//	SPI_Flash_Read((u8*)&TP_DATAINFO[2],SAVE_ADDR_BASE+8,2);//����xƫ����
//	SPI_Flash_Read((u8*)&TP_DATAINFO[3],SAVE_ADDR_BASE+10,2);//����yƫ����
//	SPI_Flash_Read((u8*)&TP_DATAINFO[4],SAVE_ADDR_BASE+12,1);//���津������	
//	SPI_Flash_Read((u8*)&TP_DATAINFO[5],SAVE_ADDR_BASE+13,1);//У׼���
//}
//�õ�������EEPROM�����У׼ֵ
//����ֵ��1���ɹ���ȡ����
//        0����ȡʧ�ܣ�Ҫ����У׼
u8 TP_Get_Adjdata(void)
{	
	s32 tempfac;
	//tempfac=AT24CXX_ReadOneByte(SAVE_ADDR_BASE+13);//��ȡ�����,���Ƿ�У׼����
	SPI_Flash_Read((u8*)&tempfac,SAVE_ADDR_BASE+13,1);//��ȡ�����,���Ƿ�У׼���� 
	if(tempfac==0X0A)//�������Ѿ�У׼����			   
	{    												 
		//tempfac=AT24CXX_ReadLenByte(SAVE_ADDR_BASE,4);	
		SPI_Flash_Read((u8*)&tempfac,SAVE_ADDR_BASE,4);
		tp_dev.xfac=(float)tempfac/100000000;//�õ�xУ׼����
		//tempfac=AT24CXX_ReadLenByte(SAVE_ADDR_BASE+4,4);	
		SPI_Flash_Read((u8*)&tempfac,SAVE_ADDR_BASE+4,4);
		tp_dev.yfac=(float)tempfac/100000000;//�õ�yУ׼����
	    //�õ�xƫ����
		//tp_dev.xoff=AT24CXX_ReadLenByte(SAVE_ADDR_BASE+8,2);
    SPI_Flash_Read((u8*)&tp_dev.xoff,SAVE_ADDR_BASE+8,2);		
 	    //�õ�yƫ����
		//tp_dev.yoff=AT24CXX_ReadLenByte(SAVE_ADDR_BASE+10,2);	
    SPI_Flash_Read((u8*)&tp_dev.yoff,SAVE_ADDR_BASE+10,2);		
 		//tp_dev.touchtype=AT24CXX_ReadOneByte(SAVE_ADDR_BASE+12);//��ȡ�������ͱ��
		SPI_Flash_Read((u8*)&tp_dev.touchtype,SAVE_ADDR_BASE+12,1);
		if(tp_dev.touchtype)//X,Y��������Ļ�෴
		{
			CMD_RDX=0X90;
			CMD_RDY=0XD0;	 
		}else				   //X,Y��������Ļ��ͬ
		{
			CMD_RDX=0XD0;
			CMD_RDY=0X90;	 
		}		 
		return 1;	 
	}
	return 0;
}	 
//��ʾ�ַ���
const u8* TP_REMIND_MSG_TBL="Please use the stylus click the cross on the screen.The cross will always move until the screen adjustment is completed.";
 					  
//��ʾУ׼���(��������)
void TP_Adj_Info_Show(u16 x0,u16 y0,u16 x1,u16 y1,u16 x2,u16 y2,u16 x3,u16 y3,u16 fac)
{	  
	POINT_COLOR=RED;
	LCD_ShowString((lcddev.width-160)/2,140,lcddev.width,lcddev.height,16,"x1:");
 	LCD_ShowString((lcddev.width-160)/2+80,140,lcddev.width,lcddev.height,16,"y1:");
 	LCD_ShowString((lcddev.width-160)/2,160,lcddev.width,lcddev.height,16,"x2:");
 	LCD_ShowString((lcddev.width-160)/2+80,160,lcddev.width,lcddev.height,16,"y2:");
	LCD_ShowString((lcddev.width-160)/2,180,lcddev.width,lcddev.height,16,"x3:");
 	LCD_ShowString((lcddev.width-160)/2+80,180,lcddev.width,lcddev.height,16,"y3:");
	LCD_ShowString((lcddev.width-160)/2,200,lcddev.width,lcddev.height,16,"x4:");
 	LCD_ShowString((lcddev.width-160)/2+80,200,lcddev.width,lcddev.height,16,"y4:");  
 	LCD_ShowString((lcddev.width-160)/2,220,lcddev.width,lcddev.height,16,"fac is:");   
  
	LCD_ShowNum((lcddev.width-160)/2+24,140,x0,4,16);		//��ʾ��ֵ
	LCD_ShowNum((lcddev.width-160)/2+24+80,140,y0,4,16);	//��ʾ��ֵ
	LCD_ShowNum((lcddev.width-160)/2+24,160,x1,4,16);		//��ʾ��ֵ
	LCD_ShowNum((lcddev.width-160)/2+24+80,160,y1,4,16);	//��ʾ��ֵ
	LCD_ShowNum((lcddev.width-160)/2+24,180,x2,4,16);		//��ʾ��ֵ
	LCD_ShowNum((lcddev.width-160)/2+24+80,180,y2,4,16);	//��ʾ��ֵ
	LCD_ShowNum((lcddev.width-160)/2+24,200,x3,4,16);		//��ʾ��ֵ
	LCD_ShowNum((lcddev.width-160)/2+24+80,200,y3,4,16);	//��ʾ��ֵ
 	LCD_ShowNum((lcddev.width-160)/2+56,220,fac,4,16); 	//��ʾ��ֵ,����ֵ������95~105��Χ֮��.

}
		 
//������У׼����
//�õ��ĸ�У׼����
void TP_Adjust(void)
{								 
	u16 pos_temp[4][2];//���껺��ֵ
	u8  cnt=0;	
	u16 d1,d2;
	u32 tem1,tem2;
	float fac; 	
	u16 outtime=0;
 	cnt=0;				
	POINT_COLOR=BLUE;
	BACK_COLOR =WHITE;
	LCD_Clear(WHITE);//����   
	POINT_COLOR=RED;//��ɫ 
	LCD_Clear(WHITE);//���� 	   
	POINT_COLOR=BLACK;
	LCD_ShowString((lcddev.width-160)/2,40,160,100,16,(u8*)TP_REMIND_MSG_TBL);//��ʾ��ʾ��Ϣ
	TP_Drow_Touch_Point(20,20,RED);//����1 
	tp_dev.sta=0;//���������ź� 
	tp_dev.xfac=0;//xfac��������Ƿ�У׼��,����У׼֮ǰ�������!�������
	
	while(1)//�������10����û�а���,���Զ��˳�
	{	
		tp_dev.scan(1);//ɨ����������
		if((tp_dev.sta&0xc0)==TP_CATH_PRES)//����������һ��(��ʱ�����ɿ���.)
		{
			outtime=0;		
			tp_dev.sta&=~(1<<6);//��ǰ����Ѿ����������.
						   			   
			pos_temp[cnt][0]=tp_dev.x[0];
			pos_temp[cnt][1]=tp_dev.y[0];
			cnt++;	  
			switch(cnt)
			{			   
				case 1:						 
					TP_Drow_Touch_Point(20,20,WHITE);				//�����1 
					TP_Drow_Touch_Point(lcddev.width-20,20,RED);	//����2
					break;
				case 2:
 					TP_Drow_Touch_Point(lcddev.width-20,20,WHITE);	//�����2
					TP_Drow_Touch_Point(20,lcddev.height-20,RED);	//����3
					break;
				case 3:
 					TP_Drow_Touch_Point(20,lcddev.height-20,WHITE);			//�����3
 					TP_Drow_Touch_Point(lcddev.width-20,lcddev.height-20,RED);	//����4
					break;
				case 4:	 //ȫ���ĸ����Ѿ��õ�
	    		    //�Ա����
					tem1=abs(pos_temp[0][0]-pos_temp[1][0]);//x1-x2
					tem2=abs(pos_temp[0][1]-pos_temp[1][1]);//y1-y2
					tem1*=tem1;
					tem2*=tem2;
					d1=sqrt(tem1+tem2);//�õ�1,2�ľ���
					
					tem1=abs(pos_temp[2][0]-pos_temp[3][0]);//x3-x4
					tem2=abs(pos_temp[2][1]-pos_temp[3][1]);//y3-y4
					tem1*=tem1;
					tem2*=tem2;
					d2=sqrt(tem1+tem2);//�õ�3,4�ľ���
					fac=(float)d1/d2;
					if(fac<0.95||fac>1.05||d1==0||d2==0)//���ϸ�
					{
						cnt=0;
 				    	TP_Drow_Touch_Point(lcddev.width-20,lcddev.height-20,WHITE);	//�����4
   	 					TP_Drow_Touch_Point(20,20,RED);								//����1
 						TP_Adj_Info_Show(pos_temp[0][0],pos_temp[0][1],pos_temp[1][0],pos_temp[1][1],pos_temp[2][0],pos_temp[2][1],pos_temp[3][0],pos_temp[3][1],fac*100);//��ʾ����   
 						continue;
					}
					tem1=abs(pos_temp[0][0]-pos_temp[2][0]);//x1-x3
					tem2=abs(pos_temp[0][1]-pos_temp[2][1]);//y1-y3
					tem1*=tem1;
					tem2*=tem2;
					d1=sqrt(tem1+tem2);//�õ�1,3�ľ���
					
					tem1=abs(pos_temp[1][0]-pos_temp[3][0]);//x2-x4
					tem2=abs(pos_temp[1][1]-pos_temp[3][1]);//y2-y4
					tem1*=tem1;
					tem2*=tem2;
					d2=sqrt(tem1+tem2);//�õ�2,4�ľ���
					fac=(float)d1/d2;
					if(fac<0.95||fac>1.05)//���ϸ�
					{
						cnt=0;
 				    	TP_Drow_Touch_Point(lcddev.width-20,lcddev.height-20,WHITE);	//�����4
   	 					TP_Drow_Touch_Point(20,20,RED);								//����1
 						TP_Adj_Info_Show(pos_temp[0][0],pos_temp[0][1],pos_temp[1][0],pos_temp[1][1],pos_temp[2][0],pos_temp[2][1],pos_temp[3][0],pos_temp[3][1],fac*100);//��ʾ����   
						continue;
					}//��ȷ��
								   
					//�Խ������
					tem1=abs(pos_temp[1][0]-pos_temp[2][0]);//x1-x3
					tem2=abs(pos_temp[1][1]-pos_temp[2][1]);//y1-y3
					tem1*=tem1;
					tem2*=tem2;
					d1=sqrt(tem1+tem2);//�õ�1,4�ľ���
	
					tem1=abs(pos_temp[0][0]-pos_temp[3][0]);//x2-x4
					tem2=abs(pos_temp[0][1]-pos_temp[3][1]);//y2-y4
					tem1*=tem1;
					tem2*=tem2;
					d2=sqrt(tem1+tem2);//�õ�2,3�ľ���
					fac=(float)d1/d2;
					if(fac<0.95||fac>1.05)//���ϸ�
					{
						cnt=0;
 				    	TP_Drow_Touch_Point(lcddev.width-20,lcddev.height-20,WHITE);	//�����4
   	 					TP_Drow_Touch_Point(20,20,RED);								//����1
 						TP_Adj_Info_Show(pos_temp[0][0],pos_temp[0][1],pos_temp[1][0],pos_temp[1][1],pos_temp[2][0],pos_temp[2][1],pos_temp[3][0],pos_temp[3][1],fac*100);//��ʾ����   
						continue;
					}//��ȷ��
					//������
					tp_dev.xfac=(float)(lcddev.width-40)/(pos_temp[1][0]-pos_temp[0][0]);//�õ�xfac		 
					tp_dev.xoff=(lcddev.width-tp_dev.xfac*(pos_temp[1][0]+pos_temp[0][0]))/2;//�õ�xoff
						  
					tp_dev.yfac=(float)(lcddev.height-40)/(pos_temp[2][1]-pos_temp[0][1]);//�õ�yfac
					tp_dev.yoff=(lcddev.height-tp_dev.yfac*(pos_temp[2][1]+pos_temp[0][1]))/2;//�õ�yoff  
					if(abs(tp_dev.xfac)>2||abs(tp_dev.yfac)>2)//������Ԥ����෴��.
					{
						cnt=0;
 				    	TP_Drow_Touch_Point(lcddev.width-20,lcddev.height-20,WHITE);	//�����4
   	 					TP_Drow_Touch_Point(20,20,RED);								//����1
						LCD_ShowString(40,26,lcddev.width,lcddev.height,16,"TP Need readjust!");
						tp_dev.touchtype=!tp_dev.touchtype;//�޸Ĵ�������.
						if(tp_dev.touchtype)//X,Y��������Ļ�෴
						{
							CMD_RDX=0X90;
							CMD_RDY=0XD0;	 
						}else				   //X,Y��������Ļ��ͬ
						{
							CMD_RDX=0XD0;
							CMD_RDY=0X90;	 
						}			    
						continue;
					}		
					POINT_COLOR=BLUE;
					LCD_Clear(WHITE);//����
					LCD_ShowString((lcddev.width-184)/2,110,184,16,16,"Touch Screen Adjust OK!");//У�����
					delay_ms(1000);
						
					TP_Save_Adjdata();  
 				//	LCD_Clear(WHITE);//����   
					return;//У�����				 
			}
		}
		delay_ms(10);
		outtime++;
		if(outtime>1000)
		{
			TP_Get_Adjdata();
	//		LCD_Clear(WHITE);//���� 
			break;
	 	} 
 	}
	
}
//��������ʼ��  		    
//����ֵ:0,û�н���У׼
//       1,���й�У׼
u8 TP_Init(void)
{	
//	if(lcddev.id==0X5510)		//���ݴ�����
//	{
//	//	OTT2001A_Init();
//	//	tp_dev.scan=CTP_Scan;	//ɨ�躯��ָ����ݴ�����ɨ��
//	//	tp_dev.touchtype|=0X80;	//������ 
//	//	tp_dev.touchtype|=lcddev.dir&0X01;//������������ 
//		return 0;
//	}else//mini��T_PEN---PB9
//	{ 
		GPIO_InitTypeDef  GPIO_InitStructure;

	 	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);	 //ʹ��PC�˿�ʱ��		
	 	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9;	// PC.4�˿�����
	 	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU; 		 //	��������
	 	GPIO_Init(GPIOB, &GPIO_InitStructure);
		
	 	SPI1_Init();		   	//��ʼ��TOUCH XPT2046 SPI�ӿ�
		//IIC_Init();;				//��ʼ��24CXX IIC �ӿ�
		//mini��
	  //TP_ReadDataInfo();//jee
		
		TP_Read_XY(&tp_dev.x[0],&tp_dev.y[0]);//��һ�ζ�ȡ��ʼ��	 
		if(TP_Get_Adjdata())
			return 0;//�Ѿ�У׼
		else			   //δУ׼?
		{ 										    
			LCD_Clear(WHITE);//����
			TP_Adjust();  //��ĻУ׼ 
			//TP_Save_Adjdata();	 
		}			
		TP_Get_Adjdata();	
//	}
	return 1; 									 
}
