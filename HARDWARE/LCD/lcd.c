#include "lcd.h"
#include "stdlib.h"
#include "font.h" 
#include "usart.h"
#include "delay.h"	 
#include "spi.h"
u8 LCDScanDir=0;    //�Ƿ�ı���Ļɨ�跽��		=3����ת180�ȣ�	
u8 lcddircheck=0;
//LCD�Ļ�����ɫ�ͱ���ɫ	   
u16 POINT_COLOR=0x0000;	//������ɫ
u16 BACK_COLOR=0xFFFF;  //����ɫ 
//���ò���--Ĭ����ɫ
#define COLOR_DIVISOR  0X0841  //RGB565
u16 title_fontcol=GRED ; //������ǰ��ɫ ��ɫ
u16 title_backgroundcol=LGRAYBLUE; //
u16 body_backgroundcol = GRAYBLUE;//������ɫ

//����LCD��Ҫ����
//Ĭ��Ϊ����
_lcd_dev lcddev;
//��mdk -O1ʱ���Ż�ʱ��Ҫ����
//��ʱi
void opt_delay(u8 i)
{
	while(i--);
} 					    
//д�Ĵ�������
//data:�Ĵ���ֵ---8λ
void LCD_WR_REG(u16 data)
{ 
	LCD_RS_CLR;//д����  
 //	LCD_CS_CLR; 
	//DATAOUT(data);
	SPI3_WriteByte(data&0xff);
	//SPI3_ReadWriteByte(data&0xff);
	LCD_WR_CLR; 
	LCD_WR_SET; 
 //	LCD_CS_SET;   
}
//д���ݺ���
//�������LCD_WR_DATA��,��ʱ�任�ռ�.
//data:�Ĵ���ֵ
void LCD_WR_DATA(u16 data)
{
	LCD_RS_SET;//д���� 
	//LCD_CS_CLR;
	//DATAOUT(data);
	//SPI3_ReadWriteByte(data&0xff);//8λ
  SPI3_WriteByte(data&0xff);//8λ
	LCD_WR_CLR;
	LCD_WR_SET;	
	//LCD_CS_SET;
}
//��ȡ LCD �������ļĴ�������(��GRAM����)
//����ֵ:������ֵ
u16 LCD_RD_DATA(void)
{	
  u16 t;
//LCD_CS_CLR;	
	LCD_RS_SET;//��8λ����
	LCD_RD_CLR;
	LCD_RD_SET;
	//opt_delay(2);
	t=SPI3_ReadWriteByte(0xff);	
//LCD_CS_SET;
	return t;  
}

//��ȡ LCD ��GRAM����
//����ֵ:��������ɫֵ 24bit
//����:LCD_ReadPoint
u16 LCD_RD_GRAM(u16 LCD_Reg)
{	
	u16 r=0,g=0,b=0;
//LCD_CS_CLR;	
	LCD_RS_SET;//��8λ����
	LCD_RD_CLR;
	SPI3_ReadWriteByte(LCD_Reg&0xff);//dummy read
	r=(SPI3_ReadWriteByte(LCD_Reg)&0xf8)<<8;//r5
	g=(SPI3_ReadWriteByte(LCD_Reg)&0xfc)<<3;//g6
	b=(SPI3_ReadWriteByte(LCD_Reg)&0xf8)>>3;//b5
  LCD_RD_SET;	
//LCD_CS_SET;
	return r+g+b;  
}
//д�Ĵ���
//LCD_Reg:�Ĵ������
//LCD_RegValue:Ҫд���ֵ
void LCD_WriteReg(u16 LCD_Reg,u16 LCD_RegValue)
{	
	LCD_WR_REG(LCD_Reg);  
	LCD_WR_DATA(LCD_RegValue);	    		 
}   
//���Ĵ��� ��Ҫ���ڶ�ȡid
//LCD_Reg:�Ĵ������
//mode:����λ��8/24/32
//����ֵ:������ֵ
u16 LCD_ReadReg(u16 LCD_Reg)
{	
  u16 t;
 	LCD_WR_REG(LCD_Reg);  //д��Ҫ���ļĴ�����
	 t=LCD_RD_DATA();
	return t;
} 
//��ʼдGRAM
void LCD_WriteRAM_Prepare(void)
{
	LCD_WR_REG(lcddev.wramcmd);
} 
//LCDдGRAM
//RGB_Code:��ɫֵ---RGB666��ʽ D7 D6 D5 D4 D3 D2 X X  ��6λ��Ч
void LCD_WriteRAM(u16 RGB_Code)
{	
//	LCD_CS_CLR;
	LCD_RS_SET;//дGRAM ��λ��ǰ
//rgb666 0x9486
//	SPI3_WriteByte(RGB_Code>>8&0xf8);//r
//	SPI3_WriteByte(RGB_Code>>3&0xfc);//g
//	SPI3_WriteByte(RGB_Code<<3&0xf8);//b
//rgb565
	SPI3_WriteByte(RGB_Code>>8&0xff);//r5g3
	SPI3_WriteByte(RGB_Code);//g3b5
	LCD_WR_CLR;
	LCD_WR_SET;
	//LCD_CS_SET;
}
//��ILI93xx����������ΪGBR��ʽ��������д���ʱ��ΪRGB��ʽ��
//ͨ���ú���ת��
//c:GBR��ʽ����ɫֵ
//����ֵ��RGB��ʽ����ɫֵ
u16 LCD_BGR2RGB(u16 c)
{
	u16  r,g,b,rgb;   
	b=(c>>0)&0x1f;
	g=(c>>5)&0x3f;
	r=(c>>11)&0x1f;	 
	rgb=(b<<11)+(g<<5)+(r<<0);		 
	return(rgb);
}
//RGB565--->RGB666
u32 ToRGB666(u16 c)
{
	u32  r,g,b,rgb;  
	b=(c>>0)&0x1f;
	g=(c>>5)&0x3f;
	r=(c>>11)&0x1f;	
  rgb=(r<<18)+(g<<10)+(b<<2);
	return rgb;
}



//��ȡ��ĳ�����ɫֵ	 
//x,y:����
//����ֵ:�˵����ɫ
u16 LCD_ReadPoint(u16 x,u16 y)
{
	u16 rgb=0;
	if(x>=lcddev.width||y>=lcddev.height)return 0;	//�����˷�Χ,ֱ�ӷ���		   
	LCD_Set_Window(x,y,1,1);	    
	if(lcddev.id==0X9341||lcddev.id==0X9486) 
	{
		LCD_WR_REG(0X2E);//9341 ���Ͷ�GRAMָ��
		rgb=LCD_RD_GRAM(0XFF); 
	}
	else
  {
	LCD_WR_REG(R34);      		 				//����IC���Ͷ�GRAMָ��  
  rgb=LCD_RD_GRAM(R34); 		
	}
 return rgb;						//����IC
}			 
//LCD������ʾ
void LCD_DisplayOn(void)
{					   
	if(lcddev.id==0X9341||lcddev.id==0X9486) LCD_WR_REG(0X29);	//������ʾ

	else LCD_WriteReg(R7,0x0173); 				 	//������ʾ
}	 
//LCD�ر���ʾ
void LCD_DisplayOff(void)
{	   
	if(lcddev.id==0X9341||lcddev.id==0X9486)LCD_WR_REG(0X28);	//�ر���ʾ
	else LCD_WriteReg(R7,0x0);//�ر���ʾ 
}   
//���ù��λ��
//Xpos:������
//Ypos:������
void LCD_SetCursor(u16 Xpos, u16 Ypos)
{	 
 	if(lcddev.id==0X9341||lcddev.id==0X5310)
	{		    
		LCD_WR_REG(lcddev.setxcmd); 
		LCD_WR_DATA(Xpos>>8);LCD_WR_DATA(Xpos&0XFF); 			 
		LCD_WR_REG(lcddev.setycmd); 
		LCD_WR_DATA(Ypos>>8);LCD_WR_DATA(Ypos&0XFF); 		
	}else if(lcddev.id==0X6804)
	{
		if(lcddev.dir==1)Xpos=lcddev.width-1-Xpos;//����ʱ����
		LCD_WR_REG(lcddev.setxcmd); 
		LCD_WR_DATA(Xpos>>8);LCD_WR_DATA(Xpos&0XFF); 
		LCD_WR_REG(lcddev.setycmd); 
		LCD_WR_DATA(Ypos>>8);LCD_WR_DATA(Ypos&0XFF); 
	}else if(lcddev.id==0X1963)
	{  			 		
		if(lcddev.dir==0)//x������Ҫ�任
		{
			Xpos=lcddev.width-1-Xpos;
			LCD_WR_REG(lcddev.setxcmd); 
			LCD_WR_DATA(0);LCD_WR_DATA(0); 		
			LCD_WR_DATA(Xpos>>8);LCD_WR_DATA(Xpos&0XFF);		 	 
		}else
		{
			LCD_WR_REG(lcddev.setxcmd); 
			LCD_WR_DATA(Xpos>>8);LCD_WR_DATA(Xpos&0XFF); 		
			LCD_WR_DATA((lcddev.width-1)>>8);LCD_WR_DATA((lcddev.width-1)&0XFF);		 	 			
		}	
		LCD_WR_REG(lcddev.setycmd); 
		LCD_WR_DATA(Ypos>>8);LCD_WR_DATA(Ypos&0XFF); 		
		LCD_WR_DATA((lcddev.height-1)>>8);LCD_WR_DATA((lcddev.height-1)&0XFF); 			 		
		
	}else if(lcddev.id==0X5510)
	{
		LCD_WR_REG(lcddev.setxcmd);LCD_WR_DATA(Xpos>>8); 		
		LCD_WR_REG(lcddev.setxcmd+1);LCD_WR_DATA(Xpos&0XFF);			 
		LCD_WR_REG(lcddev.setycmd);LCD_WR_DATA(Ypos>>8);  		
		LCD_WR_REG(lcddev.setycmd+1);LCD_WR_DATA(Ypos&0XFF);			
	}else
	{
		if(lcddev.dir==1)Xpos=lcddev.width-1-Xpos;//������ʵ���ǵ�תx,y����
		LCD_WriteReg(lcddev.setxcmd, Xpos);
		LCD_WriteReg(lcddev.setycmd, Ypos);
	}	 
} 		 
//����LCD���Զ�ɨ�跽��
//ע��:�����������ܻ��ܵ��˺������õ�Ӱ��(������9341/6804����������),
//����,һ������ΪL2R_U2D����,�������Ϊ����ɨ�跽ʽ,���ܵ�����ʾ������.
//dir:0~7,����8������(���嶨���lcd.h)
//9320/9325/9328/4531/4535/1505/b505/5408/9341/5310/5510/1963��IC�Ѿ�ʵ�ʲ���	   	   
void LCD_Scan_Dir(u8 dir)
{
	u16 regval=0;
	u16 dirreg=0;
	u16 temp;  
	if(lcddev.dir==1)//����ʱ���ı�ɨ�跽��
	{			   
		switch(dir)//����ת��
		{
			case 0:dir=6;break;
			case 1:dir=7;break;
			case 2:dir=4;break;
			case 3:dir=5;break;
			case 4:dir=1;break;
			case 5:dir=0;break;
			case 6:dir=3;break;
			case 7:dir=2;break;	     
		}
	}
	
	if(lcddev.id==0x9341||lcddev.id==0x9486)//9341,������
	{		
			switch(dir)
				{
					case L2R_U2D://������,���ϵ���
						regval|=(0<<7)|(0<<6)|(0<<5); 
						break;
					case L2R_D2U://������,���µ���
						regval|=(1<<7)|(0<<6)|(0<<5); 
						break;
					case R2L_U2D://���ҵ���,���ϵ���
						regval|=(0<<7)|(1<<6)|(0<<5); 
						break;
					case R2L_D2U://���ҵ���,���µ���
						regval|=(1<<7)|(1<<6)|(0<<5); 
						break;	 
					case U2D_L2R://���ϵ���,������
						regval|=(0<<7)|(0<<6)|(1<<5); 
						break;
					case U2D_R2L://���ϵ���,���ҵ���
						regval|=(0<<7)|(1<<6)|(1<<5); 
						break;
					case D2U_L2R://���µ���,������
						regval|=(1<<7)|(0<<6)|(1<<5); 
						break;
					case D2U_R2L://���µ���,���ҵ���
						regval|=(1<<7)|(1<<6)|(1<<5); 
						break;	 
				}
		
	  dirreg=0X36;
 		regval|=0X08;//��ҪBGR      
		LCD_WriteReg(dirreg,regval);
		
 		if((regval&0X20)||lcddev.dir==1)
		{
			if(lcddev.width<lcddev.height)//����X,Y
			{
				temp=lcddev.width;
				lcddev.width=lcddev.height;
				lcddev.height=temp;
 			}
		}else  
		{
			if(lcddev.width>lcddev.height)//����X,Y
			{
				temp=lcddev.width;
				lcddev.width=lcddev.height;
				lcddev.height=temp;
 			}
		}  

			LCD_WR_REG(lcddev.setxcmd); 
			LCD_WR_DATA(0);LCD_WR_DATA(0);
			LCD_WR_DATA((lcddev.width-1)>>8);LCD_WR_DATA((lcddev.width-1)&0XFF);
			LCD_WR_REG(lcddev.setycmd); 
			LCD_WR_DATA(0);LCD_WR_DATA(0);
			LCD_WR_DATA((lcddev.height-1)>>8);LCD_WR_DATA((lcddev.height-1)&0XFF);  
		
 }else 
	{
		switch(dir)
		{
			case L2R_U2D://������,���ϵ���
				regval|=(1<<5)|(1<<4)|(0<<3); 
				break;
			case L2R_D2U://������,���µ���
				regval|=(0<<5)|(1<<4)|(0<<3); 
				break;
			case R2L_U2D://���ҵ���,���ϵ���
				regval|=(1<<5)|(0<<4)|(0<<3);
				break;
			case R2L_D2U://���ҵ���,���µ���
				regval|=(0<<5)|(0<<4)|(0<<3); 
				break;	 
			case U2D_L2R://���ϵ���,������
				regval|=(1<<5)|(1<<4)|(1<<3); 
				break;
			case U2D_R2L://���ϵ���,���ҵ���
				regval|=(1<<5)|(0<<4)|(1<<3); 
				break;
			case D2U_L2R://���µ���,������
				regval|=(0<<5)|(1<<4)|(1<<3); 
				break;
			case D2U_R2L://���µ���,���ҵ���
				regval|=(0<<5)|(0<<4)|(1<<3); 
				break;	 
		} 
		dirreg=0X03;
		regval|=1<<12; 
		LCD_WriteReg(dirreg,regval);
	}
}     
//����
//x,y:����
//POINT_COLOR:�˵����ɫ
void LCD_DrawPoint(u16 x,u16 y)
{
	LCD_Set_Window(x,y,1,1); //���ù��λ��  
	LCD_WriteRAM_Prepare();	//��ʼд��GRAM
	//LCD->LCD_RAM=POINT_COLOR;
  LCD_WriteRAM(POINT_COLOR); 
}
//���ٻ���
//x,y:����
//color:��ɫ
void LCD_Fast_DrawPoint(u16 x,u16 y,u16 color)
{	LCD_Set_Window(x,y,1,1); //���ù��λ��   
	LCD_WriteRAM_Prepare();	//��ʼд��GRAM
	//LCD->LCD_RAM=color; 
	LCD_WriteRAM(color); 
}	 


//����LCD��ʾ����
//dir:0,������1,����
void LCD_Display_Dir(u8 dir)
{
	if(dir==0)			//����
	{
		lcddev.dir=0;	//����
		lcddev.width=240;
		lcddev.height=320;
		if(lcddev.id==0X9341||lcddev.id==0X9486)
		{
			lcddev.wramcmd=0X2C;
	 		lcddev.setxcmd=0X2A;
			lcddev.setycmd=0X2B;  
			if(lcddev.id==0X9486)
					{ 	 
						lcddev.width=320;
						lcddev.height=480; 			
					}			
		}		
	}else 				//����
	{	  				
		lcddev.dir=1;	//����
		lcddev.width=320;
		lcddev.height=240;
		if(lcddev.id==0X9341||lcddev.id==0X9486)
		{
			lcddev.wramcmd=0X2C;
	 		lcddev.setxcmd=0X2A;
			lcddev.setycmd=0X2B;  
			if(lcddev.id==0X9486)
					{ 	 
						lcddev.width=480;
						lcddev.height=320; 			
					}			
		}
	} 
	LCD_Scan_Dir(DFT_SCAN_DIR);	//Ĭ��ɨ�跽��
}	 
//���ô���,���Զ����û������굽�������Ͻ�(sx,sy).
//sx,sy:������ʼ����(���Ͻ�)
//width,height:���ڿ�Ⱥ͸߶�,�������0!!
//�����С:width*height. 
void LCD_Set_Window(u16 sx,u16 sy,u16 width,u16 height)
{   
	//u8 hsareg,heareg,vsareg,veareg;
//	u16 hsaval,heaval,vsaval,veaval; 
	width=sx+width-1;
	height=sy+height-1;
	if(lcddev.id==0X9341||lcddev.id==0X9486)//
	{
		LCD_WR_REG(lcddev.setxcmd); 
		LCD_WR_DATA(sx>>8); 
		LCD_WR_DATA(sx&0XFF);	 
		LCD_WR_DATA(width>>8); 
		LCD_WR_DATA(width&0XFF);  
		LCD_WR_REG(lcddev.setycmd); 
		LCD_WR_DATA(sy>>8); 
		LCD_WR_DATA(sy&0XFF); 
		LCD_WR_DATA(height>>8); 
		LCD_WR_DATA(height&0XFF); 
	}
}
//��ʼ��lcd
//�ó�ʼ���������Գ�ʼ������ALIENTEK��Ʒ��LCDҺ����
//������ռ�ýϴ�flash,�û����Ը����Լ���ʵ�����,ɾ��δ�õ���LCD��ʼ������.�Խ�ʡ�ռ�.
void LCD_Init(void)  //PA12 test
{ 
 	GPIO_InitTypeDef GPIO_InitStructure;
 	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE); //ʹ��PORTB,Cʱ�Ӻ�AFIOʱ��
	
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6|GPIO_Pin_7;	 //PB6---L_RS  //PB7----L_RST/LED
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;   
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOB, &GPIO_InitStructure); 	
	

	GPIO_SetBits(GPIOB,GPIO_Pin_6);
	
 	SPI3_Init();//SPI3	
	 LCD_RST = 0;
	 delay_ms(10);
	 LCD_RST = 1;
	
 		delay_ms(50); // delay 50 ms 
		LCD_WriteReg(0x0000,0x0001);
		delay_ms(50); // delay 50 ms 
//  	lcddev.id = LCD_ReadReg(0x0000); 
//  if(lcddev.id<0XFF||lcddev.id==0XFFFF||lcddev.id==0X9300)//����ID����ȷ,����lcddev.id==0X9300�жϣ���Ϊ9341��δ����λ������»ᱻ����9300
//	{	
// 		//����9341 ID�Ķ�ȡ		
//		 LCD_WR_REG(0XD3);		
// 		 LCD_ReadReg();   	    	//����0X00
//   	 lcddev.id=LCD_ReadReg();   	//��ȡ93								   
// 		 lcddev.id<<=8;
//		 lcddev.id|=LCD_ReadReg();  	//��ȡ41 	   			   
////  	if(lcddev.id!=0X9341&&lcddev.id!=0X9486)		//��9341 Ҳ����9486
//// 		{	
////			lcddev.id=0XFFFF;
////  	}  	
//	} 
	lcddev.id=0x9341;//test
 	printf(" LCD ID:%x\r\n",lcddev.id); //��ӡLCD ID  
	if(lcddev.id==0X9341)	//9341��ʼ��
	{	 
		LCD_WR_REG(0xCF);  
		LCD_WR_DATA(0x00); 
		LCD_WR_DATA(0xC1); 
		LCD_WR_DATA(0X30); 
		LCD_WR_REG(0xED);  
		LCD_WR_DATA(0x64); 
		LCD_WR_DATA(0x03); 
		LCD_WR_DATA(0X12); 
		LCD_WR_DATA(0X81); 
		LCD_WR_REG(0xE8);  
		LCD_WR_DATA(0x85); 
		LCD_WR_DATA(0x10); 
		LCD_WR_DATA(0x7A); 
		LCD_WR_REG(0xCB);  
		LCD_WR_DATA(0x39); 
		LCD_WR_DATA(0x2C); 
		LCD_WR_DATA(0x00); 
		LCD_WR_DATA(0x34); 
		LCD_WR_DATA(0x02); 
		LCD_WR_REG(0xF7);  
		LCD_WR_DATA(0x20); 
		LCD_WR_REG(0xEA);  
		LCD_WR_DATA(0x00); 
		LCD_WR_DATA(0x00); 
		LCD_WR_REG(0xC0);    //Power control 
		LCD_WR_DATA(0x1B);   //VRH[5:0] 
		LCD_WR_REG(0xC1);    //Power control 
		LCD_WR_DATA(0x01);   //SAP[2:0];BT[3:0] 
		LCD_WR_REG(0xC5);    //VCM control 
		LCD_WR_DATA(0x30); 	 //3F
		LCD_WR_DATA(0x30); 	 //3C
		LCD_WR_REG(0xC7);    //VCM control2 
		LCD_WR_DATA(0XB7); 
		LCD_WR_REG(0x36);    // Memory Access Control 
		LCD_WR_DATA(0x48); 
	
		
		LCD_WR_REG(0x3A);   //jee
		LCD_WR_DATA(0x55); //rgb565
		//LCD_WR_DATA(0x66);//rgb666
		
		LCD_WR_REG(0xB1);   
		LCD_WR_DATA(0x00);   
		LCD_WR_DATA(0x1A); 
		LCD_WR_REG(0xB6);    // Display Function Control 
		LCD_WR_DATA(0x0A); 		
		LCD_WR_DATA(0xA2); 
		
		LCD_WR_REG(0xF2);    // 3Gamma Function Disable 
		LCD_WR_DATA(0x00); 
		LCD_WR_REG(0x26);    //Gamma curve selected 
		LCD_WR_DATA(0x01); 
		LCD_WR_REG(0xE0);    //Set Gamma 
		LCD_WR_DATA(0x0F); 
		LCD_WR_DATA(0x2A); 
		LCD_WR_DATA(0x28); 
		LCD_WR_DATA(0x08); 
		LCD_WR_DATA(0x0E); 
		LCD_WR_DATA(0x08); 
		LCD_WR_DATA(0x54); 
		LCD_WR_DATA(0XA9); 
		LCD_WR_DATA(0x43); 
		LCD_WR_DATA(0x0A); 
		LCD_WR_DATA(0x0F); 
		LCD_WR_DATA(0x00); 
		LCD_WR_DATA(0x00); 
		LCD_WR_DATA(0x00); 
		LCD_WR_DATA(0x00); 		 
		LCD_WR_REG(0XE1);    //Set Gamma 
		LCD_WR_DATA(0x00); 
		LCD_WR_DATA(0x15); 
		LCD_WR_DATA(0x17); 
		LCD_WR_DATA(0x07); 
		LCD_WR_DATA(0x11); 
		LCD_WR_DATA(0x06); 
		LCD_WR_DATA(0x2B); 
		LCD_WR_DATA(0x56); 
		LCD_WR_DATA(0x3C); 
		LCD_WR_DATA(0x05); 
		LCD_WR_DATA(0x10); 
		LCD_WR_DATA(0x0F); 
		LCD_WR_DATA(0x3F); 
		LCD_WR_DATA(0x3F); 
		LCD_WR_DATA(0x0F); 
		LCD_WR_REG(0x2B); 
		LCD_WR_DATA(0x00);
		LCD_WR_DATA(0x00);
		LCD_WR_DATA(0x01);
		LCD_WR_DATA(0x3f);
		LCD_WR_REG(0x2A); 
		LCD_WR_DATA(0x00);
		LCD_WR_DATA(0x00);
		LCD_WR_DATA(0x00);
		LCD_WR_DATA(0xef);	 
		LCD_WR_REG(0x11); //Exit Sleep
		delay_ms(120);
		LCD_WR_REG(0x29); //display on	

 }else if(lcddev.id==0x9486) 
{	
		LCD_WR_REG(0XF2);    
		LCD_WR_DATA(0x18); 
		LCD_WR_DATA(0xA3); 
		LCD_WR_DATA(0x12); 
		LCD_WR_DATA(0x02); 
		LCD_WR_DATA(0XB2); 
		LCD_WR_DATA(0x12); 
		LCD_WR_DATA(0xFF); 
		LCD_WR_DATA(0x10); 
		LCD_WR_DATA(0x00); 
	
		LCD_WR_REG(0XF8);    
		LCD_WR_DATA(0x21); 
		LCD_WR_DATA(0x04); 
    LCD_WR_REG(0X13);//Normal Display Mode ON
	
		LCD_WR_REG(0x36); //Memory Access Control
		//LCD_WR_DATA(0x38); 
	  LCD_WR_DATA(0x00);
	
		LCD_WR_REG(0xB4); //��ʾ��ת����
		LCD_WR_DATA(0x02); 
		
		LCD_WR_REG(0xB6);    // Display Function Control 
 		LCD_WR_DATA(0x02); 
 		//LCD_WR_DATA(0xA2); 
		LCD_WR_DATA(0x22); 
	
	  LCD_WR_REG(0xC1);  // ���ʿ���
		LCD_WR_DATA(0x41); 
		
		LCD_WR_REG(0xC5); 
		LCD_WR_DATA(0x00); 
		//LCD_WR_DATA(0x0C); //18
		LCD_WR_DATA(0x18);
		
		LCD_WR_REG(0x3A);  //���ظ�ʽ����jee
		//LCD_WR_DATA(0x55); //RGB565
		LCD_WR_DATA(0x66); //RGB666
		
		LCD_WR_REG(0XF1);
		LCD_WR_DATA(0x36);
		LCD_WR_DATA(0x04);
		LCD_WR_DATA(0x00);
		LCD_WR_DATA(0x3C);
		LCD_WR_DATA(0X0F);
		LCD_WR_DATA(0x8F);

		LCD_WR_REG(0XF9);    
		LCD_WR_DATA(0x00); 
		LCD_WR_DATA(0x08); 	
		
		LCD_WR_REG(0xB0);  //spi�ӿڿ���jee
		LCD_WR_DATA(0x0F); 
				
		LCD_WR_REG(0xB1);  //֡�ʿ���
		//LCD_WR_DATA(0xB0); //70MHz
		LCD_WR_DATA(0x70); //46MHz
		LCD_WR_DATA(0x11); 
		
		LCD_WR_REG(0xC0); // ���ʿ���
		LCD_WR_DATA(0x10); 
		LCD_WR_DATA(0x10); 
		 		 
		LCD_WR_REG(0xE0);//(Positive Gamma Control) (E0h)
		LCD_WR_DATA(0x0F);
		LCD_WR_DATA(0x1F);
		LCD_WR_DATA(0x1C);
		LCD_WR_DATA(0x0C);
		LCD_WR_DATA(0x0F);
		LCD_WR_DATA(0x08);
		LCD_WR_DATA(0x48);
		LCD_WR_DATA(0x98);
		LCD_WR_DATA(0x37);
		LCD_WR_DATA(0x0A);
		LCD_WR_DATA(0x13);
		LCD_WR_DATA(0x04);
		LCD_WR_DATA(0x11);
		LCD_WR_DATA(0x0D);
		LCD_WR_DATA(0x00);

		LCD_WR_REG(0XE1);//(Negative Gamma Correction) (E1h)
		LCD_WR_DATA(0x0F);
		LCD_WR_DATA(0x32);
		LCD_WR_DATA(0x2E);
		LCD_WR_DATA(0x0B);
		LCD_WR_DATA(0x0D);
		LCD_WR_DATA(0x05);
		LCD_WR_DATA(0x47);
		LCD_WR_DATA(0x75);
		LCD_WR_DATA(0x37);
		LCD_WR_DATA(0x06);
		LCD_WR_DATA(0x10);
		LCD_WR_DATA(0x03);
		LCD_WR_DATA(0x24);
		LCD_WR_DATA(0x20);
		LCD_WR_DATA(0x00);
			
		LCD_WR_REG(0x11); //Exit Sleep
		delay_ms(120);
		LCD_WR_REG(0x29); //display on
 }
	LCD_Display_Dir(1);		 	//Ĭ��Ϊ����(0) ����Ϊ ����
	LCD_LED=1;					//��������
	LCD_Clear(WHITE);
}

//��������
//color:Ҫ���������ɫ
void LCD_Clear(u16 color)
{
	u32 index=0;      
	u32 totalpoint=lcddev.width;
	totalpoint*=lcddev.height; 			//�õ��ܵ���
	LCD_Set_Window(0,0,lcddev.width,lcddev.height);	//���ù��λ�� 
	LCD_WriteRAM_Prepare();     		//��ʼд��GRAM	 	  
	for(index=0;index<totalpoint;index++)
	{
		//LCD->LCD_RAM=color;	 
    LCD_WriteRAM(color);		
	}
}  
//��ָ����������䵥����ɫ
//(sx,sy),(ex,ey):�����ζԽ�����,�����СΪ:(ex-sx+1)*(ey-sy+1)   
//color:Ҫ������ɫ
void LCD_Fill(u16 sx,u16 sy,u16 width ,u16 height,u16 color)
{          
	u32 i;
	LCD_Set_Window(sx,sy,width,height);
	LCD_WriteRAM_Prepare();     			//��ʼд��GRAM	 
	for(i=0;i<(width*height);i++)
		{ //LCD_WR_DATA(color);	//���ù��λ�� 	
      LCD_WriteRAM(color);			
		}
	 
}  
//��ָ�����������ָ����ɫ��			 
//(sx,sy),(ex,ey):�����ζԽ�����,�����СΪ:(ex-sx+1)*(ey-sy+1)   
//color:Ҫ������ɫ
void LCD_Color_Fill(u16 sx,u16 sy,u16 width,u16 height,u16 *color)
{  
	//u16 height,width;
	u32 i;
	//width=ex-sx+1; 			//�õ����Ŀ��
	//height=ey-sy+1;			//�߶�
	LCD_Set_Window(sx,sy,width,height);
	LCD_WriteRAM_Prepare();     //��ʼд��GRAM
 	for(i=0;i<height*width;i++)
	{	
		//LCD->LCD_RAM=color[i];//д������ 
		LCD_WriteRAM(color[i]);
	}	  
}  
//����
//x1,y1:�������
//x2,y2:�յ�����  
void LCD_DrawLine(u16 x1, u16 y1, u16 x2, u16 y2)
{
	u16 t; 
	int xerr=0,yerr=0,delta_x,delta_y,distance; 
	int incx,incy,uRow,uCol; 
	delta_x=x2-x1; //������������ 
	delta_y=y2-y1; 
	uRow=x1; 
	uCol=y1; 
	if(delta_x>0)incx=1; //���õ������� 
	else if(delta_x==0)incx=0;//��ֱ�� 
	else {incx=-1;delta_x=-delta_x;} 
	if(delta_y>0)incy=1; 
	else if(delta_y==0)incy=0;//ˮƽ�� 
	else{incy=-1;delta_y=-delta_y;} 
	if( delta_x>delta_y)distance=delta_x; //ѡȡ�������������� 
	else distance=delta_y; 
	for(t=0;t<=distance+1;t++ )//������� 
	{  
		LCD_DrawPoint(uRow,uCol);//���� 
		xerr+=delta_x ; 
		yerr+=delta_y ; 
		if(xerr>distance) 
		{ 
			xerr-=distance; 
			uRow+=incx; 
		} 
		if(yerr>distance) 
		{ 
			yerr-=distance; 
			uCol+=incy; 
		} 
	}  
}    
//������	  
//(x1,y1),(x2,y2):���εĶԽ�����
void LCD_DrawRectangle(u16 x1, u16 y1, u16 x2, u16 y2)
{
	LCD_DrawLine(x1,y1,x2,y1);
	LCD_DrawLine(x1,y1,x1,y2);
	LCD_DrawLine(x1,y2,x2,y2);
	LCD_DrawLine(x2,y1,x2,y2);
}
//��ָ��λ�û�һ��ָ����С��Բ
//(x,y):���ĵ�
//r    :�뾶
void Draw_Circle(u16 x0,u16 y0,u8 r)
{
	int a,b;
	int di;
	a=0;b=r;	  
	di=3-(r<<1);             //�ж��¸���λ�õı�־
	while(a<=b)
	{
		LCD_DrawPoint(x0+a,y0-b);             //5
 		LCD_DrawPoint(x0+b,y0-a);             //0           
		LCD_DrawPoint(x0+b,y0+a);             //4               
		LCD_DrawPoint(x0+a,y0+b);             //6 
		LCD_DrawPoint(x0-a,y0+b);             //1       
 		LCD_DrawPoint(x0-b,y0+a);             
		LCD_DrawPoint(x0-a,y0-b);             //2             
  		LCD_DrawPoint(x0-b,y0-a);             //7     	         
		a++;
		//ʹ��Bresenham�㷨��Բ     
		if(di<0)di +=4*a+6;	  
		else
		{
			di+=10+4*(a-b);   
			b--;
		} 						    
	}
} 									  
//��ָ��λ����ʾһ���ַ�
//x,y:��ʼ����
//num:Ҫ��ʾ���ַ�:" "--->"~"
//size:�����С 12/16/24
//mode:���ӷ�ʽ(1)���Ƿǵ��ӷ�ʽ(0)
void LCD_ShowChar(u16 x,u16 y,u8 num,u8 size,u8 mode)
{  							  
    u8 temp,t1,t;
	u16 y0=y;
	u16 colortemp=POINT_COLOR;      			     
	//���ô���		   
	num=num-' ';//�õ�ƫ�ƺ��ֵ
	if(!mode) //�ǵ��ӷ�ʽ
	{
	    for(t=0;t<size;t++)
	    {   
			if(size==12)temp=asc2_1206[num][t];  //����1206����
			else temp=asc2_1608[num][t];		 //����1608���� 	                          
	        for(t1=0;t1<8;t1++)
			{			    
		        if(temp&0x80)POINT_COLOR=colortemp;
				else POINT_COLOR=BACK_COLOR;
				LCD_DrawPoint(x,y);	
				temp<<=1;
				y++;
				if(x>=lcddev.width){POINT_COLOR=colortemp;return;}//��������
				if((y-y0)==size)
				{
					y=y0;
					x++;
					if(x>=lcddev.width){POINT_COLOR=colortemp;return;}//��������
					break;
				}
			}  	 
	    }    
	}else//���ӷ�ʽ
	{
	    for(t=0;t<size;t++)
	    {   
			if(size==12)temp=asc2_1206[num][t];  //����1206����
			else temp=asc2_1608[num][t];		 //����1608���� 	                          
	        for(t1=0;t1<8;t1++)
			{			    
		        if(temp&0x80)LCD_DrawPoint(x,y); 
				temp<<=1;
				y++;
				if(x>=lcddev.height){POINT_COLOR=colortemp;return;}//��������
				if((y-y0)==size)
				{
					y=y0;
					x++;
					if(x>=lcddev.width){POINT_COLOR=colortemp;return;}//��������
					break;
				}
			}  	 
	    }     
	}
	POINT_COLOR=colortemp;	    	   	 	  
}   
//m^n����
//����ֵ:m^n�η�.
u32 LCD_Pow(u8 m,u8 n)
{
	u32 result=1;	 
	while(n--)result*=m;    
	return result;
}			 
//��ʾ����,��λΪ0,����ʾ
//x,y :�������	 
//len :���ֵ�λ��
//size:�����С
//color:��ɫ 
//num:��ֵ(0~4294967295);	 
void LCD_ShowNum(u16 x,u16 y,u32 num,u8 len,u8 size)
{         	
	u8 t,temp;
	u8 enshow=0;						   
	for(t=0;t<len;t++)
	{
		temp=(num/LCD_Pow(10,len-t-1))%10;
		if(enshow==0&&t<(len-1))
		{
			if(temp==0)
			{
				LCD_ShowChar(x+(size/2)*t,y,' ',size,0);
				continue;
			}else enshow=1; 
		 	 
		}
	 	LCD_ShowChar(x+(size/2)*t,y,temp+'0',size,0); 
	}
} 
//��ʾ����,��λΪ0,������ʾ
//x,y:�������
//num:��ֵ(0~999999999);	 
//len:����(��Ҫ��ʾ��λ��)
//size:�����С
//mode:
//[7]:0,�����;1,���0.
//[6:1]:����
//[0]:0,�ǵ�����ʾ;1,������ʾ.
void LCD_ShowxNum(u16 x,u16 y,u32 num,u8 len,u8 size,u8 mode)
{  
	u8 t,temp;
	u8 enshow=0;						   
	for(t=0;t<len;t++)
	{
		temp=(num/LCD_Pow(10,len-t-1))%10;
		if(enshow==0&&t<(len-1))
		{
			if(temp==0)
			{
				if(mode&0X80)LCD_ShowChar(x+(size/2)*t,y,'0',size,mode&0X01);  
				else LCD_ShowChar(x+(size/2)*t,y,' ',size,mode&0X01);  
 				continue;
			}else enshow=1; 
		 	 
		}
	 	LCD_ShowChar(x+(size/2)*t,y,temp+'0',size,mode&0X01); 
	}
} 
//��ʾ�ַ���
//x,y:�������
//width,height:�����С  
//size:�����С
//*p:�ַ�����ʼ��ַ		  
void LCD_ShowString(u16 x,u16 y,u16 width,u16 height,u8 size,u8 *p)
{         
	u8 x0=x;
	width+=x;
	height+=y;
    while((*p<='~')&&(*p>=' '))//�ж��ǲ��ǷǷ��ַ�!
    {       
        if(x>=width){x=x0;y+=size;}
        if(y>=height)break;//�˳�
        LCD_ShowChar(x,y,*p,size,0);
        x+=size/2;
        p++;
    }  
}

//�������
//����ֵ: true ���� false ����
bool LCD_Check(void)
{ u16 temp_colour; 
	temp_colour= LCD_ReadPoint(lcddev.width/2,lcddev.height/2);
 	LCD_Fast_DrawPoint(lcddev.width/2,lcddev.height/2,0xaaaa);
	if(LCD_ReadPoint(lcddev.width/2,lcddev.height/2)==0xaaaa)
	{
		LCD_Fast_DrawPoint(lcddev.width/2,lcddev.height/2,temp_colour);
		
		return true;
	}
	else
	{ return false;
	}
	
}




































