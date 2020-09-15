#include "lcd.h"
#include "stdlib.h"
#include "font.h" 
#include "usart.h"
#include "delay.h"	 
#include "spi.h"
u8 LCDScanDir=0;    //是否改变屏幕扫描方向		=3（旋转180度）	
u8 lcddircheck=0;
//LCD的画笔颜色和背景色	   
u16 POINT_COLOR=0x0000;	//画笔颜色
u16 BACK_COLOR=0xFFFF;  //背景色 
//配置参数--默认颜色
#define COLOR_DIVISOR  0X0841  //RGB565
u16 title_fontcol=GRED ; //标题栏前景色 白色
u16 title_backgroundcol=LGRAYBLUE; //
u16 body_backgroundcol = GRAYBLUE;//主背景色

//管理LCD重要参数
//默认为竖屏
_lcd_dev lcddev;
//当mdk -O1时间优化时需要设置
//延时i
void opt_delay(u8 i)
{
	while(i--);
} 					    
//写寄存器函数
//data:寄存器值---8位
void LCD_WR_REG(u16 data)
{ 
	LCD_RS_CLR;//写命令  
 //	LCD_CS_CLR; 
	//DATAOUT(data);
	SPI3_WriteByte(data&0xff);
	//SPI3_ReadWriteByte(data&0xff);
	LCD_WR_CLR; 
	LCD_WR_SET; 
 //	LCD_CS_SET;   
}
//写数据函数
//可以替代LCD_WR_DATA宏,拿时间换空间.
//data:寄存器值
void LCD_WR_DATA(u16 data)
{
	LCD_RS_SET;//写数据 
	//LCD_CS_CLR;
	//DATAOUT(data);
	//SPI3_ReadWriteByte(data&0xff);//8位
  SPI3_WriteByte(data&0xff);//8位
	LCD_WR_CLR;
	LCD_WR_SET;	
	//LCD_CS_SET;
}
//读取 LCD 控制器的寄存器数据(非GRAM数据)
//返回值:读到的值
u16 LCD_RD_DATA(void)
{	
  u16 t;
//LCD_CS_CLR;	
	LCD_RS_SET;//读8位数据
	LCD_RD_CLR;
	LCD_RD_SET;
	//opt_delay(2);
	t=SPI3_ReadWriteByte(0xff);	
//LCD_CS_SET;
	return t;  
}

//读取 LCD 的GRAM数据
//返回值:读到的颜色值 24bit
//调用:LCD_ReadPoint
u16 LCD_RD_GRAM(u16 LCD_Reg)
{	
	u16 r=0,g=0,b=0;
//LCD_CS_CLR;	
	LCD_RS_SET;//读8位数据
	LCD_RD_CLR;
	SPI3_ReadWriteByte(LCD_Reg&0xff);//dummy read
	r=(SPI3_ReadWriteByte(LCD_Reg)&0xf8)<<8;//r5
	g=(SPI3_ReadWriteByte(LCD_Reg)&0xfc)<<3;//g6
	b=(SPI3_ReadWriteByte(LCD_Reg)&0xf8)>>3;//b5
  LCD_RD_SET;	
//LCD_CS_SET;
	return r+g+b;  
}
//写寄存器
//LCD_Reg:寄存器编号
//LCD_RegValue:要写入的值
void LCD_WriteReg(u16 LCD_Reg,u16 LCD_RegValue)
{	
	LCD_WR_REG(LCD_Reg);  
	LCD_WR_DATA(LCD_RegValue);	    		 
}   
//读寄存器 主要用于读取id
//LCD_Reg:寄存器编号
//mode:读的位数8/24/32
//返回值:读到的值
u16 LCD_ReadReg(u16 LCD_Reg)
{	
  u16 t;
 	LCD_WR_REG(LCD_Reg);  //写入要读的寄存器号
	 t=LCD_RD_DATA();
	return t;
} 
//开始写GRAM
void LCD_WriteRAM_Prepare(void)
{
	LCD_WR_REG(lcddev.wramcmd);
} 
//LCD写GRAM
//RGB_Code:颜色值---RGB666格式 D7 D6 D5 D4 D3 D2 X X  高6位有效
void LCD_WriteRAM(u16 RGB_Code)
{	
//	LCD_CS_CLR;
	LCD_RS_SET;//写GRAM 高位在前
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
//从ILI93xx读出的数据为GBR格式，而我们写入的时候为RGB格式。
//通过该函数转换
//c:GBR格式的颜色值
//返回值：RGB格式的颜色值
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



//读取个某点的颜色值	 
//x,y:坐标
//返回值:此点的颜色
u16 LCD_ReadPoint(u16 x,u16 y)
{
	u16 rgb=0;
	if(x>=lcddev.width||y>=lcddev.height)return 0;	//超过了范围,直接返回		   
	LCD_Set_Window(x,y,1,1);	    
	if(lcddev.id==0X9341||lcddev.id==0X9486) 
	{
		LCD_WR_REG(0X2E);//9341 发送读GRAM指令
		rgb=LCD_RD_GRAM(0XFF); 
	}
	else
  {
	LCD_WR_REG(R34);      		 				//其他IC发送读GRAM指令  
  rgb=LCD_RD_GRAM(R34); 		
	}
 return rgb;						//其他IC
}			 
//LCD开启显示
void LCD_DisplayOn(void)
{					   
	if(lcddev.id==0X9341||lcddev.id==0X9486) LCD_WR_REG(0X29);	//开启显示

	else LCD_WriteReg(R7,0x0173); 				 	//开启显示
}	 
//LCD关闭显示
void LCD_DisplayOff(void)
{	   
	if(lcddev.id==0X9341||lcddev.id==0X9486)LCD_WR_REG(0X28);	//关闭显示
	else LCD_WriteReg(R7,0x0);//关闭显示 
}   
//设置光标位置
//Xpos:横坐标
//Ypos:纵坐标
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
		if(lcddev.dir==1)Xpos=lcddev.width-1-Xpos;//横屏时处理
		LCD_WR_REG(lcddev.setxcmd); 
		LCD_WR_DATA(Xpos>>8);LCD_WR_DATA(Xpos&0XFF); 
		LCD_WR_REG(lcddev.setycmd); 
		LCD_WR_DATA(Ypos>>8);LCD_WR_DATA(Ypos&0XFF); 
	}else if(lcddev.id==0X1963)
	{  			 		
		if(lcddev.dir==0)//x坐标需要变换
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
		if(lcddev.dir==1)Xpos=lcddev.width-1-Xpos;//横屏其实就是调转x,y坐标
		LCD_WriteReg(lcddev.setxcmd, Xpos);
		LCD_WriteReg(lcddev.setycmd, Ypos);
	}	 
} 		 
//设置LCD的自动扫描方向
//注意:其他函数可能会受到此函数设置的影响(尤其是9341/6804这两个奇葩),
//所以,一般设置为L2R_U2D即可,如果设置为其他扫描方式,可能导致显示不正常.
//dir:0~7,代表8个方向(具体定义见lcd.h)
//9320/9325/9328/4531/4535/1505/b505/5408/9341/5310/5510/1963等IC已经实际测试	   	   
void LCD_Scan_Dir(u8 dir)
{
	u16 regval=0;
	u16 dirreg=0;
	u16 temp;  
	if(lcddev.dir==1)//横屏时，改变扫描方向！
	{			   
		switch(dir)//方向转换
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
	
	if(lcddev.id==0x9341||lcddev.id==0x9486)//9341,很特殊
	{		
			switch(dir)
				{
					case L2R_U2D://从左到右,从上到下
						regval|=(0<<7)|(0<<6)|(0<<5); 
						break;
					case L2R_D2U://从左到右,从下到上
						regval|=(1<<7)|(0<<6)|(0<<5); 
						break;
					case R2L_U2D://从右到左,从上到下
						regval|=(0<<7)|(1<<6)|(0<<5); 
						break;
					case R2L_D2U://从右到左,从下到上
						regval|=(1<<7)|(1<<6)|(0<<5); 
						break;	 
					case U2D_L2R://从上到下,从左到右
						regval|=(0<<7)|(0<<6)|(1<<5); 
						break;
					case U2D_R2L://从上到下,从右到左
						regval|=(0<<7)|(1<<6)|(1<<5); 
						break;
					case D2U_L2R://从下到上,从左到右
						regval|=(1<<7)|(0<<6)|(1<<5); 
						break;
					case D2U_R2L://从下到上,从右到左
						regval|=(1<<7)|(1<<6)|(1<<5); 
						break;	 
				}
		
	  dirreg=0X36;
 		regval|=0X08;//需要BGR      
		LCD_WriteReg(dirreg,regval);
		
 		if((regval&0X20)||lcddev.dir==1)
		{
			if(lcddev.width<lcddev.height)//交换X,Y
			{
				temp=lcddev.width;
				lcddev.width=lcddev.height;
				lcddev.height=temp;
 			}
		}else  
		{
			if(lcddev.width>lcddev.height)//交换X,Y
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
			case L2R_U2D://从左到右,从上到下
				regval|=(1<<5)|(1<<4)|(0<<3); 
				break;
			case L2R_D2U://从左到右,从下到上
				regval|=(0<<5)|(1<<4)|(0<<3); 
				break;
			case R2L_U2D://从右到左,从上到下
				regval|=(1<<5)|(0<<4)|(0<<3);
				break;
			case R2L_D2U://从右到左,从下到上
				regval|=(0<<5)|(0<<4)|(0<<3); 
				break;	 
			case U2D_L2R://从上到下,从左到右
				regval|=(1<<5)|(1<<4)|(1<<3); 
				break;
			case U2D_R2L://从上到下,从右到左
				regval|=(1<<5)|(0<<4)|(1<<3); 
				break;
			case D2U_L2R://从下到上,从左到右
				regval|=(0<<5)|(1<<4)|(1<<3); 
				break;
			case D2U_R2L://从下到上,从右到左
				regval|=(0<<5)|(0<<4)|(1<<3); 
				break;	 
		} 
		dirreg=0X03;
		regval|=1<<12; 
		LCD_WriteReg(dirreg,regval);
	}
}     
//画点
//x,y:坐标
//POINT_COLOR:此点的颜色
void LCD_DrawPoint(u16 x,u16 y)
{
	LCD_Set_Window(x,y,1,1); //设置光标位置  
	LCD_WriteRAM_Prepare();	//开始写入GRAM
	//LCD->LCD_RAM=POINT_COLOR;
  LCD_WriteRAM(POINT_COLOR); 
}
//快速画点
//x,y:坐标
//color:颜色
void LCD_Fast_DrawPoint(u16 x,u16 y,u16 color)
{	LCD_Set_Window(x,y,1,1); //设置光标位置   
	LCD_WriteRAM_Prepare();	//开始写入GRAM
	//LCD->LCD_RAM=color; 
	LCD_WriteRAM(color); 
}	 


//设置LCD显示方向
//dir:0,竖屏；1,横屏
void LCD_Display_Dir(u8 dir)
{
	if(dir==0)			//竖屏
	{
		lcddev.dir=0;	//竖屏
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
	}else 				//横屏
	{	  				
		lcddev.dir=1;	//横屏
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
	LCD_Scan_Dir(DFT_SCAN_DIR);	//默认扫描方向
}	 
//设置窗口,并自动设置画点坐标到窗口左上角(sx,sy).
//sx,sy:窗口起始坐标(左上角)
//width,height:窗口宽度和高度,必须大于0!!
//窗体大小:width*height. 
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
//初始化lcd
//该初始化函数可以初始化各种ALIENTEK出品的LCD液晶屏
//本函数占用较大flash,用户可以根据自己的实际情况,删掉未用到的LCD初始化代码.以节省空间.
void LCD_Init(void)  //PA12 test
{ 
 	GPIO_InitTypeDef GPIO_InitStructure;
 	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE); //使能PORTB,C时钟和AFIO时钟
	
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
//  if(lcddev.id<0XFF||lcddev.id==0XFFFF||lcddev.id==0X9300)//读到ID不正确,新增lcddev.id==0X9300判断，因为9341在未被复位的情况下会被读成9300
//	{	
// 		//尝试9341 ID的读取		
//		 LCD_WR_REG(0XD3);		
// 		 LCD_ReadReg();   	    	//读到0X00
//   	 lcddev.id=LCD_ReadReg();   	//读取93								   
// 		 lcddev.id<<=8;
//		 lcddev.id|=LCD_ReadReg();  	//读取41 	   			   
////  	if(lcddev.id!=0X9341&&lcddev.id!=0X9486)		//非9341 也不是9486
//// 		{	
////			lcddev.id=0XFFFF;
////  	}  	
//	} 
	lcddev.id=0x9341;//test
 	printf(" LCD ID:%x\r\n",lcddev.id); //打印LCD ID  
	if(lcddev.id==0X9341)	//9341初始化
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
	
		LCD_WR_REG(0xB4); //显示反转设置
		LCD_WR_DATA(0x02); 
		
		LCD_WR_REG(0xB6);    // Display Function Control 
 		LCD_WR_DATA(0x02); 
 		//LCD_WR_DATA(0xA2); 
		LCD_WR_DATA(0x22); 
	
	  LCD_WR_REG(0xC1);  // 功率控制
		LCD_WR_DATA(0x41); 
		
		LCD_WR_REG(0xC5); 
		LCD_WR_DATA(0x00); 
		//LCD_WR_DATA(0x0C); //18
		LCD_WR_DATA(0x18);
		
		LCD_WR_REG(0x3A);  //像素格式设置jee
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
		
		LCD_WR_REG(0xB0);  //spi接口控制jee
		LCD_WR_DATA(0x0F); 
				
		LCD_WR_REG(0xB1);  //帧率控制
		//LCD_WR_DATA(0xB0); //70MHz
		LCD_WR_DATA(0x70); //46MHz
		LCD_WR_DATA(0x11); 
		
		LCD_WR_REG(0xC0); // 功率控制
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
	LCD_Display_Dir(1);		 	//默认为竖屏(0) 设置为 横屏
	LCD_LED=1;					//点亮背光
	LCD_Clear(WHITE);
}

//清屏函数
//color:要清屏的填充色
void LCD_Clear(u16 color)
{
	u32 index=0;      
	u32 totalpoint=lcddev.width;
	totalpoint*=lcddev.height; 			//得到总点数
	LCD_Set_Window(0,0,lcddev.width,lcddev.height);	//设置光标位置 
	LCD_WriteRAM_Prepare();     		//开始写入GRAM	 	  
	for(index=0;index<totalpoint;index++)
	{
		//LCD->LCD_RAM=color;	 
    LCD_WriteRAM(color);		
	}
}  
//在指定区域内填充单个颜色
//(sx,sy),(ex,ey):填充矩形对角坐标,区域大小为:(ex-sx+1)*(ey-sy+1)   
//color:要填充的颜色
void LCD_Fill(u16 sx,u16 sy,u16 width ,u16 height,u16 color)
{          
	u32 i;
	LCD_Set_Window(sx,sy,width,height);
	LCD_WriteRAM_Prepare();     			//开始写入GRAM	 
	for(i=0;i<(width*height);i++)
		{ //LCD_WR_DATA(color);	//设置光标位置 	
      LCD_WriteRAM(color);			
		}
	 
}  
//在指定区域内填充指定颜色块			 
//(sx,sy),(ex,ey):填充矩形对角坐标,区域大小为:(ex-sx+1)*(ey-sy+1)   
//color:要填充的颜色
void LCD_Color_Fill(u16 sx,u16 sy,u16 width,u16 height,u16 *color)
{  
	//u16 height,width;
	u32 i;
	//width=ex-sx+1; 			//得到填充的宽度
	//height=ey-sy+1;			//高度
	LCD_Set_Window(sx,sy,width,height);
	LCD_WriteRAM_Prepare();     //开始写入GRAM
 	for(i=0;i<height*width;i++)
	{	
		//LCD->LCD_RAM=color[i];//写入数据 
		LCD_WriteRAM(color[i]);
	}	  
}  
//画线
//x1,y1:起点坐标
//x2,y2:终点坐标  
void LCD_DrawLine(u16 x1, u16 y1, u16 x2, u16 y2)
{
	u16 t; 
	int xerr=0,yerr=0,delta_x,delta_y,distance; 
	int incx,incy,uRow,uCol; 
	delta_x=x2-x1; //计算坐标增量 
	delta_y=y2-y1; 
	uRow=x1; 
	uCol=y1; 
	if(delta_x>0)incx=1; //设置单步方向 
	else if(delta_x==0)incx=0;//垂直线 
	else {incx=-1;delta_x=-delta_x;} 
	if(delta_y>0)incy=1; 
	else if(delta_y==0)incy=0;//水平线 
	else{incy=-1;delta_y=-delta_y;} 
	if( delta_x>delta_y)distance=delta_x; //选取基本增量坐标轴 
	else distance=delta_y; 
	for(t=0;t<=distance+1;t++ )//画线输出 
	{  
		LCD_DrawPoint(uRow,uCol);//画点 
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
//画矩形	  
//(x1,y1),(x2,y2):矩形的对角坐标
void LCD_DrawRectangle(u16 x1, u16 y1, u16 x2, u16 y2)
{
	LCD_DrawLine(x1,y1,x2,y1);
	LCD_DrawLine(x1,y1,x1,y2);
	LCD_DrawLine(x1,y2,x2,y2);
	LCD_DrawLine(x2,y1,x2,y2);
}
//在指定位置画一个指定大小的圆
//(x,y):中心点
//r    :半径
void Draw_Circle(u16 x0,u16 y0,u8 r)
{
	int a,b;
	int di;
	a=0;b=r;	  
	di=3-(r<<1);             //判断下个点位置的标志
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
		//使用Bresenham算法画圆     
		if(di<0)di +=4*a+6;	  
		else
		{
			di+=10+4*(a-b);   
			b--;
		} 						    
	}
} 									  
//在指定位置显示一个字符
//x,y:起始坐标
//num:要显示的字符:" "--->"~"
//size:字体大小 12/16/24
//mode:叠加方式(1)还是非叠加方式(0)
void LCD_ShowChar(u16 x,u16 y,u8 num,u8 size,u8 mode)
{  							  
    u8 temp,t1,t;
	u16 y0=y;
	u16 colortemp=POINT_COLOR;      			     
	//设置窗口		   
	num=num-' ';//得到偏移后的值
	if(!mode) //非叠加方式
	{
	    for(t=0;t<size;t++)
	    {   
			if(size==12)temp=asc2_1206[num][t];  //调用1206字体
			else temp=asc2_1608[num][t];		 //调用1608字体 	                          
	        for(t1=0;t1<8;t1++)
			{			    
		        if(temp&0x80)POINT_COLOR=colortemp;
				else POINT_COLOR=BACK_COLOR;
				LCD_DrawPoint(x,y);	
				temp<<=1;
				y++;
				if(x>=lcddev.width){POINT_COLOR=colortemp;return;}//超区域了
				if((y-y0)==size)
				{
					y=y0;
					x++;
					if(x>=lcddev.width){POINT_COLOR=colortemp;return;}//超区域了
					break;
				}
			}  	 
	    }    
	}else//叠加方式
	{
	    for(t=0;t<size;t++)
	    {   
			if(size==12)temp=asc2_1206[num][t];  //调用1206字体
			else temp=asc2_1608[num][t];		 //调用1608字体 	                          
	        for(t1=0;t1<8;t1++)
			{			    
		        if(temp&0x80)LCD_DrawPoint(x,y); 
				temp<<=1;
				y++;
				if(x>=lcddev.height){POINT_COLOR=colortemp;return;}//超区域了
				if((y-y0)==size)
				{
					y=y0;
					x++;
					if(x>=lcddev.width){POINT_COLOR=colortemp;return;}//超区域了
					break;
				}
			}  	 
	    }     
	}
	POINT_COLOR=colortemp;	    	   	 	  
}   
//m^n函数
//返回值:m^n次方.
u32 LCD_Pow(u8 m,u8 n)
{
	u32 result=1;	 
	while(n--)result*=m;    
	return result;
}			 
//显示数字,高位为0,则不显示
//x,y :起点坐标	 
//len :数字的位数
//size:字体大小
//color:颜色 
//num:数值(0~4294967295);	 
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
//显示数字,高位为0,还是显示
//x,y:起点坐标
//num:数值(0~999999999);	 
//len:长度(即要显示的位数)
//size:字体大小
//mode:
//[7]:0,不填充;1,填充0.
//[6:1]:保留
//[0]:0,非叠加显示;1,叠加显示.
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
//显示字符串
//x,y:起点坐标
//width,height:区域大小  
//size:字体大小
//*p:字符串起始地址		  
void LCD_ShowString(u16 x,u16 y,u16 width,u16 height,u8 size,u8 *p)
{         
	u8 x0=x;
	width+=x;
	height+=y;
    while((*p<='~')&&(*p>=' '))//判断是不是非法字符!
    {       
        if(x>=width){x=x0;y+=size;}
        if(y>=height)break;//退出
        LCD_ShowChar(x,y,*p,size,0);
        x+=size/2;
        p++;
    }  
}

//白屏检测
//返回值: true 正常 false 故障
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




































