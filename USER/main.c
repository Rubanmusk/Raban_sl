#include "delay.h"
#include "sys.h"
#include "usart.h"
#include "lcd.h"
#include "malloc.h"  
//#include "MMC_SD.h" 
#include "sdio_sdcard.h"
#include "ff.h"  
#include "exfuns.h"
#include "fontupd.h"
#include "text.h"	
#include "marlin.h"
#include "spi.h"
#include "flash.h"
#include "gui.h"
#include "touch.h"
#include "ConfigurationStore.h"
#include "planner.h"
#include "icon_code.h"
#include "gcodeplayer.h"
#include "lcdmenu.h"

#define  LOGO_HOLD_TIME logo_hold_time
u8 logo_hold_time =3;
extern u8 lcddircheck; 
//工装程序

void tool_test()
{
  
				 u8 tool_cnt=0;
				 char TempBuffer[32];
				 
				 static bool old_x_min=true;
				 static bool old_y_min=true;
				 static bool old_z_min=true;
				 bool x_min;
				 bool y_min;
				 bool z_min;
				 
				 u32 tool_time=millis();

				  while(1)
					{
						
							if(buflen < (BUFSIZE-1))
							 {
								 get_command();                                                                                                     //读取指令
							 }
							#ifdef SDSUPPORT
							 if(SD_CD)
							 {
								card.cardOK = false;
							 }
							card_checkautostart(); //检测SD 卡
							#endif

							 if(buflen) // 如果有指令
							 {
									#ifdef SDSUPPORT
										if(card.saving)
										{	 
											if(strstr(cmdbuffer[bufindr], PSTR("M29")) == NULL)//如果命令包中没有M29(停止向SD卡中写入数据)
											{
												card_write_command(cmdbuffer[bufindr]);
//												printf(MSG_OK);
											}
											else                                             //如果没有
											{
												card_closefile();
					//							exfuns_init();							//为fatfs相关变量申请内存	
					//							f_mount(0,fs[0]); 					 	//挂载SD卡 //20160412
					//							card_initsd();//20160409遇到M29说明传输完成，然后关闭文件重新初始化SD卡
//												printf(MSG_FILE_SAVED);
												__set_FAULTMASK(1); //关闭所有中断
												NVIC_SystemReset();// 复位
											}
											 printf("\n");
										}
										else
										{
												process_commands();                                                                                        //解析命令
										}
									#else
										process_commands();  //指令解析
									#endif //SDSUPPORT
										if(buflen > 0) 
										{						//解析完后命令个数-1//20160403
											buflen = (buflen-1);
											bufindr = (bufindr + 1)%BUFSIZE;
										}
							 }
						
						manage_heater();
						manage_inactivity();
						checkHitEndstops();

						checkDTR();//20160228
						//lcd_update();//5.循环显示 主循环显示
						
						  switch(tool_cnt)//
							{   
							  	case 0: 
                         if((current_temperature[0]>180)&&(current_temperature[0]<200))
												 {
												  gui_show_strmid(0,30,200,20,BLACK,16,"打印头温度检测：    OK   ",1);
													//gui_show_strmid(0,30,200,20,BLACK,16,"打印头温度检测：    ERROR",1);
													 
                           sprintf(TempBuffer, "M104 S%.1f", 270.0);
												   menu_action_gcode(TempBuffer);	
												 }
												 else
													gui_show_strmid(0,30,200,20,BLACK,16,"打印头温度检测：    ERROR",1);

													
												 tool_cnt++;
												 break;
									case 1:	
                         if((current_temperature_bed>2)&&(current_temperature_bed<40))
												 {
												  gui_show_strmid(0,60,200,20,BLACK,16,"热床温度检测：      OK   ",1);
													//gui_show_strmid(0,60,200,20,BLACK,16,"热床温度检测：      ERROR",1);
													 
													sprintf(TempBuffer, "M140 S%.1f", 90.0);
												   menu_action_gcode(TempBuffer);	
													
												 }
												 else
													gui_show_strmid(0,60,200,20,BLACK,16,"热床温度检测：      ERROR",1);
												 
												 tool_cnt++;
													break;						 
									case 2:	
									      {
													 
													 menu_action_gcode("G91");
													 sprintf(TempBuffer, "G1 X%.1f F%d",10.0,80*60);
													 menu_action_gcode(TempBuffer);
													 menu_action_gcode("G90");
													
													 menu_action_gcode("G91");
													 sprintf(TempBuffer, "G1 X-%.1f F%d",10.0,80*60);
													 menu_action_gcode(TempBuffer);
													 menu_action_gcode("G90");
													tool_cnt++;
												}

												break;
                  case 3:	
										   	{
												   x_min= X_MIN_PIN != X_ENDSTOPS_INVERTING;
													 if(x_min && old_x_min) 
													 {													
														 gui_show_strmid(0,90,200,20,BLACK,16,"X轴限位检测：       OK   ",1);													
														 tool_cnt++;
													 }
														old_x_min = x_min;
												}
												break;													
                  case 4:	
									      {

													 menu_action_gcode("G91");
													 sprintf(TempBuffer, "G1 Y%.1f F%d",10.0,80*60);
													 menu_action_gcode(TempBuffer);
													 menu_action_gcode("G90");
													
													 menu_action_gcode("G91");
													 sprintf(TempBuffer, "G1 Y-%.1f F%d",10.0,80*60);
													 menu_action_gcode(TempBuffer);
													 menu_action_gcode("G90");
													tool_cnt++;
													
												}

												break;	
                  case 5:	
												{
													 old_y_min=true;
												   y_min= Y_MIN_PIN != Y_ENDSTOPS_INVERTING;
													 if(y_min && old_y_min) 
													 {										
														 gui_show_strmid(0,120,200,20,BLACK,16,"Y轴限位检测：       OK   ",1);
														 tool_cnt++;
													 }
														old_y_min = y_min;
												}
												break;													
                  case 6:	
									      {
													 
													 menu_action_gcode("G91");
													 sprintf(TempBuffer, "G1 Z%.1f F%d",1.0,80*60);
													 menu_action_gcode(TempBuffer);
													 menu_action_gcode("G90");
													
													 menu_action_gcode("G91");
													 sprintf(TempBuffer, "G1 Z-%.1f F%d",1.0,80*60);
													 menu_action_gcode(TempBuffer);
													 menu_action_gcode("G90");
													
													
													menu_action_gcode("G91");
													 sprintf(TempBuffer, "G1 E%.1f F%d",20.0,80*60);
													 menu_action_gcode(TempBuffer);
													 menu_action_gcode("G90"); 
													
													 menu_action_gcode("G91");
													 sprintf(TempBuffer, "G1 E-%.1f F%d",20.0,80*60);
													 menu_action_gcode(TempBuffer);
													 menu_action_gcode("G90");
													 
													 tool_cnt++;
												}

												break;	
									case 7:	
												{
													 old_z_min=true;
												   z_min= Z_MIN_PIN != Z_ENDSTOPS_INVERTING;
													 if(z_min && old_z_min) 
													 {												
														 gui_show_strmid(0,150,200,20,BLACK,16,"Z轴限位检测：       OK   ",1);
														 tool_cnt++;
													 }
														old_z_min = z_min;
												}
												break;													
                  case 8:	

												break;													
									default: break;
							}
							
							if((tool_cnt>=8)||((millis()-tool_time)>=40000))
							{
								  tool_cnt=0;
									target_temperature[0] = 0;
									target_temperature_bed = 0;
									E0_FAN=0;
									disable_x();
									disable_y();
									disable_z();
									disable_e0();
							
								if(lcddev.width==480&&lcddev.height==320)
								 gui_draw_color_bmp(0,0,lcddev.width,lcddev.height,START_PIC0);
								 if(lcddev.width==320&&lcddev.height==240)
								 gui_draw_color_bmp(0,0,lcddev.width,lcddev.height,START_PIC2);
//								 HMI_Reset();
								 delay_ms(50000);	  
								 delay_ms(50000);	 
								  
								currentMenu = tool_screen; /* function pointer to the currently active menu */
                nextMenu = home_screen;  //进入下一个界面
								
									break;	
							}
					 }
}

 int main(void)
 { 

	SystemInit();			//系统初始化	
	delay_init();	    	//延时函数初始化
	NVIC_Configuration(); 	//设置NVIC中断分组2:2位抢占优先级，2位响应优先级

	TIM4_Int_Init(9,7199);//72M  7199分频，1s需要10000（72000000/7199=10000Hz）个时钟频率，1ms需要10个，此处填9代表1ms
	uart1_init(115200);	 	//串口初始化为115200
	//uart2_init(115200);
//	usart3_init(115200);

	SPI1_Init();		   	//初始化FLASH SPI接口 
	SPI2_Init();        //TLV5618
	 
	{
			 //
		  u8 txData[3];
		   u32 inputShiftData = 0x00187FFF;

			txData[0]=inputShiftData>>16;

			txData[1]=inputShiftData>>8;

			txData[2]=inputShiftData;
		 GALVO_SS_PIN = 0 ;
	 SPI2_WriteByte(txData[0]);
	SPI2_WriteByte(txData[1]);	 
	SPI2_WriteByte(txData[2]);	
	delay_ms(1);		 
		  GALVO_SS_PIN = 1 ;
		 
		
		 inputShiftData = 0x0019FFFF;

			txData[0]=inputShiftData>>16;

			txData[1]=inputShiftData>>8;

			txData[2]=inputShiftData;
		 GALVO_SS_PIN = 0 ;
	 SPI2_WriteByte(txData[0]);
	SPI2_WriteByte(txData[1]);	 
	SPI2_WriteByte(txData[2]);	
	delay_ms(1);		 
		  GALVO_SS_PIN = 1 ;

	} 
	 

	mem_init(SRAMIN);		//初始化内部内存池
	 LCD_Init();         //LCD
	Abnormal_Init();     //异常处理
	tp_dev.init();      //初始化触摸屏组件

	if(SPI_Flash_ReadID()!= W25Q64)	
		LCD_ShowString(60,70,200,16,16,"W25Q64 Check Failed!");	  //检测不到W25Q16
	//else LCD_ShowString(60,70,200,16,16,"W25Q16 ready!");
  //lcd初始化前需要先初始化flash
	//要先初始化flash
//  FLASH_READ_VAR(LCD_SCANDIR_ADRASS,lcddircheck);	
//  if(LCDScanDir!=lcddircheck)
//	{
//	 LCDScanDir= R2L_D2U;	
//	 FLASH_WRITE_VAR(LCD_SCANDIR_ADRASS,LCDScanDir);
//	 LCD_Init();				//初始化LCD
//	 TP_Adjust();
//	 TP_Get_Adjdata();
//	}	
	if(SD_Init())	//20180111
	{
	 //LCD_ShowString(60,90,200,16,16,"SD Card Check Failed!");	//检测不到SD卡
	}
//	else LCD_ShowString(60,90,200,16,16,"SD Card ready!");
 	
 	exfuns_init();							//为fatfs相关变量申请内存				 
  f_mount(0,fs[0]); 					 	//挂载SD卡 
	if(font_init())	//检测字体,如果字体不存在,则更新字库
	{	
		while(SD_CD);
		delay_ms(10);
	  while(SD_Init());
		LCD_Clear(WHITE);//清屏
		if(update_font(5,0,16,0)==0 ) {}//从SD卡更新
		else
		{
			LCD_ShowString(5,80,200,200,16, "Font Updata Failed!");	
			while(1);
		}
		//LCD_Clear(WHITE);//清屏		
 	}
  if(theme_init())
	{
		LCD_Clear(WHITE);
		if(update_theme(5,0,16)==0 ) {}
		else
		{
			LCD_ShowString(5,80,200,200,16, "Theme Update Failed!");	
			while(1);
		}
//		LCD_Clear(WHITE);
	}

 	   gui_init();			//gui初始化		  //初始化为简体中文
	//piclib_init();		//piclib初始化

  	
			 {
      GPIO_InitTypeDef  GPIO_InitStructure;
			RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA,ENABLE);//使能PORTC,PORTE时钟
			GPIO_InitStructure.GPIO_Pin  = GPIO_Pin_13;//PA13
			GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU; //设置成上拉输入
			GPIO_Init(GPIOA, &GPIO_InitStructure);//初始化13  
			 
		//	 if(PAin(13)==0) 
			// {
			//	 setup();
			//	 LCD_Clear(WHITE);
			//	 gui_show_strmid(0,0,200,20,RED,16,"进入工装测试程序:",1);
				 
			//	 tool_test();
			// }
		//	 else
			// {
			  	 if(lcddev.width==480&&lcddev.height==320)
					 gui_draw_color_bmp(0,0,lcddev.width,lcddev.height,START_PIC0);
					 if(lcddev.width==320&&lcddev.height==240)
					 gui_draw_color_bmp(0,0,lcddev.width,lcddev.height,START_PIC2);
				
					 while(LOGO_HOLD_TIME--)
					  delay_ms(1000);
					 setup();  
			// }
	   }

	 loop();
	
	} 

