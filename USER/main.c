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
//��װ����

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
								 get_command();                                                                                                     //��ȡָ��
							 }
							#ifdef SDSUPPORT
							 if(SD_CD)
							 {
								card.cardOK = false;
							 }
							card_checkautostart(); //���SD ��
							#endif

							 if(buflen) // �����ָ��
							 {
									#ifdef SDSUPPORT
										if(card.saving)
										{	 
											if(strstr(cmdbuffer[bufindr], PSTR("M29")) == NULL)//����������û��M29(ֹͣ��SD����д������)
											{
												card_write_command(cmdbuffer[bufindr]);
//												printf(MSG_OK);
											}
											else                                             //���û��
											{
												card_closefile();
					//							exfuns_init();							//Ϊfatfs��ر��������ڴ�	
					//							f_mount(0,fs[0]); 					 	//����SD�� //20160412
					//							card_initsd();//20160409����M29˵��������ɣ�Ȼ��ر��ļ����³�ʼ��SD��
//												printf(MSG_FILE_SAVED);
												__set_FAULTMASK(1); //�ر������ж�
												NVIC_SystemReset();// ��λ
											}
											 printf("\n");
										}
										else
										{
												process_commands();                                                                                        //��������
										}
									#else
										process_commands();  //ָ�����
									#endif //SDSUPPORT
										if(buflen > 0) 
										{						//��������������-1//20160403
											buflen = (buflen-1);
											bufindr = (bufindr + 1)%BUFSIZE;
										}
							 }
						
						manage_heater();
						manage_inactivity();
						checkHitEndstops();

						checkDTR();//20160228
						//lcd_update();//5.ѭ����ʾ ��ѭ����ʾ
						
						  switch(tool_cnt)//
							{   
							  	case 0: 
                         if((current_temperature[0]>180)&&(current_temperature[0]<200))
												 {
												  gui_show_strmid(0,30,200,20,BLACK,16,"��ӡͷ�¶ȼ�⣺    OK   ",1);
													//gui_show_strmid(0,30,200,20,BLACK,16,"��ӡͷ�¶ȼ�⣺    ERROR",1);
													 
                           sprintf(TempBuffer, "M104 S%.1f", 270.0);
												   menu_action_gcode(TempBuffer);	
												 }
												 else
													gui_show_strmid(0,30,200,20,BLACK,16,"��ӡͷ�¶ȼ�⣺    ERROR",1);

													
												 tool_cnt++;
												 break;
									case 1:	
                         if((current_temperature_bed>2)&&(current_temperature_bed<40))
												 {
												  gui_show_strmid(0,60,200,20,BLACK,16,"�ȴ��¶ȼ�⣺      OK   ",1);
													//gui_show_strmid(0,60,200,20,BLACK,16,"�ȴ��¶ȼ�⣺      ERROR",1);
													 
													sprintf(TempBuffer, "M140 S%.1f", 90.0);
												   menu_action_gcode(TempBuffer);	
													
												 }
												 else
													gui_show_strmid(0,60,200,20,BLACK,16,"�ȴ��¶ȼ�⣺      ERROR",1);
												 
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
														 gui_show_strmid(0,90,200,20,BLACK,16,"X����λ��⣺       OK   ",1);													
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
														 gui_show_strmid(0,120,200,20,BLACK,16,"Y����λ��⣺       OK   ",1);
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
														 gui_show_strmid(0,150,200,20,BLACK,16,"Z����λ��⣺       OK   ",1);
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
                nextMenu = home_screen;  //������һ������
								
									break;	
							}
					 }
}

 int main(void)
 { 

	SystemInit();			//ϵͳ��ʼ��	
	delay_init();	    	//��ʱ������ʼ��
	NVIC_Configuration(); 	//����NVIC�жϷ���2:2λ��ռ���ȼ���2λ��Ӧ���ȼ�

	TIM4_Int_Init(9,7199);//72M  7199��Ƶ��1s��Ҫ10000��72000000/7199=10000Hz����ʱ��Ƶ�ʣ�1ms��Ҫ10�����˴���9����1ms
	uart1_init(115200);	 	//���ڳ�ʼ��Ϊ115200
	//uart2_init(115200);
//	usart3_init(115200);

	SPI1_Init();		   	//��ʼ��FLASH SPI�ӿ� 
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
	 

	mem_init(SRAMIN);		//��ʼ���ڲ��ڴ��
	 LCD_Init();         //LCD
	Abnormal_Init();     //�쳣����
	tp_dev.init();      //��ʼ�����������

	if(SPI_Flash_ReadID()!= W25Q64)	
		LCD_ShowString(60,70,200,16,16,"W25Q64 Check Failed!");	  //��ⲻ��W25Q16
	//else LCD_ShowString(60,70,200,16,16,"W25Q16 ready!");
  //lcd��ʼ��ǰ��Ҫ�ȳ�ʼ��flash
	//Ҫ�ȳ�ʼ��flash
//  FLASH_READ_VAR(LCD_SCANDIR_ADRASS,lcddircheck);	
//  if(LCDScanDir!=lcddircheck)
//	{
//	 LCDScanDir= R2L_D2U;	
//	 FLASH_WRITE_VAR(LCD_SCANDIR_ADRASS,LCDScanDir);
//	 LCD_Init();				//��ʼ��LCD
//	 TP_Adjust();
//	 TP_Get_Adjdata();
//	}	
	if(SD_Init())	//20180111
	{
	 //LCD_ShowString(60,90,200,16,16,"SD Card Check Failed!");	//��ⲻ��SD��
	}
//	else LCD_ShowString(60,90,200,16,16,"SD Card ready!");
 	
 	exfuns_init();							//Ϊfatfs��ر��������ڴ�				 
  f_mount(0,fs[0]); 					 	//����SD�� 
	if(font_init())	//�������,������岻����,������ֿ�
	{	
		while(SD_CD);
		delay_ms(10);
	  while(SD_Init());
		LCD_Clear(WHITE);//����
		if(update_font(5,0,16,0)==0 ) {}//��SD������
		else
		{
			LCD_ShowString(5,80,200,200,16, "Font Updata Failed!");	
			while(1);
		}
		//LCD_Clear(WHITE);//����		
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

 	   gui_init();			//gui��ʼ��		  //��ʼ��Ϊ��������
	//piclib_init();		//piclib��ʼ��

  	
			 {
      GPIO_InitTypeDef  GPIO_InitStructure;
			RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA,ENABLE);//ʹ��PORTC,PORTEʱ��
			GPIO_InitStructure.GPIO_Pin  = GPIO_Pin_13;//PA13
			GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU; //���ó���������
			GPIO_Init(GPIOA, &GPIO_InitStructure);//��ʼ��13  
			 
		//	 if(PAin(13)==0) 
			// {
			//	 setup();
			//	 LCD_Clear(WHITE);
			//	 gui_show_strmid(0,0,200,20,RED,16,"���빤װ���Գ���:",1);
				 
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

