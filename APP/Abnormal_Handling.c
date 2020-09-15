#include "Abnormal_Handling.h"
#include "delay.h"//delay_ms(100);掉电中断消抖延时
#include "gcodeplayer.h"//card.sdprinting和card.fgcode//而CardReader card;是在marlin.c中定义，在gcodeplayer.h中extern的
//#include "Marlin.h"
#include "planner.h"//plan_clear_block(); //这里是仅仅将block_buffer_tail和block_buffer_head清零 ，用在有些地方需要将block清空后再清零  //20160330
#include "stepper.h"//quickStop();////舍弃block  buffer中的所有block和当前在运行的block,但是并没有将block_buffer_head和block_buffer_tail清零   但是这个.h里也包含了planner.h
#include "temperature.h"//target_temperature[0]
#include "lcdmenu.h"
#include "ConfigurationStore.h"
//#include "beep.h"
//#include "hmi.h"
#include "Marlin.h"
//#include "usart2.h"
//#include "hmi.h"

//extern volatile unsigned char block_buffer_head;           // Index of the block to process now
//extern volatile unsigned char block_buffer_tail;           // Index of the block to process now  //planner.c//20160603
extern volatile long count_position[NUM_AXIS];//stepper.c//20160603
extern float destination[NUM_AXIS];//Marlin.c//20160603
extern float feedrate; //20160404
extern int buflen;

static float current_seek_pos[4];
static float target_seek_pos[4];
//static float pause_count_position[4];

volatile u8 Abnormal_Flag = 0x00;

u8 Pause_flag = 0x00;

u8 fgcode_fptr_flag = 0x00;

unsigned long fgcode_fptr=0;
//unsigned long fgcode_clust;
//unsigned long fgcode_dsect;
//char Backup_pname[50];//20160315
//FIL fgcode_tail;//20160330
static int Backup_target_temperature = 0;
static int Backup_target_temperature2 = 0;
//static u32 Backup_feedrate = 0;
static int Backup_buflen = 0;
//static long stop_steps[4] = {0,0,0,0};

static u8 Backup_active_extruder=0;
//////new
static int Backup_fanSpeed=0;
static int Backup_target_bed=0;

u16 Store_selindex;
u16 Read_selindex;

//u8 Button_continue_flag = 0x00;

//volatile u8 Start_move_flag = 0x00;//从开始打印到开始走完第一个Blcok(打印模型的第一个点)的标志位
//volatile u8 Print_all_process = 0x00;//打印过程中，从点击打印，到打印完成或者取消打印的过程

//volatile u8 Power_Status = 0x00;//掉电和上电中断标志位   0x01：掉电     0x02：上电，都在中断结束后回到主循环中的Power_off_manage()函数中判断和清零

//static volatile u8 Power_off_process = 0x00;//掉电处理过程标志，在掉电归零后清零
//volatile u8 Power_on_process = 0x00; //上电处理过程标志，这个比较特殊，不是在上电归位清零，而是在上电归零后再回到掉电点后再清零，也就是在block_buffer_tail==0x01时清零


volatile u8 Outage_Flag = 0x00;

u8 Outage_cnt_flag=0x00;
u16 Outage_cnt=0;
u16 Upage_cnt=0;
u32 Outage_check_millis=0;

//static u8 Outage_enable_Flag = 0x00;


void Abnormal_Init(void) //异常处理：目前包括断电、断料和暂停
{
//   Outage_Pin_Init();
// 	 Outage_Set_IRQ_Rising();//20170218
//   Outage_EXIT_Set(ENABLE); //这儿使能没关系，下面有判断会再判断是否使能
	Material_Pin_Init();
	Material_IRQ_Init();
	//Check_Outage_Init();
}


//PE.6 检测 220V有无:        低电平:有200V表示有电     高电平：无220V表示断电   PE.5断电续打模块上的电源总开关：高电平为开，低电平关
//void Outage_Pin_Init(void) 
//{
// 	GPIO_InitTypeDef GPIO_InitStructure;
// 	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOE,ENABLE);//使能PORTE时钟

//	GPIO_InitStructure.GPIO_Pin  = GPIO_Pin_6;//PE6
//	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;//GPIO_Mode_IN_FLOATING; //设置成上拉输入  //20160412
// 	GPIO_Init(GPIOE, &GPIO_InitStructure);//初始化GPIOE3,6

//  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_5;				 //-->PE.5 端口配置 
//  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP; 		 //推挽输出
//  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;	 //速度为50MHz
//  GPIO_Init(GPIOE, &GPIO_InitStructure);	 //根据参数初始化GPIOE.5
//	POWER_KEY = 1;//GPIO_SetBits(GPIOE,GPIO_Pin_5);
//}

//void Outage_Set_IRQ_Rising(void) 
//{
//    EXTI_InitTypeDef EXTI_InitStructure;
//	
//    RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO,ENABLE);	//使能复用功能时钟
// //GPIOE.6	  中断线以及中断初始化配置 下降沿触发
//  	GPIO_EXTILineConfig(GPIO_PortSourceGPIOE,GPIO_PinSource6);
//	  EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;	//外部中断，并非外部事件
//  	EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Rising;//EXTI_Trigger_Rising;//只触发上升沿中断
//  	EXTI_InitStructure.EXTI_Line=EXTI_Line6;
//	  EXTI_InitStructure.EXTI_LineCmd = ENABLE;//中断不使能
//  	EXTI_Init(&EXTI_InitStructure);	  	//根据EXTI_InitStruct中指定的参数初始化外设EXTI寄存器
//}


//void Outage_Set_IRQ_Falling(void) 
//{
//    EXTI_InitTypeDef EXTI_InitStructure;
//	
//    RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO,ENABLE);	//使能复用功能时钟
// //GPIOE.6	  中断线以及中断初始化配置 下降沿触发
//  	GPIO_EXTILineConfig(GPIO_PortSourceGPIOE,GPIO_PinSource6);
//	  EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;	//外部中断，并非外部事件
//  	EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Falling;//EXTI_Trigger_Rising;//只触发上升沿中断
//  	EXTI_InitStructure.EXTI_Line=EXTI_Line6;
//	  EXTI_InitStructure.EXTI_LineCmd = ENABLE;//中断不使能
//  	EXTI_Init(&EXTI_InitStructure);	  	//根据EXTI_InitStruct中指定的参数初始化外设EXTI寄存器
//}

//void Outage_EXIT_Set(FunctionalState NewState) 
//{

//	  NVIC_InitTypeDef NVIC_InitStructure;
//	
//	  NVIC_InitStructure.NVIC_IRQChannel = EXTI9_5_IRQn;			//使能所在的外部中断通道
//  	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0x01;	//抢占优先级2， 
//  	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0x00;					//子优先级2
//  	NVIC_InitStructure.NVIC_IRQChannelCmd = NewState;								//使能外部中断通道
//  	NVIC_Init(&NVIC_InitStructure);
//}

//E-DEF--->PA12
void Material_Pin_Init(void) //IO初始化
{
 	GPIO_InitTypeDef GPIO_InitStructure;
 	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA|RCC_APB2Periph_GPIOD,ENABLE);//使能PORTG时钟

	GPIO_InitStructure.GPIO_Pin  = GPIO_Pin_12;//PA12
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING; //设置成上拉输入
 	GPIO_Init(GPIOA, &GPIO_InitStructure);//初始化GPIOA12
	
//	GPIO_InitStructure.GPIO_Pin  = GPIO_Pin_3;//PD3
//	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING; //设置成上拉输入
// 	GPIO_Init(GPIOD, &GPIO_InitStructure);//初始化GPIOG11	
}

void Material_EXIT_Set(FunctionalState NewState) 
{
 	  NVIC_InitTypeDef NVIC_InitStructure;
	
    NVIC_InitStructure.NVIC_IRQChannel = EXTI15_10_IRQn;			//使能按键KEY2所在的外部中断通道
  	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0x01;	//抢占优先级， 
  	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0x00;					//子优先级
  	NVIC_InitStructure.NVIC_IRQChannelCmd = NewState;								//使能外部中断通道
  	NVIC_Init(&NVIC_InitStructure);
	
//	  NVIC_InitStructure.NVIC_IRQChannel = EXTI3_IRQn;	//使能按键KEY2所在的外部中断通道
//  	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0x01;	//抢占优先级， 
//  	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0x01;					//子优先级
//  	NVIC_InitStructure.NVIC_IRQChannelCmd = NewState;								//使能外部中断通道
//  	NVIC_Init(&NVIC_InitStructure);
	
}

void Material_IRQ_Init(void)////PG11外中断使用EXTI9_5_IRQn中断通道，抢占优先级0，响应优先级0   
{
 
 	EXTI_InitTypeDef EXTI_InitStructure;
 	NVIC_InitTypeDef NVIC_InitStructure;


  	RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO,ENABLE);	//使能复用功能时钟

 //GPIOA12	  中断线以及中断初始化配置 下降沿触发 //KEY1
  	GPIO_EXTILineConfig(GPIO_PortSourceGPIOA,GPIO_PinSource12);
	  EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;	//外部中断，并非外部事件

  	 EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Rising_Falling;//EXTI_Trigger_Rising_Falling;//EXTI_Trigger_Rising;//只触发上升沿中断 //20160412
	  EXTI_InitStructure.EXTI_Line=EXTI_Line12;
	  EXTI_InitStructure.EXTI_LineCmd = ENABLE;//外部中断使能
  	EXTI_Init(&EXTI_InitStructure);	  	//根据EXTI_InitStruct中指定的参数初始化外设EXTI寄存器

    NVIC_InitStructure.NVIC_IRQChannel = EXTI15_10_IRQn;			//使能按键KEY2所在的外部中断通道
  	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0x01;	//抢占优先级2， 
  	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0x00;					//子优先级2
  	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;								//使能外部中断通道
  	NVIC_Init(&NVIC_InitStructure);

}


void EXTI15_10_IRQHandler(void)
{   
	if(EXTI_GetITStatus(EXTI_Line12) == SET)//PA12
	{
		EXTI_ClearITPendingBit(EXTI_Line12);    
		delay_ms(20);
		if(e0_endstops_inverting==0)
		{
			 if((PAin(12) == 1)&&(card.sdprinting == true)&&(count_position[E_AXIS]/axis_steps_per_unit[E_AXIS] != 0))                                                                        // 掉电    对于PE6来说是上升沿
			 {
						 card.sdprinting=false;	 
						 Abnormal_Flag = 0x01;
						 Pause_flag = 0x01;
				}
		 }
			 else
			 {
			   if((PAin(12) == 0)&&(card.sdprinting == true)&&(count_position[E_AXIS]/axis_steps_per_unit[E_AXIS] != 0))                                                                        // 掉电    对于PE6来说是上升沿
				  {
							 card.sdprinting=false;	 
							 Abnormal_Flag = 0x01;
							 Pause_flag = 0x01;
//							 beep_enable();
//							 nextMenu =  Material_over_screen;

					}
			 }
	 }
}



void Abnormal_Handling(u8 state)
{
        switch(state)
					{ case 0: 
//						        Outage_check();
						       // Auto_Check_Outage();//防止掉电上电再掉电再上电这么快速的操作会误触发错误的选项
										break;
						case 1:	//pause
							      Pause_flag = 0x01;
			              //beep_enable();
						        Material_Occur_Handling();
						        Abnormal_Flag = 0x00;
							      break;
						case 2: //play
							      Pause_flag = 0x00;
						        nextMenu = sdprint_screen;
						        
						        active_extruder = Backup_active_extruder;
				        
							      Material_Ok_Handling();//这里就是
						        Abnormal_Flag = 0x00;
						        break;		
            case 3: //stop
										Pause_flag = 0x00;
						        starttime=0;
						        //beep_enable();
										nextMenu=home_screen;	
						        
						        Cancle_Print_Handling();
						        Abnormal_Flag = 0x00;
						        break;	
           case 4:	
//							      Pause_flag = 0x01;
			              //beep_enable();
//						        Outage_Occur_Handling();
//						        Abnormal_Flag = 0x00;
							      break;
						case 5: 
//							      Pause_flag = 0x00;
//						        nextMenu = sdprint_screen;
//						        
//						        active_extruder = Backup_active_extruder;
//						        
////						        {
////											memset(send_buf, 0, sizeof(send_buf));		
////											strcpy((char*)send_buf,"page printing");	
////											strcat((char*)send_buf,(const char*)HMI_end);
////											u2_printf("%s",send_buf);
////							      }
//						        
//							      Outage_Ok_Handling();//这里就是
//						        Abnormal_Flag = 0x00;
						        break;
            case 6://打印完是跳转
										Pause_flag = 0x00;
										starttime=0;						
										nextMenu=end_print_screen;	
						        
//						        {
//											memset(send_buf, 0, sizeof(send_buf));		
//											strcpy((char*)send_buf,"page main");	
//											strcat((char*)send_buf,(const char*)HMI_end);
//											u2_printf("%s",send_buf);
//										}
//						
						        Cancle_Print_Handling();
						        Abnormal_Flag = 0x00;
						
                 break;							
						default: break;
					}			
}

void Material_Occur_Handling(void) 
{

    char TempBuffer[60];//20170303
				
	     current_seek_pos[X_AXIS] = current_position[X_AXIS];
			 current_seek_pos[Y_AXIS] = current_position[Y_AXIS];
			 current_seek_pos[Z_AXIS] = current_position[Z_AXIS];
			 current_seek_pos[E_AXIS] = current_position[E_AXIS];
		
			 fgcode_fptr =  card.fgcode.fptr;
			 Backup_target_temperature = target_temperature[0];
	         //if(temp_sensor_num==2)
	        // Backup_target_temperature2 = target_temperature[1];
			 Backup_buflen = buflen;
         
	     Backup_fanSpeed=LaserPower;
         Backup_target_bed=target_temperature_bed;
	     Backup_active_extruder = active_extruder;
	 
						

					
			if(current_position[X_AXIS]>pause_length)//暂停时的坐标大于15 
				target_seek_pos[X_AXIS] = current_position[X_AXIS]-pause_length;//就将暂停移动的目标值设置
			else
				target_seek_pos[X_AXIS] = current_position[X_AXIS]/2;
			
			if(current_position[Y_AXIS]>pause_length)
				target_seek_pos[Y_AXIS] = current_position[Y_AXIS]-pause_length;
			else
				target_seek_pos[Y_AXIS] = current_position[Y_AXIS]/2;
		
			if((max_pos[Z_AXIS]-(current_position[Z_AXIS]))>pause_length)
				target_seek_pos[Z_AXIS] = current_position[Z_AXIS]+pause_length;
			else
				target_seek_pos[Z_AXIS] = (max_pos[Z_AXIS]+current_position[Z_AXIS])/2;

			target_seek_pos[E_AXIS] = current_position[E_AXIS];
			
					
			memset(TempBuffer, 0, sizeof(TempBuffer));//填充目标位置
			sprintf(TempBuffer, "G0 X%5.2f Y%5.2f Z%5.2f E%8.2f F%d",target_seek_pos[X_AXIS],target_seek_pos[Y_AXIS],target_seek_pos[Z_AXIS],target_seek_pos[E_AXIS],80*60);
			enquecommand(TempBuffer);//enquecommand(PSTR("G1 Z50 F100"));
		
	

}

void Material_Ok_Handling(void) 
{
    char TempBuffer[60];//20170303

		memset(TempBuffer, 0, sizeof(TempBuffer));
		sprintf(TempBuffer, "G92 X%5.2f Y%5.2f Z%5.2f E%8.2f",target_seek_pos[X_AXIS],target_seek_pos[Y_AXIS],target_seek_pos[Z_AXIS],target_seek_pos[E_AXIS]);                                                        
		enquecommand(TempBuffer);//准备移动前，将当前位置设置为暂停位置(其实就是赋值count_position[X_AXIS])，这里只是保证
		
		memset(TempBuffer, 0, sizeof(TempBuffer));//填充目标位置
		sprintf(TempBuffer, "G0 X%5.2f Y%5.2f Z%5.2f E%8.2f F%d",current_seek_pos[X_AXIS],current_seek_pos[Y_AXIS],current_seek_pos[Z_AXIS],current_seek_pos[E_AXIS],40*60);
		enquecommand(TempBuffer);
		card.sdprinting = true;

}


//void Outage_Occur_Handling(void) 
//{
//		 setTargetHotend(0, 0);   //设置打印头1目标温度为0 并关闭风扇 
//	     if(temp_sensor_num==2)
//         setTargetHotend(0, 1);   //设置加热块2目标温度为0 并关闭风扇 20170611	
//         setTargetBed(0);
//         LaserPower=0;   
//	
//	     enquecommand(PSTR("G28 X0 Y0"));//11.填充X轴归位cmdbuffer  //bairen  enquecommand(PSTR("G28 X0 Y0 Z0"))   luozhong  enquecommand(PSTR("G28 X0"))
//			 
//			 POWER_KEY=0; 
//       //delay_ms(1000);
//}

//void Outage_Ok_Handling(void) 
//{
//	
//	   //  Discard_Buf_and_Block();//7.清除所有cmdbuffer和Block
//	
//  //6.自己填写添加一个预热命令  和XY归位命令   M109+Backup_target_temperature(备份断电前的打印目标温度)
//	              
//	                {
//									 char pname[12]="M190 S";
//									 char pname1[16]="M109 T0 S";
//					         char pname2[16]="M109 T1 S";
//									 char TempBuffer[3];
//									 sprintf(TempBuffer, "%3d",Backup_target_bed);
//									 strcat(pname,TempBuffer);	//拷贝path到pname里面
//									 enquecommand(pname);
//									 target_temperature_bed = Backup_target_bed;//**************************************************
//					
//							
//					          target_temperature[0] = Backup_target_temperature;
//									 sprintf(TempBuffer, "%3d",Backup_target_temperature);					                
//									 strcat(pname1,TempBuffer);	//拷贝path到pname里面
//									 enquecommand(pname1);
//					          if((temp_sensor_num==2)&&(Backup_target_temperature2>=EXTRUDE_MINTEMP))//20170825 
//									  {										  
//					                 target_temperature[1] = Backup_target_temperature2;
//					                 sprintf(TempBuffer, "%3d",Backup_target_temperature2);					                 
//					                 strcat(pname2,TempBuffer);	//拷贝path到pname里面
//									 enquecommand(pname2);
//									  }
//									//**************************************************
//									
//              	   }
//				   
//				  
//				   
//				  LaserPower=Backup_fanSpeed;
//	              active_extruder=Backup_active_extruder;
//				   

//								enquecommand(PSTR("G28 X0 Y0"));//20170303

//							
//								                                  
////										if(PEin(6) == 0)//此处防止在执行该函数过程中掉电，本来进入该函数已经确定说明有电，可是还是执行这个函数过程中会掉电  //20170218 
//										{
//											EXTI->EMR |= (1<<6);
//											Outage_EXIT_Set(ENABLE); //这儿使能没关系，下面有判断会再判断是否使能
//											nextMenu = sdprint_screen;
//										  Abnormal_Gcode_seek();	
//										}		
//}



void Cancle_Print_Handling(void) 
{
						quickStop();//Discard_Buf_and_Block();//quickStop(); //此处不能使用Discard_Buf_and_Block
						card_closefile();
						starttime=0;
						autotempShutdown();
						//setTargetHotend(0,tmp_extruder);//20161218
						{//20170111
						 int e; 
						 for(e = 0; e < EXTRUDERS; e++) 
						 setTargetHotend(0,e);//20161125  20170111
						}
						target_temperature_bed=0;
						{//20170111
						 int e; 
						 for(e = 0; e < EXTRUDERS; e++) 
						 heater_temp[e]=0;//20161125  20170111
						}
//											heater_0_temp = 0;//20160409
						bed_temp = 0;
						active_extruder = 0;//20161223
						tmp_extruder = 0;
						feedmultiply = 100;
						LaserPower = 0;
						
//						Print_end();//HMI 20170213
					
//						USART_ITConfig(USART3, USART_IT_RXNE, ENABLE);////20170213-8266
//		        TIM_ITConfig(TIM7,TIM_IT_Update,ENABLE );//TIM_Cmd(TIM7, ENABLE); 
												
						menu_action_gcode("G28 X0 Y0");
						//jee2018断电续打变量
//						appoint_z_height=0;
//						FLASH_WRITE_VAR(BACKUP_Z_HEIGHT_ADDRESS,appoint_z_height);
						SPI_Flash_Erase_Sector(FILE_NAME_ADDRESS/1024/4);//清除存储文件名2018
						SPI_Flash_Erase_Sector(FLASH_POWER_OFF_STORE_OFFSET/1024/4);//清除存储文件名2018
						
//							appoint_z_height=0;
//  						FLASH_WRITE_VAR(BACKUP_Z_HEIGHT_ADDRESS,appoint_z_height);

						if(SD_FINISHED_STEPPERRELEASE)
						{
						 //   finishAndDisableSteppers();
								menu_action_gcode(PSTR(SD_FINISHED_RELEASECOMMAND));
						}
//					starttime=0;//20160412
//					fgcode_fptr_flag = 0x00;
//				  SPI_Flash_Erase_Sector(FGCODE_FPTR_ADDRESS/1024/4);//两个地方擦除，1.在发现不是断电前的文件时2.是断电前文件，在移动到断电点时
					
}

void Abnormal_Gcode_seek(void) 
{
	 u8 cnt = 0;
	 char serial_char;
	 char TempBuffer[50];
	 unsigned long current_fptr;  
	 float seek_pos[4];

	f_lseek (&card.fgcode,fgcode_fptr);
		do
		{
			 f_lseek (&card.fgcode,card.fgcode.fptr-2);
			 serial_char = (BYTE)card_get();//读取该地址指针位置的值，并将地址指针+1
				if(serial_char == 'G')
				{
					if(Gcode_seek_valid() == true)//遇到G继续往前找，先找到换行说明card.fgcode.fptr指向'G'  cnt++,   如果先找到分号';'或者或者到头了就返回false
					cnt++;

					if (cnt>=(BLOCK_BUFFER_SIZE+Backup_buflen+1))
					{
            current_fptr = card.fgcode.fptr;//current_fptr指向'G'
						seek_pos[X_AXIS] = Gcode_seek_XYZE(current_fptr,X_AXIS,1);//向后查找
						seek_pos[Y_AXIS] = Gcode_seek_XYZE(current_fptr,Y_AXIS,1);//向后查找
						seek_pos[Z_AXIS] = Gcode_seek_XYZE(current_fptr,Z_AXIS,0);//向前查找
						seek_pos[E_AXIS] = Gcode_seek_XYZE(current_fptr,E_AXIS,0);//向前查找
						
						f_lseek (&card.fgcode,current_fptr);
						
						
						memset(TempBuffer, 0, sizeof(TempBuffer));//填充目标位置
						sprintf(TempBuffer, "G92 X%5.2f Y%5.2f Z%5.2f E%8.2f",base_home_pos[X_AXIS],base_home_pos[Y_AXIS],seek_pos[Z_AXIS],seek_pos[E_AXIS]-5);
						enquecommand(TempBuffer);
						
						memset(TempBuffer, 0, sizeof(TempBuffer));//填充目标位置
						sprintf(TempBuffer, "G0 X%5.2f Y%5.2f Z%5.2f E%8.2f F%d",seek_pos[X_AXIS],seek_pos[Y_AXIS],seek_pos[Z_AXIS],seek_pos[E_AXIS],40*60);
						enquecommand(TempBuffer);//enquecommand(PSTR("G1 Z50 F100"));

			

						card.sdprinting = true;
					  break;
					}
				}
				
			if ((card_eof())||(card.fgcode.fptr<=2))//如果朝前没有找到，就将指针指向暂停时的最后一个点的位置
			{
				f_lseek (&card.fgcode,fgcode_fptr);
				
				memset(TempBuffer, 0, sizeof(TempBuffer));//填充目标位置
					sprintf(TempBuffer, "G0 Z%5.2f F%d",current_seek_pos[Z_AXIS],40*60);
					enquecommand(TempBuffer);//enquecommand(PSTR("G1 Z50 F100"));
				
				card.sdprinting = true;
        break;	
			}
				
		}while(1);

}

void Abnormal_Z_Height(void) 
{
	 char TempBuffer[30];
	 float seek_pos[4]={0};
	 u8 seek_flag=0x00; 
//	 unsigned long backup_fptr=0;

//	  f_lseek (&card.fgcode,fgcode_fptr);
		do
		{
			seek_pos[Z_AXIS] = Gcode_seek_XYZE(card.fgcode.fptr,Z_AXIS,1);//向后查找 有效的'Z'并将返回该Z后面跟的数值  无效的是分号后的 'X', 'Y', 'Z', 'E'
			if(seek_flag == 0x00)
			{
				if((seek_pos[Z_AXIS]>0)&&(seek_pos[Z_AXIS]<1))
				 seek_flag = 0x01;
			}
			else
			{
			  if((temp_z_height-seek_pos[Z_AXIS])<floor_height/5)
				{		
					  FLASH_READ_VAR(BACKUP_BED_TARGET_TEM_ADDRESS,target_temperature_bed); //JEE2018
						FLASH_READ_VAR(BACKUP_TARGET_TEM_ADDRESS,target_temperature[0]);  //JEE2018					
					 
					  if(target_temperature_bed>0&&target_temperature_bed<bed_maxtemp)
						{
						memset(TempBuffer, 0, sizeof(TempBuffer));
					  sprintf(TempBuffer, "M190 S%d",target_temperature_bed);                                                        
					  enquecommand(TempBuffer);
					  }else
						{
						target_temperature_bed=0;
						}
             if(target_temperature[0]>0&&target_temperature[0]<heater_0_maxtemp)
						 {
							memset(TempBuffer, 0, sizeof(TempBuffer));
							sprintf(TempBuffer, "M109 S%d",target_temperature[0]);                                                        
							enquecommand(TempBuffer);
						 }else//默认
						 {					 
						  enquecommand("M109 S210");
						 }			
//						seek_pos[X_AXIS] = Gcode_seek_XYZE(card.fgcode.fptr,X_AXIS,1);
//						seek_pos[Y_AXIS] = Gcode_seek_XYZE(card.fgcode.fptr,Y_AXIS,0);
						seek_pos[E_AXIS] = Gcode_seek_XYZE(card.fgcode.fptr,E_AXIS,0);
						 
						enquecommand("G91");								 				                                                      
						enquecommand("G1 Z5 F900");//上升5
						enquecommand("G90");								 
						enquecommand(PSTR("G28 X0 Y0"));
						 
						memset(TempBuffer, 0, sizeof(TempBuffer));
						sprintf(TempBuffer, "G92 Z%.3f E%.3f",temp_z_height+z_homeing_ofsize+5,seek_pos[E_AXIS]-5.0);                                                        
						enquecommand(TempBuffer);
						
//						memset(TempBuffer, 0, sizeof(TempBuffer));					 
//						sprintf(TempBuffer, "G1 X%.2f Y%.2f Z%.2f F1200",seek_pos[X_AXIS],seek_pos[Y_AXIS],temp_z_height);
//						menu_action_gcode(TempBuffer);
	
					break;
				}
			}
			
			if ((card_eof())||(card.fgcode.fptr<=1))//如果朝前没有找到，就将指针指向暂停时的最后一个点的位置
			{
				//f_lseek (&card.fgcode,fgcode_fptr);
				
				nextMenu = home_screen;
				card.sdprinting=false;
				Discard_Buf_and_Block();
				Abnormal_Flag = 0x03;
				
//				appoint_z_height=0;
//      	FLASH_WRITE_VAR(BACKUP_Z_HEIGHT_ADDRESS,appoint_z_height);
        break;	
			}
				
		}while(1);

}
bool Gcode_seek_valid()
{
	   char serial_char;
        do
				{
				  f_lseek (&card.fgcode,card.fgcode.fptr-2);
					serial_char = (char)card_get();
					if(serial_char == ';')
						return false;
					else if((serial_char == '\n')||(serial_char == '\r'))
						return true;
					
				if((card_eof())||(card.fgcode.fptr<=2))
					{
					 return false;
					} 
				}while(1);
				

}

float Gcode_seek_XYZE(u32 cur_fptr,int axis,bool direction)//前提是该文件已经打开
{
	 char serial_char;
	 char buffer[8];
	 u8 i;
	 const char axis_xyze[4] = {'X', 'Y', 'Z', 'E'};
	 unsigned long backup_fptr;

		f_lseek (&card.fgcode,cur_fptr+1);
    do
		{
			if(direction == 0)
			 f_lseek (&card.fgcode,card.fgcode.fptr-2);
			 serial_char = (char)card_get();
			if (axis==X_AXIS ? (serial_char == axis_xyze[X_AXIS]) :
					axis==Y_AXIS ? (serial_char == axis_xyze[Y_AXIS]) :
					axis==Z_AXIS ? (serial_char == axis_xyze[Z_AXIS]) :
					axis==E_AXIS ? (serial_char == axis_xyze[E_AXIS]) :
					0)
			{
				 backup_fptr = card.fgcode.fptr;
						if(Gcode_seek_valid() == true)//检测是合格的XYZE
						{
							 f_lseek (&card.fgcode,backup_fptr);//定位到合格点
							for(i=0;i<8;i++)
							{
								 buffer[i] = (char)card_get();//取出合格的值的字符串
							}
						 return (strtod(buffer,NULL));//计算返回合格的值
						}
						else//不合格继续查找
						f_lseek (&card.fgcode,backup_fptr);
			}
			
			if((card_eof())||(card.fgcode.fptr<=2))
			{
			 return 0.0;
			} 
		}while(1);
}

void Discard_Buf_and_Block() //清除所有的Block和cmdbuffer,但是并没有将block_buffer_head和block_buffer_tail清零   只在暂停和取消中才用到，在继续打印和上电中不用，因为如果在掉电或暂停上移动的过程中又上电了，那么没有走够15就会停了
{
		Clear_cmdbuffer();//这里清空，也清零cmdbuffer[bufindr][bufindw]   //20160330
		quickStop(); ////舍弃block  buffer中的所有block和当前在运行的block,但是并没有将block_buffer_head和block_buffer_tail清零
	
}

void Abnormal_Store()//总共6个地方保存，4个点暂停，1个延时掉电判断，一个断料中断里
{
	//char strore_power_off[14];
	char strore_power_off[27];
		fgcode_fptr =  card.fgcode.fptr;
		Backup_target_temperature = target_temperature[0];
       // if(temp_sensor_num==1)
			 Backup_target_temperature2=0;	
	    //else Backup_target_temperature2 = target_temperature[1];
		Backup_buflen = buflen;

		Backup_fanSpeed=LaserPower;
	    Backup_target_bed= target_temperature_bed; 
	    Backup_active_extruder = active_extruder;	
	
	strore_power_off[0] = BREAK_UINT32(fgcode_fptr,0);
	strore_power_off[1] = BREAK_UINT32(fgcode_fptr,1);
	strore_power_off[2] = BREAK_UINT32(fgcode_fptr,2);
	strore_power_off[3] = BREAK_UINT32(fgcode_fptr,3);
	
  strore_power_off[4] = LO_UINT16(Store_selindex);
	strore_power_off[5] = HI_UINT16(Store_selindex);

	
	strore_power_off[6] = BREAK_UINT32(Backup_target_temperature,0);
	strore_power_off[7] = BREAK_UINT32(Backup_target_temperature,1);
	strore_power_off[8] = BREAK_UINT32(Backup_target_temperature,2);
	strore_power_off[9] = BREAK_UINT32(Backup_target_temperature,3);

	
	strore_power_off[10] = BREAK_UINT32(Backup_buflen,0);
	strore_power_off[11] = BREAK_UINT32(Backup_buflen,1);
	strore_power_off[12] = BREAK_UINT32(Backup_buflen,2);
	strore_power_off[13] = BREAK_UINT32(Backup_buflen,3);

	strore_power_off[14] = BREAK_UINT32(Backup_fanSpeed,0);
	strore_power_off[15] = BREAK_UINT32(Backup_fanSpeed,1);
	strore_power_off[16] = BREAK_UINT32(Backup_fanSpeed,2);
	strore_power_off[17] = BREAK_UINT32(Backup_fanSpeed,3);
	
	strore_power_off[18] = BREAK_UINT32(Backup_target_bed,0);
	strore_power_off[19] = BREAK_UINT32(Backup_target_bed,1);
	strore_power_off[20] = BREAK_UINT32(Backup_target_bed,2);
	strore_power_off[21] = BREAK_UINT32(Backup_target_bed,3);
	
	strore_power_off[22] = Backup_active_extruder;
	
	strore_power_off[23] = BREAK_UINT32(Backup_target_temperature2,0);
	strore_power_off[24] = BREAK_UINT32(Backup_target_temperature2,1);
	strore_power_off[25] = BREAK_UINT32(Backup_target_temperature2,2);
	strore_power_off[26] = BREAK_UINT32(Backup_target_temperature2,3);
	
	FLASH_WRITE_VAR(FGCODE_FPTR_ADDRESS,strore_power_off);
}

void Abnormal_Read()
{
   FLASH_READ_VAR(FGCODE_FPTR_ADDRESS,fgcode_fptr);            //244        unsigned long      4个字节   掉电时的文件位置指针//20160506 
	 FLASH_READ_VAR(FGCODE_FPTR_ADDRESS+4,Read_selindex);
	 FLASH_READ_VAR(FGCODE_FPTR_ADDRESS+6,Backup_target_temperature);
	 FLASH_READ_VAR(FGCODE_FPTR_ADDRESS+10,Backup_buflen);//BACKUP_BUFLEN_ADDRESS
	FLASH_READ_VAR(FGCODE_FPTR_ADDRESS+14,Backup_fanSpeed);
	FLASH_READ_VAR(FGCODE_FPTR_ADDRESS+18,Backup_target_bed);
	FLASH_READ_VAR(FGCODE_FPTR_ADDRESS+22,Backup_active_extruder);
	FLASH_READ_VAR(FGCODE_FPTR_ADDRESS+23,Backup_target_temperature2);
}
