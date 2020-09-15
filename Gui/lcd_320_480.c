#include "lcd_320_480.h"
#include "ConfigurationStore.h"
#include "beep.h" 
#include "usart2.h"	
#include "material_over.h"



extern char showString[30];//输入界面使用
extern volatile long count_position[NUM_AXIS];

extern  uint32_t display_next_update_millis;

extern bool redraw_screen ;  //界面重画标志
extern bool windows_flag ;//窗口打开标志 每个界面的窗口模式

extern u8 ioc_key;//图标按键
extern _data_input input_data;
extern _filelistbox_obj * flistbox;
extern _filelistbox_list * filelistx; 	//文件 

extern _btn_obj **screen_key_group;
extern _progressbar_obj* sd_printing_prgb;

extern _graph_obj* temp_graph;

extern float manual_move_length; //手动控制移动的距离
extern u32 manual_move_xy_speed; //手动控制移动的速度(X、Y轴)
extern u32 manual_move_z_speed; //手动控制移动的速度(Z轴)

extern float preheat_e0_length; //挤出机1挤出回抽长度
extern u32 preheat_e0_speed; //挤出机1挤出回抽速度

extern float heater_0_temp ;
extern float bed_temp ;
extern int fanSpeed_temp;

extern u8 Cancel_Flag ;
////HMI
extern u8 send_buf[500];
extern const u8 HMI_end[];
/***********************************
	功能：主界面：各种状态显示  
	state： true 	退出当前界面
					false 显示当前界面
  返回： 无
************************************/
void home_screen_320_480(bool state)	//主页
{
	u8 i;
	u8 selx=0XFF;
//  u32 time;
	char TempBuffer[32];
	u8 key_num=4;//4个按键	
	if(redraw_screen==true)			//重画界面
	{
			redraw_screen=false;			//关闭重画
			display_next_update_millis=0;
		//绘制背景		
			gui_fill_rectangle(0,0,lcddev.width,lcddev.height,GROUNDCOL );//填充背景色
		
		//绘制背景	
			screen_key_group =(_btn_obj **)gui_memin_malloc(sizeof(_btn_obj *)*key_num);	//申请一组按钮
			if(screen_key_group)
			{		
					for(i=0;i<2;i++)
					{	
						screen_key_group[i]=btn_creat(10+(i%2)*155,40+(i%2)*140,305,130,i,BTN_TYPE_PIC);											
					 	screen_key_group[i]->caption=(u8*)MAINMENU_ICONNAME[gui_phy.language][i];	//图标标题					          					
					  screen_key_group[i]->caption_top=110;	 //文字在按钮的偏移位置
						screen_key_group[i]->caption_left=0;  //文字在按钮的偏移位置
					}
           
					for(i=2;i<key_num;i++)
					{	
						screen_key_group[i]=btn_creat(325-(i%2)*315,40+(i%2)*140,145,130,i,BTN_TYPE_PIC);
						screen_key_group[i]->caption_top=110;	 //文字在按钮的偏移位置
						screen_key_group[i]->caption_left=5;  //文字在按钮的偏移位置						
						screen_key_group[i]->caption=(u8*)MAINMENU_ICONNAME[gui_phy.language][i];	//图标标题												
					}
						screen_key_group[0]->picbtnpathu=UNLOAD0;
						screen_key_group[1]->picbtnpathu=PRINT0;					
						screen_key_group[2]->picbtnpathu=APPCATION0;						  
				  	screen_key_group[3]->picbtnpathu=SETTING0;
						
					
						screen_key_group[0]->picbtnpathd=UNLOAD1;
						screen_key_group[1]->picbtnpathd=PRINT1;					
						screen_key_group[2]->picbtnpathd=APPCATION1;	
						screen_key_group[3]->picbtnpathd=SETTING1;
					
							
				
		  	for(i=0;i<key_num;i++)
				{btn_draw(screen_key_group[i]);
				}
			}	
		}
	if(state)   //正常显示
	{	
		gui_phy.back_color=GROUNDCOL;
		if(display_next_update_millis<millis())//实时扫描显示带变化的信息
			{
				display_next_update_millis = millis() + 500;//  
               
				gui_draw_single_color_icos(160,10,20,20,SYSTEM_HEATER1,WHITE,GROUNDCOL);
				sprintf(TempBuffer, "%3.0f/%-3.0f",degHotend(0),degTargetHotend(0));
				gui_show_strmid(180,10,100,20,WHITE,16,(u8 *)TempBuffer,0);      
				//显示打印头1的当前温度/目标温度   0为不叠加
			#if EXTRUDERS > 1
				 if(temp_sensor_num==2)
				 {
					gui_draw_single_color_icos(280,10,20,20,SYSTEM_HEATER2,WHITE,GROUNDCOL);	//显示打印头2小图标	
					sprintf(TempBuffer, "%3.0f/%-3.0f",degHotend(1),degTargetHotend(1));
					gui_show_strmid(320,10,100,20,WHITE,16,(u8 *)TempBuffer,0);
				 }	   
			#endif
		    gui_draw_single_color_icos(20,10,20,20,SYSTEM_BED,WHITE,GROUNDCOL);
				sprintf(TempBuffer, "%3.0f/%-3.0f",degBed(),degTargetBed());
				gui_show_strmid(40,10,100,20,WHITE,16,(u8 *)TempBuffer,0);             //显示热床的当前温度/目标温度														
		 }	
			  gui_phy.back_color=WHITE;
     		
      	 
				if(card.cardOK==true)
	      gui_draw_single_color_icos(440,10,20,20,SYSTEM_SD,WHITE,GROUNDCOL);
		    else
		   {
		    gui_fill_rectangle(440,10,20,20,GROUNDCOL);
		   }

			selx=screen_key_chk(screen_key_group,&in_obj,key_num);   //按键判断
			if(selx&(1<<6))//按键值有效 并且相应
			{
				switch(selx & ~(3<<6))
					{
				    case 0: nextMenu = preheat_head_1_screen;
										break;
						case 1: nextMenu = gecodelist_screen;
										break;
						case 2:	nextMenu = tool_screen;
										break;
						case 3:	nextMenu = settinglist_screen;
						        break;							
						default: break;
					}	
	     }
  }
	else				//释放界面内存
	{
		for(i=0;i<key_num;i++)
		{btn_delete(screen_key_group[i]);
		}
	
		gui_memin_free(screen_key_group);//释放内存	
		display_next_update_millis=0;		
	}

}
/************************************
	功能：打印中的SD卡界面 ：弹出的取消和暂停窗口
	state： true 	退出当前界面
					false 显示当前界面
  返回： 无
***********************************/
void sdprint_screen_320_480(bool state)  //工具
{  
   u8 selx=0XFF;
	 static u8 S=0;	 
	 u32 time=0;
	  u8 i;
    u8 key_num=3;
    char TempBuffer[32];
	if(redraw_screen==true)				//重画界面
	{ 
		redraw_screen=false;			//关闭重画
		//背景
		 
		 LCD_Fill(0,0,lcddev.width,40,GROUNDCOL);
		 //LCD_Fill(0,40,lcddev.width,lcddev.height-40,LIGHTGRAY);
		 LCD_Fill(0,40,lcddev.width,lcddev.height-40,GROUNDCOL1);
	   sd_printing_prgb=progressbar_creat(165,120,300,20,0,0X61);	//进度条	  			
			screen_key_group =(_btn_obj **)gui_memin_malloc(sizeof(_btn_obj *)*key_num);
			if(sd_printing_prgb)
			{	
				  sd_printing_prgb->inbkcolora=BLUECOL;
					sd_printing_prgb->inbkcolorb=BLUECOL;
					sd_printing_prgb->infcolora=REDCOL;
					sd_printing_prgb->infcolorb=REDCOL;
				  sd_printing_prgb->btncolor=WHITE;//百分比颜色
					
					progressbar_draw_progressbar(sd_printing_prgb);	
			}
		if(screen_key_group)
		{	
				 for(i=0;i<key_num;i++)//
				{
				 screen_key_group[i]=btn_creat(11+(i%3)*156,250,145,60,i,BTN_TYPE_PIC);	    	
				}
			//更多
				screen_key_group[0]->picbtnpathu=PRINT_MORE0;
				screen_key_group[0]->picbtnpathd=PRINT_MORE1;
        //暂停继续				
        if(card.sdprinting)
				{ 
					screen_key_group[1]->picbtnpathu=PRINT_PAUSE;    				
				}
				else
				{  	
				 screen_key_group[1]->picbtnpathu=PRINT_PLAY;	    						
				}
        //停止				
				screen_key_group[2]->picbtnpathu=MOVE_STOP0;
				screen_key_group[2]->picbtnpathd=MOVE_STOP1;
			
														  							
			for(i=0;i<key_num;i++)
			{
				btn_draw(screen_key_group[i]);
			}
		}
	}
	if(state)  //正常显示
	{
		
		 if(display_next_update_millis<millis())
			{		
		   display_next_update_millis = millis() + 500;//  
        gui_phy.back_color=GROUNDCOL;      
				gui_draw_single_color_icos(160,10,20,20,SYSTEM_HEATER1,WHITE,GROUNDCOL);
				sprintf(TempBuffer, "%3.0f/%-3.0f",degHotend(0),degTargetHotend(0));
				gui_show_strmid(180,10,100,20,WHITE,16,(u8 *)TempBuffer,0);      
				//显示打印头1的当前温度/目标温度   0为不叠加
			#if EXTRUDERS > 1
				 if(temp_sensor_num==2)
				 {
					gui_draw_single_color_icos(280,10,20,20,SYSTEM_HEATER2,WHITE,GROUNDCOL);	//显示打印头2小图标	
					sprintf(TempBuffer, "%3.0f/%-3.0f",degHotend(1),degTargetHotend(1));
					gui_show_strmid(320,10,100,20,WHITE,16,(u8 *)TempBuffer,0);
				 }	   
			#endif
		    gui_draw_single_color_icos(20,10,20,20,SYSTEM_BED,WHITE,GROUNDCOL);
				sprintf(TempBuffer, "%3.0f/%-3.0f",degBed(),degTargetBed());
				gui_show_strmid(40,10,100,20,WHITE,16,(u8 *)TempBuffer,0);             //显示热床的当前温度/目标温度														
				      		      	 
				if(card.cardOK==true)
				{					            						
	         gui_draw_single_color_icos(440,10,20,20,SYSTEM_SD,WHITE,GROUNDCOL);									  						 						
				}
		    else
		   {			
		    gui_fill_rectangle(440,10,20,20,GROUNDCOL);
		   }	
       gui_phy.back_color=GROUNDCOL1;			 
			 ////Z
			if(current_position[Z_AXIS]>=1000)     
			  sprintf(TempBuffer, "Z:%5.1fcm",current_position[Z_AXIS]/10);
			else 
			{
				if(current_position[Z_AXIS]>=0)//if(count_position[X_AXIS]/axis_steps_per_unit[X_AXIS]>=0)
				   sprintf(TempBuffer, "Z:%5.1fmm",current_position[Z_AXIS]);
			  else
					sprintf(TempBuffer, "Z:%5.1fmm",current_position[Z_AXIS]);
			}
			if(Z_MIN_PIN != Z_ENDSTOPS_INVERTING)//if((X_MIN_PIN != X_ENDSTOPS_INVERTING)||(X_MAX_PIN != X_ENDSTOPS_INVERTING))
		      gui_show_strmid(10,150,140,20,RED,16,(u8*)TempBuffer,0);			  
			else 
			  gui_show_strmid(10,150,140,20,WHITE,16,(u8*)TempBuffer,0);	
										
			 /////////画百分比和进度条			
			 sd_printing_prgb->curpos=f_tell(&card.fgcode);
			 sd_printing_prgb->totallen=f_size(&card.fgcode);
//		   sprintf(TempBuffer, "%.1f%%",(float)f_tell(&card.fgcode)*100/f_size(&card.fgcode));
//			 gui_show_strmid(270,123,40,30,WHITE,16,(u8*)TempBuffer,0);			
			 progressbar_draw_progressbar(sd_printing_prgb);	//画进度条
			  	//打印时间			 			 			   			
					if(starttime)	 //打印中：暂停或者正在打印	  starttime
					{
						time = millis()/1000 - starttime/1000;		
						sprintf(TempBuffer, "%3d:%02d:%02d",time/3600,(time/60)%60,time%60);
						gui_show_strmid(165,100,300,20,WHITE,16,(u8 *)TempBuffer,0);						
					}															       						
//			//打印速度
//			 sprintf(TempBuffer, "%3d%%",feedmultiply);	           			
//			 gui_show_strmid(110,60,100,30,WHITE,16,(u8*)TempBuffer,0);
			//继续暂停
				 if(card.sdprinting)
				{ 
					 gui_draw_single_color_icos(50,80,80,65,PRINTING_0+S*4,WHITE,GROUNDCOL1);	         									  						 
					 S++;
					 if(S>=9)S=0;
					screen_key_group[1]->picbtnpathu=PRINT_PLAY;  									
				}
				else
				{ 				
         gui_draw_single_color_icos(50,80,80,65,PRINTING_0+S*4,ORANGECOL,GROUNDCOL1);						
				 screen_key_group[1]->picbtnpathu=PRINT_PAUSE;	      				
				}
			
				////
				btn_draw(screen_key_group[1]);
												    
			}		
       gui_phy.back_color=WHITE;			
			/////文件名
			gui_show_strmid(165,145,300,20,WHITE,16,card.filename,1);							
		//
		selx=screen_key_chk(screen_key_group,&in_obj,key_num);
		if(selx&(1<<6))//按键值有效 并且相应
		{
			switch(selx & ~(3<<6))//
					{ case 0: nextMenu=sdprintmore_screen;
						        break;
						case 1:	if(card.sdprinting)
									{									
										screen_key_group[1]->picbtnpathu=PRINT_PLAY;																		
										btn_draw(screen_key_group[1]);
										
											if(residencyStart > -1)
											{
													card.sdprinting=false;	
												 if(Machine_type == 2)
													Discard_Buf_and_Block();
												 
												 Abnormal_Flag = 0x01;  													 
												 nextMenu = preheat_head_1_screen;
												 Pause_flag = 0x01;
												 beep_enable();
										  }								
									}
									else
									{ 								
										screen_key_group[1]->picbtnpathu=PRINT_PAUSE;																											
										btn_draw(screen_key_group[1]);

										  if(blocks_queued() ==false)// if (block_buffer_head == block_buffer_tail)
										   Abnormal_Flag = 0x02;	
									}
										break;
           	case 2:	nextMenu = Confirm_cancel_screen;	
                     
										break;						
//						case 4:							
//										if(emc_switch == 0)
//										{
//											emc_switch = 1;
//											gui_show_strmid(122,76,116,16,RED,16,(u8*)TOOLS_MENU[gui_phy.language][1],1);
//											screen_key_group[4]->bkctbl[2] = RED;  //松开时的图标颜色
//											screen_key_group[4]->bkctbl[3] = RED;	//按下时的图标颜色		//20160228
//										}
//										else
//										{
//											emc_switch = 0;
//											gui_show_strmid(122,76,116,16,BLACK,16,(u8*)TOOLS_MENU[gui_phy.language][1],1);
//											screen_key_group[4]->bkctbl[2] = DARKBLUE;  //松开时的图标颜色
//											screen_key_group[4]->bkctbl[3] = DARKBLUE;	//按下时的图标颜色		//20160228
//										}
//										btn_draw(screen_key_group[4]);
//										FLASH_WRITE_VAR(FLASH_SET_STORE_OFFSET+152,emc_switch);                                             //244        bool               1个字节   EMC开关   //20160412 
//										break;
//						case 5:						
//										if(no_material_switch == 0)
//										{
//											 no_material_switch = 1;
//											Material_EXIT_Set(ENABLE);
//											gui_show_strmid(242,76,116,16,RED,16,(u8*)TOOLS_MENU[gui_phy.language][2],1);
//											 screen_key_group[5]->bkctbl[2] = RED;  //松开时的图标颜色
//											 screen_key_group[5]->bkctbl[3] = RED;	//按下时的图标颜色		//20160228	
//										}
//										else
//										{
//											no_material_switch = 0;
//											Material_EXIT_Set(DISABLE);
//											gui_show_strmid(242,76,116,16,BLACK,16,(u8*)TOOLS_MENU[gui_phy.language][2],1);
//											screen_key_group[5]->bkctbl[2] = DARKBLUE;  //松开时的图标颜色
//											screen_key_group[5]->bkctbl[3] = DARKBLUE;	//按下时的图标颜色		//20160228
//										}
//										FLASH_WRITE_VAR(FLASH_SET_STORE_OFFSET+154,no_material_switch); 
//										btn_draw(screen_key_group[5]);
//										break;
//								case 3:			
//								nextMenu = UpdateOK_screen;
//										 break;
//								case 4:	
//								 	
//								   fanSpeed=fanSpeed*100/255;								
//								  enter_data_input(&fanSpeed,FLOAT_TYPE,100,0,false,NULL); 
//								   fanSpeed=fanSpeed*255/100;								
//								 btn_draw(screen_key_group[4]);
//										 break;
//								case 5:			
//								nextMenu = home_screen;
//										 break;
						default: break;
					}
		}
	}
	else   //释放界面内存
	{
		for(i=0;i<key_num;i++)		
			{							
				btn_delete(screen_key_group[i]);
			}
		gui_memin_free(screen_key_group);//释放内存	 
	} 	
}

/************************************
	功能：打印中的SD卡界面 ：弹出的取消和暂停窗口
	state： true 	退出当前界面
					false 显示当前界面
  返回： 无
***********************************/
void sdprintmore_screen_320_480 (bool state) //打印过程中SD卡的界面
{	
		u8 selx=0XFF;

		u8 i;
		u8 key_num=6; //三个按钮
	  // char TempBuffer[32]={0};	
//	  u8 *BACKGROUND = PRINT_PIC0;
		if(redraw_screen==true)				//重画界面
		{ 	
			redraw_screen=false;			//关闭重画		
			//背景
	       gui_fill_rectangle(0,0,lcddev.width,lcddev.height,GROUNDCOL );//填充背景色		     		 
		     gui_show_strmid(0,10,lcddev.width,20,WHITE,16,(u8*)TOOLS_MENU[gui_phy.language][6],1);				
				
			screen_key_group =(_btn_obj **)gui_memin_malloc(sizeof(_btn_obj *)*key_num);		   																
			if(screen_key_group)
			{									
				 for(i=0;i<key_num;i++)//
				{
				 screen_key_group[i]=btn_creat(11+(i%3)*156,40+(i/3)*140,145,130,i,BTN_TYPE_PIC);	
                 //screen_key_group[i]->caption=(u8*)TOOLS_MENU[gui_phy.language][i];
				 screen_key_group[i]->caption_left=5;
				 screen_key_group[i]->caption_top=110;			
				}				
				/////////////	
			screen_key_group[0]->picbtnpathu=APP_BED0;
			screen_key_group[0]->picbtnpathd=APP_BED1;//20160228
			screen_key_group[0]->caption=gui_memin_malloc(20);
			if(screen_key_group[0]->caption)
			   sprintf((char*)screen_key_group[0]->caption, "%3.0f/%-3.0f",degBed(),degTargetBed());
			
            screen_key_group[1]->picbtnpathu=PRINT_HEAD10;
			screen_key_group[1]->picbtnpathd=PRINT_HEAD11;			
			screen_key_group[1]->caption=gui_memin_malloc(20);
			if(screen_key_group[1]->caption)
			  sprintf((char*)screen_key_group[1]->caption, "%3.0f/%-3.0f",degHotend(0),degTargetHotend(0));
			
			screen_key_group[2]->picbtnpathu=PRINT_HEAD20;
			screen_key_group[2]->picbtnpathd=PRINT_HEAD21;			
            screen_key_group[2]->caption=gui_memin_malloc(20);			
			if(screen_key_group[2]->caption)
			{
			  if(temp_sensor_num==2)
			       sprintf((char*)screen_key_group[2]->caption, "%3.0f/%-3.0f",degHotend(1),degTargetHotend(1));
			  else screen_key_group[2]->caption=(u8*)"--/--";
			}
            screen_key_group[3]->picbtnpathu=PRINT_SPEED0;
			screen_key_group[3]->picbtnpathd=PRINT_SPEED1;  
			screen_key_group[3]->caption=gui_memin_malloc(20);
			  if(screen_key_group[3]->caption)
                sprintf((char*)(screen_key_group[3]->caption), "%3d%%",feedmultiply);

				  
					screen_key_group[4]->picbtnpathu=APP_FAN0;
					screen_key_group[4]->picbtnpathd=APP_FAN1;//20160228
					screen_key_group[4]->caption=gui_memin_malloc(20);
					if(screen_key_group[4]->caption)
			   sprintf((char*)(screen_key_group[4]->caption),"%.1f%%",fanSpeed);//fan	        
				
			
		      screen_key_group[5]->picbtnpathu=SET_BACK0;
			  screen_key_group[5]->picbtnpathd=SET_BACK1;//20160228      
				
				for(i=0;i<key_num;i++)
				{	btn_draw(screen_key_group[i]);
				}
			}
		}
		if(state)   //正常显示 
		{						
			selx=screen_key_chk(screen_key_group,&in_obj,key_num);	
			if(selx&(1<<6))//按键值有效 并且相应
			{	
				switch(selx & ~(3<<6))//
						{  
		       	case 0: //Bed
							 if(gui_phy.language==2)	
								 sprintf(showString,"Bed Setting...");
							 else		
							   sprintf(showString,"热床目标温度设置中...");
							   enter_data_input(&target_temperature_bed,INT_TYPE,bed_maxtemp,0,false,NULL);
						        break;
						case 1://Head1	
							  if(gui_phy.language==2)	
							 	 sprintf(showString,"Head Setting...");
							  else	
                    sprintf(showString,"打印头目标温度设置中...");							
						        enter_data_input(&target_temperature[0],INT_TYPE,heater_0_maxtemp,0,false,NULL);						
								break;
           	case 2://Head2	
								  if(temp_sensor_num==2)
								  {	
										 if(gui_phy.language==2)	
								    sprintf(showString,"Head2 Setting...");
							     else	
                   sprintf(showString,"打印头2目标温度设置中...");									  
									 enter_data_input(&target_temperature[1],INT_TYPE,heater_0_maxtemp,0,false,NULL);
								  }
						         break;						
						case 3:	//Speed	
							 if(gui_phy.language==2)	
								 sprintf(showString,"Velocity Setting...");
							 else	
                sprintf(showString,"打印速度设置中...");							
							    enter_data_input(&feedmultiply,U32_TYPE,999,1,false,NULL);
								 break;
						case 4:	//Fan
									 if(gui_phy.language==2)	
										 sprintf(showString,"Fan Setting...");
									 else	
							      sprintf(showString,"风扇速度设置中...");
							       fanSpeed=fanSpeed*100/255;
								 enter_data_input(&fanSpeed,FLOAT_TYPE,100,0,false,NULL); 							
                     fanSpeed =	fanSpeed*255/100;	              						 
								 break;
						case 5:	//Back		
								 nextMenu = sdprint_screen;
								 break;
						
								default: break;
						}
			}
		}
		else  			//释放界面内存  
		{
			for(i=0;i<key_num;i++)			
			{	
        if(i<key_num-1)		    
				gui_memin_free(screen_key_group[i]->caption);
				btn_delete(screen_key_group[i]);
			}
			gui_memin_free(screen_key_group);//释放内存			
		}
}
	
/***********************************
	功能：非打印状态的SD卡界面：进入SD卡中文件列表界面
	state： true 	退出当前界面
					false 显示当前界面
  返回： 无
************************************/
void gecodelist_screen_320_480(bool state) ///非打印过程中SD卡的界面
 { 
	u8 selx=0XFF;
	u8 i;	
	u8 *pname=0;
    u8 key_num=3;
	//弹窗大小
	u16 width =lcddev.width/2;
	u16 height=lcddev.height/2;	
	u16 left = lcddev.width/4;
	u16 top = lcddev.height/4;
	u16 btnwidth=(width-20)/2;
	u16 btnheight=(btnwidth-10)/2;
	 
 	_filelistbox_list * filelisttemp;	
   
 	if(windows_flag==false)  //非弹窗
	{	 
		if(redraw_screen==true)				//重画界面
		{ 
			redraw_screen=false;			//关闭重画	

			//画背景	
			gui_fill_rectangle(0,0,lcddev.width,lcddev.height,WHITE );//填充背景色
			gui_fill_rectangle(0,0,lcddev.width,40,GREENCOL);  //填充区域1	
		
			gui_show_strmid(0,0,lcddev.width,40,WHITE,16,(u8*)HOME_ICONNAME[gui_phy.language][5],1);//显示标题	
			//画背景							
			screen_key_group =(_btn_obj **)gui_memin_malloc(sizeof(_btn_obj *)*1);	//申请一组按钮	
			flistbox=filelistbox_creat(1,40,lcddev.width-2,FLBOX_ITEM_HEIGHT*7,1,16);	//创建一个filelistbox
			if(screen_key_group&&flistbox)
				{	
//						screen_key_group[0]=btn_creat(5,5,40,40,0,BTN_TYPE_PIC);//带文字的图标											
//						screen_key_group[0]->picbtnpathu=FILE_BACK0;	 	//松开时的图标
//						screen_key_group[0]->picbtnpathd=FILE_BACK1;		//按下时的图标
            screen_key_group[0]=btn_creat(5,5,30,30,0,BTN_TYPE_SGICON_TEXT);//带文字的图标                      								
						screen_key_group[0]->picbtnpathu=SYSTEM_BACK;	 	//松开时的图标
						screen_key_group[0]->picbtnpathd=SYSTEM_BACK;		//按下时的图标
				    screen_key_group[0]->bkctbl[0]=GREENCOL;//松开时背景色
				    screen_key_group[0]->bkctbl[1]=GREENCOL;//按下时背景色
				    screen_key_group[0]->bkctbl[2]=WHITE;//松开时图标颜色
				    screen_key_group[0]->bkctbl[3]=GROUNDCOL;//按下时图标颜色
					
						flistbox->fliter=FLBOX_FLT_GCODE;	//查找GCODE文件
						flistbox->type=0;
						//flistbox->path=(u8*)"0:\\GCODE";
					  flistbox->path=(u8*)"0:";
						filelistbox_scan_filelist(flistbox);	//重新扫描列表
						filelistbox_draw_listbox(flistbox);		
						
						for(i=0;i<1;i++)
							{	btn_draw(screen_key_group[i]);
							}
				}
		}
		if(state)//正常显示
		{
			selx=filelistbox_check(flistbox,&in_obj);//扫描文件	 双击的动作 0x01 进入下一个目录 0x02 返回上一级目录 0x3 目标文件  NULL 无双击动作			
			selx=screen_key_chk(screen_key_group,&in_obj,1);
			
			if(selx&(1<<6))//按键值有效 并且相应  //点击主页
			{	
				switch(selx & ~(3<<6))//返回主页
						{   case 0: 
                     if(filelistbox_get_pathdepth(flistbox->path)>0)							  
											  filelistbox_back(flistbox);
							       else
							         nextMenu = home_screen;
													
									   break;			   
								default: break;
						}
			}
			else  //列表相应
			{
				if(flistbox->dbclick==0X81)//双击文件了
					{
						flistbox->dbclick=0x00;//标记已处理  
						redraw_screen=true;
						windows_flag=true; //进入窗口模式
						
//						filelisttemp=filelist_search(flistbox->list,flistbox->selindex);//得到选中的list的详细信息
//						if((Read_selindex == flistbox->selindex)&&(fgcode_fptr_flag == 0x01))
//							continue_flag = 0x01;
//						else
//							continue_flag = 0x00;
							
					}
			}
		}
		else //释放界面内存
		{
				for(i=0;i<1;i++)
				{	btn_delete(screen_key_group[i]);
				}
				gui_memin_free(screen_key_group);//释放内存	 
				filelistbox_delete(flistbox);	//删除filelist
		}
	}
	else //弹窗
	{	
			if(redraw_screen==true)				//重画界面
			{ 
				redraw_screen=false;			//关闭重画
				//画背景
				gui_draw_arcrectangle(left,top,width,height,4,1,GREENCOL,GREENCOL);//				
				gui_draw_arcrectangle(left+2,top+lcddev.height/8,width-4,lcddev.height*3/8-2,4,1,WHITE,WHITE);
				
				gui_show_strmid(left,top+5,width,20,WHITE,16,(u8*)CONTINUE_PRINT_LABE[gui_phy.language][2],1);//显示标题
				filelisttemp=filelist_search(flistbox->list,flistbox->selindex);//得到选中的list的详细信息
				gui_show_strmid(left,top+5+lcddev.height/8,width,20,GREENCOL,12,filelisttemp->name,1);//显示文件
															
				if(screen_key_group)
					{
                     for(i=1;i<key_num;i++)
					 {	
						 screen_key_group[i]=btn_creat(left+5+(i-1)*(btnwidth+10),top+height-5-btnheight,btnwidth,btnheight,i,BTN_TYPE_TEXTB);//							
					     screen_key_group[i]->bkctbl[0]=REDCOL;
						 screen_key_group[i]->bkctbl[1]=ORANGECOL;
					  }
					 screen_key_group[1]->caption=(u8*)GUI_OK_CAPTION_TBL[gui_phy.language];//确定
					 screen_key_group[2]->caption=(u8*)GUI_CANCEL_CAPTION_TBL[gui_phy.language];//取消					
						for(i=1;i<key_num;i++)
						{	btn_draw(screen_key_group[i]);
						}
					}
			}	
			if(state)  //正常显示
			{
				selx=screen_key_chk(screen_key_group,&in_obj,3);
				if(selx&(1<<6))//按键值有效 并且相应
				{	
					switch(selx & ~(3<<6))//
 							{   case 1:	windows_flag=false; //退出窗口模式 
								          filelisttemp=filelist_search(flistbox->list,flistbox->selindex);//得到选中的list的详细信息
													strcpy((char *)card.filename,(const char*)filelisttemp->name);//复制保存文件名到card供后面显示
													Store_selindex = flistbox->selindex;//20160508          
								
													pname=gui_memin_malloc(strlen((const char*)filelisttemp->name)+strlen((const char*)flistbox->path)+2);//申请内存
													if(pname)  
													{	
														pname=gui_path_name(pname,flistbox->path,filelisttemp->name);	//文件名加入路径 					
														selx=f_open(&card.fgcode,(const TCHAR*)pname,FA_READ);	//只读方式打开文件
														gui_memin_free(pname);			//释放内存	
														if (selx==FR_OK)//打开成功. 
														{
															nextMenu = sdprint_screen;										
															card.sdprinting = true;
															quickStop();				//20160412									
															starttime=millis();
															feedmultiply = 100;															
															   {
											            strcpy((char*)send_buf,"page main");	
	                                strcat((char*)send_buf,(const char*)HMI_end);
	                                u2_printf("%s",send_buf);
												          }
//															USART_ITConfig(USART3, USART_IT_RXNE, DISABLE);////20170213-8266
//															TIM_ITConfig(TIM7,TIM_IT_Update,DISABLE );//TIM_Cmd(TIM7, DISABLE);
														}
													}
										break;
									case 2:		windows_flag=false; //退出窗口模式 
													for(i=1;i<key_num;i++)
													{	
														btn_delete(screen_key_group[i]);
													}
													filelistbox_draw_listbox(flistbox);
										break;						
									default: break;
							}
					}
			}
			else   //释放界面内存
			{
				windows_flag=false;	
				for(i=0;i<key_num;i++)
				{	btn_delete(screen_key_group[i]);
				}
				gui_memin_free(screen_key_group);//释放内存	 
				filelistbox_delete(flistbox);	//删除filelist	
			}		 
	}

}


/***********************************
	功能：手动控制
  返回： 无
************************************/
void manual_screen_320_480(bool state)
{ 
	u8 selx=0XFF;
	u8 i;
	u8 key_num=10;	
	char TempBuffer[32];
	static u8 flag_x=0,flag_y=0,flag_z=0;
//	float data_temp;
	if(redraw_screen==true)				//重画界面
		{ 
			redraw_screen=false;			//关闭重画	
			//画背景	
			gui_fill_rectangle(0,0,lcddev.width,lcddev.height,GROUNDCOL );//填充背景色
			gui_show_strmid(0,10,lcddev.width,20,WHITE,16,(u8*)TOOLS_MENU[gui_phy.language][1],1);	
			
			screen_key_group =(_btn_obj **)gui_memin_malloc(sizeof(_btn_obj *)*key_num);	//申请一组按钮		
			if(screen_key_group)
				{
						 for(i=0;i<4;i++)//X,Y,Z,归零
						 {
					   screen_key_group[i]=btn_creat(11,(i%4)*70+40,145,60,i,BTN_TYPE_PIC);			
						 }						
						 for(i=4;i<6;i++) //显示长度,速度
						 {
						  screen_key_group[i]=btn_creat(167+(i-4)*156,40,145,60,i,BTN_TYPE_PIC);						
						 }												 
						 for(i=6;i<8;i++)//左移,右移
						 {
						  screen_key_group[i]=btn_creat(167+(i-6)*156,110,145,130,i,BTN_TYPE_PIC);						 	
						 }						 
						 for(i=8;i<key_num;i++)//急停,返回
						 {
						 screen_key_group[i]=btn_creat(167+(i-8)*156,250,145,60,i,BTN_TYPE_PIC);
						
						 }
						 if(flag_x==0)
						 screen_key_group[0]->picbtnpathu=MOVE_X0;
						 else
						 screen_key_group[0]->picbtnpathu=MOVE_X1;
						 if(flag_y==0)
						 screen_key_group[1]->picbtnpathu=MOVE_Y0;
						 else
						 screen_key_group[1]->picbtnpathu=MOVE_Y1;
						 if(flag_z==0)
						 screen_key_group[2]->picbtnpathu=MOVE_Z0;
						 else
						 screen_key_group[2]->picbtnpathu=MOVE_Z1;
						 
						 screen_key_group[3]->picbtnpathu=HOMEZERO0;
						 screen_key_group[3]->picbtnpathd=HOMEZERO1;
						 
						 screen_key_group[4]->picbtnpathu=DIS0_BLUE;
						 screen_key_group[4]->picbtnpathd=DIS0_BLUE;
						 
						 screen_key_group[5]->picbtnpathu=DIS0_BLUE;
						 screen_key_group[5]->picbtnpathd=DIS0_BLUE;
						 						 
						 screen_key_group[4]->caption=gui_memin_malloc(sizeof(u8)*8);
				     sprintf((char*)screen_key_group[4]->caption,"%3.1fmm",manual_move_length);//移动速度XY
						 screen_key_group[5]->caption=gui_memin_malloc(sizeof(u8)*8);
				     sprintf((char*)screen_key_group[5]->caption,"%3dmm/s",manual_move_xy_speed);//移动速度XY	
						 
						 screen_key_group[6]->picbtnpathu=MOVE_L0;
				  	 screen_key_group[6]->picbtnpathd=MOVE_L1;	
						 
					  screen_key_group[7]->picbtnpathu=MOVE_R0;
						 screen_key_group[7]->picbtnpathd=MOVE_R1;
						 
						 screen_key_group[8]->picbtnpathu=MOVE_STOP0;
						 screen_key_group[8]->picbtnpathd=MOVE_STOP1;
						 
						 screen_key_group[9]->picbtnpathu=HEAT_BACK0;
						 screen_key_group[9]->picbtnpathd=HEAT_BACK1;
						 
						for(i=0;i<key_num;i++)
						{
							btn_draw(screen_key_group[i]);
						}

				}					
		}	//重画界面结束		
				
	if(state)  //正常显示
	{
		selx=screen_key_chk(screen_key_group,&in_obj,key_num);
		if(selx&(1<<6))//按键值有效 并且相应
		{	
//			if((selx & ~(3<<6))==0) nextMenu = home_screen;//返回主页
		 if(card.sdprinting == false)
			{
			  switch(selx & ~(3<<6))//
					{   case 	0: 
						           flag_y=0;flag_z=0;
						           flag_x=(~flag_x)&0x01;
						           if(flag_x==0)
												screen_key_group[0]->picbtnpathu=MOVE_X0;
											 else
											 screen_key_group[0]->picbtnpathu=MOVE_X1;
											 if(flag_y==0)
											 screen_key_group[1]->picbtnpathu=MOVE_Y0;
											 else
											 screen_key_group[1]->picbtnpathu=MOVE_Y1;
											 if(flag_z==0)
											 screen_key_group[2]->picbtnpathu=MOVE_Z0;
											 else
											 screen_key_group[2]->picbtnpathu=MOVE_Z1;
											 for(i=0;i<3;i++)
						            btn_draw(screen_key_group[i]);
						             break;//
						  case 	1: 
						           flag_x=0;flag_z=0;
						           flag_y=(~flag_y)&0x01;
						           if(flag_x==0)
												screen_key_group[0]->picbtnpathu=MOVE_X0;
											 else
											 screen_key_group[0]->picbtnpathu=MOVE_X1;
											 if(flag_y==0)
											 screen_key_group[1]->picbtnpathu=MOVE_Y0;
											 else
											 screen_key_group[1]->picbtnpathu=MOVE_Y1;
											 if(flag_z==0)
											 screen_key_group[2]->picbtnpathu=MOVE_Z0;
											 else
											 screen_key_group[2]->picbtnpathu=MOVE_Z1;
											 for(i=0;i<3;i++)
						            btn_draw(screen_key_group[i]);
						             break;//					 
							 case 2: 
						           flag_y=0;flag_x=0;
						           flag_z=(~flag_z)&0x01;
						           if(flag_x==0)
												screen_key_group[0]->picbtnpathu=MOVE_X0;
											 else
											 screen_key_group[0]->picbtnpathu=MOVE_X1;
											 if(flag_y==0)
											 screen_key_group[1]->picbtnpathu=MOVE_Y0;
											 else
											 screen_key_group[1]->picbtnpathu=MOVE_Y1;
											 if(flag_z==0)
											 screen_key_group[2]->picbtnpathu=MOVE_Z0;
											 else
											 screen_key_group[2]->picbtnpathu=MOVE_Z1;
											 for(i=0;i<3;i++)
						            btn_draw(screen_key_group[i]);
						             break;//				 
							case 3:if(flag_x+flag_y+flag_z==0)//三轴归零		
							       {
										 	menu_action_gcode("G28") ;
										 }else if(flag_x+flag_y+flag_z==1)
                                       {
											 if(flag_x==1) menu_action_gcode("G28 X0") ;//X轴归零
											 if(flag_y==1) menu_action_gcode("G28 Y0") ;//Y轴归零
											 if(flag_z==1) menu_action_gcode("G28 Z0") ;//Z轴归零											
											}						           
											 break;
                      case 4: 
				                if(gui_phy.language==2)	
												 sprintf(showString,"Length Setting...");
											 else													
						          sprintf(showString,"移动距离设置中...");
						        enter_data_input(&manual_move_length,FLOAT_TYPE,min(x_max_pos-min_pos[0], min(y_max_pos-min_pos[1], z_max_pos-min_pos[2])),0.1,false,NULL);//3轴最大值中最小的那个的一半//20160404
											 break;//移动距离输入			
							case 5: 
													if(gui_phy.language==2)	
													 sprintf(showString,"Move_Speed Setting...");
												 else	
                            sprintf(showString,"移动速度设置中...");								
          						enter_data_input(&manual_move_xy_speed,U32_TYPE,min(max_feedrate[0],max_feedrate[1]),1,false,NULL);//X,Y轴中最大速度中最小的那个
											manual_move_z_speed=manual_move_xy_speed/5;
							        break;//XY速度输入
							case 6:  if(flag_x+flag_y+flag_z==1)
							        {
											  if(flag_x==1)
											  {
												menu_action_gcode("G91");
											  sprintf(TempBuffer, "G1 X-%.1f F%d",manual_move_length,manual_move_xy_speed*60);
											  menu_action_gcode(TempBuffer);
											  menu_action_gcode("G90");
												}else if(flag_y==1)
												{
												menu_action_gcode("G91");
											  sprintf(TempBuffer, "G1 Y-%.1f F%d",manual_move_length,manual_move_xy_speed*60);
											  menu_action_gcode(TempBuffer);
											  menu_action_gcode("G90");												
												}else
												{
												menu_action_gcode("G91");
											  sprintf(TempBuffer, "G1 Z-%.1f F%d",manual_move_length,manual_move_z_speed*60);
											  menu_action_gcode(TempBuffer);
											  menu_action_gcode("G90");
												}											
											}                     							        
							        break;
              case 7:  if(flag_x+flag_y+flag_z==1)
							        {
											  if(flag_x==1)
											  {
												menu_action_gcode("G91");
											  sprintf(TempBuffer, "G1 X%.1f F%d",manual_move_length,manual_move_xy_speed*60);
											  menu_action_gcode(TempBuffer);
											  menu_action_gcode("G90");
												}else if(flag_y==1)
												{
												menu_action_gcode("G91");
											  sprintf(TempBuffer, "G1 Y%.1f F%d",manual_move_length,manual_move_xy_speed*60);
											  menu_action_gcode(TempBuffer);
											  menu_action_gcode("G90");												
												}else
												{
												menu_action_gcode("G91");
											  sprintf(TempBuffer, "G1 Z%.1f F%d",manual_move_length,manual_move_z_speed*60);
											  menu_action_gcode(TempBuffer);
											  menu_action_gcode("G90");
												}											
											}                     							        
							        break;							
 							case 	8: stop_brake_time=millis();break;	//20160409	急停按钮，将所有Block和当前block都清空//Discard_Buf_and_Block();//7.清除所有cmdbuffer和Block	
							case  9: nextMenu = tool_screen; break;
//							case 	0: menu_action_gcode("G28 Y0") ;break;//X轴归零
//							case 15: menu_action_gcode("G28 X0") ;break;//Y轴归零
//							case 12: menu_action_gcode("G28 Z0") ;break;//Z轴归零
//							case 10: menu_action_gcode("G91");
//											 sprintf(TempBuffer, "G1 X%.1f F%d",manual_move_length,manual_move_xy_speed*60);
//											 menu_action_gcode(TempBuffer);
//											 menu_action_gcode("G90");
//											 break;//X正方向移动
//						case 	8: menu_action_gcode("G91");
//											 sprintf(TempBuffer, "G1 X-%.1f F%d",manual_move_length,manual_move_xy_speed*60);
//											 menu_action_gcode(TempBuffer);
//											 menu_action_gcode("G90");
//											 break;//X负方向移动						
//							case 	4: menu_action_gcode("G91");
//											 sprintf(TempBuffer, "G1 Y%.1f F%d",manual_move_length,manual_move_xy_speed*60);
//											 menu_action_gcode(TempBuffer);
//											 menu_action_gcode("G90");
//											 break;//Y正方向移动
//							case 14: menu_action_gcode("G91");
//											 sprintf(TempBuffer, "G1 Y-%.1f F%d",manual_move_length,manual_move_xy_speed*60);
//											 menu_action_gcode(TempBuffer);
//											 menu_action_gcode("G90");
//											 break;//Y负方向移动
//							case  1: menu_action_gcode("G91");
//											 sprintf(TempBuffer, "G1 Z%.1f F%d",manual_move_length,manual_move_z_speed*60);
//											 menu_action_gcode(TempBuffer);
//											 menu_action_gcode("G90");
//											 break;//Z方向移动	
//							case 11: menu_action_gcode("G91");
//											 sprintf(TempBuffer, "G1 Z-%.1f F%d",manual_move_length,manual_move_z_speed*60);
//											 menu_action_gcode(TempBuffer);
//											 menu_action_gcode("G90");
//											 break;//Z负方向移动	
//						
//							case 6:  enter_data_input(&manual_move_z_speed,U32_TYPE,max_feedrate[2],1,false,NULL);
//											 break;//Z速度输入						
					   	default: break;
					}
				}
		}
	}
	else       //释放界面内存
	{		
			for(i=0;i<key_num;i++)
				{ if(i==4||i==5)
					{	gui_memin_free(screen_key_group[i]->caption);//释放按钮标题内存
					}
					btn_delete(screen_key_group[i]);
				}
			gui_memin_free(screen_key_group);//释放内存	 
		} 
	
}

/***********************************
	功能：进入打印头1预热
  返回： 无
************************************/
void preheat_head_1_screen_320_480(bool state)
{ 
	u8 selx=0XFF;
	u8 i;
	u8 key_num=6;//9个按钮
//	u32 temp;
	char TempBuffer[32];
	static bool switch_status=false;
//	int temp;
	if(redraw_screen==true)				//重画界面
		{ 
			redraw_screen=false;			//关闭重画	
			display_next_update_millis=0;			
			//画背景	
			gui_fill_rectangle(0,0,lcddev.width,lcddev.height,GROUNDCOL);//填充背景色LIGHTGRAY
		  gui_fill_rectangle(11,40,301,200,WHITE);

//			gui_show_strmid(0,0,lcddev.width,50,WHITE,16,(u8*)TOOL_ICONNAME[gui_phy.language][1],1);//显示标题	
//      if(tmp_extruder==0)
			 gui_show_strmid(0,0,lcddev.width-150,40,WHITE,16,(u8*)TOOL_ICONNAME[gui_phy.language][1],1);//显示标题		
//			else if(tmp_extruder==1)
//			 gui_show_strmid(0,0,lcddev.width-150,40,WHITE,16,(u8*)TOOL_ICONNAME[gui_phy.language][1],1);//显示标题	
						
			
			screen_key_group =(_btn_obj **)gui_memin_malloc(sizeof(_btn_obj *)*key_num);	//申请一组按钮	
			temp_graph=graph_creat(50,60,240,150,20,6);		//创建曲线图	
			if(screen_key_group&&temp_graph)                                 
				{											
					  for(i=0;i<4;i++)//回抽,打印头,挤出,返回
						{
						 screen_key_group[i]=btn_creat(323,40+(i%4)*70,145,60,i,BTN_TYPE_PIC);
						
						}
					 for(i=4;i<6;i++)//开关和显示温度按钮
						{
						 screen_key_group[i]=btn_creat(11+(i%2)*155,250,145,60,i,BTN_TYPE_PIC);
						
						}
						
						//回抽按钮														
						screen_key_group[0]->picbtnpathu=E_UP0;
						screen_key_group[0]->picbtnpathd=E_UP1;							
						//T0 or T1
						if(active_extruder==0) 
						{							
						 screen_key_group[1]->caption=(u8*)"T0";										 														
					    }else 
						{
						screen_key_group[1]->caption=(u8*)"T1";
						}
						screen_key_group[1]->picbtnpathu=DIS1_GREEN;
						screen_key_group[1]->picbtnpathd=DIS1_GREEN;						
						//挤出按钮						
						screen_key_group[2]->picbtnpathu=E_DOWN0;
						screen_key_group[2]->picbtnpathd=E_DOWN1;			
           //返回按钮						
						screen_key_group[3]->picbtnpathu=HEAT_BACK0;
						screen_key_group[3]->picbtnpathd=HEAT_BACK1;
						
						
						
						if((switch_status == 1)&&(target_temperature[tmp_extruder] != heater_temp[tmp_extruder]))//20160403
							{
								target_temperature[tmp_extruder] = heater_temp[tmp_extruder];
								sprintf(TempBuffer, "M104 S%.1f",heater_temp[tmp_extruder]);
								menu_action_gcode(TempBuffer);
							}
							
							if(target_temperature[tmp_extruder]>0)//if(switch_status==0 && !Print_all_process)//20160402
							{
									 switch_status = 1;
									 heater_temp[tmp_extruder] = target_temperature[tmp_extruder];
									if(card.sdprinting == true)//打印过程中温度可调，但是这个开关按钮不能随便按
									{
										 screen_key_group[4]->picbtnpathu=KEY_ON0;
										 screen_key_group[4]->picbtnpathd=KEY_ON0;
									}
									else
									{
										 screen_key_group[4]->picbtnpathu=KEY_ON0;
										 screen_key_group[4]->picbtnpathd=KEY_ON0;
									}
							}
							else
							{
								switch_status = 0;
								screen_key_group[4]->picbtnpathu=KEY_OFF0;
								screen_key_group[4]->picbtnpathd=KEY_OFF0;
							}
						
						
						
						
						
						//温度显示按钮						
						screen_key_group[5]->picbtnpathu=DIS3_ORANGE;
						screen_key_group[5]->picbtnpathd=DIS3_ORANGE;
						screen_key_group[5]->caption =gui_memin_malloc(12);
						if(screen_key_group[5]->caption)
							sprintf((char*)(screen_key_group[5]->caption), "%4.1f/%-4.1f℃",degHotend(tmp_extruder),heater_temp[tmp_extruder]);
									
//						if((switch_status == 1)&&(target_temperature[tmp_extruder] != heater_temp[tmp_extruder]))//20160403
//						{
////							target_temperature[tmp_extruder] = heater_temp[tmp_extruder];
//							sprintf(TempBuffer, "M104 S%.1f",heater_temp[tmp_extruder]);
//							menu_action_gcode(TempBuffer);
//						}
					
																					
						////////
						temp_graph->data_0_temp=degHotend(0);
						temp_graph->data_1_temp=heater_temp[tmp_extruder];
						graph_draw(temp_graph); //绘制曲线
						
						gui_show_strmid(temp_graph->left-10,temp_graph->top+temp_graph->height+2,90,20,temp_graph->data_0_color,16,(u8*)HEAD_1_ICONNAME[gui_phy.language][0],1);
						gui_show_strmid(temp_graph->left+100,temp_graph->top+temp_graph->height+2,90,20,temp_graph->data_1_color,16,(u8*)HEAD_1_ICONNAME[gui_phy.language][1],1);
						gui_show_strmid(temp_graph->left+230,temp_graph->top+temp_graph->height+2,40,20,BLACK,16,"1S",1);

//						gui_fill_rectangle(screen_key_group[4]->left,screen_key_group[4]->top+20,50,30,LIGHTBLUE);
//						gui_show_strmid(screen_key_group[4]->left,screen_key_group[4]->top+30,50,20,WHITE,16,(u8*)UNIT_CAPTION_TBL[0],1);

//						gui_fill_rectangle(screen_key_group[3]->left,screen_key_group[3]->top+20,50,30,LIGHTBLUE);
//						gui_show_strmid(screen_key_group[3]->left,screen_key_group[3]->top+30,50,20,WHITE,16,(u8*)UNIT_CAPTION_TBL[1],1);
//	

						for(i=0;i<key_num;i++)
						{
							btn_draw(screen_key_group[i]);
						}
				}
		
		}		//重画界面		
		
	if(state)   //正常显示
	{
		if(display_next_update_millis<millis())
		{	 display_next_update_millis = millis() + 1000;//
			 temp_graph->data_0_temp=degHotend(tmp_extruder);//当前温度
			 temp_graph->data_1_temp=heater_temp[tmp_extruder]; //注意，此处并不是目标温度，只是中间转换的温度
			 graph_draw(temp_graph); //绘制曲线
//			 gui_phy.back_color=LIGHTGRAY;	//修改背景色
			 gui_show_strmid(30,temp_graph->top+temp_graph->height-8-(temp_graph->height/temp_graph->ordinate_unit_num)*6,20,20,BLACK,16,"℃",0);
			 for(i=0;i<temp_graph->ordinate_unit_num;i++)
				{
					sprintf(TempBuffer,"%4d",temp_graph->ordinate_start_value+ORDINZTEE_UNIT_TABL[temp_graph->ordinate_unit_value]*i);
					gui_show_strmid(15,temp_graph->top+temp_graph->height-10-(temp_graph->height/temp_graph->ordinate_unit_num)*i,35,20,BLACK,16,(u8 *)TempBuffer,0);
				}
//				gui_phy.back_color=BACK_COLOR;		//恢复背景色
				
				sprintf((char*)(screen_key_group[5]->caption), "%4.1f/%-4.0f℃",degHotend(tmp_extruder),heater_temp[tmp_extruder]);
				btn_draw(screen_key_group[5]);//更新温度按钮
				//btn_draw(screen_key_group[4]);
				
//				if(active_extruder)
//				 gui_show_strmid(0,0,lcddev.width-150,40,RED,16,(u8*)TOOL_ICONNAME[gui_phy.language][1],1);
//				else
//					gui_show_strmid(0,0,lcddev.width-150,40,WHITE,16,(u8*)TOOL_ICONNAME[gui_phy.language][1],1);
		}		 
		selx=screen_key_chk(screen_key_group,&in_obj,key_num);
		if(selx&(1<<6))//按键值有效 并且相应
		{	
			  switch(selx & ~(3<<6))//
					{   case 3:  
												if(Pause_flag == 0x00)
												  nextMenu = home_screen;
												else if(Pause_flag == 0x01)
													nextMenu = Material_ok_screen;
											 	break;	
							case 0:  if(card.sdprinting == false)
												{         
												 menu_action_gcode("G91");     //回抽按钮
												 sprintf(TempBuffer, "G1 E-%.1f F%d",preheat_e0_length,preheat_e0_speed*60);
												 menu_action_gcode(TempBuffer);
												 menu_action_gcode("G90");
												}
											 break;
							case 2:  if(card.sdprinting == false)
												{
												 menu_action_gcode("G91");     //挤出按钮											
												 sprintf(TempBuffer, "G1 E%.1f F%d",preheat_e0_length,preheat_e0_speed*60);
												 menu_action_gcode(TempBuffer);
												 menu_action_gcode("G90");
												}
											 break;
							case 1:  
								
							         if(active_extruder==1)
											 {					
												 screen_key_group[1]->caption=(u8*)"T0";
												 active_extruder=0;													 
											 }else
											 {
											  screen_key_group[1]->caption=(u8*)"T1";	
												active_extruder=1;												 
											 }
											 if(temp_sensor_num==2)
												 tmp_extruder = active_extruder;	
                        else
	                       tmp_extruder=0;
																					 
//									        menu_action_gcode((const char*)screen_key_group[1]->caption);//////20170611	 
											btn_draw(screen_key_group[1]);
											 
											if(target_temperature[tmp_extruder]==0)
											{
											  switch_status = 0;
											  heater_temp[tmp_extruder] = 0;
												screen_key_group[4]->picbtnpathu=KEY_OFF0;
											  screen_key_group[4]->picbtnpathd=KEY_OFF0;
											}
											else
											{
											  switch_status = 1;
											  heater_temp[tmp_extruder] = target_temperature[tmp_extruder];
												screen_key_group[4]->picbtnpathu=KEY_ON0;
											  screen_key_group[4]->picbtnpathd=KEY_ON0;
											}

											 btn_draw(screen_key_group[4]);
											 break;
							case 5: 
								if(gui_phy.language==2)	
								 sprintf(showString,"Head Setting...");
							 else	
								   sprintf(showString,"打印头目标温度设置中...");
								   enter_data_input(&heater_temp[tmp_extruder],FLOAT_TYPE,heater_0_maxtemp,0,false,NULL);//enter_data_input(&preheat_e0_length,FLOAT_TYPE,50,0.1,false,NULL);
											//挤出或回抽长度输入																				
											 break;						 
							case 4: 							       							
        						if(card.sdprinting == false)					      
							        {
												if(switch_status == 0)                          //开关按钮  打开
													{
														switch_status = 1;
//														target_temperature[tmp_extruder] = heater_temp[tmp_extruder];
//														heater_temp[tmp_extruder]=200;
														sprintf(TempBuffer, "M104 S%.1f",heater_temp[tmp_extruder]);//target_temperature[0]
														screen_key_group[4]->picbtnpathu=KEY_ON0;
								            screen_key_group[4]->picbtnpathd=KEY_ON0;
													}
												 else                                            //开关按钮  关掉
												 {
													 switch_status = 0;
													 heater_temp[tmp_extruder] = 0;
													 target_temperature[tmp_extruder] = 0;
													 sprintf(TempBuffer, "M104 S%.1f", 0.0);
													 screen_key_group[4]->picbtnpathu=KEY_OFF0;
								           screen_key_group[4]->picbtnpathd=KEY_OFF0;
												 }
												 menu_action_gcode(TempBuffer);
												 btn_draw(screen_key_group[4]);
												 //btn_draw(screen_key_group[5]);
											 }
											 
											 break;		
										 
					   	default: break;
					}
		}	
	}
	else        //释放界面内存
	{
		for(i=0;i<key_num;i++)
			{	if(i==5)
				{	gui_memin_free(screen_key_group[i]->caption);//释放按钮标题内存
				}
				btn_delete(screen_key_group[i]);
			}
		graph_delete(temp_graph);	
		gui_memin_free(screen_key_group);
	} 
	
}


/***********************************
	功能：进入热床预热
  返回： 无
************************************/
void preheat_bed_screen_320_480(bool state)
{
	u8 selx=0XFF;
	u8 i;
	u8 key_num=3;//9个按钮
//	u32 temp;
	char TempBuffer[32];
//	float data_temp;
	
	static bool switch_status = 0;
	
//	int temp;
	if(redraw_screen==true)				//重画界面
		{ 
			redraw_screen=false;			//关闭重画	
			display_next_update_millis=0;			
			//画背景	
			gui_fill_rectangle(0,0,lcddev.width,lcddev.height,GROUNDCOL);//填充背景色LIGHTGRAY
		  gui_fill_rectangle(11,40,462,200,WHITE);

		  gui_show_strmid(0,0,lcddev.width,50,WHITE,16,(u8*)TOOL_ICONNAME[gui_phy.language][2],1);//显示标题	
    									
			screen_key_group =(_btn_obj **)gui_memin_malloc(sizeof(_btn_obj *)*key_num);	//申请一组按钮	
			temp_graph=graph_creat(60,60,360,150,30,6);		//创建曲线图	
			if(screen_key_group&&temp_graph)                                 
				{											
					  for(i=0;i<3;i++)//返回
						{
						 screen_key_group[i]=btn_creat(11+(i%3)*156,250,145,60,i,BTN_TYPE_PIC);
						
						}
					 
						if((switch_status == 1)&&(target_temperature_bed != bed_temp))//20160403
						{
							target_temperature_bed = bed_temp;
							sprintf(TempBuffer, "M140 S%.1f",bed_temp);
							menu_action_gcode(TempBuffer);
						}	
						
						if(target_temperature_bed>0)//if(switch_status==0 && !card.sdprinting)
						{
						    switch_status = 1;
								bed_temp = target_temperature_bed;
							if(card.sdprinting == true)
								{
								 screen_key_group[0]->picbtnpathu=KEY_ON0;
								 screen_key_group[0]->picbtnpathd=KEY_ON0; 
								}
								else
								{
								 screen_key_group[0]->picbtnpathu=KEY_ON0;
								 screen_key_group[0]->picbtnpathd=KEY_ON0; 
								}
						}
						else
						{
							switch_status = 0;
							screen_key_group[0]->picbtnpathu=KEY_OFF0;
						  screen_key_group[0]->picbtnpathd=KEY_OFF0;	
						}
															
						///热床温度
						screen_key_group[1]->picbtnpathu=DIS3_ORANGE;
						screen_key_group[1]->picbtnpathd=DIS3_ORANGE;						
						screen_key_group[1]->caption =gui_memin_malloc(12);
						if(screen_key_group[1]->caption)
							sprintf((char*)(screen_key_group[1]->caption), "%4.1f/%-4.1f℃",degBed(),bed_temp);
									
           //返回按钮						
						screen_key_group[2]->picbtnpathu=HEAT_BACK0;
						screen_key_group[2]->picbtnpathd=HEAT_BACK1;
											
	
						
						temp_graph->data_0_temp=degBed();
			      temp_graph->data_1_temp=bed_temp;
						graph_draw(temp_graph); //绘制曲线
						
						gui_show_strmid(temp_graph->left+10,temp_graph->top+temp_graph->height+2,90,20,temp_graph->data_0_color,16,(u8*)HEAD_1_ICONNAME[gui_phy.language][0],1);
						gui_show_strmid(temp_graph->left+140,temp_graph->top+temp_graph->height+2,90,20,temp_graph->data_1_color,16,(u8*)HEAD_1_ICONNAME[gui_phy.language][1],1);
						gui_show_strmid(temp_graph->left+290,temp_graph->top+temp_graph->height+2,40,20,BLACK,16,"1S",1);
						for(i=0;i<key_num;i++)
						{
							btn_draw(screen_key_group[i]);
						}
				}							
		}		//重画界面		
		
	if(state)   //正常显示
	{
		if(display_next_update_millis<millis())
		{	
 			 display_next_update_millis = millis() + 1000;//
			 temp_graph->data_0_temp=degBed();
			 temp_graph->data_1_temp=bed_temp;
			 graph_draw(temp_graph); //绘制曲线
//			 gui_phy.back_color=LIGHTGRAY;	//修改背景色
			 gui_show_strmid(30,temp_graph->top+temp_graph->height-8-(temp_graph->height/temp_graph->ordinate_unit_num)*6,20,20,BLACK,16,"℃",0);
			 for(i=0;i<temp_graph->ordinate_unit_num;i++)
				{
					sprintf(TempBuffer,"%4d",temp_graph->ordinate_start_value+ORDINZTEE_UNIT_TABL[temp_graph->ordinate_unit_value]*i);
					gui_show_strmid(20,temp_graph->top+temp_graph->height-10-(temp_graph->height/temp_graph->ordinate_unit_num)*i,35,20,BLACK,16,(u8 *)TempBuffer,0);
				}
//				gui_phy.back_color=BACK_COLOR;		//恢复背景色
				
				sprintf((char*)(screen_key_group[1]->caption), "%4.1f/%-4.1f℃",degBed(),bed_temp );
				btn_draw(screen_key_group[1]);//更新温度按钮
		}		
		selx=screen_key_chk(screen_key_group,&in_obj,key_num);
		if(selx&(1<<6))//按键值有效 并且相应
		{	
			   switch(selx & ~(3<<6))//
					{
						case 2:  nextMenu = tool_screen;
										 break;	
				  	case 1:
               if(gui_phy.language==2)	
								 sprintf(showString,"Bed Setting...");
							 else								
						    sprintf(showString,"热床目标温度设置中...");
    						enter_data_input(&bed_temp,FLOAT_TYPE,bed_maxtemp,0.1,false,NULL);//温度设定输入	
										 break;						 
						case 0:  if(card.sdprinting == false)
											{
													if(switch_status == 0)
													{
														switch_status = 1;												
														sprintf(TempBuffer, "M140 S%.1f",bed_temp);//target_temperature[0]																	
						               	screen_key_group[0]->picbtnpathu=KEY_ON0;		
                                        screen_key_group[0]->picbtnpathd=KEY_ON0;														
													}
													 else
													 {
														 switch_status = 0;
														 bed_temp = 0;
														 target_temperature_bed = 0;
														 sprintf(TempBuffer, "M140 S%.1f",0.0);
															screen_key_group[0]->picbtnpathu=KEY_OFF0;
														  screen_key_group[0]->picbtnpathd=KEY_OFF0;
													 }
													 menu_action_gcode(TempBuffer);
													 btn_draw(screen_key_group[0]);
											 }
											 break;

					   	default: break;
					}
		}
		
 	}
	else       //释放界面内存
	{
		gui_memin_free(screen_key_group[1]->caption);//释放按钮标题内存
		for(i=0;i<key_num;i++)
			{	
				btn_delete(screen_key_group[i]);
			}
		graph_delete(temp_graph);			
		gui_memin_free(screen_key_group);
	} 
	
}

/***********************************
	功能：工具界面
  返回： 无
************************************/
void tool_screen_320_480(bool state)  //工具
{  
   u8 selx=0XFF;
	  u8 i;
    u8 key_num=6;
//    char* TempBuffer[20];
	if(redraw_screen==true)				//重画界面
	{ 
		redraw_screen=false;			//关闭重画
	//绘制背景		
		gui_fill_rectangle(0,0,lcddev.width,lcddev.height,GROUNDCOL );//填充背景色					
		gui_show_strmid(0,10,lcddev.width,20,WHITE,16,(u8*)MAINMENU_ICONNAME[gui_phy.language][2],1);
		
//		if(emc_switch ) 	gui_show_strmid(122,76,116,16,RED,16,(u8*)TOOLS_MENU[gui_phy.language][1],1);
//			else	gui_show_strmid(122,76,116,16,BLACK,16,(u8*)TOOLS_MENU[gui_phy.language][1],1);
//		
//		if(no_material_switch ) 	gui_show_strmid(242,76,116,16,RED,16,(u8*)TOOLS_MENU[gui_phy.language][2],1);
//			else	gui_show_strmid(242,76,116,16,BLACK,16,(u8*)TOOLS_MENU[gui_phy.language][2],1);
		
		
	//绘制背景	
		screen_key_group =(_btn_obj **)gui_memin_malloc(sizeof(_btn_obj *)*key_num);	//申请一组按钮
		if(screen_key_group)
		{	
				 for(i=0;i<6;i++)//
				{
				 screen_key_group[i]=btn_creat(11+(i%3)*156,40+(i/3)*140,145,130,i,BTN_TYPE_PIC);	
                 screen_key_group[i]->caption=(u8*)TOOLS_MENU[gui_phy.language][i];
				 screen_key_group[i]->caption_left=5;
				 screen_key_group[i]->caption_top=110;			
				}
				screen_key_group[0]->picbtnpathu=APP_BED0;
				screen_key_group[0]->picbtnpathd=APP_BED1;//20160228
				
				screen_key_group[1]->picbtnpathu=APP_MOVING0;
				screen_key_group[1]->picbtnpathd=APP_MOVING1;//20160228
		
			  screen_key_group[2]->picbtnpathu=APP_DELTA0;			
			  screen_key_group[2]->picbtnpathd=APP_DELTA1;//20160228
		
				
			  screen_key_group[3]->picbtnpathu=APP_WIFI0;				
			  screen_key_group[3]->picbtnpathd=APP_WIFI1;//20160228
				
			  screen_key_group[4]->picbtnpathu=APP_FAN0;
			  screen_key_group[4]->picbtnpathd=APP_FAN1;//20160228         
				screen_key_group[4]->caption=gui_memin_malloc(20);
				  if(screen_key_group[4]->caption)
				  sprintf((char*)(screen_key_group[4]->caption), "%.1f%%",fanSpeed);//fan	        
//          screen_key_group[4]->caption=(u8*)TempBuffer;	
					
		      screen_key_group[5]->picbtnpathu=SET_BACK0;
			  screen_key_group[5]->picbtnpathd=SET_BACK1;//20160228
														  			
				
			for(i=0;i<key_num;i++)
			{
				btn_draw(screen_key_group[i]);
			}
		}
	}
	if(state)  //正常显示
	{
		selx=screen_key_chk(screen_key_group,&in_obj,key_num);
		if(selx&(1<<6))//按键值有效 并且相应
		{
			switch(selx & ~(3<<6))//
					{ case 0: nextMenu = preheat_bed_screen;
						        break;
						case 1:	nextMenu = manual_screen;
										break;
           	case 2:		
							
                 if(Machine_type == 2)
									 {
                    bed_level_switch = 0;//点击一键调平时关闭调平补偿
						        calculate_delta(current_position);//计算current_position对应的delta
//										adjust_delta(current_position);
										plan_set_position(delta[X_AXIS], delta[Y_AXIS], delta[Z_AXIS], current_position[E_AXIS]);
							      FLASH_WRITE_VAR(FLASH_SET_STORE_OFFSET+183,bed_level_switch); 					
					    
										menu_action_gcode("G28");
						        menu_action_gcode("G29");
						        nextMenu = Bed_level_screen;
									 }else
									 {	
                    menu_action_gcode("G28");										 
									 nextMenu = Bed_level_xyz_screen;									 
									 }
										break;						
//						case 4:							
//										if(emc_switch == 0)
//										{
//											emc_switch = 1;
//											gui_show_strmid(122,76,116,16,RED,16,(u8*)TOOLS_MENU[gui_phy.language][1],1);
//											screen_key_group[4]->bkctbl[2] = RED;  //松开时的图标颜色
//											screen_key_group[4]->bkctbl[3] = RED;	//按下时的图标颜色		//20160228
//										}
//										else
//										{
//											emc_switch = 0;
//											gui_show_strmid(122,76,116,16,BLACK,16,(u8*)TOOLS_MENU[gui_phy.language][1],1);
//											screen_key_group[4]->bkctbl[2] = DARKBLUE;  //松开时的图标颜色
//											screen_key_group[4]->bkctbl[3] = DARKBLUE;	//按下时的图标颜色		//20160228
//										}
//										btn_draw(screen_key_group[4]);
//										FLASH_WRITE_VAR(FLASH_SET_STORE_OFFSET+152,emc_switch);                                             //244        bool               1个字节   EMC开关   //20160412 
//										break;
//						case 5:						
//										if(no_material_switch == 0)
//										{
//											 no_material_switch = 1;
//											Material_EXIT_Set(ENABLE);
//											gui_show_strmid(242,76,116,16,RED,16,(u8*)TOOLS_MENU[gui_phy.language][2],1);
//											 screen_key_group[5]->bkctbl[2] = RED;  //松开时的图标颜色
//											 screen_key_group[5]->bkctbl[3] = RED;	//按下时的图标颜色		//20160228	
//										}
//										else
//										{
//											no_material_switch = 0;
//											Material_EXIT_Set(DISABLE);
//											gui_show_strmid(242,76,116,16,BLACK,16,(u8*)TOOLS_MENU[gui_phy.language][2],1);
//											screen_key_group[5]->bkctbl[2] = DARKBLUE;  //松开时的图标颜色
//											screen_key_group[5]->bkctbl[3] = DARKBLUE;	//按下时的图标颜色		//20160228
//										}
//										FLASH_WRITE_VAR(FLASH_SET_STORE_OFFSET+154,no_material_switch); 
//										btn_draw(screen_key_group[5]);
//										break;
								case 3:	
                   // nextMenu=wifi_screen;									
								  //nextMenu = UpdateOK_screen;
										 break;
								case 4:	
									if(gui_phy.language==2)	
								   sprintf(showString,"Fan Setting...");
							    else	
								  sprintf(showString,"风扇速度设置中...");	
								   fanSpeed=fanSpeed*100/255;								
								  enter_data_input(&fanSpeed,FLOAT_TYPE,100,0,false,NULL); 
								   fanSpeed=fanSpeed*255/100;								
								 btn_draw(screen_key_group[4]);
										 break;
								case 5:			
								nextMenu = home_screen;
										 break;
						default: break;
					}
		}
	}
	else   //释放界面内存
	{
		for(i=0;i<key_num;i++)		
			{	
				if(i==4)
				gui_memin_free(screen_key_group[4]->caption);
				btn_delete(screen_key_group[i]);
			}
		gui_memin_free(screen_key_group);//释放内存	 
	} 	
}


/***********************************
	功能：进入  机器设置   界面
  返回： 无
************************************/
void settinglist_screen_320_480(bool state)
{ 	
	u8 selx=0XFF;
	u8 i;	

	u8 key_num=4;
//	u8** items;//这样定义相当于定义了一个数组   ，而u8* items是定义一个变量
//	items= (u8**)MACHINE_MENU;//这样就是把数组MACHINE_MENU的数组首地址(第一个元素的地址)赋给items(也就是数组首地址，也是数组元素items[0]的地址)，所以items[2]是指第2个元素
	
//	u8* items;//定义一个变量
//	items= (u8*)MACHINE_MENU;//这样是将数组MACHINE_MENU的首地址赋给一个变量items，
	
//	items= (u8**)MACHINE_MENU[0];
////MACHINE_MENU[3][19]里存的就是各字符串的首地址，而sysset_mmenu_tbl[2]就是数组第二组的首地址 ，即第二组第一个元素的地址
////而(u8**)MACHINE_MENU[2]是指将数组第二组的首地址强制赋给items，   即items[8]第二组第8个元素
////所以*items即数组第二组的第一个元素(元素即是字符串的地址)，*items也就是items[0]是第一个元素，而items[8]是第8各元素
////所以**items即是这个字符串的第一个字符

	if(redraw_screen==true)				//重画界面
	{
			redraw_screen=false;			//关闭重画	
			//画背景	
			gui_fill_rectangle(0,0,lcddev.width,lcddev.height,GROUNDCOL );//填充背景色
			
			gui_show_strmid(0,10,lcddev.width,20,WHITE,16,(u8*)MAINMENU_ICONNAME[gui_phy.language][3],1);//显示标题	//“机器设置”
			//画背景				
			
			screen_key_group =(_btn_obj **)gui_memin_malloc(sizeof(_btn_obj *)*key_num);	//申请一组按钮	
			
				if(screen_key_group)
				 {
						for(i=0;i<2;i++)//关于，返回
						{		
//						screen_key_group[i]=btn_creat((i%2)*210+5,(i%2)*105+30,100,100,i,BTN_TYPE_PIC);	
            screen_key_group[i]=btn_creat((i%2)*315+10,(i%2)*140+40,145,130,i,BTN_TYPE_PIC);							
					  screen_key_group[i]->caption=(u8*)SYSTEM_MENU[gui_phy.language][i];	
            screen_key_group[i]->caption_top=110;	 //文字在按钮的偏移位置
						screen_key_group[i]->caption_left=5;  //文字在按钮的偏移位置									
						}						 
					 	for(i=2;i<4;i++)	//语言，恢复出厂设置
						 {		
//						 screen_key_group[i]=btn_creat(110-(i%2)*105,(i%2)*105+30,205,100,i,BTN_TYPE_PIC);
						screen_key_group[i]=btn_creat(165-(i%2)*155,(i%2)*140+40,305,130,i,BTN_TYPE_PIC);	 
						screen_key_group[i]->caption=(u8*)SYSTEM_MENU[gui_phy.language][i];	
					  screen_key_group[i]->caption_top=110;	 //文字在按钮的偏移位置
						screen_key_group[i]->caption_left=5;  //文字在按钮的偏移位置		
						 }
						 						
						screen_key_group[0]->picbtnpathu=ABOUT0;	
            screen_key_group[0]->picbtnpathd=ABOUT1;							 
		   												
						screen_key_group[1]->picbtnpathu=SET_BACK0;				 
		     		screen_key_group[1]->picbtnpathd=SET_BACK1;					 											 				
						 	
						if(gui_phy.language == 0)
							screen_key_group[2]->caption="中文（简体）";
						else if(gui_phy.language == 1)
							screen_key_group[2]->caption="中文（繁體）";
						else if(gui_phy.language == 2)
						  screen_key_group[2]->caption="English";
						screen_key_group[2]->picbtnpathu=LANGUAGE0;				 
		      	screen_key_group[2]->picbtnpathd=LANGUAGE1;
						
						screen_key_group[3]->picbtnpathu=RESTORE0;				 
		        screen_key_group[3]->picbtnpathd=RESTORE1;
						
		       	for(i=0;i<key_num;i++)
						btn_draw(screen_key_group[i]);                                             //画按键
					 

				}
		}
	if(state)
	{

	selx=screen_key_chk(screen_key_group,&in_obj,key_num);
	if(selx&(1<<6))
	 {
		 switch(selx & ~(3<<6))                                          //窗口界面
			{
			  case 0:
						//nextMenu = Machine_Model_Selection;//movelist_screen;
							nextMenu = about_screen;
								break;	
				case 1:
							  nextMenu = home_screen;				
								break;
				case 2: if(gui_phy.language<2)
					       gui_phy.language++;
				        else gui_phy.language=0;
				     								
				        for(i=0;i<key_num;i++) 
                {
                screen_key_group[i]->caption=(u8*)SYSTEM_MENU[gui_phy.language][i];									
								 btn_draw(screen_key_group[i]); 
								}									
								FLASH_WRITE_VAR(FLASH_SET_STORE_OFFSET+151,gui_phy.language);														
								break;
        case 3:
								nextMenu = Restore_factory_Setting_screen;//20160323
													
								break;				
				default: break;
			}
		}

	}
	else
	{	
    for(i=0;i<key_num;i++)		
		btn_delete(screen_key_group[i]);//删除按键		
		gui_memin_free(screen_key_group);//释放内存	
	}
}




/***********************************
	功能：进入 关于  界面
  返回： 无
************************************/
void about_screen_320_480(bool state)
{ 
	
	u8 selx=0XFF;
	u8 key_num=1;
//  char mac[17];//mac地址ver1---17个字符
//  u8 i=0;
//	
  char TempBuffer[16];

	if(redraw_screen==true)				//重画界面
	{
			redraw_screen=false;			//关闭重画	

			//画背景	
			gui_fill_rectangle(0,0,lcddev.width,lcddev.height,WHITE );//填充背景色
			gui_fill_rectangle(0,0,lcddev.width,40,GREENCOL);  //填充区域1	
			gui_draw_color_bmp(0,40,lcddev.width,lcddev.height-50,SET_ABOUT0);
			gui_show_strmid(200,0,80,40,WHITE,16,(u8*)SYSTEM_MENU[gui_phy.language][0],1);//显示标题	//20160409
      sprintf(TempBuffer, "%s","V ");						
			strcat(TempBuffer,(char *)FLASH_SET_EEPROM_VERSION);	
			gui_show_strmid(440,0,40,40,WHITE,16,(u8*)FLASH_SET_EEPROM_VERSION,1);//20160330	
//			sprintf(mac,"aa-bb-99-88-77");//外部传入
//		 	//二维码 
//     	DISPLAY_RENCODE_TO_TFT((u8*)mac);
		
			screen_key_group =(_btn_obj **)gui_memin_malloc(sizeof(_btn_obj *)*key_num);	//申请一组按钮				
			if(screen_key_group)
			 {
//					screen_key_group[0]=btn_creat(5,5,40,40,0,BTN_TYPE_PIC);//带文字的图标                      
//								
//						screen_key_group[0]->picbtnpathu=FILE_BACK0;	 	//松开时的图标
//						screen_key_group[0]->picbtnpathd=FILE_BACK1;		//按下时的图标
				    screen_key_group[0]=btn_creat(5,5,30,30,0,BTN_TYPE_SGICON_TEXT);//带文字的图标                      								
						screen_key_group[0]->picbtnpathu=SYSTEM_BACK;	 	//松开时的图标
						screen_key_group[0]->picbtnpathd=SYSTEM_BACK;		//按下时的图标
				    screen_key_group[0]->bkctbl[0]=GREENCOL;//松开时背景色
				    screen_key_group[0]->bkctbl[1]=GREENCOL;//按下时背景色
				    screen_key_group[0]->bkctbl[2]=WHITE;//松开时图标颜色
				    screen_key_group[0]->bkctbl[3]=GRAY;//按下时图标颜色				 				 
				 
					btn_draw(screen_key_group[0]);                                             //画按键

				}
		}
	if(state)
	{
	
		selx=screen_key_chk(screen_key_group,&in_obj,key_num);	                                //扫描按键
		if(selx&(1<<6))//按键值有效 并且相应
			{
				switch(selx & ~(3<<6))
						{
							case 0: nextMenu = settinglist_screen;
									break;
								default: break;
						}
			}
	}
	else
	{
		btn_delete(screen_key_group[0]);//删除按键
		gui_memin_free(screen_key_group);//释放内存	 
	}
}





