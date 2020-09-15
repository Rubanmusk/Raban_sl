#include "lcd_240_320.h"
#include "ConfigurationStore.h"
//#include "beep.h" 
//#include "material_over.h"
//#include "usart2.h"	


extern char showString[30];//�������ʹ��

extern volatile long count_position[NUM_AXIS];

extern  uint32_t display_next_update_millis;

extern bool redraw_screen ;  //�����ػ���־
extern bool windows_flag ;//���ڴ򿪱�־ ÿ������Ĵ���ģʽ

extern u8 ioc_key;//ͼ�갴��
extern _data_input input_data;
extern _filelistbox_obj * flistbox;
extern _filelistbox_list * filelistx; 	//�ļ� 

extern _btn_obj **screen_key_group;
extern _progressbar_obj* sd_printing_prgb;

extern _graph_obj* temp_graph;

extern float manual_move_length; //�ֶ������ƶ��ľ���
extern u32 manual_move_xy_speed; //�ֶ������ƶ����ٶ�(X��Y��)
extern u32 manual_move_z_speed; //�ֶ������ƶ����ٶ�(Z��)

extern float preheat_e0_length; //������1�����س鳤��
extern u32 preheat_e0_speed; //������1�����س��ٶ�

extern float heater_0_temp ;
extern float bed_temp ;
extern int fanSpeed_temp;

extern u8 Cancel_Flag ;
/////HMI
//extern u8 send_buf[500];
//extern const u8 HMI_end[];
/***********************************
	���ܣ������棺����״̬��ʾ  
	state�� true 	�˳���ǰ����
					false ��ʾ��ǰ����
  ���أ� ��
************************************/
void home_screen_240_320(bool state)	//��ҳ
{
	u8 i;
	u8 selx=0XFF;
//	u32 time;
	char TempBuffer[32];
	u8 key_num=4;//10������	
	if(redraw_screen==true)			//�ػ�����
	{
			redraw_screen=false;			//�ر��ػ�
			display_next_update_millis=0;
		//���Ʊ���		
			gui_fill_rectangle(0,0,lcddev.width,lcddev.height,GUI_TITLE_BACKCOL);//��䱳��ɫ	  
		
		//���Ʊ���	
			screen_key_group =(_btn_obj **)gui_memin_malloc(sizeof(_btn_obj *)*key_num);	//����һ�鰴ť

			if(screen_key_group)
			{		
					for(i=0;i<2;i++)
					{	
						screen_key_group[i]=btn_creat(5+(i%2)*105,30+(i%2)*105,205,100,i,BTN_TYPE_PIC);											
					 	screen_key_group[i]->caption=(u8*)MAINMENU_ICONNAME[gui_phy.language][i];	//ͼ�����					          					
					  screen_key_group[i]->caption_top=80;	 //�����ڰ�ť��ƫ��λ��
						screen_key_group[i]->caption_left=5;  //�����ڰ�ť��ƫ��λ��
					}
           
					for(i=2;i<4;i++)
					{	
						screen_key_group[i]=btn_creat(215-(i%2)*210,30+(i%2)*105,100,100,i,BTN_TYPE_PIC);
						screen_key_group[i]->caption_top=80;	 //�����ڰ�ť��ƫ��λ��
						screen_key_group[i]->caption_left=5;  //�����ڰ�ť��ƫ��λ��						
						screen_key_group[i]->caption=(u8*)MAINMENU_ICONNAME[gui_phy.language][i];	//ͼ�����												
					}
						screen_key_group[0]->picbtnpathu=UNLOAD2;
						screen_key_group[1]->picbtnpathu=PRINT2;					
						screen_key_group[2]->picbtnpathu=APPCATION2;						  
				  	screen_key_group[3]->picbtnpathu=SETTING2;
						
					
						screen_key_group[0]->picbtnpathd=UNLOAD3;
						screen_key_group[1]->picbtnpathd=PRINT3;					
						screen_key_group[2]->picbtnpathd=APPCATION3;	
						screen_key_group[3]->picbtnpathd=SETTING3;
					
							
				
		  	for(i=0;i<key_num;i++)
				{btn_draw(screen_key_group[i]);
				}
			}	
		}
	if(state)   //������ʾ
	{
		
		if(display_next_update_millis<millis())//ʵʱɨ����ʾ���仯����Ϣ
		{
				display_next_update_millis = millis() + 500;//  
					gui_phy.back_color=GUI_TITLE_BACKCOL;									
				gui_draw_single_color_icos(102,5,20,20,SYSTEM_HEATER1,GUI_ICON_FONTCOL,GUI_TITLE_BACKCOL);
				sprintf(TempBuffer, "%3.0f/%-3.0f",degHotend(0),degTargetHotend(0));
				gui_show_strmid(122,5,70,20,GUI_ICON_FONTCOL,16,(u8 *)TempBuffer,0);      
				//��ʾ��ӡͷ1�ĵ�ǰ�¶�/Ŀ���¶�   0Ϊ������
			#if EXTRUDERS > 1
				 if(temp_sensor_num==2)
				 {
					gui_draw_single_color_icos(198,5,20,20,SYSTEM_HEATER2,GUI_ICON_FONTCOL,GUI_TITLE_BACKCOL);//��ʾ��ӡͷ2Сͼ��	
					sprintf(TempBuffer, "%3.0f/%-3.0f",degHotend(1),degTargetHotend(1));
					gui_show_strmid(218,5,70,20,GUI_ICON_FONTCOL,16,(u8 *)TempBuffer,0);
				 }								    
			#endif
				gui_draw_single_color_icos(6,5,20,20,SYSTEM_BED,GUI_ICON_FONTCOL,GUI_TITLE_BACKCOL);
				sprintf(TempBuffer, "%3.0f/%-3.0f",degBed(),degTargetBed());
				gui_show_strmid(26,5,70,20,GUI_ICON_FONTCOL,16,(u8 *)TempBuffer,0);             //��ʾ�ȴ��ĵ�ǰ�¶�/Ŀ���¶�		
          gui_phy.back_color=WHITE;					 
		 }				 
     		                		 
			  if(card.cardOK==true)
	        gui_draw_single_color_icos(294,5,20,20,SYSTEM_SD,GUI_ICON_FONTCOL,GUI_TITLE_BACKCOL);
		    else
		   {
		    gui_fill_rectangle(294,5,20,20,GUI_TITLE_BACKCOL);
		   }

			selx=screen_key_chk(screen_key_group,&in_obj,key_num);   //�����ж�
			if(selx&(1<<6))//����ֵ��Ч ������Ӧ
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
	else				//�ͷŽ����ڴ�
	{
		for(i=0;i<key_num;i++)
		{btn_delete(screen_key_group[i]);
		}
	
		gui_memin_free(screen_key_group);//�ͷ��ڴ�	
		display_next_update_millis=0;		
	}

}


/************************************
	���ܣ���ӡ�е�SD������ ��������ȡ������ͣ����
	state�� true 	�˳���ǰ����
					false ��ʾ��ǰ����
  ���أ� ��
***********************************/
void sdprint_screen_240_320 (bool state) //��ӡ������SD���Ľ���
{	
		u8 selx=0XFF;
		u8 i;
	  static u8 S=0;	
	  // u32 time=0;
		u8 key_num=3; //������ť
	    char TempBuffer[32];	
		if(redraw_screen==true)				//�ػ�����
		{ 	
			redraw_screen=false;			//�ر��ػ�		
			//����
			gui_fill_rectangle(0,0,lcddev.width,lcddev.height/8-5,GUI_TITLE_BACKCOL );//������������ɫ
			gui_fill_rectangle(0,lcddev.height/8-5,lcddev.width,lcddev.height-lcddev.height/8+5,GUI_BODY_BACKCOL );//��䱳��ɫ
			
        sd_printing_prgb=progressbar_creat(110,100,205,20,0,0X61);	//������	
   			
			screen_key_group =(_btn_obj **)gui_memin_malloc(sizeof(_btn_obj *)*key_num);
			if(sd_printing_prgb)
			{	
				    sd_printing_prgb->inbkcolora=BLUECOL;
					sd_printing_prgb->inbkcolorb=BLUECOL;
					sd_printing_prgb->infcolora=REDCOL;
					sd_printing_prgb->infcolorb=REDCOL;
				    sd_printing_prgb->btncolor=GUI_ICON_FONTCOL;//�ٷֱ���ɫ
					
					progressbar_draw_progressbar(sd_printing_prgb);	
			}	   			
			if(screen_key_group)
			{									
				for(i=0;i<key_num;i++)
				{					        
				 screen_key_group[i]=btn_creat(5+(i%3)*105,190,100,45,i,BTN_TYPE_PIC);		
				}				
            /////////////		              				
				//����
				screen_key_group[0]->picbtnpathu=PRINT_MORE2;
				screen_key_group[0]->picbtnpathd=PRINT_MORE3;
                //��ͣ����				
       if(card.sdprinting)
				{ 
					screen_key_group[1]->picbtnpathu=PRINT_PAUSE2;      			
				}
				else
				{  	
				 screen_key_group[1]->picbtnpathu=PRINT_PLAY2;	        			
				}
                //ֹͣ				
				screen_key_group[2]->picbtnpathu=MOVE_STOP2;
				screen_key_group[2]->picbtnpathd=MOVE_STOP3;
											
									
				for(i=0;i<key_num;i++)
				{	btn_draw(screen_key_group[i]);
				}
			}
		}
		if(state)   //������ʾ 
		{		
     		
      if(display_next_update_millis<millis())
			{		
				display_next_update_millis = millis() + 500;//  
				gui_phy.back_color=GUI_TITLE_BACKCOL;	
				
				gui_draw_single_color_icos(102,5,20,20,SYSTEM_HEATER1,WHITE,GUI_TITLE_BACKCOL);
				sprintf(TempBuffer, "%3.0f/%-3.0f",degHotend(0),degTargetHotend(0));
				gui_show_strmid(122,5,70,20,WHITE,16,(u8 *)TempBuffer,0);      
				//��ʾ��ӡͷ1�ĵ�ǰ�¶�/Ŀ���¶�   0Ϊ������
			  #if EXTRUDERS > 1
				 if(temp_sensor_num==2)
				 {
					gui_draw_single_color_icos(198,5,20,20,SYSTEM_HEATER2,WHITE,GUI_TITLE_BACKCOL);//��ʾ��ӡͷ2Сͼ��	
					sprintf(TempBuffer, "%3.0f/%-3.0f",degHotend(1),degTargetHotend(1));
					gui_show_strmid(218,5,70,20,WHITE,16,(u8 *)TempBuffer,0);
				 }								    
			 #endif
				gui_draw_single_color_icos(6,5,20,20,SYSTEM_BED,WHITE,GUI_TITLE_BACKCOL);
				sprintf(TempBuffer, "%3.0f/%-3.0f",degBed(),degTargetBed());
				gui_show_strmid(26,5,70,20,WHITE,16,(u8 *)TempBuffer,0);             //��ʾ�ȴ��ĵ�ǰ�¶�/Ŀ���¶�														
				     		                		 
			  if(card.cardOK==true)
				{					  			 					           					
	         gui_draw_single_color_icos(294,5,20,20,SYSTEM_SD,WHITE,GUI_TITLE_BACKCOL);			
				}
		    else
		   {				
		    gui_fill_rectangle(294,5,20,20,GUI_TITLE_BACKCOL);
		   }
													
			//XYZ����	
//		  if(current_position[X_AXIS]>=1000)     
//			  sprintf(TempBuffer, "X:%5.1fcm",current_position[X_AXIS]/10);
//			else 
//			{
//				if(current_position[X_AXIS]>=0)//if(count_position[X_AXIS]/axis_steps_per_unit[X_AXIS]>=0)
//				   sprintf(TempBuffer, "X:%5.1fmm",current_position[X_AXIS]);
//			  else
//					sprintf(TempBuffer, "X:%5.1fmm",current_position[X_AXIS]);
//			}
//		
//			if(X_MIN_PIN != X_ENDSTOPS_INVERTING)//if((X_MIN_PIN != X_ENDSTOPS_INVERTING)||(X_MAX_PIN != X_ENDSTOPS_INVERTING))
//		      gui_show_strmid(5,35,100,20,RED,12,(u8*)TempBuffer,0);			
//			else
//			 gui_show_strmid(5,35,100,20,WHITE,12,(u8*)TempBuffer,0);					
//			///////	Y				
//			if(current_position[Y_AXIS]>=1000)     
//			  sprintf(TempBuffer, "Y:%5.1fcm",current_position[Y_AXIS]/10);
//			else 
//			{
//				if(current_position[Y_AXIS]>=0)//if(count_position[X_AXIS]/axis_steps_per_unit[X_AXIS]>=0)
//				   sprintf(TempBuffer, "Y:%5.1fmm",current_position[Y_AXIS]);
//			  else
//					sprintf(TempBuffer, "Y:%5.1fmm",current_position[Y_AXIS]);
//			}
//			if(Y_MIN_PIN != Y_ENDSTOPS_INVERTING)//if((X_MIN_PIN != X_ENDSTOPS_INVERTING)||(X_MAX_PIN != X_ENDSTOPS_INVERTING))
//		    gui_show_strmid(110,35,100,20,RED,12,(u8*)TempBuffer,0);			 
//			else				
//				gui_show_strmid(110,35,100,20,WHITE,12,(u8*)TempBuffer,0);
						
		  //////////Z							
     gui_phy.back_color=GUI_BODY_BACKCOL;			
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
		      gui_show_strmid(5,120,100,20,RED,16,(u8*)TempBuffer,0);			  
			else 
			  gui_show_strmid(5,120,100,20,WHITE,16,(u8*)TempBuffer,0);	
										
			 /////////���ٷֱȺͽ�����
			 sd_printing_prgb->curpos=f_tell(&card.fgcode);
			 sd_printing_prgb->totallen=f_size(&card.fgcode);
//		     sprintf(TempBuffer, "%.1f%%",(float)f_tell(&card.fgcode)*100/f_size(&card.fgcode));
//			 gui_show_strmid(270,123,40,30,WHITE,16,(u8*)TempBuffer,0);			
			 progressbar_draw_progressbar(sd_printing_prgb);	//��������
																												
			//��ӡʱ��			 			 			   			
			if(starttime)	 //��ӡ�У���ͣ�������ڴ�ӡ	  starttime
			{
				print_time = millis()/1000 - starttime/1000;		
				sprintf(TempBuffer, "%3d:%02d:%02d",print_time/3600,(print_time/60)%60,print_time%60);
				gui_show_strmid(110,80,200,20,WHITE,16,(u8 *)TempBuffer,0); 		
			}
//			//��ӡ�ٶ�
//			 sprintf(TempBuffer, "%3d%%",feedmultiply);	           			
//			 gui_show_strmid(110,60,100,30,WHITE,16,(u8*)TempBuffer,0);
			//������ͣ
				 if(card.sdprinting)
				{ 
					gui_draw_single_color_icos(15,50,80,65,PRINTING_0+S*4,GREENCOL,GUI_BODY_BACKCOL);  
					screen_key_group[1]->picbtnpathu=PRINT_PLAY2;   
          S++;
          if(S>=9)S=0;					
				}
				else
				{  
          gui_draw_single_color_icos(15,50,80,65,PRINTING_0+S*4,REDCOL,GUI_BODY_BACKCOL);						
				 screen_key_group[1]->picbtnpathu=PRINT_PAUSE2;	      					
				}
				btn_draw(screen_key_group[1]);

				gui_phy.back_color=WHITE;						    
		     }													    					
			/////�ļ���
			gui_show_strmid(110,125,200,20,WHITE,16,card.filename,1);	
        
			selx=screen_key_chk(screen_key_group,&in_obj,key_num);	
			if(selx&(1<<6))//����ֵ��Ч ������Ӧ
			{	
				switch(selx & ~(3<<6))//
						{  
					    case 0://����
							      nextMenu=sdprintmore_screen;
						         break;
							case 1://������ͣ
								if(card.sdprinting)
									{
										
										screen_key_group[1]->picbtnpathu=PRINT_PLAY2;																		
										btn_draw(screen_key_group[1]);
										
											if(residencyStart > -1)
											{
												card.sdprinting=false;	
											 
												 Abnormal_Flag = 0x01;                       												 
												 nextMenu = preheat_head_1_screen;
												 Pause_flag = 0x01;
												 //beep_enable();
										  }								
									}
									else
									{ 										
										screen_key_group[1]->picbtnpathu=PRINT_PAUSE2;																											
										btn_draw(screen_key_group[1]);

										  if(blocks_queued() ==false)// if (block_buffer_head == block_buffer_tail)
										   Abnormal_Flag = 0x02;	
									}
								    break;	
							 case 2: //�Ƿ�ֹͣ��ӡ	
									nextMenu = Confirm_cancel_screen;
									break;
								default: break;
						}
			}
		}
		else  			//�ͷŽ����ڴ�  
		{
			 for(i=0;i<key_num;i++)			
			 {			
				btn_delete(screen_key_group[i]);
		     }
			gui_memin_free(screen_key_group);//�ͷ��ڴ�			
		}
}	
/***********************************
	���ܣ��Ǵ�ӡ״̬��SD�����棺����SD�����ļ��б����
	state�� true 	�˳���ǰ����
					false ��ʾ��ǰ����
  ���أ� ��
************************************/
void gecodelist_screen_240_320(bool state) ///�Ǵ�ӡ������SD���Ľ���
 { 
	u8 selx=0XFF;
	u8 i;	
	u8 *pname=0;
  u8 key_num=4;
	 	//������С
	u16 width =lcddev.height-lcddev.width/8;
	u16 height=lcddev.height/2;	
	u16 left = (lcddev.width-width)/2;
	u16 top = lcddev.height/4;
	u16 btnwidth=(width-30)/3;
	u16 btnheight=btnwidth-30;
 	_filelistbox_list * filelisttemp;	
   
 	if(windows_flag==false)  //�ǵ���
	{	 
		if(redraw_screen==true)				//�ػ�����
		{ 
			redraw_screen=false;			//�ر��ػ�	
      lcd_reset_flag=false;     //�ļ��б��ֹˢ��  
			//������	
			gui_fill_rectangle(0,0,lcddev.width,lcddev.height,WHITE );//��䱳��ɫ
			gui_fill_rectangle(0,0,lcddev.width,40,GREENCOL);  //�������1	
		
			gui_show_strmid(0,0,lcddev.width,40,GUI_ICON_FONTCOL,16,(u8*)MAINMENU_ICONNAME[gui_phy.language][1],1);//��ʾ����	
			//������							
			screen_key_group =(_btn_obj **)gui_memin_malloc(sizeof(_btn_obj *)*1);	//����һ�鰴ť	
			flistbox=filelistbox_creat(1,40,lcddev.width-2,FLBOX_ITEM_HEIGHT*5,1,16);	//����һ��filelistbox
			if(screen_key_group&&flistbox)
				{
//						screen_key_group[0]=btn_creat(5,5,30,30,0,BTN_TYPE_PIC);//�����ֵ�ͼ��											
//						screen_key_group[0]->picbtnpathu=FILE_BACK2;	 	//�ɿ�ʱ��ͼ��
//						screen_key_group[0]->picbtnpathd=FILE_BACK3;		//����ʱ��ͼ��
            screen_key_group[0]=btn_creat(5,5,30,30,0,BTN_TYPE_SGICON_TEXT);//�����ֵ�ͼ��                      								
						screen_key_group[0]->picbtnpathu=SYSTEM_BACK;	 	//�ɿ�ʱ��ͼ��
						screen_key_group[0]->picbtnpathd=SYSTEM_BACK;		//����ʱ��ͼ��
				    screen_key_group[0]->bkctbl[0]=GREENCOL;//�ɿ�ʱ����ɫ
				    screen_key_group[0]->bkctbl[1]=GREENCOL;//����ʱ����ɫ
				    screen_key_group[0]->bkctbl[2]=GUI_ICON_FONTCOL;//�ɿ�ʱͼ����ɫ
				    screen_key_group[0]->bkctbl[3]=GUI_TITLE_BACKCOL;//����ʱͼ����ɫ

					
						flistbox->fliter=FLBOX_FLT_GCODE;	//����GCODE�ļ�
						flistbox->type=0;
						flistbox->path="0:";
						filelistbox_scan_filelist(flistbox);	//����ɨ���б�
						filelistbox_draw_listbox(flistbox);		
						
						for(i=0;i<1;i++)
							{	btn_draw(screen_key_group[i]);
							}
				}
		}
		if(state)//������ʾ
		{
			selx=filelistbox_check(flistbox,&in_obj);//ɨ���ļ�	 ˫���Ķ��� 0x01 ������һ��Ŀ¼ 0x02 ������һ��Ŀ¼ 0x3 Ŀ���ļ�  NULL ��˫������			
			selx=screen_key_chk(screen_key_group,&in_obj,1);
			
			if(selx&(1<<6))//����ֵ��Ч ������Ӧ  //�����ҳ
			{	
				switch(selx & ~(3<<6))//������ҳ
						{   case 0: 	
											if(filelistbox_get_pathdepth(flistbox->path)>0)							  
										    filelistbox_back(flistbox);
							        else{
												lcd_reset_flag=true;
							         nextMenu = home_screen;
											 }
									   break;			   
								default: break;
						}
			}
			else  //�б���Ӧ
			{
				if(flistbox->dbclick==0X81)//˫���ļ���
					{
//		      	FLASH_READ_VAR(FILE_GCODE_PATH_ADDRESS,fnameDirPath);//jee2018
//						if(strcmp((char*)flistbox->path,fnameDirPath)==0)
//						{
//							FLASH_READ_VAR(FILE_NAME_ADDRESS,card.filename);
//							if(strcmp((char*)flistbox->fname,(char*)card.filename)==0)//���
//							{
//							 again_print_flag=true;
//							}
//							else
//							{
//							 again_print_flag=false;
//							}
//						}else 
//						{
						 again_print_flag=false;
//						}
						flistbox->dbclick=0x00;//����Ѵ���  
						redraw_screen=true;
					  windows_flag=true; //���봰��ģʽ	
					}
			}
		}
		else //�ͷŽ����ڴ�
		{
				for(i=0;i<1;i++)
				{	btn_delete(screen_key_group[i]);
				}
				gui_memin_free(screen_key_group);//�ͷ��ڴ�	 
				filelistbox_delete(flistbox);	//ɾ��filelist
		}
	}
	else //����
	{	
			if(redraw_screen==true)				//�ػ�����
			{ 
				redraw_screen=false;			//�ر��ػ�
				//������
				gui_draw_arcrectangle(left,top,width,height,4,1,BLUECOL,BLUECOL);//				
				gui_draw_arcrectangle(left+2,top+lcddev.height/8,width-4,height-lcddev.height/8-2,4,1,WHITE,WHITE);
				
				gui_show_strmid(left,top+5,width,20,GUI_ICON_FONTCOL,16,(u8*)CONTINUE_PRINT_LABE[gui_phy.language][2],1);//��ʾ����
				filelisttemp=filelist_search(flistbox->list,flistbox->selindex);//�õ�ѡ�е�list����ϸ��Ϣ
				gui_show_strmid(left,top+15+lcddev.height/8,width,20,REDCOL,16,filelisttemp->name,1);//��ʾ�ļ�
															
				if(screen_key_group)
					{
            for(i=1;i<key_num-1;i++)
					 {	
						screen_key_group[i]=btn_creat(left+5+(i-1)*(btnwidth+10)*2,top+height-5-btnheight,btnwidth,btnheight,i,BTN_TYPE_TEXTB);//
             					 
						 screen_key_group[i]->bkctbl[0]= ORANGECOL;
						 screen_key_group[i]->bkctbl[1]=ORANGECOL;
					  }
					 if(again_print_flag)
					 {	
					 screen_key_group[1]->caption=(u8*)GUI_AGAIN_CAPTION_TBL[gui_phy.language];//�ش�	
						 
           screen_key_group[3]= btn_creat(left+btnwidth+15,top+height-5-btnheight,btnwidth,btnheight,3,BTN_TYPE_TEXTB);						 
					 screen_key_group[3]->caption=(u8*)GUI_CONTINUE_CAPTION_TBL[gui_phy.language];//����
					 screen_key_group[3]->bkctbl[0]= ORANGECOL;
					 screen_key_group[3]->bkctbl[1]= ORANGECOL;
					
					 }
					 else
					 {
           gui_fill_rectangle(left+btnwidth+15,top+height-5-btnheight,btnwidth,btnheight,WHITE);				 
					 screen_key_group[1]->caption=(u8*)GUI_OK_CAPTION_TBL[gui_phy.language];//ȷ��
					 }
					 screen_key_group[2]->caption=(u8*)GUI_CANCEL_CAPTION_TBL[gui_phy.language];//ȡ��
//						 screen_key_group[1]->picbtnpathu=PRINT_YES0;	 	//�ɿ�ʱ��ͼ��
//						 screen_key_group[1]->picbtnpathd=PRINT_YES1;		//����ʱ��ͼ��
//             screen_key_group[2]->picbtnpathu=PRINT_NO0;	 	//�ɿ�ʱ��ͼ��
//						 screen_key_group[2]->picbtnpathd=PRINT_NO1;		//����ʱ��ͼ��						
						for(i=1;i<key_num-1;i++)
						{	btn_draw(screen_key_group[i]);
						}
						if(again_print_flag)btn_draw(screen_key_group[3]);
					}
			}	
			if(state)  //������ʾ
			{
				if(again_print_flag)
				 selx=screen_key_chk(screen_key_group,&in_obj,key_num);
				else  selx=screen_key_chk(screen_key_group,&in_obj,key_num-1);
				if(selx&(1<<6))//����ֵ��Ч ������Ӧ
				{	
					switch(selx & ~(3<<6))//
 							{ 
						    case 3://����
//								      	windows_flag=false; //�˳�����ģʽ
//												lcd_reset_flag=true;									
//								          filelisttemp=filelist_search(flistbox->list,flistbox->selindex);//�õ�ѡ�е�list����ϸ��Ϣ
//								          memset(card.filename, 0, sizeof(card.filename));////20180115
//													strcpy((char *)card.filename,(const char*)filelisttemp->name);//���Ʊ����ļ�����card��������ʾ
//													Store_selindex = flistbox->selindex;//20160508          
//								
//													pname=gui_memin_malloc(strlen((const char*)filelisttemp->name)+strlen((const char*)flistbox->path)+2);//�����ڴ�
//													if(pname)  
//													{	
//														pname=gui_path_name(pname,flistbox->path,filelisttemp->name);	//�ļ�������·�� 					
//														selx=f_open(&card.fgcode,(const TCHAR*)pname,FA_READ);	//ֻ����ʽ���ļ�											
//														if (selx==FR_OK)//�򿪳ɹ�. 
//														{
//															nextMenu = sdprint_screen;										
//															card.sdprinting = true;
//															quickStop();				//20160412									
//															starttime=millis();
//															feedmultiply = 100;
//															//�ϵ�����
//															FLASH_WRITE_VAR(FILE_NAME_ADDRESS,card.filename);//�洢�ļ���
//															FLASH_WRITE_VAR(FILE_GCODE_PATH_ADDRESS,pname);//�洢·��+�ļ���
//													    FLASH_READ_VAR(BACKUP_Z_HEIGHT_ADDRESS,temp_z_height);//��ȡZ�߶�
//															if(temp_z_height>floor_height)
//															  Abnormal_Z_Height();	
//													  																												
//															USART_ITConfig(USART3, USART_IT_RXNE, DISABLE);////20170213-8266
//															TIM_ITConfig(TIM7,TIM_IT_Update,DISABLE );//TIM_Cmd(TIM7, DISABLE);
//														}
//															gui_memin_free(pname);			//�ͷ��ڴ�	
//													}
//													break;
								case 1:	//�ش�
									      windows_flag=false; //�˳�����ģʽ
												lcd_reset_flag=true;									
								          filelisttemp=filelist_search(flistbox->list,flistbox->selindex);//�õ�ѡ�е�list����ϸ��Ϣ
								          memset(card.filename, 0, sizeof(card.filename));////20180115
//								          mymemcpy(card.filename,filelisttemp->name,32);
													strcpy((char *)card.filename,(const char*)filelisttemp->name);//���Ʊ����ļ�����card��������ʾ
													Store_selindex = flistbox->selindex;//20160508          
								
													pname=gui_memin_malloc(strlen((const char*)filelisttemp->name)+strlen((const char*)flistbox->path)+2);//�����ڴ�
													if(pname)  
													{	
														pname=gui_path_name(pname,flistbox->path,filelisttemp->name);	//�ļ�������·�� 					
														selx=f_open(&card.fgcode,(const TCHAR*)pname,FA_READ);	//ֻ����ʽ���ļ�																																							
														if(selx==FR_OK)//�򿪳ɹ�. 
														{
															nextMenu = sdprint_screen;										
															card.sdprinting = true;
															quickStop();				//20160412									
															starttime=millis();
															feedmultiply = 100;
															
															FLASH_WRITE_VAR(FILE_NAME_ADDRESS,card.filename);//�洢�ļ���	
															strcpy(fnameDirPath,(char*)pname);
//															mymemcpy(fnameDirPath,pname,sizeof(pname));														
															FLASH_WRITE_VAR(FILE_GCODE_PATH_ADDRESS,fnameDirPath);//�洢·��+�ļ���
															
															USART_ITConfig(USART3, USART_IT_RXNE, DISABLE);////20170213-8266
															TIM_ITConfig(TIM7,TIM_IT_Update,DISABLE );//TIM_Cmd(TIM7, DISABLE);
														}
														gui_memin_free(pname);			//�ͷ��ڴ�	
													}
										   break;
									case 2:		windows_flag=false; //�˳�����ģʽ 
                          	again_print_flag=false;
																										
												 	for(i=1;i<key_num-1;i++)
													{	
														btn_delete(screen_key_group[i]);
													}
													if(again_print_flag)	btn_delete(screen_key_group[3]);
													filelistbox_draw_listbox(flistbox);
										break;						
									default: break;
							}
					}
			}
			else   //�ͷŽ����ڴ�
			{
				windows_flag=false;	
				for(i=0;i<key_num;i++)
				{	btn_delete(screen_key_group[i]);
				}
				gui_memin_free(screen_key_group);//�ͷ��ڴ�	 
				filelistbox_delete(flistbox);	//ɾ��filelist	
			}		 
	}

 }


/***********************************
	���ܣ��ֶ�����
  ���أ� ��
************************************/
void manual_screen_240_320(bool state)
{ 
	u8 selx=0XFF;
	u8 i;
	u8 key_num=10;	
	char TempBuffer[32];
	static u8 flag_x=0,flag_y=0,flag_z=0;
//	float data_temp;
	if(redraw_screen==true)				//�ػ�����
		{ 
			redraw_screen=false;			//�ر��ػ�	
			//������	
			gui_fill_rectangle(0,0,lcddev.width,lcddev.height,GUI_TITLE_BACKCOL );//������������ɫ	
			//gui_show_strmid(0,0,lcddev.width,30,GUI_ICON_FONTCOL,16,(u8*)TOOLS_MENU[gui_phy.language][1],1);	
			
			screen_key_group =(_btn_obj **)gui_memin_malloc(sizeof(_btn_obj *)*key_num);	//����һ�鰴ť		
			if(screen_key_group)
				{
						 for(i=0;i<4;i++)//X,Y,Z,����
						 {//��϶Ϊ8+8+8+6=30
					   screen_key_group[i]=btn_creat(5,(i%4)*53+30,100,45,i,BTN_TYPE_PIC);			
						 }						
						 for(i=4;i<6;i++) //��ʾ����,�ٶ�
						 {
						  screen_key_group[i]=btn_creat(110+(i-4)*105,30,100,45,i,BTN_TYPE_PIC);						
						 }												 
						 for(i=6;i<8;i++)//����,����
						 {
						  screen_key_group[i]=btn_creat(110+(i-6)*105,85,100,95,i,BTN_TYPE_PIC);						 	
						 }						 
						 for(i=8;i<key_num;i++)//��ͣ,����
						 {
						 screen_key_group[i]=btn_creat(110+(i-8)*105,189,100,45,i,BTN_TYPE_PIC);
						
						 }
						 if(flag_x==0)
						 screen_key_group[0]->picbtnpathu=MOVE_X2;
						 else
						 screen_key_group[0]->picbtnpathu=MOVE_X3;
						 if(flag_y==0)
						 screen_key_group[1]->picbtnpathu=MOVE_Y2;
						 else
						 screen_key_group[1]->picbtnpathu=MOVE_Y3;
						 if(flag_z==0)
						 screen_key_group[2]->picbtnpathu=MOVE_Z2;
						 else
						 screen_key_group[2]->picbtnpathu=MOVE_Z3;
						 
						 screen_key_group[3]->picbtnpathu=HOMEZERO2;
						 screen_key_group[3]->picbtnpathd=HOMEZERO3;
						 
						 screen_key_group[4]->picbtnpathu=DIS0_BLUE2;
						 screen_key_group[4]->picbtnpathd=DIS0_BLUE2;
						 
						 screen_key_group[5]->picbtnpathu=DIS0_BLUE2;
						 screen_key_group[5]->picbtnpathd=DIS0_BLUE2;
						 						 
						 screen_key_group[4]->caption=gui_memin_malloc(sizeof(u8)*8);
				     sprintf((char*)screen_key_group[4]->caption,"%3.1fmm",manual_move_length);//�ƶ��ٶ�XY
						 screen_key_group[5]->caption=gui_memin_malloc(sizeof(u8)*8);
				     sprintf((char*)screen_key_group[5]->caption,"%3dmm/s",manual_move_xy_speed);//�ƶ��ٶ�XY	
						 
						 screen_key_group[6]->picbtnpathu=MOVE_L2;
				  	 screen_key_group[6]->picbtnpathd=MOVE_L3;	
						 
					  screen_key_group[7]->picbtnpathu=MOVE_R2;
						 screen_key_group[7]->picbtnpathd=MOVE_R3;
						 
						 screen_key_group[8]->picbtnpathu=MOVE_STOP2;
						 screen_key_group[8]->picbtnpathd=MOVE_STOP3;
						 
						 screen_key_group[9]->picbtnpathu=HEAT_BACK2;
						 screen_key_group[9]->picbtnpathd=HEAT_BACK3;
						 
						for(i=0;i<key_num;i++)
						{
							btn_draw(screen_key_group[i]);
						}

				}					
		}	//�ػ��������		
				
	if(state)  //������ʾ
	{
   //��ʾxyz����
		if(display_next_update_millis<millis())
		{		
			display_next_update_millis = millis() + 800;// 
			 gui_phy.back_color=GUI_TITLE_BACKCOL;		
			gui_fill_rectangle(5,5,lcddev.width,20,GUI_TITLE_BACKCOL );//������������ɫ	
			
      if(current_position[X_AXIS]>=1000)     
			  sprintf(TempBuffer, "X:%5.1fcm",current_position[X_AXIS]/10);
			else 
			{
				if(current_position[X_AXIS]>=0)//if(count_position[X_AXIS]/axis_steps_per_unit[X_AXIS]>=0)
				   sprintf(TempBuffer, "X:%5.1fmm",current_position[X_AXIS]);
			  else
					sprintf(TempBuffer, "X:%5.1fmm",current_position[X_AXIS]);
			}
		
			if(X_MIN_PIN != X_ENDSTOPS_INVERTING)//if((X_MIN_PIN != X_ENDSTOPS_INVERTING)||(X_MAX_PIN != X_ENDSTOPS_INVERTING))
		      gui_show_strmid(5,5,100,20,RED,12,(u8*)TempBuffer,0);			
			else
			 gui_show_strmid(5,5,100,20,WHITE,12,(u8*)TempBuffer,0);					
			///////	Y				
			if(current_position[Y_AXIS]>=1000)     
			  sprintf(TempBuffer, "Y:%5.1fcm",current_position[Y_AXIS]/10);
			else 
			{
				if(current_position[Y_AXIS]>=0)//if(count_position[X_AXIS]/axis_steps_per_unit[X_AXIS]>=0)
				   sprintf(TempBuffer, "Y:%5.1fmm",current_position[Y_AXIS]);
			  else
					sprintf(TempBuffer, "Y:%5.1fmm",current_position[Y_AXIS]);
			}
			if(Y_MIN_PIN != Y_ENDSTOPS_INVERTING)//if((X_MIN_PIN != X_ENDSTOPS_INVERTING)||(X_MAX_PIN != X_ENDSTOPS_INVERTING))
		    gui_show_strmid(110,5,100,20,RED,12,(u8*)TempBuffer,0);			 
			else				
				gui_show_strmid(110,5,100,20,WHITE,12,(u8*)TempBuffer,0);
						
		  //////////Z							
    	
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
		      gui_show_strmid(215,5,100,20,RED,12,(u8*)TempBuffer,0);			  
			else 
			  gui_show_strmid(215,5,100,20,WHITE,12,(u8*)TempBuffer,0);	
			
			
			 gui_phy.back_color=WHITE;		
	
		}
		
		selx=screen_key_chk(screen_key_group,&in_obj,key_num);
		if(selx&(1<<6))//����ֵ��Ч ������Ӧ
		{	
		 if(card.sdprinting == false)
			{
			  switch(selx & ~(3<<6))//
					{   case 	0: 
						           flag_y=0;flag_z=0;
						           flag_x=(~flag_x)&0x01;
						           if(flag_x==0)
												screen_key_group[0]->picbtnpathu=MOVE_X2;
											 else
											 screen_key_group[0]->picbtnpathu=MOVE_X3;
											 if(flag_y==0)
											 screen_key_group[1]->picbtnpathu=MOVE_Y2;
											 else
											 screen_key_group[1]->picbtnpathu=MOVE_Y3;
											 if(flag_z==0)
											 screen_key_group[2]->picbtnpathu=MOVE_Z2;
											 else
											 screen_key_group[2]->picbtnpathu=MOVE_Z3;
											 for(i=0;i<3;i++)
						            btn_draw(screen_key_group[i]);
						             break;//
						  case 	1: 
						           flag_x=0;flag_z=0;
						           flag_y=(~flag_y)&0x01;
						           if(flag_x==0)
												screen_key_group[0]->picbtnpathu=MOVE_X2;
											 else
											 screen_key_group[0]->picbtnpathu=MOVE_X3;
											 if(flag_y==0)
											 screen_key_group[1]->picbtnpathu=MOVE_Y2;
											 else
											 screen_key_group[1]->picbtnpathu=MOVE_Y3;
											 if(flag_z==0)
											 screen_key_group[2]->picbtnpathu=MOVE_Z2;
											 else
											 screen_key_group[2]->picbtnpathu=MOVE_Z3;
											 for(i=0;i<3;i++)
						            btn_draw(screen_key_group[i]);
						             break;//					 
							 case 2: 
						           flag_y=0;flag_x=0;
						           flag_z=(~flag_z)&0x01;
						           if(flag_x==0)
												screen_key_group[0]->picbtnpathu=MOVE_X2;
											 else
											 screen_key_group[0]->picbtnpathu=MOVE_X3;
											 if(flag_y==0)
											 screen_key_group[1]->picbtnpathu=MOVE_Y2;
											 else
											 screen_key_group[1]->picbtnpathu=MOVE_Y3;
											 if(flag_z==0)
											 screen_key_group[2]->picbtnpathu=MOVE_Z2;
											 else
											 screen_key_group[2]->picbtnpathu=MOVE_Z3;
											 for(i=0;i<3;i++)
						            btn_draw(screen_key_group[i]);
						             break;//				 
							case 3:								    
										if(flag_x+flag_y+flag_z==0)//�������		
							       {									  
										 	menu_action_gcode("G28") ;
										 }else if(flag_x+flag_y+flag_z==1)
                      {
											 if(flag_x==1) menu_action_gcode("G28 X0") ;//X�����
											 if(flag_y==1) menu_action_gcode("G28 Y0") ;//Y�����
											 if(flag_z==1) menu_action_gcode("G28 Z0") ;//Z�����											
											}						           
											 break;
              case 4: 
								if(gui_phy.language==2)	
								 sprintf(showString,"Length Setting...");
							 else	
				           sprintf(showString,"�ƶ�����������...");
				         enter_data_input(&manual_move_length,FLOAT_TYPE,min(x_max_pos-min_pos[0], min(y_max_pos-min_pos[1], z_max_pos-min_pos[2])),0.1,false,NULL);//3�����ֵ����С���Ǹ���һ��//20160404
											 break;//�ƶ���������			
							case 5: 
												if(gui_phy.language==2)	
												 sprintf(showString,"Move_Speed Setting...");
											 else	
                          sprintf(showString,"�ƶ��ٶ�������...");								
          						enter_data_input(&manual_move_xy_speed,U32_TYPE,min(max_feedrate[0],max_feedrate[1]),1,false,NULL);//X,Y��������ٶ�����С���Ǹ�
											manual_move_z_speed=manual_move_xy_speed/5;
							        break;//XY�ٶ�����
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
 							case 	8: stop_brake_time=millis();break;	//20160409	��ͣ��ť��������Block�͵�ǰblock�����//Discard_Buf_and_Block();//7.�������cmdbuffer��Block	
							case  9: nextMenu = tool_screen; break;
				
					   	default: break;
					}
				}
		}
	}
	else       //�ͷŽ����ڴ�
	{		
			for(i=0;i<key_num;i++)
				{ if(i==4||i==5)
					{	gui_memin_free(screen_key_group[i]->caption);//�ͷŰ�ť�����ڴ�
					}
					btn_delete(screen_key_group[i]);
				}
			gui_memin_free(screen_key_group);//�ͷ��ڴ�	 
		} 
	
}

/***********************************
	���ܣ������ӡͷ1Ԥ��
  ���أ� ��
************************************/
void preheat_head_1_screen_240_320(bool state)
{ 
	u8 selx=0XFF;
	u8 i;
	u8 key_num=6;//9����ť
//	u32 temp;
	char TempBuffer[32];
	static bool switch_status=false;
//	int temp;
	if(redraw_screen==true)				//�ػ�����
		{ 
			redraw_screen=false;			//�ر��ػ�	
			display_next_update_millis=0;			
			//������	
			gui_fill_rectangle(0,0,lcddev.width,lcddev.height,GUI_TITLE_BACKCOL );//��䱳��ɫ
		  gui_fill_rectangle(5,30,205,151,WHITE);

//			gui_show_strmid(0,0,lcddev.width,50,WHITE,16,(u8*)TOOL_ICONNAME[gui_phy.language][1],1);//��ʾ����	
//      if(tmp_extruder==0)
			 gui_show_strmid(0,0,lcddev.width-105,30,GUI_ICON_FONTCOL,16,(u8*)HEAT_ICONNAME[gui_phy.language][0],1);//��ʾ����		
//			else if(tmp_extruder==1)
//			 gui_show_strmid(0,0,lcddev.width-105,30,WHITE,16,(u8*)TOOL_ICONNAME[gui_phy.language][1],1);//��ʾ����	
						
			
			screen_key_group =(_btn_obj **)gui_memin_malloc(sizeof(_btn_obj *)*key_num);	//����һ�鰴ť	
			temp_graph=graph_creat(35,55,150,100,15,6);		//��������ͼ	
			if(screen_key_group&&temp_graph)                                 
				{											
					  for(i=0;i<4;i++)//�س�,��ӡͷ,����,����
						{//��϶Ϊ8
						 screen_key_group[i]=btn_creat(215,30+(i%4)*53,100,45,i,BTN_TYPE_PIC);
						
						}
					 for(i=4;i<6;i++)//���غ���ʾ�¶Ȱ�ť
						{
						 screen_key_group[i]=btn_creat(5+(i%2)*105,189,100,45,i,BTN_TYPE_PIC);
						
						}
						
						//�س鰴ť														
						screen_key_group[0]->picbtnpathu=E_UP2;
						screen_key_group[0]->picbtnpathd=E_UP3;							
						//T0 or T1
						if(active_extruder==0) 
						{							
						 screen_key_group[1]->caption=(u8*)"T0";										 														
					    }else 
						{
						screen_key_group[1]->caption=(u8*)"T1";
						}
						screen_key_group[1]->picbtnpathu=DIS1_GREEN2;
						screen_key_group[1]->picbtnpathd=DIS1_GREEN2;						
						//������ť						
						screen_key_group[2]->picbtnpathu=E_DOWN2;
						screen_key_group[2]->picbtnpathd=E_DOWN3;			
           //���ذ�ť						
						screen_key_group[3]->picbtnpathu=HEAT_BACK2;
						screen_key_group[3]->picbtnpathd=HEAT_BACK3;
						//���ذ�ť	
//						if(switch_status==true)
//						{
//							screen_key_group[4]->picbtnpathu=KEY_ON2;						 
//						}
//						else 
//						{
//							screen_key_group[4]->picbtnpathu=KEY_OFF2;
//						}
						
						
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
									if(card.sdprinting == true)//��ӡ�������¶ȿɵ�������������ذ�ť������㰴
									{
										 screen_key_group[4]->picbtnpathu=KEY_ON2;
										 screen_key_group[4]->picbtnpathd=KEY_ON2;
									}
									else
									{
										 screen_key_group[4]->picbtnpathu=KEY_ON2;
										 screen_key_group[4]->picbtnpathd=KEY_ON2;
									}
							}
							else
							{
								switch_status = 0;
								screen_key_group[4]->picbtnpathu=KEY_OFF2;
								screen_key_group[4]->picbtnpathd=KEY_OFF2;
							}
						
						
						//�¶���ʾ��ť						
						screen_key_group[5]->picbtnpathu=DIS3_ORANGE2;
						screen_key_group[5]->picbtnpathd=DIS3_ORANGE2;
						screen_key_group[5]->caption =gui_memin_malloc(12);
						if(screen_key_group[5]->caption)
						{
							if(gui_phy.language==3)
							  sprintf((char*)(screen_key_group[5]->caption), "%4.1f/%-4.1f��",degHotend(tmp_extruder),heater_temp[tmp_extruder]);
							else
						 	  sprintf((char*)(screen_key_group[5]->caption), "%4.1f/%-4.1f��",degHotend(tmp_extruder),heater_temp[tmp_extruder]);
						}		
//						if((switch_status == 1)&&(target_temperature[tmp_extruder] != heater_temp[tmp_extruder]))//20160403
//						{
////							target_temperature[tmp_extruder] = heater_temp[tmp_extruder];
//							sprintf(TempBuffer, "M104 S%.1f",heater_temp[tmp_extruder]);
//							menu_action_gcode(TempBuffer);
//						}
					
																					
						////////
						temp_graph->data_0_temp=degHotend(tmp_extruder);
						temp_graph->data_1_temp=heater_temp[tmp_extruder];
						graph_draw(temp_graph); //��������
						
						gui_show_strmid(temp_graph->left-15,temp_graph->top+temp_graph->height+2,75,20,temp_graph->data_0_color,16,(u8*)HEAT_ICONNAME[gui_phy.language][3],1);
						gui_show_strmid(temp_graph->left+65,temp_graph->top+temp_graph->height+2,75,20,temp_graph->data_1_color,16,(u8*)HEAT_ICONNAME[gui_phy.language][4],1);
						gui_show_strmid(temp_graph->left+150,temp_graph->top+temp_graph->height+2,20,20,BLACK,12,(u8*)"1S",1);

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
		
		}		//�ػ�����		
		
	if(state)   //������ʾ
	{
		if(display_next_update_millis<millis())
		{	 display_next_update_millis = millis() + 1000;//
			 temp_graph->data_0_temp=degHotend(tmp_extruder);//��ǰ�¶�
			 temp_graph->data_1_temp=heater_temp[tmp_extruder]; //ע�⣬�˴�������Ŀ���¶ȣ�ֻ���м�ת�����¶�
			 graph_draw(temp_graph); //��������
//			 gui_phy.back_color=LIGHTGRAY;	//�޸ı���ɫ
			if(gui_phy.language==3)//����
				gui_show_strmid(10,temp_graph->top+temp_graph->height-8-(temp_graph->height/temp_graph->ordinate_unit_num)*6,20,20,BLACK,16,(u8*)"��",0);
			else 
			  gui_show_strmid(10,temp_graph->top+temp_graph->height-8-(temp_graph->height/temp_graph->ordinate_unit_num)*6,20,20,BLACK,16,(u8*)"��",0);
			 for(i=0;i<temp_graph->ordinate_unit_num;i++)
				{
					sprintf(TempBuffer,"%4d",temp_graph->ordinate_start_value+ORDINZTEE_UNIT_TABL[temp_graph->ordinate_unit_value]*i);
					gui_show_strmid(5,temp_graph->top+temp_graph->height-10-(temp_graph->height/temp_graph->ordinate_unit_num)*i,28,20,BLACK,12,(u8 *)TempBuffer,0);
				}
//				gui_phy.back_color=BACK_COLOR;		//�ָ�����ɫ
				if(gui_phy.language==3)//����
					sprintf((char*)(screen_key_group[5]->caption), "%4.1f/%-4.0f��",degHotend(tmp_extruder),heater_temp[tmp_extruder]);
				else
				  sprintf((char*)(screen_key_group[5]->caption), "%4.1f/%-4.0f��",degHotend(tmp_extruder),heater_temp[tmp_extruder]);
				btn_draw(screen_key_group[5]);//�����¶Ȱ�ť
				//btn_draw(screen_key_group[4]);
		}		
		selx=screen_key_chk(screen_key_group,&in_obj,key_num);
		if(selx&(1<<6))//����ֵ��Ч ������Ӧ
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
												 menu_action_gcode("G91");     //�س鰴ť
												 sprintf(TempBuffer, "G1 E-%.1f F%d",preheat_e0_length,preheat_e0_speed*60);
//												sprintf(TempBuffer, "G1 E-0.1 F%d",preheat_e0_speed*60);
												 menu_action_gcode(TempBuffer);
												 menu_action_gcode("G90");
												}
											 break;
							case 2:  if(card.sdprinting == false)
												{
												 menu_action_gcode("G91");     //������ť											
												 sprintf(TempBuffer, "G1 E%.1f F%d",preheat_e0_length,preheat_e0_speed*60);
//												 sprintf(TempBuffer, "G1 E0.1 F%d",preheat_e0_speed*60);
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
									
//											menu_action_gcode((const char*)screen_key_group[1]->caption);//////20170611	 
									         
											btn_draw(screen_key_group[1]);
											 
											if(target_temperature[tmp_extruder]==0)
											{
											  switch_status = 0;
											  heater_temp[tmp_extruder] = 0;
												screen_key_group[4]->picbtnpathu=KEY_OFF2;
											  screen_key_group[4]->picbtnpathd=KEY_OFF2;
											}
											else
											{
											  switch_status = 1;
											  heater_temp[tmp_extruder] = target_temperature[tmp_extruder];
											  screen_key_group[4]->picbtnpathu=KEY_ON2;
											  screen_key_group[4]->picbtnpathd=KEY_ON2;
											}
											 btn_draw(screen_key_group[4]);
							          	 
											 break;
							case 5:    if(gui_phy.language==2)	
												 sprintf(showString,"Head Setting...");
											 else	
             								sprintf(showString,"��ӡͷĿ���¶�������...");
											 enter_data_input(&heater_temp[tmp_extruder],FLOAT_TYPE,heater_0_maxtemp,0,false,NULL);//�¶��趨����//enter_data_input(&preheat_e0_length,FLOAT_TYPE,50,0.1,false,NULL);
											//������س鳤������																				
											 break;						 
							case 4: 							       							
        						if(card.sdprinting == false)					      
							        {
												if(switch_status == 0)                          //���ذ�ť  ��
													{
														switch_status = 1;
//														target_temperature[tmp_extruder] = heater_temp[tmp_extruder];
//														heater_temp[tmp_extruder]=200;
														sprintf(TempBuffer, "M104 S%.1f",heater_temp[tmp_extruder]);//target_temperature[0]
														screen_key_group[4]->picbtnpathu=KEY_ON2;
								            screen_key_group[4]->picbtnpathd=KEY_ON2;
													}
												 else                                            //���ذ�ť  �ص�
												 {
													 switch_status = 0;
													 heater_temp[tmp_extruder] = 0;
													 target_temperature[tmp_extruder] = 0;
													 sprintf(TempBuffer, "M104 S%.1f", 0.0);
													 screen_key_group[4]->picbtnpathu=KEY_OFF2;
								           screen_key_group[4]->picbtnpathd=KEY_OFF2;
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
	else        //�ͷŽ����ڴ�
	{
		for(i=0;i<key_num;i++)
			{	if(i==5)
				{	gui_memin_free(screen_key_group[i]->caption);//�ͷŰ�ť�����ڴ�
				}
				btn_delete(screen_key_group[i]);
			}
		graph_delete(temp_graph);	
		gui_memin_free(screen_key_group);
	} 
	
}


/***********************************
	���ܣ������ȴ�Ԥ��
  ���أ� ��
************************************/
void preheat_bed_screen_240_320(bool state)
{
	u8 selx=0XFF;
	u8 i;
	u8 key_num=3;//9����ť
//	u32 temp;
	char TempBuffer[32];
//	float data_temp;
	
	static bool switch_status = 0;
	
//	int temp;
	if(redraw_screen==true)				//�ػ�����
		{ 
			redraw_screen=false;			//�ر��ػ�	
			display_next_update_millis=0;			
			//������	
			gui_fill_rectangle(0,0,lcddev.width,lcddev.height,GUI_TITLE_BACKCOL );//������������ɫ			
		  gui_fill_rectangle(5,30,310,155,WHITE);

		  gui_show_strmid(0,0,lcddev.width,30,GUI_ICON_FONTCOL,16,(u8*)HEAT_ICONNAME[gui_phy.language][1],1);//��ʾ����	
    									
			screen_key_group =(_btn_obj **)gui_memin_malloc(sizeof(_btn_obj *)*key_num);	//����һ�鰴ť	
			temp_graph=graph_creat(45,55,240,100,30,6);		//��������ͼ	
			if(screen_key_group&&temp_graph)                                 
				{											
					  for(i=0;i<3;i++)//����
						{
						 screen_key_group[i]=btn_creat(5+(i%3)*105,190,100,45,i,BTN_TYPE_PIC);
						
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
								 screen_key_group[0]->picbtnpathu=KEY_ON2;
								 screen_key_group[0]->picbtnpathd=KEY_ON2; 
								}
								else
								{
								 screen_key_group[0]->picbtnpathu=KEY_ON2;
								 screen_key_group[0]->picbtnpathd=KEY_ON2; 
								}
						}
						else
						{
							switch_status = 0;
							screen_key_group[0]->picbtnpathu=KEY_OFF2;
						  screen_key_group[0]->picbtnpathd=KEY_OFF2;	
						}
						
						
						///�ȴ��¶�
						screen_key_group[1]->picbtnpathu=DIS3_ORANGE2;
						screen_key_group[1]->picbtnpathd=DIS3_ORANGE2;						
						screen_key_group[1]->caption =gui_memin_malloc(12);
						if(screen_key_group[1]->caption)
						{
							if(gui_phy.language==3)							 
								sprintf((char*)(screen_key_group[1]->caption), "%4.1f/%-4.1f��",degBed(),bed_temp);
							else
                sprintf((char*)(screen_key_group[1]->caption), "%4.1f/%-4.1f��",degBed(),bed_temp);							
						}
           //���ذ�ť						
						screen_key_group[2]->picbtnpathu=HEAT_BACK2;
						screen_key_group[2]->picbtnpathd=HEAT_BACK3;
											
	
						
						temp_graph->data_0_temp=degBed();
			      temp_graph->data_1_temp=bed_temp;
						graph_draw(temp_graph); //��������
						
						gui_show_strmid(temp_graph->left+10,temp_graph->top+temp_graph->height+2,90,20,temp_graph->data_0_color,16,(u8*)HEAT_ICONNAME[gui_phy.language][3],1);
						gui_show_strmid(temp_graph->left+110,temp_graph->top+temp_graph->height+2,90,20,temp_graph->data_1_color,16,(u8*)HEAT_ICONNAME[gui_phy.language][4],1);
						gui_show_strmid(temp_graph->left+240,temp_graph->top+temp_graph->height+2,40,20,BLACK,16,(u8*)"1S",1);
						for(i=0;i<key_num;i++)
						{
							btn_draw(screen_key_group[i]);
						}
				}							
		}		//�ػ�����		
		
	if(state)   //������ʾ
	{
		if(display_next_update_millis<millis())
		{	
 			 display_next_update_millis = millis() + 1000;//
			 temp_graph->data_0_temp=degBed();
			 temp_graph->data_1_temp=bed_temp;
			 graph_draw(temp_graph); //��������
//			 gui_phy.back_color=LIGHTGRAY;	//�޸ı���ɫ
			if(gui_phy.language==3)	//����	
			 gui_show_strmid(20,temp_graph->top+temp_graph->height-8-(temp_graph->height/temp_graph->ordinate_unit_num)*6,20,20,BLACK,16,(u8*)"��",0);
			else
			 gui_show_strmid(20,temp_graph->top+temp_graph->height-8-(temp_graph->height/temp_graph->ordinate_unit_num)*6,20,20,BLACK,16,(u8*)"��",0);
			for(i=0;i<temp_graph->ordinate_unit_num;i++)
				{
					sprintf(TempBuffer,"%4d",temp_graph->ordinate_start_value+ORDINZTEE_UNIT_TABL[temp_graph->ordinate_unit_value]*i);
					gui_show_strmid(10,temp_graph->top+temp_graph->height-10-(temp_graph->height/temp_graph->ordinate_unit_num)*i,35,20,BLACK,12,(u8 *)TempBuffer,0);
				}
//				gui_phy.back_color=BACK_COLOR;		//�ָ�����ɫ
				
			if(gui_phy.language==3)							 
				sprintf((char*)(screen_key_group[1]->caption), "%4.1f/%-4.1f��",degBed(),bed_temp);
			else
				sprintf((char*)(screen_key_group[1]->caption), "%4.1f/%-4.1f��",degBed(),bed_temp);
				btn_draw(screen_key_group[1]);//�����¶Ȱ�ť
		}		
		selx=screen_key_chk(screen_key_group,&in_obj,key_num);
		if(selx&(1<<6))//����ֵ��Ч ������Ӧ
		{	
			   switch(selx & ~(3<<6))//
					{
						case 2:  nextMenu = tool_screen;
										 break;	
				  	case 1:  
							if(gui_phy.language==2)	
								 sprintf(showString,"Bed Setting...");
							 else	
						  sprintf(showString,"�ȴ�Ŀ���¶�������...");
						enter_data_input(&bed_temp,FLOAT_TYPE,bed_maxtemp,0.1,false,NULL);//�¶��趨����	
										 break;						 
						case 0:  if(card.sdprinting == false)
											{
													if(switch_status == 0)
													{
														switch_status = 1;												
														sprintf(TempBuffer, "M140 S%.1f",bed_temp);//target_temperature[0]																	
						               	screen_key_group[0]->picbtnpathu=KEY_ON2;		
                            screen_key_group[0]->picbtnpathd=KEY_ON2;														
													}
													 else
													 {
														 switch_status = 0;
														 bed_temp = 0;
														 target_temperature_bed = 0;
														 sprintf(TempBuffer, "M140 S%.1f",0.0);
															screen_key_group[0]->picbtnpathu=KEY_OFF2;
														  screen_key_group[0]->picbtnpathd=KEY_OFF2;
													 }
													 menu_action_gcode(TempBuffer);
													 btn_draw(screen_key_group[0]);
											 }
											 break;
					   	default: break;
					}
		}
		
 	}
	else       //�ͷŽ����ڴ�
	{
		gui_memin_free(screen_key_group[1]->caption);//�ͷŰ�ť�����ڴ�
		for(i=0;i<key_num;i++)
			{	
				btn_delete(screen_key_group[i]);
			}
		graph_delete(temp_graph);			
		gui_memin_free(screen_key_group);
	} 
	
}
/***********************************
	���ܣ����߽���
  ���أ� ��
************************************/
void tool_screen_240_320(bool state)  //����
{  
   u8 selx=0XFF;
	  u8 i;
    u8 key_num=6;
//   char TempBuffer[20];
	if(redraw_screen==true)				//�ػ�����
	{ 
		redraw_screen=false;			//�ر��ػ�
	//���Ʊ���		
		gui_fill_rectangle(0,0,lcddev.width,lcddev.height,GUI_TITLE_BACKCOL );//������������ɫ					
		gui_show_strmid(0,0,lcddev.width,30,GUI_ICON_FONTCOL,16,(u8*)MAINMENU_ICONNAME[gui_phy.language][2],1);

	//���Ʊ���	
		screen_key_group =(_btn_obj **)gui_memin_malloc(sizeof(_btn_obj *)*key_num);	//����һ�鰴ť
		if(screen_key_group)
		{	
				 for(i=0;i<6;i++)//
				{
				 screen_key_group[i]=btn_creat(5+(i%3)*105,30+(i/3)*105,100,100,i,BTN_TYPE_PIC);	
				 if(i<3)
         screen_key_group[i]->caption=(u8*)TOOLS_MENU[gui_phy.language][i];
         screen_key_group[i]->caption_left=5;
      	 screen_key_group[i]->caption_top=75;			
				}
				
				
        screen_key_group[0]->picbtnpathu=APP_BED2;
			  screen_key_group[0]->picbtnpathd=APP_BED3;//20160228
				
		    screen_key_group[1]->picbtnpathu=APP_MOVING2;
			  screen_key_group[1]->picbtnpathd=APP_MOVING3;//20160228
			
			  screen_key_group[2]->picbtnpathu=APP_DELTA2;			
			  screen_key_group[2]->picbtnpathd=APP_DELTA3;//20160228
			
				
				screen_key_group[3]->picbtnpathu=APP_WIFI2;				
			  screen_key_group[3]->picbtnpathd=APP_WIFI3;//20160228
				screen_key_group[3]->caption=(u8*)TOOLS_MENU[gui_phy.language][4];
							
			  screen_key_group[4]->picbtnpathu=APP_FAN2;
		    screen_key_group[4]->picbtnpathd=APP_FAN3;//20160228
				screen_key_group[4]->caption=gui_memin_malloc(20);    
      
				
		    screen_key_group[5]->picbtnpathu=SET_BACK2;
			  screen_key_group[5]->picbtnpathd=SET_BACK3;//20160228
				screen_key_group[5]->caption=(u8*)TOOLS_MENU[gui_phy.language][5];										  			
				
			for(i=0;i<key_num;i++)
			{
				btn_draw(screen_key_group[i]);
			}
		}
	}
	if(state)  //������ʾ
	{
		if(display_next_update_millis<millis())//ʵʱɨ����ʾ���仯����Ϣ
		{
			display_next_update_millis = millis() + 1000;// 
			if(screen_key_group[4]->caption)
			btn_draw(screen_key_group[4]);//ˢ��
		}
		selx=screen_key_chk(screen_key_group,&in_obj,key_num);
		if(selx&(1<<6))//����ֵ��Ч ������Ӧ
		{
			switch(selx & ~(3<<6))//
					{ case 0: nextMenu = preheat_bed_screen;
						        break;
						case 1:	nextMenu = manual_screen;
										break;
           	case 2:	
										menu_action_gcode("G28"); 									
									  nextMenu = Bed_level_xyz_screen;

										break;						
					      case 3:			                     
					           nextMenu = UpdateOK_screen;
										 break;
								case 4:	
									if(gui_phy.language==2)	
								 sprintf(showString,"Fan Setting...");
													
										 break;
								case 5:			
                     nextMenu = home_screen;
										 break;
						default: break;
					}
		}
	}
	else   //�ͷŽ����ڴ�
	{
		for(i=0;i<key_num;i++)
			{
				if(i==4)
				gui_memin_free(screen_key_group[4]->caption);
				btn_delete(screen_key_group[i]);
			}
		gui_memin_free(screen_key_group);//�ͷ��ڴ�	 
	} 	
}

/***********************************
  ���ܣ���ӡ�������
  ���أ� ��
************************************/
void sdprintmore_screen_240_320(bool state)  //
{  
    u8 selx=0XFF;
	u8 i;
    u8 key_num=6;
//   char* TempBuffer[20];
	if(redraw_screen==true)				//�ػ�����
	{ 
		redraw_screen=false;			//�ر��ػ�
	//���Ʊ���		
		gui_fill_rectangle(0,0,lcddev.width,lcddev.height,GUI_TITLE_BACKCOL );//������������ɫ							
		gui_show_strmid(0,5,lcddev.width,20,GUI_ICON_FONTCOL,16,(u8*)TOOLS_MENU[gui_phy.language][6],1);			
	//���Ʊ���	
		screen_key_group =(_btn_obj **)gui_memin_malloc(sizeof(_btn_obj *)*key_num);	//����һ�鰴ť
		if(screen_key_group)
		{	
			 for(i=0;i<6;i++)//
			{
			 screen_key_group[i]=btn_creat(5+(i%3)*105,30+(i/3)*105,100,100,i,BTN_TYPE_PIC);	
			 //screen_key_group[i]->caption=(u8*)TOOLS_MENU[gui_phy.language][i];
			 screen_key_group[i]->caption_left=5;
			 screen_key_group[i]->caption_top=75;			
			}
			
       screen_key_group[0]->picbtnpathu=APP_BED2;
			screen_key_group[0]->picbtnpathd=APP_BED3;//20160228
			screen_key_group[0]->caption=gui_memin_malloc(20);
			if(screen_key_group[0]->caption)
			   sprintf((char*)screen_key_group[0]->caption, "%3.0f/%-3.0f",degBed(),degTargetBed());
			
            screen_key_group[1]->picbtnpathu=PRINT_HEAD12;
			screen_key_group[1]->picbtnpathd=PRINT_HEAD13;//20160228			
			screen_key_group[1]->caption=gui_memin_malloc(20);
			if(screen_key_group[1]->caption)
			  sprintf((char*)screen_key_group[1]->caption, "%3.0f/%-3.0f",degHotend(0),degTargetHotend(0));
			
            screen_key_group[2]->picbtnpathu=PRINT_HEAD22;
			screen_key_group[2]->picbtnpathd=PRINT_HEAD23;//20160228			
            screen_key_group[2]->caption=gui_memin_malloc(20);			
			if(screen_key_group[2]->caption)
			  if(temp_sensor_num==2)
			       sprintf((char*)screen_key_group[2]->caption, "%3.0f/%-3.0f",degHotend(1),degTargetHotend(1));
			  else screen_key_group[2]->caption="--/--";
			  
            screen_key_group[3]->picbtnpathu=PRINT_SPEED2;
			screen_key_group[3]->picbtnpathd=PRINT_SPEED3;//20160228
			screen_key_group[3]->caption=gui_memin_malloc(20);
			  if(screen_key_group[3]->caption)
                sprintf((char*)(screen_key_group[3]->caption), "%3d%%",feedmultiply);
 
		
			screen_key_group[4]->picbtnpathu=APP_FAN2;
			screen_key_group[4]->picbtnpathd=APP_FAN3;//20160228
			screen_key_group[4]->caption=gui_memin_malloc(20);   
				
		      screen_key_group[5]->picbtnpathu=SET_BACK2;
			  screen_key_group[5]->picbtnpathd=SET_BACK3;//20160228
														  			
				
			for(i=0;i<key_num;i++)
			{
				btn_draw(screen_key_group[i]);
			}
		}
	}
	if(state)  //������ʾ
	{
		if(display_next_update_millis<millis())//ʵʱɨ����ʾ���仯����Ϣ
		{
			display_next_update_millis = millis() + 1000;// 
			btn_draw(screen_key_group[4]);//ˢ��
		}
		selx=screen_key_chk(screen_key_group,&in_obj,key_num);
		if(selx&(1<<6))//����ֵ��Ч ������Ӧ
		{
			switch(selx & ~(3<<6))//
					{ 
						case 0: //Bed
							if(gui_phy.language==2)	
								 sprintf(showString,"Bed Setting...");
							 else	
							  sprintf(showString,"�ȴ�Ŀ���¶�������...");
							   enter_data_input(&target_temperature_bed,INT_TYPE,bed_maxtemp,0,false,NULL);
						        break;
						case 1://Head1	
							if(gui_phy.language==2)	
								 sprintf(showString,"Head Setting...");
							 else	
                       sprintf(showString,"��ӡͷĿ���¶�������...");							
						        enter_data_input(&target_temperature[0],INT_TYPE,heater_0_maxtemp,0,false,NULL);						
								break;
                     	case 2://Head2	
								  if(temp_sensor_num==2)
								  {	
										if(gui_phy.language==2)	
											 sprintf(showString,"Head2 Setting...");
										 else	
                         sprintf(showString,"��ӡͷ2Ŀ���¶�������...");									  
									 enter_data_input(&target_temperature[1],INT_TYPE,heater_0_maxtemp,0,false,NULL);
								  }
						         break;						
						case 3:	//Speed	
							   if(gui_phy.language==2)	
								 sprintf(showString,"Velocity Setting...");
							  else	
                    sprintf(showString,"��ӡ�ٶ�������...");							
							    enter_data_input(&feedmultiply,U32_TYPE,999,1,false,NULL);
								 break;
						case 5:	//Back		
								 nextMenu = sdprint_screen;
								 break;
						default: break;
					}
		}
	}
	else   //�ͷŽ����ڴ�
	{
		for(i=0;i<key_num;i++)
			{
				if(i<key_num-1)
				gui_memin_free(screen_key_group[i]->caption);
				btn_delete(screen_key_group[i]);
			}
		gui_memin_free(screen_key_group);//�ͷ��ڴ�	 
	} 	
}

/***********************************
	���ܣ�����  ��������   ����
  ���أ� ��
************************************/
void settinglist_screen_240_320(bool state)
{ 	

	u8 selx=0XFF;
	u8 i;	

	u8 key_num=4;
//	u8** items;//���������൱�ڶ�����һ������   ����u8* items�Ƕ���һ������
//	items= (u8**)MACHINE_MENU;//�������ǰ�����MACHINE_MENU�������׵�ַ(��һ��Ԫ�صĵ�ַ)����items(Ҳ���������׵�ַ��Ҳ������Ԫ��items[0]�ĵ�ַ)������items[2]��ָ��2��Ԫ��
	
//	u8* items;//����һ������
//	items= (u8*)MACHINE_MENU;//�����ǽ�����MACHINE_MENU���׵�ַ����һ������items��
	
//	items= (u8**)MACHINE_MENU[0];
////MACHINE_MENU[3][19]���ľ��Ǹ��ַ������׵�ַ����sysset_mmenu_tbl[2]��������ڶ�����׵�ַ �����ڶ����һ��Ԫ�صĵ�ַ
////��(u8**)MACHINE_MENU[2]��ָ������ڶ�����׵�ַǿ�Ƹ���items��   ��items[8]�ڶ����8��Ԫ��
////����*items������ڶ���ĵ�һ��Ԫ��(Ԫ�ؼ����ַ����ĵ�ַ)��*itemsҲ����items[0]�ǵ�һ��Ԫ�أ���items[8]�ǵ�8��Ԫ��
////����**items��������ַ����ĵ�һ���ַ�

	if(redraw_screen==true)				//�ػ�����
	{
			redraw_screen=false;			//�ر��ػ�	
			//������	
			gui_fill_rectangle(0,0,lcddev.width,lcddev.height,GUI_TITLE_BACKCOL );//������������ɫ
			
			gui_show_strmid(0,5,lcddev.width,20,GUI_ICON_FONTCOL,16,(u8*)MAINMENU_ICONNAME[gui_phy.language][3],1);//��ʾ����	//���������á�
			//������				
			
			screen_key_group =(_btn_obj **)gui_memin_malloc(sizeof(_btn_obj *)*key_num);	//����һ�鰴ť	
			
				if(screen_key_group)
				 {
						for(i=0;i<2;i++)//���ڣ�����
						{		
						screen_key_group[i]=btn_creat((i%2)*210+5,(i%2)*105+30,100,100,i,BTN_TYPE_PIC);								
					  screen_key_group[i]->caption=(u8*)SYSTEM_MENU[gui_phy.language][i];	
            screen_key_group[i]->caption_top=80;	 //�����ڰ�ť��ƫ��λ��
						screen_key_group[i]->caption_left=5;  //�����ڰ�ť��ƫ��λ��									
						}						 
					 	for(i=2;i<4;i++)	//���ԣ��ָ���������
						 {		
					 screen_key_group[i]=btn_creat(110-(i%2)*105,(i%2)*105+30,205,100,i,BTN_TYPE_PIC);					 
						screen_key_group[i]->caption=(u8*)SYSTEM_MENU[gui_phy.language][i];	
					  screen_key_group[i]->caption_top=80;	 //�����ڰ�ť��ƫ��λ��
						screen_key_group[i]->caption_left=5;  //�����ڰ�ť��ƫ��λ��		
						 }
						 						
						screen_key_group[0]->picbtnpathu=ABOUT2;	
            screen_key_group[0]->picbtnpathd=ABOUT3;							 
		   												
						screen_key_group[1]->picbtnpathu=SET_BACK2;				 
		     		screen_key_group[1]->picbtnpathd=SET_BACK3;					 											 				
						 	
//						if(gui_phy.language == 0)
//							screen_key_group[2]->caption=(u8*)LANGUAGE_LABE[0];
//						else if(gui_phy.language == 1)
//						screen_key_group[2]->caption=(u8*)LANGUAGE_LABE[1];
//						else if(gui_phy.language == 2)
//						  screen_key_group[2]->caption=(u8*)LANGUAGE_LABE[2];
						screen_key_group[2]->picbtnpathu=LANGUAGE2;				 
		      	screen_key_group[2]->picbtnpathd=LANGUAGE3;
						
						screen_key_group[3]->picbtnpathu=RESTORE2;				 
		        screen_key_group[3]->picbtnpathd=RESTORE3;
						
		       	for(i=0;i<key_num;i++)
						btn_draw(screen_key_group[i]);                                             //������
					 

				}
		}
	if(state)
	{

	selx=screen_key_chk(screen_key_group,&in_obj,key_num);
	if(selx&(1<<6))
	 {
		 switch(selx & ~(3<<6))                                          //���ڽ���
			{
			  case 0:			
							nextMenu = about_screen;
								break;	
				case 1:
							  nextMenu = home_screen;				
								break;
				case 2:
//      					if(gui_phy.language<2)
//					       gui_phy.language++;
//				        else gui_phy.language=0;
//				     								
//				        for(i=0;i<key_num;i++) 
//                {
//                screen_key_group[i]->caption=(u8*)SYSTEM_MENU[gui_phy.language][i];									
//								 btn_draw(screen_key_group[i]); 
//								}																
//						  	FLASH_WRITE_VAR(FLASH_SET_STORE_OFFSET+151,gui_phy.language);		
								nextMenu = gui_set_language	;		
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
		btn_delete(screen_key_group[i]);//ɾ������		
		gui_memin_free(screen_key_group);//�ͷ��ڴ�	
	}
}


/***********************************
	���ܣ����� ����  ����
  ���أ� ��
************************************/
void about_screen_240_320(bool state)
{ 
	u8 selx=0XFF;
	u8 key_num=1;
//	u8 i=0;
	char TempBuffer[16];
  //char mac[17];

	if(redraw_screen==true)				//�ػ�����
	{
			redraw_screen=false;			//�ر��ػ�	

			//������	
			gui_fill_rectangle(0,0,lcddev.width,lcddev.height,GUI_BODY_BACKCOL );//��䱳��ɫ
			gui_fill_rectangle(0,0,lcddev.width,40,GREENCOL);  //�������1	
			gui_draw_color_bmp(0,40,lcddev.width,lcddev.height-40,SET_ABOUT2);
  		gui_show_strmid(120,0,80,30,GUI_ICON_FONTCOL,16,(u8*)SYSTEM_MENU[gui_phy.language][0],1);//��ʾ����	//20160409
			 
		  sprintf(TempBuffer, "%s","V ");						
			strcat(TempBuffer,(char *)FLASH_SET_EEPROM_VERSION);	
			gui_show_strmid(270,0,50,30,GUI_ICON_FONTCOL,16,(u8*)FLASH_SET_EEPROM_VERSION,1);//20160330					 
//			//��ά��
//	    sprintf(mac,"aa-bb-cc-dd-ee");
//		  DISPLAY_RENCODE_TO_TFT((u8*)mac);
			screen_key_group =(_btn_obj **)gui_memin_malloc(sizeof(_btn_obj *)*key_num);	//����һ�鰴ť				
			if(screen_key_group)//��ɫͼ��
			 {
					screen_key_group[0]=btn_creat(5,0,30,30,0,BTN_TYPE_SGICON_TEXT);//�����ֵ�ͼ��                      
								
						screen_key_group[0]->picbtnpathu=SYSTEM_BACK;	 	//�ɿ�ʱ��ͼ��
						screen_key_group[0]->picbtnpathd=SYSTEM_BACK;		//����ʱ��ͼ��
				    screen_key_group[0]->bkctbl[0]=GREENCOL;//�ɿ�ʱ����ɫ
				    screen_key_group[0]->bkctbl[1]=GREENCOL;//����ʱ����ɫ
				    screen_key_group[0]->bkctbl[2]=GUI_ICON_FONTCOL;//�ɿ�ʱͼ����ɫ
				    screen_key_group[0]->bkctbl[3]=GUI_TITLE_BACKCOL;//����ʱͼ����ɫ
					btn_draw(screen_key_group[0]);                                             //������

				}
		}
	if(state)
	{
		selx=screen_key_chk(screen_key_group,&in_obj,key_num);	                                //ɨ�谴��
		if(selx&(1<<6))//����ֵ��Ч ������Ӧ
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
		btn_delete(screen_key_group[0]);//ɾ������
		gui_memin_free(screen_key_group);//�ͷ��ڴ�	 
	}
}





