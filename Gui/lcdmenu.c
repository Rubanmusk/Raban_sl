#include "lcdmenu.h"
#include "ConfigurationStore.h"
#include "lcd_240_320.h"
#include "spi.h"
//#include "QR_Encode.h"

//extern bool wifi_flag;
//  u8 Password_num[6];
//	static u8 num_cnt = 0;
char showString[30];//���������ʾ
//�ļ�·��
char fnameDirPath[100] = "0:";
extern volatile long count_position[NUM_AXIS];//stepper.c//20160404

//typedef void (*menuFunc_t)();
uint32_t tp_scan_next_update_millis=0;
uint32_t display_next_update_millis=0;

menuFunc_t lastMenu;  //������һ������
menuFunc_t currentMenu = home_screen; /* function pointer to the currently active menu */
menuFunc_t nextMenu = home_screen;  //������һ������


bool redraw_screen = true;  //�����ػ���־
bool windows_flag = false;//���ڴ򿪱�־ ÿ������Ĵ���ģʽ

u8 ioc_key=0x00;//ͼ�갴��
_data_input input_data;
_filelistbox_obj * flistbox;
_filelistbox_list * filelistx; 	//�ļ� 

_btn_obj **screen_key_group;
_progressbar_obj* sd_printing_prgb;

_graph_obj* temp_graph;

float manual_move_length=6; //�ֶ������ƶ��ľ���
u32 manual_move_xy_speed=50; //�ֶ������ƶ����ٶ�(X��Y��)
u32 manual_move_z_speed=10; //�ֶ������ƶ����ٶ�(Z��)

float preheat_e0_length=5; //������1�����س鳤��
u32 preheat_e0_speed=5; //������1�����س��ٶ�

float heater_temp[EXTRUDERS] = { 0 };
float heater_0_temp = 0;//20160409
float bed_temp = 0;
int fanSpeed_temp=0;

u8 Password_set_flag;//������ж��ǿ�����������
bool again_print_flag=false;
bool lcd_reset_flag=true;
/***********************************
	���ܣ�SCREEN_SCAN_TIME ms����һ����Ļ ����һ���������н����˳����л�
  ���أ�   ��   
************************************/
void lcd_update(void)
{
	static u8 TP_cnt=0;
	char TempBuffer[32];

	if (tp_scan_next_update_millis < millis())// tp_scan_next_update_millis  <  timer4_millis
	{
	  tp_scan_next_update_millis = millis() + SCREEN_SCAN_TIME;//   ������һ����Ļˢ��ʱ��
		
		if(PEN==0)//�а�������
		{	
			TP_cnt++;
      if(TP_cnt>=40)
			{
				if(lcd_reset_flag)
				{
			 TP_cnt=0;
			 LCD_Init();
			 redraw_screen = true;
				//beep_enable();
				}
			}
		}
		else if(TP_cnt!=0)
			TP_cnt=0;
		
		//if(LCD_Check()) //Һ��������
		{
			/*
			tp_dev.scan(0);
			in_obj.get_key(&tp_dev,IN_TYPE_TOUCH);	//ÿ��ɨ��õ�һ�ΰ�����ֵ
			if(currentMenu!=nextMenu)   // �����һ�������뵱ǰ���治ͬ �˳���ǰ���� �������ػ���־
			{ 
				lastMenu = currentMenu;
				(*currentMenu)(false);  //�˳���ǰ���� ���ͷ��ڴ�   false�����ͷ��ڴ�
				redraw_screen=true;
				currentMenu=nextMenu;  //������һ������
			}
			(*currentMenu)(true);//������һ������
		
*/


		}
		if(axis_adjust)
		{
			if(buflen < BUFSIZE -5)
			 {
				float x =  40;
				float y =  40; 
				sprintf(TempBuffer, "G1 F500000 X-%.1f Y-%.1f", x,y);
				menu_action_gcode(TempBuffer);
				sprintf(TempBuffer, "G1 F500000 X%.1f Y-%.1f", x,y);
				menu_action_gcode(TempBuffer);	 
				sprintf(TempBuffer, "G1 F500000 X%.1f Y%.1f", x,y);
				menu_action_gcode(TempBuffer);
				sprintf(TempBuffer, "G1 F500000 X-%.1f Y%.1f", x,y);
				menu_action_gcode(TempBuffer);
				sprintf(TempBuffer, "G1 F500000 X-%.1f Y-%.1f", x,y);
				menu_action_gcode(TempBuffer);				 				 
			 }		
		}

		//else          	//Һ���������ȹ��� ��Ҫ���¸�λ��ʼ��
		//{
//			LCD_Init();				//���³�ʼ��LCD
//			(*currentMenu)(false);  //�˳���ǰ���� 	
//			redraw_screen=true;				
// 			currentMenu=home_screen; //��ǰ����ǿ������Ϊ��ҳ
// 			nextMenu=home_screen; //��һ������Ϊ��ҳ			
//			(*currentMenu)(true); //������һ������
		//}
				
	}
}
void menu_action_gcode(const char* pgcode)
{
    enquecommand(pgcode);
}
/***********************************
	���ܣ������棺����״̬��ʾ  
	state�� true 	�˳���ǰ����
					false ��ʾ��ǰ����
  ���أ� ��
************************************/
void home_screen(bool state)	//��ҳ
{
		nextMenu=home_screen_240_320;				
	
}
/***********************************
	���ܣ���ӡ�е�SD������ ��������ȡ������ͣ����
	state�� true 	�˳���ǰ����
					false ��ʾ��ǰ����
  ���أ� ��
************************************/
void sdprint_screen (bool state) //��ӡ������SD���Ľ���
{	
 nextMenu=sdprint_screen_240_320;
}
	
void sdprintmore_screen (bool state) //��ӡ������SD���Ľ���
{	
 nextMenu=sdprintmore_screen_240_320;
}
/***********************************
	���ܣ��Ǵ�ӡ״̬��SD�����棺����SD�����ļ��б����
	state�� true 	�˳���ǰ����
					false ��ʾ��ǰ����
  ���أ� ��
************************************/
void gecodelist_screen(bool state) //�Ǵ�ӡ������SD���Ľ���
{	
 nextMenu=gecodelist_screen_240_320;
}
/***********************************
	���ܣ��ֶ�����
  ���أ� ��
************************************/
void manual_screen(bool state)
{ 
nextMenu=manual_screen_240_320;
}
/***********************************
	���ܣ������ӡͷ1Ԥ��
  ���أ� ��
************************************/
void preheat_head_1_screen(bool state)
{ 
 nextMenu=preheat_head_1_screen_240_320;
}
/***********************************
	���ܣ������ȴ�Ԥ��
  ���أ� ��
************************************/
void preheat_bed_screen(bool state)
{
 nextMenu=preheat_bed_screen_240_320;
}
/***********************************
	���ܣ����߽���
  ���أ� ��
************************************/
void tool_screen(bool state)  //����
{  
nextMenu=tool_screen_240_320;
}
///***********************************
//	���ܣ���ѯ���������������
//  ���أ���
//************************************/
//u8 * set_search_caption(const u8 *mcaption)
//{
//	while(*mcaption!='.')mcaption++;
//	return (u8*)(++mcaption);
//}
/***********************************
	���ܣ�����  ��������   ����
  ���أ� ��
************************************/
void settinglist_screen(bool state)
{ 
 nextMenu=settinglist_screen_240_320;
}
/***********************************
	���ܣ����� ����  ����
  ���أ� ��
************************************/
void about_screen(bool state)
{ 
 nextMenu=about_screen_240_320;
}


/***********************************
	���ܣ��ָ���������
  ���أ� ��
************************************/
void Restore_factory_Setting_screen(bool state)//20160323
{
		u8 selx=0XFF;
		u8 i;
	  u8 key_num = 2; 
	  u16 width =240;
	  u16 height =210;	
		u16 left = (lcddev.width-width)/2;
		u16 top = (lcddev.height-height)/2;
		if(redraw_screen==true)				//�ػ�����
		{
			redraw_screen=false;			//�ر��ػ�		
			//����
			gui_draw_arcrectangle(left,top,width,40,6,1,BLUECOL,BLUECOL);//��������
			gui_show_strmid(left,top,width,30,BLACK,16,(u8*)SYSTEM_MENU[gui_phy.language][3],1);//��ʾ����	
			gui_fill_rectangle(left,top+30,width,height-30,WHITE );//������
			gui_draw_rectangle(left,top+30,width,height-30,BLUECOL);//������
			
     if(gui_phy.language < 2)
			{
				gui_show_strmid(left,top+40,width,40,BLUECOL,16,"�ָ��������ã�����",1);//
			  gui_show_strmid(left,top+80,width,40,BLUECOL,16,"�������ָ�������״̬",1);//
				gui_show_strmid(left,top+120,width,40,BLUECOL,16,"  �Ƿ�ȷ����",1);//
			}
			else if(gui_phy.language == 3)
			{
				gui_show_strmid(left,top+40,width,40,BLUECOL,16,"�������ϻ��¸������ҽñ����",1);//
			  gui_show_strmid(left,top+80,width,40,BLUECOL,16,"�Ķ���Ͱ����ϻ��·κ����˴ϴ�.",1);//
				gui_show_strmid(left,top+120,width,40,BLUECOL,16,"Ȯ����Ŭ���Ͻðڽ��ϱ�?",1);//
			} else 
			{
				gui_show_strmid(left,top+40,width,40,BLUECOL,16,"FACTORY DEFAULT,ALL",1);//
				gui_show_strmid(left,top+80,width,40,BLUECOL,16,"PARAMETERS WILL RESTORE",1);//
				gui_show_strmid(left,top+120,width,40,BLUECOL,16,"ARE YOU SURE?",1);//
			}
			

			screen_key_group =(_btn_obj **)gui_memin_malloc(sizeof(_btn_obj *)*key_num);	//����һ�鰴ť
			if(screen_key_group)
			{
					screen_key_group[0]=btn_creat(left+40,top+160,65,35,0,BTN_TYPE_TEXTB);//�����ֵ�ͼ��
					screen_key_group[1]=btn_creat(left+135,top+160,65,35,1,BTN_TYPE_TEXTB);//�����ֵ�ͼ��
						for(i=0;i<key_num;i++)
						{							
							screen_key_group[i]->bkctbl[0]=REDCOL;
							screen_key_group[i]->bkctbl[1]=ORANGECOL;																	  
						}
						screen_key_group[0]->caption=(u8*)GUI_OK_CAPTION_TBL[gui_phy.language];
						screen_key_group[1]->caption=(u8*)GUI_CANCEL_CAPTION_TBL[gui_phy.language];
						
						for(i=0;i<key_num;i++)
						{
							btn_draw(screen_key_group[i]);
						}
				}

		}
		if(state)
		{
			selx=screen_key_chk(screen_key_group,&in_obj,key_num);	
			if(selx&(1<<6))//����ֵ��Ч ������Ӧ
			{	
				switch(selx & ~(3<<6))//
						{   
								case 0:	
									      Read_Factor_Refault();//Config_ResetDefault();
												
								case 1:	nextMenu = settinglist_screen;
												break;						
								default: break;
						}
			}
		}
		else
		{
			for(i=0;i<key_num;i++)
			{	
				btn_delete(screen_key_group[i]);
			}
			gui_memin_free(screen_key_group);//�ͷ���
		}
}

/***********************************
	���ܣ����¶Ի���
  ���أ� ��
************************************/
void UpdateOK_screen(bool state)//20160323
{
		u8 selx=0XFF;
		u8 i;
	  u8 key_num = 2; 
	  u16 width =240;
	  u16 height =180;	
		u16 left = (lcddev.width-width)/2;
		u16 top = (lcddev.height-height)/2;
		if(redraw_screen==true)				//�ػ�����
		{
			redraw_screen=false;			//�ر��ػ�		
			//����
			gui_draw_arcrectangle(left,top,width,40,6,1,ORANGECOL,ORANGECOL);//��������
			//gui_show_strmid(left,top,width,30,BLACK,16,(u8*)TOOLS_MENU[gui_phy.language][3],1);//��ʾ����	
			gui_fill_rectangle(left,top+30,width,height-30,WHITE );//������
			gui_draw_rectangle(left,top+30,width,height-30,BLUECOL);//������
			
			gui_show_strmid(left,top+60,width,40,BLUECOL,16,(u8*)TOOLS_MENU[gui_phy.language][4],1);//

			screen_key_group =(_btn_obj **)gui_memin_malloc(sizeof(_btn_obj *)*key_num);	//����һ�鰴ť
			if(screen_key_group)
			{
					screen_key_group[0]=btn_creat(left+40,top+120,65,35,0,BTN_TYPE_TEXTB);//�����ֵ�ͼ��
					screen_key_group[1]=btn_creat(left+135,top+120,65,35,1,BTN_TYPE_TEXTB);//�����ֵ�ͼ��
						for(i=0;i<key_num;i++)
						{							
							screen_key_group[i]->bkctbl[0]=REDCOL;
							screen_key_group[i]->bkctbl[1]=ORANGECOL;																	  
						}
						screen_key_group[0]->caption=(u8*)GUI_OK_CAPTION_TBL[gui_phy.language];
						screen_key_group[1]->caption=(u8*)GUI_CANCEL_CAPTION_TBL[gui_phy.language];
						
						for(i=0;i<key_num;i++)
						{
							btn_draw(screen_key_group[i]);
						}
				}

		}
		if(state)
		{
			selx=screen_key_chk(screen_key_group,&in_obj,key_num);	
			if(selx&(1<<6))//����ֵ��Ч ������Ӧ
			{	
				switch(selx & ~(3<<6))//
						{   
								case 0:												
													LCD_Clear(WHITE);
													if(update_theme(5,0,12)==0) 
													{	
                            LCD_ShowString(5,80,200,200,12, "Theme Update Succeed!");														
											  	 }
													else
													{
														LCD_ShowString(5,80,200,200,12, "Theme Update Failed!");															
													}	
                          delay_ms(3000);													
									      nextMenu = home_screen;
								        break;
								case 1:	nextMenu = tool_screen;
												break;						
								default: break;
						}
			}
		}
		else
		{
			for(i=0;i<key_num;i++)
			{	
				btn_delete(screen_key_group[i]);
			}
			gui_memin_free(screen_key_group);//�ͷ���
		}
}


/***********************************
	���ܣ�������������
  ���أ� ��
************************************/
void enter_data_input(void * date_address ,u8 type,double up_limit,double low_limit,bool judge,u32 store_address)
{	
	input_data.address	= date_address; //���ݵ�ַ
	input_data.type		= type; 	//������������
	input_data.up_limit			=	up_limit; //������������
	input_data.low_limit			=	low_limit;// ����������?
	input_data.judge=	judge;
	input_data.store_address=	store_address;
	lastMenu = currentMenu ;
	nextMenu = data_input_screen ;//
}
/***********************************
	���ܣ������������
  ���أ� ��
************************************/
void data_input_screen(bool state)
{
	u8 selx=0XFF;
	u8 i;
	static char *input_data_buf;		//�������12���ַ�+������,�ܹ�13��
	static u8 input_data_sta;
	static u8 num_re_input;
	static u8 maxlen;
	double data_temp;
	double backup_data;
	u8 res;
	u8 key_num=14;
	u16 left;
	u8 xi;//��϶
	u16 btnwidth;//��ť���	
	u16 btnheight;
	u16 top;
	 if(lcddev.id==0x9341)
	 {
	    left = 4;
		 xi=3;
		 btnwidth=60;
  		 btnheight=40;		 
	 }
	 else
	 {		 
		 left=5;
		 xi=5;
		 btnwidth=90;
		 btnheight=60;		
	 }
	top=lcddev.height-2*xi-5-3*btnheight;
	if(redraw_screen==true)				//�ػ�����
	{ 
			redraw_screen=false;			//�ر��ػ�
		 //���Ʊ���
           LCD_Clear(GUI_BODY_BACKCOL);				
		   gui_draw_arcrectangle(lcddev.width/4,lcddev.height/8+10,lcddev.width/2,lcddev.height/8,4,1,WHITE,WHITE );//��ʾ��
//		   gui_show_string((u8*)showString,5,2,lcddev.width/2,30,12,WHITE);	//������ʾ
		//���Ʊ���			
			num_re_input=1;//�������������־
			input_data_buf=(char*)gui_memin_malloc(15);	//���뻺����
			screen_key_group =(_btn_obj **)gui_memin_malloc(sizeof(_btn_obj *)*key_num);	//����һ�鰴ť
			if(input_data_buf&&screen_key_group)	
			{	
				///0~9,С����  ����		
				for(i=1;i<10;i++)
				{						
					screen_key_group[i]=btn_creat(left+(i-1)%3*(btnwidth+xi),top+(i-1)/3*(btnheight+xi),btnwidth,btnheight,i,BTN_TYPE_TEXTB);//�����ֵ�ͼ��						
					screen_key_group[i]->bkctbl[0] = BLUECOL;//�ɿ�ʱ�ı���ɫ
					screen_key_group[i]->bkctbl[1] = GRAY;	//����ʱ�ı���ɫ						
				}	       	
		  	    screen_key_group[1]->caption="1";
		        screen_key_group[2]->caption="2";
				screen_key_group[3]->caption="3";
				screen_key_group[4]->caption="4";
				screen_key_group[5]->caption="5";
				screen_key_group[6]->caption="6";
				screen_key_group[7]->caption="7";
				screen_key_group[8]->caption="8";
				screen_key_group[9]->caption="9";
				
				screen_key_group[10]=btn_creat(left+(btnwidth+xi)*3,top+(btnheight+xi)*2,btnwidth,btnheight,10,BTN_TYPE_TEXTB);				
                screen_key_group[10]->caption=".";
	            screen_key_group[10]->bkctbl[0] = ORANGECOL;//�ɿ�ʱ�ı���ɫ
				screen_key_group[10]->bkctbl[1] = GRAY;	//����ʱ�ı���?
								
                screen_key_group[0]=btn_creat(left+(btnwidth+xi)*3,top+btnheight+xi,btnwidth,btnheight,0,BTN_TYPE_TEXTB);				
                screen_key_group[0]->caption="0";
	            screen_key_group[0]->bkctbl[0] = BLUECOL;//�ɿ�ʱ�ı���ɫ
				screen_key_group[0]->bkctbl[1] = GRAY;	//����ʱ�ı���ɫ
				
				screen_key_group[11]=btn_creat(left+(btnwidth+xi)*3,top,btnwidth,btnheight,11,BTN_TYPE_TEXTB);				
				screen_key_group[11]->caption="+/-";
				screen_key_group[11]->bkctbl[0] = ORANGECOL;//�ɿ�ʱ�ı���ɫ
				screen_key_group[11]->bkctbl[1] = GRAY;	//����ʱ�ı���?
				
				screen_key_group[12]=btn_creat(left+(btnwidth+xi)*4,top,btnwidth,btnheight,12,BTN_TYPE_TEXTB);
				screen_key_group[12]->bkctbl[0] = ORANGECOL;//�ɿ�ʱ�ı���ɫ
				screen_key_group[12]->bkctbl[1] = GRAY;	//����ʱ�ı���ɫ					
				screen_key_group[12]->caption="Del";	
				
				screen_key_group[13]=btn_creat(left+(btnwidth+xi)*4,top+btnheight+xi,btnwidth,btnheight*2+xi,13,BTN_TYPE_TEXTB);
				screen_key_group[13]->bkctbl[0] = REDCOL;//�ɿ�ʱ�ı���ɫ
				screen_key_group[13]->bkctbl[1] = GRAY;	//����ʱ�ı���ɫ	
				screen_key_group[13]->caption="OK";	
				
							
								
				for(i=0;i<key_num;i++)
				{	btn_draw(screen_key_group[i]);//�����ֵ�ͼ��
				}
				
				gui_memset(input_data_buf,0,14);				//���뻺������

				switch(input_data.type)
					{case U8_TYPE:	sprintf(input_data_buf," %d",*(u8*)input_data.address);
													break;
					 case U16_TYPE:	sprintf(input_data_buf," %d",*(u16*)input_data.address);
													break;
					 case U32_TYPE:	sprintf(input_data_buf," %d",*(u32*)input_data.address);
													break;
					 case INT_TYPE:	if(*(int*)input_data.address >= 0)
														sprintf(input_data_buf," %d",*(int*)input_data.address);
													else
														sprintf(input_data_buf,"%d",*(int*)input_data.address);
													break;
					 case FLOAT_TYPE:	if(*(float*)input_data.address >= 0)
																sprintf(input_data_buf," %.3f",*(float*)input_data.address);
														else
																sprintf(input_data_buf,"%.3f",*(float*)input_data.address);
														break;
					 case DOUBLE_TYPE: if(*(double*)input_data.address >= 0)	
																sprintf(input_data_buf," %.3f",*(double*)input_data.address);
														else
																sprintf(input_data_buf,"%.3f",*(double*)input_data.address);
														 break;
					 default : break;
				}	
				if(strchr(input_data_buf,'.'))maxlen=14;
				else maxlen=13;
				input_data_sta=strlen(input_data_buf);
           ///////��ʾ��
				gui_show_strmid(lcddev.width/4,lcddev.height/8+10,lcddev.width/2,lcddev.height/8,BLACK,16,(u8*)input_data_buf,1);

								
			}				
		}
	if(state)
	{
		selx=screen_key_chk(screen_key_group,&in_obj,key_num);
		if(selx&(1<<6))//����ֵ��Ч ������Ӧ
		{
		selx=screen_key_group[selx & ~(3<<6)]->id;
		
			res=input_data_sta&~0x80;
			switch(selx)//
					{ case 0:	
									if(num_re_input)
										{	num_re_input=0;
											maxlen=13;//�ָ�12�����ݳ���
											for(res=2;res<maxlen;res++)
											{ input_data_buf[res]='\0';
											}
											input_data_buf[1]='0';//���һ����������Ϊ0
											input_data_buf[0]=' ';
											res=2;
											input_data_sta=input_data_sta&0x80|res;
										}

										if(res!=2||input_data_buf[1]!='0')
										{
											if(res<maxlen)
											input_data_buf[res++]='0';//����0							
											input_data_sta=input_data_sta&0x80|res;
										} 						
									break; 
						case 1:		/////1~9			
						case 2:
						case 3:						
						case 4:
						case 5:
						case 6:
						case 7:
						case 8:	
            case 9:							
										if(num_re_input)
										{	num_re_input=0;
											maxlen=13;//�ָ�12�����ݳ���
											for(res=0;res<maxlen;res++)
											{ input_data_buf[res]='\0';
											}
											input_data_buf[1]='0';//���һ����������Ϊ0
											input_data_buf[0]=' ';
											res=2;
											input_data_sta=input_data_sta&0x80|res;
										}
										if(maxlen==13)
											{
												if(res>1&&input_data_buf[1]=='0')res=1;					//��һ�����ݲ���Ϊ0 
											}
										if(res<maxlen)input_data_buf[res++]='0'+selx;		
										input_data_sta=input_data_sta&0x80|res;
										
										break;
						case 11://�����л�
										num_re_input=0;
										if(input_data.type>U32_TYPE)
										{
												if(input_data_sta&(0x80))input_data_sta&=~(0x80);	 
												else input_data_sta|=0x80;
												if(input_data_sta&0X80)//	��ʾ����
															input_data_buf[0]='-';
												else	
													input_data_buf[0]=' ';
												if((res<3)&&input_data_buf[1]=='0')
													input_data_buf[0]=' ';
										}
										break;
						case 12://�˸�
										num_re_input=0;
										if(res>2)
										{	       
											res--;
											if(input_data_buf[res]=='.')
											{	input_data_buf[0]=' ';
												maxlen=13;//�ָ�12�����ݳ���
											}
											input_data_buf[res]='\0';
										}else if(input_data_buf[1]!='0')
														{	input_data_buf[1]='0';//���һ����������Ϊ0
															input_data_buf[0]=' ';
														}
										input_data_sta=input_data_sta&0x80|res;					
										break;
						case 10://С����
										num_re_input=0;
										if(maxlen==13)
										{	
											if(res<maxlen)
												{
													if(res==0)
														{	input_data_buf[res++]=' ';//������.
															input_data_buf[res++]='0';//������.	
														}															
													input_data_buf[res++]='.';//����.
													input_data_sta=input_data_sta&0x80|res;	
													maxlen=14;
												}
										}
										break;
						case 13://ȷ����ť	input_data.address
										data_temp=atof(input_data_buf);
										if(data_temp==0)	data_temp=0;
									
										if(data_temp>input_data.up_limit) data_temp=input_data.up_limit;
										else if(data_temp<input_data.low_limit) data_temp=input_data.low_limit;
										switch(input_data.type)
										{	 case U8_TYPE:	backup_data=	*(u8*)input_data.address;
											                *(u8 *)input_data.address =(u8)data_temp;	
                                      if((input_data.judge==true)&&(backup_data!=data_temp))
											                  FLASH_WRITE_VAR(input_data.store_address,*(u8 *)input_data.address);											
																			break;
											 case U16_TYPE:	backup_data=	*(u16*)input_data.address;
												              *(u16 *)input_data.address =(u16)data_temp;
																			if((input_data.judge==true)&&(backup_data!=data_temp))
											                  FLASH_WRITE_VAR(input_data.store_address,*(u16 *)input_data.address);	
																			break;
											 case U32_TYPE:	backup_data=	*(u32*)input_data.address;
												              *(u32 *)input_data.address =(u32)data_temp;
																			if((input_data.judge==true)&&(backup_data!=data_temp))
											                  FLASH_WRITE_VAR(input_data.store_address,*(u32 *)input_data.address);	
																			break;
											 case INT_TYPE:	backup_data=	*(int*)input_data.address;
												              *(int *)input_data.address =(int)data_temp;
																			if((input_data.judge==true)&&(backup_data!=data_temp))
											                  FLASH_WRITE_VAR(input_data.store_address,*(int *)input_data.address);	
																			break;
											 case FLOAT_TYPE: backup_data=	*(float*)input_data.address;	
												               *(float *)input_data.address =(float)data_temp;
																			 if((input_data.judge==true)&&(backup_data!=data_temp))
											                  FLASH_WRITE_VAR(input_data.store_address,*(float *)input_data.address);	
																			break;
											 case DOUBLE_TYPE: backup_data=	*(double*)input_data.address;	 
																				*(double *)input_data.address =(double)data_temp;
																				if((input_data.judge==true)&&(backup_data!=data_temp))
											                  FLASH_WRITE_VAR(input_data.store_address,*(double *)input_data.address);	
																			break;
											 default : break;
										}
										nextMenu = lastMenu ; 
										break;						
//						case 14: //������һ���˵�
//										nextMenu = lastMenu ; 
//										break;
						default: break;	
					}
			  gui_draw_arcrectangle(lcddev.width/4,lcddev.height/8+10,lcddev.width/2,lcddev.height/8,4,1,WHITE,WHITE );//��ʾ��
		            gui_show_strmid(lcddev.width/4,lcddev.height/8+10,lcddev.width/2,lcddev.height/8,BLACK,16,(u8*)input_data_buf,1);

		}
	}
	else
	{
		gui_memin_free(input_data_buf);
		for(i=0;i<key_num;i++)
			{	btn_delete(screen_key_group[i]);
			}
		gui_memin_free(screen_key_group);//�ͷ��ڴ�					
	} 
	

}


/***********************************
	���ܣ����� �ػ�����
  ���أ� ��
************************************/
//void shutdown_screen(bool state)
//{
//	
//	  if(redraw_screen==true)				//�ػ�����
//		{ redraw_screen=false;			//�ر��ػ�
//	    gui_fill_rectangle(0,0,lcddev.width,lcddev.height,GUI_BODY_BACKCOL );//��䱳��ɫ

//			if(gui_phy.language == 2)
//			{
//				gui_show_strmid(0,0,lcddev.width,lcddev.height,WHITE,16,"SHUTDOWN...",1);
//			}
//			else
//			{
//			  gui_show_strmid(0,0,lcddev.width,lcddev.height,WHITE,16,"���ڹػ�...",1);	
//			}
//			delay_ms(666);		
//			POWER_KEY=0;  //�ػ���ť
//			display_next_update_millis = millis() + 1000;// ��ʱ1s
//		}	

//		if(display_next_update_millis<millis())//���1S���� ˵���ػ�ʧ��
//		{	
//			nextMenu = failed_to_shutdown_screen ;			
//		}
//}
/***********************************
	���ܣ��ػ��������
  ���أ� ��
************************************/
void failed_to_shutdown_screen(bool state)
{ 
    if(redraw_screen==true)				//�ػ�����
		{ redraw_screen=false;			//�ر��ػ�
	    gui_fill_rectangle(0,0,lcddev.width,lcddev.height,GUI_BODY_BACKCOL );//��䱳��ɫ

			if(gui_phy.language == 2)
			{
				gui_show_strmid(0,0,lcddev.width,lcddev.height-20,WHITE,16,"FAILED TO SHUTDOWN!!!",1);
				gui_show_strmid(0,20,lcddev.width,lcddev.height-40,WHITE,16,"PLEASE INSTALL Dlion POWER MODULE!",1);
			}
			else
			{
			  gui_show_strmid(0,0,lcddev.width,lcddev.height-20,WHITE,16,"�ػ�ʧ��!!!",1);
			  gui_show_strmid(0,20,lcddev.width,lcddev.height,WHITE,16,"�밲װDlion��Դģ��!",1);		
			}
			display_next_update_millis = millis() + 3000;// ��ʱ3s
		}	
		if(display_next_update_millis<millis())//���3S���� ����home ����
		{		
			POWER_KEY=1;  //�ػ���ť
			nextMenu = home_screen ;
			 //beep_enable();
		}			
}
/***********************************
	���ܣ����� �ϵ� ����
  ���أ� ��
************************************/
//void Power_Off_screen(bool state)
//{
//		u16 width =200;
//		u16 height =100;	
//		u16 left = (lcddev.width-width)/2;
//		u16 top = (lcddev.height-height)/2;	 
//      if(redraw_screen==true)				//�ػ�����
//		{ redraw_screen=false;			//�ر��ػ�
//	        gui_fill_rectangle(0,0,lcddev.width,lcddev.height,LIGHTBLUE );//��䱳��ɫ
//		 	gui_fill_rectangle(left,top,width,height,WHITE );//������
//			gui_draw_rectangle(left,top,width,height,LIGHTBLUE);//����

//			gui_show_strmid(left,top+10,width,40,RED,16,(u8*)POWER_OFF_LABE[gui_phy.language][0],1);
//			gui_show_strmid(left,top+50,width,40,RED,16,(u8*)POWER_OFF_LABE[gui_phy.language][1],1);	

//		}			
//}

/***********************************
	���ܣ����Ͻ���
  ���أ� ��
************************************/
void Material_over_screen (bool state)
{	
		u8 selx=0XFF;
		u8 i;
	    u8 key_num = 1; 
		u16 width =lcddev.width/2;
		u16 height=lcddev.height/2;	
		u16 left = lcddev.width/4;
		u16 top = lcddev.height/4; 
	  u16 btnwidth=(width-40)/2;
    u16 btnheight=lcddev.height/8;	
		if(redraw_screen==true)				//�ػ�����
		{
			redraw_screen=false;			//�ر��ػ�		
			//����		
      gui_fill_rectangle(left,top,width,30,BLUECOL);//������
			gui_fill_rectangle(left,top+30,width,height-30,WHITE );//������
			gui_draw_rectangle(left+1,top+1,width-2,height-2,BLUECOL);//2������
	    gui_draw_rectangle(left,top,width,height,BLUECOL);
			
			gui_show_strmid(left,top+5,width,20,WHITE,16,(u8*)MATERIAL_OVER_LABE[gui_phy.language][0],1);//����
			gui_show_strmid(left,top+40,width,20,BLUECOL,16,(u8*)MATERIAL_OVER_LABE[gui_phy.language][1],1);	

			screen_key_group =(_btn_obj **)gui_memin_malloc(sizeof(_btn_obj *)*key_num);	//����һ�鰴ť
			if(screen_key_group)
			{
					screen_key_group[0]=btn_creat(lcddev.width/2-btnwidth/2,top+height-5-btnheight,btnwidth,btnheight,0,4);//�����ֵ�ͼ��

						screen_key_group[0]->bcfucolor=LDARKBLUE;
						screen_key_group[0]->bcfdcolor=WHITE;	
						screen_key_group[0]->bkctbl[0]=REDCOL;
						screen_key_group[0]->bkctbl[1]=RED;
				
			
						screen_key_group[0]->caption=(u8*)GUI_OK_CAPTION_TBL[gui_phy.language];

						for(i=0;i<key_num;i++)
						{
							btn_draw(screen_key_group[i]);
						}
			 }



		}
		if(state)
		{
			selx=screen_key_chk(screen_key_group,&in_obj,key_num);	
			if(selx&(1<<6))//����ֵ��Ч ������Ӧ
			{	
				switch(selx & ~(3<<6))//
						{
								case 0: nextMenu = preheat_head_1_screen;
												break;								
								default: break;
						}
			}
		}
		else
		{
				for(i=0;i<key_num;i++)
				{	
					btn_delete(screen_key_group[i]);
				}
				gui_memin_free(screen_key_group);//�ͷ���
		}
}


/***********************************
	���ܣ����ϻ�����ɼ�����ӡ
  ���أ� ��
************************************/
void Material_ok_screen(bool state)//20160412
{	
		u8 selx=0XFF;
		u8 i;
	    u8 key_num = 2; 
		u16 width =lcddev.width/2;
		u16 height=lcddev.height/2;	
		u16 left = lcddev.width/4;
		u16 top = lcddev.height/4;
    u16 btnwidth=(width-40)/2;
    u16 btnheight=lcddev.height/8;	
		if(redraw_screen==true)				//�ػ�����
		{
			redraw_screen=false;			//�ر��ػ�		
			//����		

			gui_fill_rectangle(left,top,width,30,BLUECOL);//������
			gui_fill_rectangle(left,top+30,width,height-30,WHITE );//������
			gui_draw_rectangle(left+1,top+1,width-2,height-2,BLUECOL);//2������
	    gui_draw_rectangle(left,top,width,height,BLUECOL);
			
			gui_show_strmid(left,top+5,width,20,WHITE,16,(u8*)MATERIAL_OK_LABE[gui_phy.language][0],1);
			gui_show_strmid(left,top+40,width,20,BLUECOL,16,(u8*)MATERIAL_OK_LABE[gui_phy.language][1],1);	


			screen_key_group =(_btn_obj **)gui_memin_malloc(sizeof(_btn_obj *)*key_num);	//����һ�鰴ť
			if(screen_key_group)
			{
				
				screen_key_group[0]=btn_creat(left+5,top+height-5-btnheight,btnwidth,btnheight,0,4);//�����ֵ�ͼ��
				screen_key_group[1]=btn_creat(left+width-5-btnwidth,top+height-5-btnheight,btnwidth,btnheight,1,4);//�����ֵ�ͼ��
				
				for(i=0;i<key_num;i++)
				{
					screen_key_group[i]->bcfucolor=LDARKBLUE;
					screen_key_group[i]->bcfdcolor=WHITE;
					screen_key_group[i]->bkctbl[0]=REDCOL;
					screen_key_group[i]->bkctbl[1]=RED;
			
				}					
			
						screen_key_group[0]->caption=(u8*)GUI_OK_CAPTION_TBL[gui_phy.language];
						screen_key_group[1]->caption=(u8*)GUI_CANCEL_CAPTION_TBL[gui_phy.language];

						for(i=0;i<key_num;i++)
						{
							btn_draw(screen_key_group[i]);
						}
			 }



		}
		if(state)
		{
			selx=screen_key_chk(screen_key_group,&in_obj,key_num);	
			if(selx&(1<<6))//����ֵ��Ч ������Ӧ
			{	
				switch(selx & ~(3<<6))//
						{
								case 0:	nextMenu = sdprint_screen;
												TIM_ITConfig(TIM3,TIM_IT_Update, ENABLE);//20160323
//												Material_Status = 0x02;//20160412
												break;	
								case 1: nextMenu = preheat_head_1_screen;
												break;												
								default: break;
						}
						
			}
		}
		else
		{
				for(i=0;i<key_num;i++)
				{	
					btn_delete(screen_key_group[i]);
				}
				gui_memin_free(screen_key_group);//�ͷ���
		}
		
}


/***********************************
  ���ܣ� �Ƿ�ֹͣ��ӡ
  ���أ� ��
************************************/
void Confirm_cancel_screen(bool state)//20160412
{	
		u8 selx=0XFF;
		u8 i;
	    u8 key_num = 2; 
		u16 width =220;
		u16 height =150;	
		u16 left = (lcddev.width-width)/2;
		u16 top = (lcddev.height-height)/2;	
		if(redraw_screen==true)				//�ػ�����
		{
			redraw_screen=false;			//�ر��ػ�		
			//����		

			gui_fill_rectangle(left,top,width,30,BLUECOL);
			gui_fill_rectangle(left,top+30,width,height-30,WHITE );//������			
			gui_draw_rectangle(left-1,top,width+2,height+1,BLUECOL);//	
			
			gui_show_strmid(left,top+30,width,40,BLUECOL,16,(u8*)CONFIRM_CANCEL_LABE[gui_phy.language][0],1);


			screen_key_group =(_btn_obj **)gui_memin_malloc(sizeof(_btn_obj *)*key_num);	//����һ�鰴ť
			if(screen_key_group)
			{
					screen_key_group[0]=btn_creat(left+5,top+100,100,45,0,4);//�����ֵ�ͼ��
					screen_key_group[1]=btn_creat(left+115,top+100,100,45,1,4);//�����ֵ�ͼ��
				 

				for(i=0;i<key_num;i++)
				{
					screen_key_group[i]->bcfucolor=LDARKBLUE;
					screen_key_group[i]->bcfdcolor=WHITE;
					screen_key_group[i]->bkctbl[0]=REDCOL;
					screen_key_group[i]->bkctbl[1]=RED;					
			
				}					
			
						screen_key_group[0]->caption=(u8*)GUI_OK_CAPTION_TBL[gui_phy.language];
						screen_key_group[1]->caption=(u8*)GUI_CANCEL_CAPTION_TBL[gui_phy.language];

						for(i=0;i<key_num;i++)
						{
							btn_draw(screen_key_group[i]);
						}
			 }



		}
		if(state)
		{
			selx=screen_key_chk(screen_key_group,&in_obj,key_num);	
			if(selx&(1<<6))//����ֵ��Ч ������Ӧ
			{	
				switch(selx & ~(3<<6))//
						{
								case 0:	//ֹͣ��ӡ
												card.sdprinting=false;
												Discard_Buf_and_Block();

												Abnormal_Flag = 0x03;
												break;	
								case 1: 
									        nextMenu = sdprint_screen;
												break;												
								default: break;
						}
						
			}
		}
		else
		{
				for(i=0;i<key_num;i++)
				{	
					btn_delete(screen_key_group[i]);
				}
				gui_memin_free(screen_key_group);//�ͷ���
		}
		
}



/***********************************
	���ܣ���ȫ�ϵ�����ϵ��ȡ������δ�������
  ���أ� ��
************************************/
void Continue_Print_screen(bool state)//20160412
{	
		u8 selx=0XFF;
		u8 i;
	  u8 res;
	  u8 key_num = 2; 	
//    u8 *pname=0;
		u16 width =220;
		u16 height =150;	
		u16 left = (lcddev.width-width)/2;
		u16 top = (lcddev.height-height)/2;	
// 	   _filelistbox_list * filelisttemp;		
	
		if(redraw_screen==true)				//�ػ�����
		{
			redraw_screen=false;			//�ر��ػ�		
			//����		
			LCD_Clear(WHITE);
			gui_fill_rectangle(left,top,width,30,BLUECOL );
			gui_fill_rectangle(left,top+30,width,height-30,WHITE );//������			
			gui_draw_rectangle(left-1,top,width+2,height+1,BLUECOL);//����	

			gui_show_strmid(left,top+30,width,20,BLUECOL,16,(u8*)CONTINUE_PRINT_LABE[gui_phy.language][0],1);
			gui_show_strmid(left,top+60,width,20,BLUECOL,16,(u8*)CONTINUE_PRINT_LABE[gui_phy.language][1],1);	

			screen_key_group =(_btn_obj **)gui_memin_malloc(sizeof(_btn_obj *)*key_num);	//����һ�鰴ť
			if(screen_key_group)
			{
				screen_key_group[0]=btn_creat(left+5,top+100,100,45,0,BTN_TYPE_TEXTB);//�����ֵ�ͼ��
				screen_key_group[1]=btn_creat(left+115,top+100,100,45,1,BTN_TYPE_TEXTB);//�����ֵ�ͼ��
				 
				for(i=0;i<key_num;i++)
				{
					screen_key_group[i]->bcfucolor=LDARKBLUE;
					screen_key_group[i]->bcfdcolor=WHITE;
					screen_key_group[i]->bkctbl[0]=REDCOL;
					screen_key_group[i]->bkctbl[1]=RED;
				
				}					
			
						screen_key_group[0]->caption=(u8*)GUI_OK_CAPTION_TBL[gui_phy.language];
						screen_key_group[1]->caption=(u8*)GUI_CANCEL_CAPTION_TBL[gui_phy.language];

						for(i=0;i<key_num;i++)
						{
							btn_draw(screen_key_group[i]);
						}
			 }



		}
		if(state)
		{
			selx=screen_key_chk(screen_key_group,&in_obj,key_num);	
			if(selx&(1<<6))//����ֵ��Ч ������Ӧ
			{	
				switch(selx & ~(3<<6))//
						{
								case 0:	
//									Abnormal_Read();
//								  card_closefile();
									//flistbox=filelistbox_creat(2,44,lcddev.width-4,192,1,16);	//����һ��filelistbo
//								flistbox=filelistbox_creat(1,41,lcddev.width-2,FLBOX_ITEM_HEIGHT*6,1,16);	//����һ��filelistbox
//								if(flistbox)
//								{
//									flistbox->fliter=FLBOX_FLT_GCODE;	//����GCODE�ļ�
//									flistbox->type=0;
//									flistbox->path="0:\\GCODE";
//									filelistbox_scan_filelist(flistbox);	//����ɨ���б�
//									filelistbox_draw_listbox(flistbox);		
//								
//									flistbox->selindex = Read_selindex;//���ܹؼ��ǽ��洢�ĵڼ����ļ��ĺ�ȡ�����ҵ����ļ�
//									filelisttemp=filelist_search(flistbox->list,flistbox->selindex);//�õ�ѡ�е�list����ϸ��Ϣ
//										strcpy((char *)card.filename,(const char*)filelisttemp->name);//���Ʊ����ļ�����card��������ʾ
//										Store_selindex = flistbox->selindex;//20160508          
					
//										pname=gui_memin_malloc(strlen((const char*)filelisttemp->name)+strlen((const char*)flistbox->path)+2);//�����ڴ�
									     FLASH_READ_VAR(FILE_GCODE_PATH_ADDRESS,fnameDirPath);//��ȡ·��+�ļ���
												res=f_open(&card.fgcode,(const TCHAR*)fnameDirPath,FA_READ);
													if(res==FR_OK)
													{
														 FLASH_READ_VAR(FILE_NAME_ADDRESS,card.filename);//�ļ���
															nextMenu = sdprint_screen;										
															card.sdprinting = true;
															quickStop();				//20160412									
															starttime=millis();
															feedmultiply = 100;
														  FLASH_READ_VAR(BACKUP_Z_HEIGHT_ADDRESS,temp_z_height);//��ȡZ�߶�
															if(temp_z_height>floor_height)
															{
																appoint_z_height=temp_z_height;//�ϵ�����¼�¼Z�߶�
															  Abnormal_Z_Height();	
															}	
															
															USART_ITConfig(USART3, USART_IT_RXNE, DISABLE);////20170213-8266
															TIM_ITConfig(TIM7,TIM_IT_Update,DISABLE );//TIM_Cmd(TIM7, DISABLE);
//															Abnormal_Flag = 0x05;//�ϵ�������ӡ
//															fgcode_fptr_flag = 0x00;
////														isStoreFlag=0x00;//jee2018
////														SPI_Flash_Erase_Sector(FGCODE_FPTR_ADDRESS/1024/4);
//														  SPI_Flash_Erase_Sector(FILE_NAME_ADDRESS/1024/4);
//															starttime=millis();							
													}
													else
													{
																nextMenu = home_screen;
																card.sdprinting=false;
																Discard_Buf_and_Block();
																Abnormal_Flag = 0x03;
													}
												break;	
								case 1: nextMenu = home_screen;
//										    card.sdprinting=false;
//									      Discard_Buf_and_Block();
//												Abnormal_Flag = 0x03;
												fgcode_fptr_flag = 0x00;
//												card_closefile();	
									      SPI_Flash_Erase_Sector(FGCODE_FPTR_ADDRESS/1024/4);
												SPI_Flash_Erase_Sector(FILE_NAME_ADDRESS/1024/4);
												break;												
								default: break;
						}
						
			}
		}
		else
		{
				for(i=0;i<key_num;i++)
				{	
					btn_delete(screen_key_group[i]);
				}
				gui_memin_free(screen_key_group);//�ͷ���
				
		}
		
}

/***********************************
	���ܣ�xyz�ͻ����ֶ���ƽ���̽���
  ���أ� ��
************************************/
void Bed_level_xyz_screen (bool state)
{ 
	u8 selx=0XFF;
	u8 i;
	char tempbuff[20];
	u8 key_num=3;
	//char points[20];
	u8 xi=5;
	static u8 steps=0;
  u16	btnwidth=0,btnheight=0;	
	if(lcddev.id==0x9341)
	{
		btnwidth=100;
		btnheight=45;
		xi=5;
	}
  if(lcddev.id==0x9486)
	{
		btnwidth=145;
		btnheight=60;
		xi=10;		
	}	

//	float data_temp;
	if(redraw_screen==true)				//�ػ�����
		{ 
			redraw_screen=false;			//�ر��ػ�	
			//������	
			gui_fill_rectangle(0,0,lcddev.width,lcddev.height,GUI_BODY_BACKCOL);//��䱳��ɫ	
			gui_fill_rectangle(0,0,lcddev.width,lcddev.height/8-5,GUI_TITLE_BACKCOL);//��䱳��ɫ			
			gui_show_strmid(0,0,lcddev.width,lcddev.height/8,GUI_ICON_FONTCOL,16,(u8*)TOOLS_MENU[gui_phy.language][2],1);//��ʾ����	һ����ƽ
			
      gui_fill_rectangle(xi,lcddev.height/8,btnwidth,lcddev.height-2*xi-btnheight-lcddev.height/8,BLUECOL);  //�������1˵����
			gui_show_strmid(xi,40,btnwidth,20,GUI_ICON_FONTCOL,16,(u8*)"Instruction:",1);				  
		  gui_show_strmid(xi,70,btnwidth,20,WHITE,12,(u8*)"Click Next Btn!",1);
			gui_show_strmid(xi,90,btnwidth,20,WHITE,12,(u8*)"Waiting...",1);
		
			
		  gui_fill_rectangle(2*xi+btnwidth,lcddev.height/8,2*btnwidth+xi,lcddev.height-2*xi-btnheight-lcddev.height/8,WHITE); 
		  for(i=0;i<4;i++)
			 gui_fill_circle(btnwidth+4*xi+(i%2)*(2*btnwidth-5*xi),lcddev.height/8+2*xi+(i/2)*(lcddev.height-6*xi-btnheight-lcddev.height/8),xi,GRAY);
			
			screen_key_group =(_btn_obj **)gui_memin_malloc(sizeof(_btn_obj *)*key_num);	//����һ�鰴ť		
			if(screen_key_group)
				{
					  screen_key_group[0]=btn_creat(btnwidth+2*xi,lcddev.height-xi-btnheight,btnwidth,btnheight,0,4);
					  screen_key_group[1]=btn_creat(xi,lcddev.height-xi-btnheight,btnwidth,btnheight,1,BTN_TYPE_PIC);						
	          screen_key_group[2]=btn_creat(2*btnwidth+3*xi,lcddev.height-xi-btnheight,btnwidth,btnheight,2,BTN_TYPE_PIC);
					  					
					  screen_key_group[0]->arcbtnr=0;
						screen_key_group[0]->bkctbl[0] = ORANGECOL;//�ɿ�ʱ�ı���ɫ
						screen_key_group[0]->bkctbl[1] = GRAY;	//����ʱ�ı���ɫ
						screen_key_group[0]->caption="Next";				
						if(lcddev.id==0x9341)
						{
					  screen_key_group[1]->picbtnpathu=MOVE_STOP2;
					  screen_key_group[1]->picbtnpathd=MOVE_STOP3;
						screen_key_group[2]->picbtnpathu=HEAT_BACK2;
					  screen_key_group[2]->picbtnpathd=HEAT_BACK3;
 	          }else if(lcddev.id==0x9486)
						{
						screen_key_group[1]->picbtnpathu=MOVE_STOP0;
					  screen_key_group[1]->picbtnpathd=MOVE_STOP1;
            screen_key_group[2]->picbtnpathu=HEAT_BACK0;
					  screen_key_group[2]->picbtnpathd=HEAT_BACK1;							
						}
						
						for(i=0;i<key_num;i++)
						{
							btn_draw(screen_key_group[i]);
						}

				}					
		}	//�ػ��������		
				
	if(state)  //������ʾ
	{
		selx=screen_key_chk(screen_key_group,&in_obj,key_num);
		if(selx&(1<<6))//����ֵ��Ч ������Ӧ
		{	
			  switch(selx & ~(3<<6))//
					{  
						  case 0:                           								
//			               gui_fill_circle(btnwidth+4*xi+(steps%2)*(2*btnwidth-5*xi),lcddev.height/8+2*xi+(steps/2)*(lcddev.height-6*xi-btnheight-lcddev.height/8),xi,REDCOL);	                   
										  if(steps<5)
											 {												 
												 if(steps==0)
												 {													 
													 menu_action_gcode("G1 Z10");
													 menu_action_gcode("G1 X20 Y20 F1000");
													 menu_action_gcode("G28 Z0");
												 }
												 else if(steps==1) 
												 {
													 gui_fill_rectangle(xi,90,btnwidth,20,BLUECOL);//�������
													 gui_show_strmid(xi,90,btnwidth,20,WHITE,12,(u8*)"Point1: OK",1);	
													 gui_show_strmid(xi,110,btnwidth,20,WHITE,12,(u8*)"Waiting...",1);													 
													 menu_action_gcode("G1 Z10");
													 gui_fill_circle(btnwidth+4*xi,lcddev.height/8+2*xi,xi,REDCOL);
													 sprintf(tempbuff,"G1 X%f Y20",x_max_pos-20);
													 menu_action_gcode(tempbuff);
													 menu_action_gcode("G28 Z0");
												 }
												 else if(steps==2) 
												 {
													 menu_action_gcode("G1 Z10");
												 gui_fill_circle(btnwidth+4*xi+(2*btnwidth-5*xi),lcddev.height/8+2*xi,xi,REDCOL);
													 sprintf(tempbuff,"G1 X%f Y%f",x_max_pos-20,y_max_pos-20);
													 menu_action_gcode(tempbuff);
													 menu_action_gcode("G28 Z0");
													 gui_fill_rectangle(xi,110,btnwidth,20,BLUECOL);
													 gui_show_strmid(xi,110,btnwidth,20,WHITE,12,(u8*)"Point2: OK",1);
													 gui_show_strmid(xi,130,btnwidth,20,WHITE,12,(u8*)"Waiting...",1);
												 }
												 else if(steps==3)
												 {	
													 menu_action_gcode("G1 Z10");
                          gui_fill_circle(btnwidth+4*xi+(2*btnwidth-5*xi),\
												  lcddev.height/8+2*xi+(lcddev.height-6*xi-btnheight-lcddev.height/8),xi,REDCOL);		
                           sprintf(tempbuff,"G1 X20 Y%f",y_max_pos-20);													 
												    menu_action_gcode(tempbuff);
													  menu_action_gcode("G28 Z0");
													 gui_fill_rectangle(xi,130,btnwidth,20,BLUECOL);
                          gui_show_strmid(xi,130,btnwidth,20,WHITE,12,(u8*)"Point3: OK",1);	
													gui_show_strmid(xi,150,btnwidth,20,WHITE,12,(u8*)"Waiting...",1);													 
												 }else if(steps==4)
                          {
													 gui_fill_circle(btnwidth+4*xi,\
												   lcddev.height/8+2*xi+(lcddev.height-6*xi-btnheight-lcddev.height/8),xi,REDCOL);	
														menu_action_gcode("G28");
														gui_fill_rectangle(xi,150,btnwidth,20,BLUECOL);														
													 gui_show_strmid(xi,150,btnwidth,20,WHITE,12,(u8*)"Point4: OK",1);
													 gui_fill_rectangle(xi,70,btnwidth,20,BLUECOL);	//���click next btn
														screen_key_group[0]->caption="Finish";
														btn_draw(screen_key_group[0]);
													}                												 
						               steps++;
												}else{
														steps=0;																												
													  stop_brake_time=millis(); 
														nextMenu=tool_screen;												
													}																
											 break;	
							case 1:  
                       stop_brake_time=millis(); 
                       break;													
							case 2: steps=0;	
											stop_brake_time=millis(); 
								      nextMenu=tool_screen;                           
											 break;					
					   	default: break;
					}

		}
	}
	else
		{ 
			for(i=0;i<key_num;i++)
			{	
				if(i==0)
				{	gui_memin_free(screen_key_group[i]->caption);//�ͷŰ�ť�����ڴ�
				}
				btn_delete(screen_key_group[i]);
			}
			gui_memin_free(screen_key_group);//�ͷ���
		}
	
}
/***********************************
	���ܣ�һ����ƽ���̽���
  ���أ� ��
************************************/
void Bed_level_screen (bool state)
{ 
	u8 selx=0XFF;
	u8 i;
	u8 key_num=1;	
	u16 left = (lcddev.width-320)/2;
	u16 top = (lcddev.height-240)/2;	

	//u16 offset=(lcddev.width-lcddev.height)/4;

//	float data_temp;
	if(redraw_screen==true)				//�ػ�����
		{ 
			redraw_screen=false;			//�ر��ػ�	
			//������	
			gui_fill_rectangle(0,0,lcddev.width,lcddev.height/8,GUI_TITLE_BACKCOL);//��䱳��ɫ
			gui_fill_rectangle(0,lcddev.height/8,lcddev.width,lcddev.height-lcddev.height/8,WHITE);
			//gui_fill_rectangle(0,0,lcddev.width,40,LIGHTBLUE);  //�������1	
			//gui_draw_hline(0,40,lcddev.width,GRAY);//���ֽ���
			gui_show_strmid(0,0,lcddev.width,lcddev.height/8,WHITE,16,(u8*)TOOLS_MENU[gui_phy.language][2],1);//��ʾ����	һ����ƽ


		    gui_draw_arc(37+left,48+top,221+left,232+top,129+left,140+top,92,REDCOL,0);
			//gui_draw_arc(offset+left,top,3*left-offset,3*top,lcddev.width/2,lcddev.height/2,lcddev.height/2,REDCOL,0);
			
			screen_key_group =(_btn_obj **)gui_memin_malloc(sizeof(_btn_obj *)*key_num);	//����һ�鰴ť		
			if(screen_key_group)
				{
						screen_key_group[0]=btn_creat(lcddev.width-50,lcddev.height/2-20,45,45,0,4);
					
							  screen_key_group[0]->arcbtnr=10;
								screen_key_group[0]->bkctbl[0]=GUI_TITLE_BACKCOL;
								screen_key_group[0]->bkctbl[1]=BLUE;
								screen_key_group[0]->caption="X";											
 	
						for(i=0;i<key_num;i++)
						{
							btn_draw(screen_key_group[i]);
						}

				}					
		}	//�ػ��������		
				
	if(state)  //������ʾ
	{
		selx=screen_key_chk(screen_key_group,&in_obj,key_num);
		if(selx&(1<<6))//����ֵ��Ч ������Ӧ
		{	
			  switch(selx & ~(3<<6))//
					{  
						  case 0:   
                        stop_brake_time=millis(); 
											 break;	
												
					   	default: break;
					}

		}
	}
	else
		{ 
			for(i=0;i<key_num;i++)
			{	
				gui_memin_free(screen_key_group[0]->caption);
				btn_delete(screen_key_group[i]);
			}
			gui_memin_free(screen_key_group);//�ͷ���
		}
	
}


/***********************************
	���ܣ�һ����ƽ��ɽ���
  ���أ� ��
************************************/
void Bed_level_finish_screen (bool state)
{	
		u8 selx=0XFF;
		u8 i;
		u8 key_num = 1; 
	
	  u16 width =lcddev.width/2;
		u16 height =lcddev.height/2;	
		u16 left = lcddev.width/4;
		u16 top = lcddev.height/4;	
    u16 btnwidth=(width-20)/2;
    u16 btnheight=(btnwidth-10)/2;	
		if(redraw_screen==true)				//�ػ�����
		{
			
				redraw_screen=false;			//�ر��ػ�		
				//����
				gui_draw_arcrectangle(left,top,width,33,6,1,BLUECOL,BLUECOL);//��������
				gui_show_strmid(left,top,width,30,BLACK,16,(u8*)TOOLS_MENU[gui_phy.language][3],1);//��ʾ����	
				gui_fill_rectangle(left,top+30,width,height-30,WHITE );//������
				gui_draw_rectangle(left+1,top+30,width-2,height-29,BLUECOL);//������
        gui_draw_rectangle(left+2,top+30,width-4,height-28,BLUECOL);//������
		
				
				screen_key_group =(_btn_obj **)gui_memin_malloc(sizeof(_btn_obj *)*key_num);	//����һ�鰴ť
				if(screen_key_group)
				{

						screen_key_group[0]=btn_creat((lcddev.width-btnwidth)/2,3*lcddev.height/4-btnheight-5,btnwidth,btnheight,0,4);//�����ֵ�ͼ��

							
								screen_key_group[0]->bkctbl[0]=ORANGECOL;
							  screen_key_group[0]->bkctbl[1]=WHITE;
							


							screen_key_group[0]->caption=(u8*)GUI_FINISH_CAPTION_TBL[gui_phy.language];

							
							for(i=0;i<key_num;i++)
							{
								btn_draw(screen_key_group[i]);
							}
					}
			
		}
 		if(state) //������ʾ
		{
			selx=screen_key_chk(screen_key_group,&in_obj,key_num);	
			if(selx&(1<<6))//����ֵ��Ч ������Ӧ
			{	
				switch(selx & ~(3<<6))//
						{   
								case 0:	
												bed_level_switch = 1;//��ƽ��ɺ�򿪵�ƽ����
												calculate_delta(current_position);//����current_position��Ӧ��delta
												adjust_delta(current_position);
												plan_set_position(delta[X_AXIS], delta[Y_AXIS], delta[Z_AXIS], current_position[E_AXIS]);
												FLASH_WRITE_VAR(FLASH_SET_STORE_OFFSET+183,bed_level_switch);
												nextMenu = tool_screen;		
					              quickStop();
												break;			
								default: break;
						}
			}
		}
		else
		{		
			for(i=0;i<key_num;i++)
			{	
				btn_delete(screen_key_group[i]);
			}
			gui_memin_free(screen_key_group);//�ͷ���
					
		}
}



/***********************************
	���ܣ��¶��쳣����
  ���أ� ��
************************************/
//void Temperature_error_screen(bool state)
//{
//	  static u8 temp_error_cnt=0;
//		u16 width =lcddev.width/2;
//		u16 height =lcddev.height/2;	
//		u16 left = lcddev.width/4;
//		u16 top = lcddev.height/4;	 
//       if(redraw_screen==true)				//�ػ�����
//		{ 
//			redraw_screen=false;			//�ر��ػ�
//	        gui_fill_rectangle(0,0,lcddev.width,lcddev.height,GUI_BODY_BACKCOL);//��䱳��ɫ
//			gui_fill_rectangle(left,top,width,height,WHITE );//������
//			gui_draw_rectangle(left-1,top-1,width+2,height+2,BLUECOL);//����
//			gui_draw_rectangle(left-2,top-2,width+4,height+4,BLUECOL);//����
//			
//			gui_show_strmid(left,top+10,width,40,RED,16,(u8*)TEMP_ERROR_MENU[gui_phy.language][0],1);
//			gui_show_strmid(left,top+50,width,40,RED,16,(u8*)TEMP_ERROR_MENU[gui_phy.language][1],1);
//			
//			//beep_enable();
//		}			
//		temp_error_cnt++;
//		if(temp_error_cnt>=60)
//		{
//		 temp_error_cnt=0;
//		 nextMenu = lastMenu;	
//		}
//}



/***********************************
	���ܣ�δ�ﵽ�����¶�
  ���أ� ��
************************************/
void cold_extrude_screen(bool state)
{
	  static u8 cold_extrude_cnt=0;
		u16 width =lcddev.width/2;
		u16 height =lcddev.height/2;	
		u16 left = lcddev.width/4;
		u16 top = lcddev.height/4;	 
     if(redraw_screen==true)				//�ػ�����
		{ 
			redraw_screen=false;			//�ر��ػ�
	    gui_fill_rectangle(0,0,lcddev.width,lcddev.height,GUI_BODY_BACKCOL );//��䱳��ɫ
			gui_fill_rectangle(left,top+lcddev.height/8,width,height-lcddev.height/8,WHITE );//������
			gui_fill_rectangle(left,top,width,lcddev.height/8,BLUECOL);
			gui_draw_rectangle(left-1,top-1,width+2,height+2,BLUECOL);//����
            gui_draw_rectangle(left-2,top-2,width+4,height+4,BLUECOL);//����
			
			gui_show_strmid(left,top+10+lcddev.height/8,width,40,RED,16,(u8*)ERR_COLD_MENU[gui_phy.language][0],1);
			gui_show_strmid(left,top+50+lcddev.height/8,width,40,RED,16,(u8*)ERR_COLD_MENU[gui_phy.language][1],1);
			//beep_enable();
			
			//jee2018----��ӡ�������¶��쳣��ֹͣ
			if(card.sdprinting)
			{
				card.sdprinting=false;
				Discard_Buf_and_Block();

				Abnormal_Flag = 0x03;	
			}
		}
		
		cold_extrude_cnt++;
		if(cold_extrude_cnt>=60)
		{
		 cold_extrude_cnt=0;
		 nextMenu = lastMenu;	
		}
}




//BMP���뺯��
//����ǰLCD��Ļ��ָ�������ͼ,��Ϊ16λ��ʽ��BMP�ļ� RGB565��ʽ.
//����Ϊrgb565����Ҫ����,��Ҫ����ԭ���ĵ�ɫ��λ����������.���������Ѿ�����������.
//����Ϊrgb555��ʽ����Ҫ��ɫת��,��ʱ��ȽϾ�,���Ա���Ϊ565������ٵİ취.
//filename:���·��
//x,y:����Ļ�ϵ���ʼ����  
//mode:ģʽ.0,�����������ļ��ķ�ʽ����;1,���֮ǰ�����ļ�,�򸲸�֮ǰ���ļ�.���û��,�򴴽��µ��ļ�.
//����ֵ:0,�ɹ�;����,������.  
//void bmp_screen_shot(u8 shot_cnt)
//{				
//	u8 *screen_shot=0;
//	char *screen_shot_path=0;
//	char *screen_shot_name=0;
//	char *screen_shot_num=0;
//	
//	screen_shot=gui_memin_malloc(40);	//����40���ֽ��ڴ�,����"0:PAINT/PAINT20120321210633.bmp"
//	screen_shot_path=gui_memin_malloc(20);	//����40���ֽ��ڴ�,����"0:PAINT/PAINT20120321210633.bmp"
//	screen_shot_name=gui_memin_malloc(20);	//����40���ֽ��ڴ�,����"0:PAINT/PAINT20120321210633.bmp"
//	screen_shot_num=gui_memin_malloc(3);	//����40���ֽ��ڴ�,����"0:PAINT/PAINT20120321210633.bmp"
//	sprintf(screen_shot_num,"%3d",shot_cnt);
//	screen_shot_name = "-screen_shot.bmp";
//	strcat((char *)screen_shot_num,(const char*)screen_shot_name);		//�������������
//	screen_shot_path = "0:\\SCREEN";
//	screen_shot=gui_path_name(screen_shot,(u8*)screen_shot_path,(u8*)screen_shot_num);	//�ļ�������·��
//	if(!bmp_encode(screen_shot,0,0,lcddev.width,lcddev.height,1))//����BMP�ļ�
//	{
//		printf("screen shot OK\r\n");
//	}	
//	gui_memin_free(screen_shot);
//	gui_memin_free(screen_shot_path);
//	gui_memin_free(screen_shot_name);
//	gui_memin_free(screen_shot_num);

//}
/***********************************
	���ܣ���ĻУ׼����
  ���أ� ��
************************************/
void Tp_Adjust_screen(bool state)
{
	if(state)
	{
		LCD_Clear(WHITE);//����
		TP_Adjust();  //��ĻУ׼ 
    TP_Get_Adjdata();			
//		beep();
		nextMenu = home_screen;
	}
}

//��ӡ���
void end_print_screen(bool state)
{
  u8 selx=0XFF;
	u8 i;
	u8 key_num=1;	
	u16 left = lcddev.width/4;
	u16 top = lcddev.height/4;	
  u16 width = lcddev.width-left*2;
	u16 height = lcddev.height-top*2;
	char TempBuffer[20];
	if(redraw_screen==true)				//�ػ�����
		{ 
			redraw_screen=false;			//�ر��ػ�	
			//������	
			gui_fill_rectangle(left,top,width,lcddev.height/8,BLUECOL);//��䱳��ɫ
			gui_fill_rectangle(left,top+lcddev.height/8,width,height-lcddev.height/8,WHITE);
      gui_draw_rectangle(left-1,top,width+2,height+1,BLUECOL);//����	
			
			gui_show_strmid(left,top+lcddev.height/8,width,20,BLUECOL,16,(u8*)CONFIRM_CANCEL_LABE[gui_phy.language][1],1);//��ʾ����	
	   //��ӡʱ��
			sprintf(TempBuffer, "%3d:%02d:%02d",print_time/3600,(print_time/60)%60,print_time%60);
			gui_show_strmid(left,top+lcddev.height/8+20,width,20,ORANGECOL,16,(u8*)TempBuffer,1);
			
			screen_key_group =(_btn_obj **)gui_memin_malloc(sizeof(_btn_obj *)*key_num);	//����һ�鰴ť		
			if(screen_key_group)
				{
						screen_key_group[0]=btn_creat(lcddev.width/2-40,top+height-lcddev.height/8-5,80,lcddev.height/8,0,4);
					
							  screen_key_group[0]->arcbtnr=4;
								screen_key_group[0]->bkctbl[0]=ORANGECOL;
								screen_key_group[0]->bkctbl[1]=BLUECOL;
								screen_key_group[0]->caption=(u8*)GUI_FINISH_CAPTION_TBL[gui_phy.language];											
 	
						for(i=0;i<key_num;i++)
						{
							btn_draw(screen_key_group[i]);
						}

				}					
		}	//�ػ��������		
				
	if(state)  //������ʾ
	{
		selx=screen_key_chk(screen_key_group,&in_obj,key_num);
		if(selx&(1<<6))//����ֵ��Ч ������Ӧ
		{	
			  switch(selx & ~(3<<6))//
					{  
						  case 0:   
                      nextMenu = home_screen; 
							         print_time=0;//���						       
											 break;	
												
					   	default: break;
					}

		}
	}
	else
		{ 
			for(i=0;i<key_num;i++)
			{	
				gui_memin_free(screen_key_group[0]->caption);
				btn_delete(screen_key_group[i]);
			}
			gui_memin_free(screen_key_group);//�ͷ���
		}

}

//����ѡ�����
void gui_set_language(bool state)
{
  u8 selx=0XFF;
	u8 i;
	u8 res=0xff;
	u8 key_num=3;
   static u8 listsel=0;
	 static u8 listtop=0; 

  if(windows_flag==false)
	{
	if(redraw_screen==true)				//�ػ�����
		{ 
			redraw_screen=false;			//�ر��ػ�	
      //������	
			gui_fill_rectangle(0,0,lcddev.width,lcddev.height,WHITE );//��䱳��ɫ
			gui_fill_rectangle(0,0,lcddev.width,40,GREENCOL);  //�������1	
	    gui_show_strmid(0,0,lcddev.width,40,GUI_ICON_FONTCOL,16,(u8*)SYSTEM_MENU[gui_phy.language][2],1);//��ʾ����	
			//������							
			screen_key_group =(_btn_obj **)gui_memin_malloc(sizeof(_btn_obj *)*key_num);	//����һ�鰴ť	
			tlistbox=listbox_creat(4,40,lcddev.width-6,LBOX_ITEM_HEIGHT*5,0x80,16);	//����һ��listbox
     if(tlistbox&&screen_key_group)
		 {
			  screen_key_group[0]=btn_creat(5,5,30,30,0,BTN_TYPE_SGICON_TEXT);//�����ֵ�ͼ��                      								
				screen_key_group[0]->picbtnpathu=SYSTEM_BACK;	 	//�ɿ�ʱ��ͼ��
				screen_key_group[0]->picbtnpathd=SYSTEM_BACK;		//����ʱ��ͼ��
				screen_key_group[0]->bkctbl[0]=GREENCOL;//�ɿ�ʱ����ɫ
				screen_key_group[0]->bkctbl[1]=GREENCOL;//����ʱ����ɫ
				screen_key_group[0]->bkctbl[2]=GUI_ICON_FONTCOL;//�ɿ�ʱͼ����ɫ
				screen_key_group[0]->bkctbl[3]=GUI_TITLE_BACKCOL;//����ʱͼ����ɫ
			  btn_draw(screen_key_group[0]);
			 
			 for(i=0;i<GUI_LANGUAGE_NUM;i++)
			 {       			 
			 res=listbox_addlist(tlistbox,(u8*)LANGUAGE_MENU[gui_phy.language][i]);
       	
			  if(res)break;
			 }
			 if(res==0)//��ӳɹ�
			 {
				tlistbox->scbv->topitem = listtop;//������
			  tlistbox->selindex = gui_phy.language;
			  listbox_draw_listbox(tlistbox);
			 }
		 }

    }
   if(state)  //������ʾ
	 {	
		selx=screen_key_chk(screen_key_group,&in_obj,key_num);  
		if(selx&(1<<6))//����ֵ��Ч ������Ӧ
		{	
			  switch(selx & ~(3<<6))//
					{  
						  case 0:   
                      nextMenu = settinglist_screen; 
							         					       
											 break;	
												
					   	default: break;
					}

		}
		selx=listbox_check(tlistbox,&in_obj);//ɨ��
		if(tlistbox->dbclick&0X80)
		{
				tlistbox->dbclick=0x00;
				listtop = tlistbox->scbv->topitem ;
        listsel = tlistbox->selindex;			
//			  FLASH_WRITE_VAR(FLASH_SET_STORE_OFFSET+151,gui_phy.language);	
			redraw_screen=true;
			windows_flag=true; //���봰��ģʽ
		}
	 }
	else
		{ 
	
			btn_delete(screen_key_group[0]);		
			gui_memin_free(screen_key_group);//�ͷ���
			listbox_delete(tlistbox);
		}

 }else
	{
	if(redraw_screen==true)				//�ػ�����
		{ 
			redraw_screen=false;			//�ر��ػ�
			//������
			gui_draw_arcrectangle(lcddev.width/4,lcddev.height/4,lcddev.width/2,lcddev.height/2,4,1,GREENCOL,GREENCOL);			
			gui_draw_arcrectangle(lcddev.width/4+2,lcddev.height/4+lcddev.height/8,lcddev.width/2-4,lcddev.height/2-lcddev.height/8-2,4,1,WHITE,WHITE);
			  
			gui_show_strmid(lcddev.width/4,lcddev.height/4+5+lcddev.height/8,lcddev.width/2,20,GREENCOL,16,(u8*)LANGUAGE_MENU[gui_phy.language][listsel],1);//��ʾ
			if(screen_key_group)
			{
				for(i=1;i<key_num;i++)
			 {	
				screen_key_group[i]=btn_creat(lcddev.width/4+5+(i-1)*(lcddev.width/2-60),lcddev.height*3/4-5-30,50,30,i,BTN_TYPE_TEXTB);//
									 
				 screen_key_group[i]->bkctbl[0]= ORANGECOL;
				 screen_key_group[i]->bkctbl[1]= ORANGECOL;	 
			 }
			 screen_key_group[1]->caption=(u8*)GUI_OK_CAPTION_TBL[gui_phy.language];
			 screen_key_group[2]->caption=(u8*)GUI_CANCEL_CAPTION_TBL[gui_phy.language];
			 for(i=1;i<key_num;i++)
				{	btn_draw(screen_key_group[i]);
				}
			}
		}
	  if(state)  //������ʾ
		{
			 selx=screen_key_chk(screen_key_group,&in_obj,key_num);
			if(selx&(1<<6))//����ֵ��Ч ������Ӧ
				{	
					switch(selx & ~(3<<6))//				
 							{  
						    case 1:  //ȷ��
										windows_flag=false; //�˳�����ģʽ
										gui_phy.language = listsel;			
										FLASH_WRITE_VAR(FLASH_SET_STORE_OFFSET+151,gui_phy.language);
								    nextMenu = settinglist_screen;
//										listbox_draw_listbox(tlistbox);//�ػ�								
									break;
								case 2:
									windows_flag=false; //�˳�����ģʽ
								 	for(i=1;i<key_num;i++)
										{	
											btn_delete(screen_key_group[i]);
										}
										listbox_draw_listbox(tlistbox);
									break;
								default:break;
							}
				}
		}else
		{
	  	windows_flag=false;	
				for(i=0;i<key_num;i++)
				{	btn_delete(screen_key_group[i]);
				}
				gui_memin_free(screen_key_group);//�ͷ��ڴ�	 
				listbox_delete(tlistbox);	//ɾ��
		}
 }
}




