#ifndef __LCDMENU_H
#define __LCDMENU_H 			 
#include "touch.h"
#include "guix.h" //��Ӣ�ı�־λ�Ľṹ��
#include "gui.h"
#include "fontupd.h"
#include "text.h"
#include "temperature.h"
#include "gcodeplayer.h"
#include "button.h"
#include "filelistbox.h"
#include "gcodeplayer.h"
#include "stepper.h"
#include "graph.h"
#include "listbox.h"
#include "flash.h"
#include "icon_code.h"
#include "label_code.h"
#include "Abnormal_Handling.h"


#define U8_TYPE 0
#define U16_TYPE 1
#define U32_TYPE 2
#define INT_TYPE 3
#define FLOAT_TYPE 4
#define DOUBLE_TYPE 5


#define USER_DATA_ADDR   (u32)5242880 //5*1024*1024 �û������׵�ַ

//��ֵ����ṹ��.
__packed typedef struct 
{
	void *address;  		//���޸����ݵĵ�ַ
	u8 type;    //�������ݵ�����
	double up_limit;
	double low_limit;
	bool judge;
	u32 store_address;
}_data_input;

#define ICOS_SD_CARD      	0				//SD��
#define ICOS_FLASH_DISK		1				//FLASH DISK
#define ICOS_LOCATION		0				//����ͼ������λ��,���Ϊ0,���������ͼ������SD��,���Ϊ1,���������ͼ������flash disk	    

#define SCREEN_SCAN_TIME  50  //��Ļ����ʱ�� ms


extern float heater_temp[EXTRUDERS];
extern float heater_0_temp;//20160409
extern float bed_temp;

extern char fnameDirPath[100];
extern u8 Password_set_flag;

typedef void (*menuFunc_t)(bool);
extern menuFunc_t currentMenu;
extern menuFunc_t nextMenu;
extern bool again_print_flag;
extern bool lcd_reset_flag;

void lcd_update(void);
void menu_action_gcode(const char* pgcode);

void enter_data_input(void * date_address ,u8 type,double up_limit,double low_limit,bool judge,u32 store_address);
void data_input_screen(bool state);

void home_screen(bool state);	//��ҳ
void gecodelist_screen(bool state);
void sdprint_screen (bool state);
void sdprintmore_screen (bool state);//����
void manual_screen (bool state);	
void preheat_head_1_screen (bool state);	
void preheat_bed_screen(bool state);
void tool_screen(bool state);	//����

void system_screen(bool state);//ϵͳ
void settinglist_screen(bool state);
void Restore_factory_Setting_screen(bool state);

void set_print_size_screen (bool state);
void set_delta_screen (bool state);
void set_max_speed_screen (bool state);

void set_PID_screen (bool state);
void set_direction_screen (bool state);
void set_steps_per_uint_screen (bool state);
void set_endswitch_step_1_screen (bool state);
void set_endswitch_step_2_screen (bool state);
void set_max_temp_screen (bool state);
void set_target_temp_screen (bool state);
void about_screen(bool state);


void shutdown_screen(bool state);
void failed_to_shutdown_screen(bool state);

void Power_Off_screen(bool state);
void Material_over_screen (bool state);
void Material_ok_screen(bool state);//20160412

void Confirm_cancel_screen(bool state);


void Continue_Print_screen(bool state);//20160412

void energy_saving_screen(bool state);
void Language_Selection(bool state);//20160412
void Machine_Type_Selection(bool state);
void Temp_Sensor_Selection(bool state);

void Password_set_screen(bool state);
void Password_screen(bool state);

void Bed_level_screen (bool state);
void Bed_level_finish_screen (bool state);


void Temperature_error_screen(bool state);
void cold_extrude_screen(bool state);

void bmp_screen_shot(u8 shot_cnt);
void Tp_Adjust_screen(bool state);

void UpdateOK_screen(bool state);
void UpdateUsart_screen(bool state);
void Bed_level_xyz_screen (bool state);
//void DISPLAY_RENCODE_TO_TFT(u8 *qrcode_data);
//void wifi_screen(bool state);
void end_print_screen(bool state);
void gui_set_language(bool state);
#endif

















