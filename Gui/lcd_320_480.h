#ifndef __LCD_320_480_H
#define __LCD_320_480_H 			 
#include "lcdmenu.h"


void DISPLAY_RENCODE_TO_TFT(u8 *qrcode_data);
void enter_data_input_320_480(void * date_address ,u8 type,double up_limit,double low_limit,bool judge,u32 store_address);
void data_input_screen_320_480(bool state);

void home_screen_320_480(bool state);	//主页
void gecodelist_screen_320_480(bool state);
void sdprint_screen_320_480 (bool state);
void sdprintmore_screen_320_480 (bool state);
void manual_screen_320_480 (bool state);	
void preheat_head_1_screen_320_480 (bool state);	
void preheat_bed_screen_320_480(bool state);
void tool_screen_320_480(bool state);	//工具

void system_screen_320_480(bool state);//系统
void settinglist_screen_320_480(bool state);
void Restore_factory_Setting_screen_320_480(bool state);
void movelist_screen_320_480(bool state);
void templist_screen_320_480(bool state);
void set_print_size_screen_320_480 (bool state);
void set_max_speed_screen_320_480 (bool state);
void set_direction_screen_320_480 (bool state);
void set_steps_per_uint_screen_320_480 (bool state);
void set_endswitch_step_1_screen_320_480 (bool state);
void set_endswitch_step_2_screen_320_480 (bool state);
void set_max_temp_screen_320_480 (bool state);
void set_target_temp_screen_320_480 (bool state);
void about_screen_320_480(bool state);

#endif

















