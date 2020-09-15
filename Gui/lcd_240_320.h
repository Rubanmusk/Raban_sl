#ifndef __LCD_240_320_H
#define __LCD_240_320_H 			 
#include "lcdmenu.h"

void enter_data_input_240_320(void * date_address ,u8 type,double up_limit,double low_limit,bool judge,u32 store_address);
void data_input_screen_240_320(bool state);

void home_screen_240_320(bool state);	//主页
void gecodelist_screen_240_320(bool state);
void sdprint_screen_240_320 (bool state);
void sdprintmore_screen_240_320 (bool state);//
void manual_screen_240_320 (bool state);	
void preheat_head_1_screen_240_320 (bool state);	
void preheat_bed_screen_240_320(bool state);
void tool_screen_240_320(bool state);	//工具

void system_screen_240_320(bool state);//系统
void settinglist_screen_240_320(bool state);
void Restore_factory_Setting_screen_240_320(bool state);
void movelist_screen_240_320(bool state);
void templist_screen_240_320(bool state);
void set_print_size_screen_240_320 (bool state);
void set_max_speed_screen_240_320 (bool state);
void set_direction_screen_240_320 (bool state);
void set_steps_per_uint_screen_240_320 (bool state);
void set_endswitch_step_1_screen_240_320 (bool state);
void set_endswitch_step_2_screen_240_320 (bool state);
void set_max_temp_screen_240_320 (bool state);
void set_target_temp_screen_240_320 (bool state);
void about_screen_240_320(bool state);

#endif

















