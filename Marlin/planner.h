/*
  planner.h - buffers movement commands and manages the acceleration profile plan
  Part of Grbl

  Copyright (c) 2009-2011 Simen Svale Skogsrud

  Grbl is free software: you can redistribute it and/or modify
  it under the terms of the GNU General Public License as published by
  the Free Software Foundation, either version 3 of the License, or
  (at your option) any later version.

  Grbl is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
  GNU General Public License for more details.

  You should have received a copy of the GNU General Public License
  along with Grbl.  If not, see <http://www.gnu.org/licenses/>.
*/

// This module is to be considered a sub-module of stepper.c. Please don't include 
// this file from any other module.

#ifndef planner_h
#define planner_h

#include "Marlin.h"
//#define SAVE_ADDR_BASE                (5*1024+24)*1024   //触摸数据touch.c
//
#define FILE_NAME_ADDRESS 		             (5*1024+20)*1024	//  unsigned long//fgcode_fptr; //20180115
#define FILE_GCODE_PATH_ADDRESS 		        FILE_NAME_ADDRESS+64	
//
#define FACTORY_STORE_OFFSET               (5*1024+16)*1024///20161207  //恢复出厂设置数据存储地址
//wifi
#define WIFI_ACCOUNT_STORE_OFFSET          (5*1024+12)*1024  //wifi账号存储地址
#define WIFI_PASSWORD_STORE_OFFSET         (5*1024+12)*1024+32*1//wifi密码存储地址
//#define SERVER_IP_STORE_OFFSET              (5*1024+12)*1024+32*2
//#define SERVER_PORTNUM_STORE_OFFSET         (5*1024+12)*1024+32*3
//
#define BED_LEVEL_STORE_OFFSET             (5*1024+8)*1024///20161207//调平数据存储地址
//#define BED_LEVEL_STORE_OFFSET               (5*1024+8)*1024
//
#define FLASH_POWER_OFF_STORE_OFFSET         (5*1024+4)*1024//掉电数据存储地址
#define FGCODE_FPTR_ADDRESS 		             FLASH_POWER_OFF_STORE_OFFSET	//  unsigned long//fgcode_fptr; //20160506
//#define FGCODE_CLUST_ADDRESS 		             FLASH_POWER_OFF_STORE_OFFSET+4	//  unsigned long//fgcode_clust; //20160506
//#define FGCODE_DSECT_ADDRESS 		             FLASH_POWER_OFF_STORE_OFFSET+8	//  unsigned long//fgcode_dsect; //20160506
#define FILE_SELINDEX_ADDRESS 		           FLASH_POWER_OFF_STORE_OFFSET+4	//u8*32 //card.filename//20160506
#define X_STOP_STEPS_ADDRESS 		             FLASH_POWER_OFF_STORE_OFFSET+6//unsigned long
#define Y_STOP_STEPS_ADDRESS 		             FLASH_POWER_OFF_STORE_OFFSET+10//unsigned long
#define Z_STOP_STEPS_ADDRESS 		             FLASH_POWER_OFF_STORE_OFFSET+14//unsigned long
#define E_STOP_STEPS_ADDRESS 		             FLASH_POWER_OFF_STORE_OFFSET+18//unsigned long
#define BACKUP_TEMP_ADDRESS 		             FLASH_POWER_OFF_STORE_OFFSET+22//int
#define BACKUP_FEEDRATE_ADDRESS 		         FLASH_POWER_OFF_STORE_OFFSET+26//float
#define BACKUP_BUFLEN_ADDRESS 		           FLASH_POWER_OFF_STORE_OFFSET+30//float
//jee2018
#define BACKUP_Z_HEIGHT_ADDRESS              FLASH_POWER_OFF_STORE_OFFSET+36
#define BACKUP_TARGET_TEM_ADDRESS            FLASH_POWER_OFF_STORE_OFFSET+40
#define BACKUP_BED_TARGET_TEM_ADDRESS        FLASH_POWER_OFF_STORE_OFFSET+48
#define FLASH_SET_STORE_OFFSET                5*1024*1024
#define FLASH_SET_EEPROM_VERSION "150"        //20160401





extern unsigned long minsegmenttime;
extern float max_feedrate[4]; // set the max speeds
extern float axis_steps_per_unit[4];
extern unsigned long max_acceleration_units_per_sq_second[4]; // Use M201 to override by software
extern float minimumfeedrate;
extern float acceleration;         // Normal acceleration mm/s^2  THIS IS THE DEFAULT ACCELERATION for all moves. M204 SXXXX
extern float retract_acceleration; //  mm/s^2   filament pull-pack and push-forward  while standing still in the other axis M204 TXXXX
extern float max_xy_jerk; //speed than can be stopped at once, if i understand correctly.
extern float max_z_jerk;
extern float max_e_jerk;
extern float mintravelfeedrate;
extern unsigned long axis_steps_per_sqr_second[NUM_AXIS];


extern int heater_0_maxtemp;
extern int heater_0_mintemp;
extern int bed_maxtemp;
extern float heater_0_PLAtemp;
extern float heater_0_ABStemp;
extern float x_offset_pos;
extern float y_offset_pos;
extern float x_max_pos;
extern float y_max_pos;
extern float z_max_pos;
extern bool invert_x_dir;
extern bool invert_y_dir; 
extern bool invert_z_dir;
extern bool invert_e0_dir;
extern bool x_endstops_inverting;
extern bool y_endstops_inverting; 
extern bool z_endstops_inverting;

extern bool emc_switch;

extern bool no_material_switch;
extern bool brownout_switch;

extern bool screen_saving_switch;
extern bool auto_shutdown_switch;

extern u8 temp_sensor_type;
extern u8 Machine_type;

extern float diagonal;
extern float smooth_rod;
extern float effector;
extern float carriage;

extern bool level_endstops_inverting;
extern bool Password_switch;
extern char Password_read_buffer[6];

extern bool bed_level_switch;
extern u8 temp_sensor_num;

extern u8 backup_power;

extern bool e0_endstops_inverting;

extern u32 pulse_width;
extern u32 pause_length;

extern u32 extrude_length_advance;
extern float temp_z_height;
extern float appoint_z_height;
//extern float gcode_appoint_z_height;
extern u32 print_time;
extern float floor_height;
extern float add_homeing_ofsize[3];
extern float z_homeing_ofsize;
#ifdef AUTOTEMP
    extern bool autotemp_enabled;
    extern float autotemp_max;
    extern float autotemp_min;
    extern float autotemp_factor;
#endif
// This struct is used when buffering the setup for each linear movement "nominal" values are as specified in 
// the source g-code and may never actually be reached if acceleration management is active.
typedef struct {
  // Fields used by the bresenham algorithm for tracing the line
  long steps_x, steps_y, steps_z, steps_e;  //每个坐标轴所需走的步数 Step count along each axis    各轴步进电机的脉冲数
  unsigned long step_event_count;           //完成这个block所需走的步数，上面4个的最大的那个的步数   The number of step events required to complete this block
  long accelerate_until;                    //梯形曲线中的加速距离 The index of the step event on which to stop acceleration   加速的     step
  long decelerate_after;                    //加速和匀速的距离 The index of the step event on which to start decelerating      开始减速的 step
  long acceleration_rate;                   //加速率，用来计算加速度 The acceleration rate used for acceleration calculation
  unsigned char direction_bits;             //这个block的方向位，1反向，0正向，每一个位代表一个轴的方向 The direction bit set for this block (refers to *_DIRECTION_BIT in config.h)
  unsigned char active_extruder;            //所有用到的有效挤出头 Selects the active extruder

  // Fields used by the motion planner to manage acceleration  计划管理加速度
//  float speed_x, speed_y, speed_z, speed_e;        // Nominal mm/sec for each axis
  float nominal_speed;                               //匀速速度，即梯形曲线的匀速阶段速度 The nominal speed for this block in                          mm/sec 
  float entry_speed;                                 //进入速度，梯形左边的开始入口速度，即从上一个block进入到这个block时的速度                      mm/sec
  float max_entry_speed;                             //最大进入速度，进入速度不能超过这个值 Maximum allowable junction entry speed in                  mm/sec
  float millimeters;                                 //总路程，单位mm The total travel of this block in                                                mm
  float acceleration;                                //加速度，单位mm/sec^2    acceleration                                                            mm/sec^2
  unsigned char recalculate_flag;                    //连接处重新计算梯形速度曲线的标志 Planner flag to recalculate trapezoids on entry junction       
  unsigned char nominal_length_flag;                 //能达到额定速度的标志 Planner flag for nominal speed always reached                          false是三角形，true是梯形

  // Settings for the trapezoid generator     //梯形速度曲线产生器的设置参数
  unsigned long nominal_rate;                        //这个block的匀速速度    The nominal step rate for this block in step_events/sec    steps/sec 
  unsigned long initial_rate;                        //梯形曲线的初始速度/进入速度 The jerk-adjusted step rate at start of block     steps/sec 
  unsigned long final_rate;                          //梯形曲线的退出速度 The minimal rate at exit                                     steps/sec 
  unsigned long acceleration_st;                     //加速度 acceleration                                                            steps/sec^2            
	unsigned long laser_pwm;                           //laser_PWM
  #ifdef BARICUDA
  unsigned long valve_pressure;
  unsigned long e_to_p_pressure;
  #endif
  volatile char busy;                               //正在处理这个block的标志位，1表示正在执行这个block
} block_t;

void plan_init(void);
// Add a new linear movement to the buffer. x, y and z is the signed, absolute target position in 
// millimaters. Feed rate specifies the speed of the motion.
void plan_buffer_line(const float x, const float y, const float z, const float e, float feed_rate, const uint8_t extruder);

// Set position. Used for G92 instructions.
void plan_set_position(const float x, const float y, const float z, const float e);
void plan_set_e_position(const float e);

void check_axes_activity(void);
uint8_t movesplanned(void); //return the nr of buffered moves

extern float axis_steps_per_unit[4];
// Called when the current block is no longer needed. Discards the block and makes the memory
// availible for new blocks.    
void plan_discard_current_block(void);
void plan_clear_block(void);  //这里是仅仅将block_buffer_tail和block_buffer_head清零 ，用在有些地方需要将block清空后再清零  /20160330

block_t *plan_get_current_block(void);
bool blocks_queued(void);
void reset_acceleration_rates(void);

#endif
