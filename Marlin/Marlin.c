
/*
    Reprap firmware based on Sprinter and grbl.
 Copyright (C) 2011 Camiel Gubbels / Erik van der Zalm

 This program is free software: you can redistribute it and/or modify
 it under the terms of the GNU General Public License as published by
 the Free Software Foundation, either version 3 of the License, or
 (at your option) any later version.												

 This program is distributed in the hope that it will be useful,
 but WITHOUT ANY WARRANTY; without even the implied warranty of
 MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 GNU General Public License for more details.

 You should have received a copy of the GNU General Public License
 along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

/*
 This firmware is a mashup between Sprinter and grbl.
  (https://github.com/kliment/Sprinter)
  (https://github.com/simen/grbl/tree)

 It has preliminary support for Matthew Roberts advance algorithm
    http://reprap.org/pipermail/reprap-dev/2011-May/003323.html
 */
#include "Marlin.h"
#include "delay.h"
//#include "ultralcd.h"
#include "planner.h"
#include "stepper.h"
#include "temperature.h"
#include "motion_control.h"
//#include "cardreader.h"
#include "watchdog.h"
#include "ConfigurationStore.h"
#include "language.h"
//#include "pins_arduino.h"
#include "usart.h"
//#include "beep.h"
#include "gcodeplayer.h"
#include "lcdmenu.h"
#include "sdio_sdcard.h"
//#include "mmc_sd.h"
#include "Abnormal_Handling.h"
//#include "hmi.h"
//#include "apsta.h"
#include "exfuns.h"
#include "guix.h"

#define VERSION_STRING  "1.0.0"

// look here for descriptions of gcodes: http://linuxcnc.org/handbook/gcode/g-code.html
// http://objects.reprap.org/wiki/Mendel_User_Manual:_RepRapGCodes

//Implemented Codes
//-------------------
// G0  -> G1
// G1  - Coordinated Movement X Y Z E
// G2  - CW ARC
// G3  - CCW ARC
// G4  - Dwell S<seconds> or P<milliseconds>
// G10 - retract filament according to settings of M207
// G11 - retract recover filament according to settings of M208
// G28 - Home all Axis
// G90 - Use Absolute Coordinates
// G91 - Use Relative Coordinates
// G92 - Set current position to cordinates given

//RepRap M Codes
// M0   - Unconditional stop - Wait for user to press a button on the LCD (Only if ULTRA_LCD is enabled)
// M1   - Same as M0
// M104 - Set extruder target temp
// M105 - Read current temp
// M106 - Fan on
// M107 - Fan off
// M109 - Wait for extruder current temp to reach target temp.
// M114 - Display current position

//Custom M Codes
// M15  - update flash data (font data ; icon data and so on)
// M16  - screen_adjust
// M17  - Enable/Power all stepper motors
// M18  - Disable all stepper motors; same as M84
// M20  - List SD card
// M21  - Init SD card
// M22  - Release SD card
// M23  - Select SD file (M23 filename.g)
// M24  - Start/resume SD print
// M25  - Pause SD print
// M26  - Set SD position in bytes (M26 S12345)
// M27  - Report SD print status
// M28  - Start SD write (M28 filename.g)
// M29  - Stop SD write
// M30  - Delete file from SD (M30 filename.g)
// M31  - Output time since last M109 or SD card start to serial
// M42  - Change pin status via gcode Use M42 Px Sy to set pin x to value y, when omitting Px the onboard led will be used.
// M80  - Turn on Power Supply
// M81  - Turn off Power Supply
// M82  - Set E codes absolute (default)
// M83  - Set E codes relative while in Absolute Coordinates (G90) mode
// M84  - Disable steppers until next move,
//        or use S<seconds> to specify an inactivity timeout, after which the steppers will be disabled.  S0 to disable the timeout.
// M85  - Set inactivity shutdown timer with parameter S<seconds>. To disable set zero (default)
// M92  - Set axis_steps_per_unit - same syntax as G92
// M114 - Output current position to serial port
// M115 - Capabilities string
// M117 - display message
// M119 - Output Endstop status to serial port
// M126 - Solenoid Air Valve Open (BariCUDA support by jmil)
// M127 - Solenoid Air Valve Closed (BariCUDA vent to atmospheric pressure by jmil)
// M128 - EtoP Open (BariCUDA EtoP = electricity to air pressure transducer by jmil)
// M129 - EtoP Closed (BariCUDA EtoP = electricity to air pressure transducer by jmil)
// M140 - Set bed target temp
// M190 - Wait for bed current temp to reach target temp.
// M200 - Set filament diameter
// M201 - Set max acceleration in units/s^2 for print moves (M201 X1000 Y1000)
// M202 - Set max acceleration in units/s^2 for travel moves (M202 X1000 Y1000) Unused in Marlin!!
// M203 - Set maximum feedrate that your machine can sustain (M203 X200 Y200 Z300 E10000) in mm/sec
// M204 - Set default acceleration: S normal moves T filament only moves (M204 S3000 T7000) im mm/sec^2  also sets minimum segment time in ms (B20000) to prevent buffer underruns and M20 minimum feedrate
// M205 -  advanced settings:  minimum travel speed S=while printing T=travel only,  B=minimum segment time X= maximum xy jerk, Z=maximum Z jerk, E=maximum E jerk
// M206 - set additional homeing offset
// M207 - set retract length S[positive mm] F[feedrate mm/sec] Z[additional zlift/hop]
// M208 - set recover=unretract length S[positive mm surplus to the M207 S*] F[feedrate mm/sec]
// M209 - S<1=true/0=false> enable automatic retract detect if the slicer did not support G10/11: every normal extrude-only move will be classified as retract depending on the direction.
// M218 - set hotend offset (in mm): T<extruder_number> X<offset_on_X> Y<offset_on_Y>
// M220 S<factor in percent>- set speed factor override percentage
// M221 S<factor in percent>- set extrude factor override percentage
// M240 - Trigger a camera to take a photograph
// M280 - set servo position absolute. P: servo index, S: angle or microseconds
// M300 - Play beepsound S<frequency Hz> P<duration ms>
// M301 - Set PID parameters P I and D
// M302 - Allow cold extrudes
// M303 - PID relay autotune S<temperature> sets the target temperature. (default target temperature = 150C)
// M304 - Set bed PID parameters P I and D
// M400 - Finish all moves
// M500 - stores paramters in EEPROM
// M501 - reads parameters from EEPROM (if you need reset them after you changed them temporarily).
// M502 - reverts to the default "factory settings".  You still need to store them in EEPROM afterwards if you want to.
// M503 - print the current settings (from memory not from eeprom)
// M540 - Use S[0|1] to enable or disable the stop SD card print on endstop hit (requires ABORT_ON_ENDSTOP_HIT_FEATURE_ENABLED)

// M700 - Laser on
// M701 - Laser off

// M600 - Pause for filament change X[pos] Y[pos] Z[relative lift] E[initial retract] L[later retract distance for removal]
// M907 - Set digital trimpot motor current using axis codes.
// M908 - Control digital trimpot directly.
// M350 - Set microstepping mode.
// M351 - Toggle MS1 MS2 pins directly.
// M928 - Start SD logging (M928 filename.g) - ended by M29
// M999 - Restart after being stopped by error

//Stepper Movement Variables

//===========================================================================
//=============================imported variables============================
//===========================================================================
bool axis_adjust = false;


u8 enable_level_sensor = 0x00;
extern u16 logo_hold_time;
float delta[3] = {0.0, 0.0, 0.0};
float bed_level[ACCURATE_BED_LEVELING_POINTS][ACCURATE_BED_LEVELING_POINTS];
//===========================================================================
//=============================public variables=============================
//===========================================================================
#ifdef SDSUPPORT
CardReader card;
#endif
float homing_feedrate[] = HOMING_FEEDRATE;
bool axis_relative_modes[] = AXIS_RELATIVE_MODES;
int feedmultiply=100; //100->1 200->2
int saved_feedmultiply;
int extrudemultiply=100; //100->1 200->2
float current_position[NUM_AXIS] = { 0.0, 0.0, 0.0, 0.0 };
float add_homeing[3]={0,0,0};

float min_pos[3] = { X_MIN_POS, Y_MIN_POS, Z_MIN_POS };
float max_pos[3] = { X_MAX_POS, Y_MAX_POS, Z_MAX_POS };
float  base_min_pos[3]   = { X_MIN_POS, Y_MIN_POS, Z_MIN_POS };  
float  base_max_pos[3]   = { X_MAX_POS, Y_MAX_POS, Z_MAX_POS };
float  base_home_pos[3]  = { X_HOME_POS, Y_HOME_POS, Z_HOME_POS };
float  max_length[3]	    = { X_MAX_LENGTH, Y_MAX_LENGTH, Z_MAX_LENGTH };


static const float  home_retract_mm[3]= { X_HOME_RETRACT_MM, Y_HOME_RETRACT_MM, Z_HOME_RETRACT_MM };
//static const signed char home_dir[3]  = { X_HOME_DIR, Y_HOME_DIR, Z_HOME_DIR };
signed char home_dir[3]  = { X_HOME_DIR, Y_HOME_DIR, Z_HOME_DIR };
// Extruder offset, only in XY plane
#if EXTRUDERS > 1
float extruder_offset[2][EXTRUDERS] = {
#if defined(EXTRUDER_OFFSET_X) && defined(EXTRUDER_OFFSET_Y)
  EXTRUDER_OFFSET_X, EXTRUDER_OFFSET_Y
#endif
	0,0,0,0
};
#endif
uint8_t active_extruder = 0;
u8 tmp_extruder;//20161218
float LaserPower=0;
#ifdef BARICUDA
int ValvePressure=0;
int EtoPPressure=0;
#endif

#ifdef FWRETRACT
  bool autoretract_enabled=true;
  bool retracted=false;
  float retract_length=3, retract_feedrate=17*60, retract_zlift=0.8;
  float retract_recover_length=0, retract_recover_feedrate=8*60;
#endif
//===========================================================================
//=============================private variables=============================
//===========================================================================
const char axis_codes[NUM_AXIS] = {'X', 'Y', 'Z', 'E'};
 float destination[NUM_AXIS] = {  0.0, 0.0, 0.0, 0.0};
float offset[3] = {0.0, 0.0, 0.0};
static bool home_all_axis = true;
//static float feedrate = 1500.0, next_feedrate, saved_feedrate;//20160404
float feedrate = 1500.0; 
static float next_feedrate, saved_feedrate;
static long gcode_N, gcode_LastN, Stopped_gcode_LastN = 0;

volatile static bool relative_mode = false;  //Determines Absolute or Relative Coordinates

char cmdbuffer[BUFSIZE][MAX_CMD_SIZE];
volatile bool fromsd[BUFSIZE];
int bufindr = 0;
static int bufindw = 0;
int buflen = 0;
//static int i = 0;
static char serial_char;
static int serial_count = 0;
static bool comment_mode = false;
static char *strchr_pointer; // just a pointer to find chars in the cmd string like X, Y, Z, E, etc

//const int sensitive_pins[] = SENSITIVE_PINS; // Sensitive pin list for M42

//static float tt = 0;
//static float bt = 0;

//Inactivity shutdown variables
static unsigned long previous_millis_cmd = 0;
static unsigned long max_inactive_time = 0;
static unsigned long stepper_inactive_time = DEFAULT_STEPPER_DEACTIVE_TIME*10001;

u32 stop_brake_time=0;

unsigned long starttime=0;
unsigned long stoptime=0;
//static u8 tmp_extruder;//20161218


bool Stopped=false;

//#if NUM_SERVOS > 0
//  Servo servos[NUM_SERVOS];
//#endif
//===========================================================================
//=============================ROUTINES=============================
//===========================================================================

void get_arc_coordinates(void);
bool setTargetedHotend(int code);

static u8 config_ok = 0x00;

long residencyStart;

// Reset calibration results to zero.//??bed_level[][],?G28?G29????
void reset_bed_level(void) 
{
	int x,y;
  for ( y = 0; y < ACCURATE_BED_LEVELING_POINTS; y++) {
    for ( x = 0; x < ACCURATE_BED_LEVELING_POINTS; x++) {
      bed_level[x][y] = 0.0;
    }
  }
}



void Clear_cmdbuffer(void)//这里清空，也清零cmdbuffer[bufindr][bufindw]   //20160330
{
	 u8 i,j;
	for(i=0;i<BUFSIZE;i++)
	{
		for(j=0;j<MAX_CMD_SIZE;j++)
		cmdbuffer[i][j] = 0;
	}
	bufindr = 0;
	bufindw = 0;
	buflen = 0;
}


void enquecommand(const char *cmd)
{
  if(buflen < BUFSIZE)
  {
    //this is dangerous if a mixing of serial and this happsens
    strcpy(&(cmdbuffer[bufindw][0]),cmd);
   // SERIAL_ECHO_START;
   // printf("enqueing \"%s\"",cmdbuffer[bufindw]);
    bufindw= (bufindw + 1)%BUFSIZE;
    buflen += 1;
  }
}


void Print_parameter_init()
{
  	max_pos[0] = x_max_pos;//x_max_pos和X_MAX_POS值会改变，不能使用max_pos[0]代替x_max_pos，因为X_MAX_POS与X_MAX_LENGTH挂钩，如果X_MAX_POS固定宏，那么max_pos[0]改变后无法传递竂_MAX_LENGTH?
		max_pos[1] = y_max_pos;
		max_pos[2] = z_max_pos;
	
		base_max_pos[0] = x_max_pos;
		base_max_pos[1] = y_max_pos;
		base_max_pos[2] = z_max_pos;

		
	  min_pos[0] = 0;//x_max_pos和X_MAX_POS值会改变，不能使用max_pos[0]代替x_max_pos，因为X_MAX_POS与X_MAX_LENGTH挂钩，如果X_MAX_POS固定宏，那么max_pos[0]改变后无法传递竂_MAX_LENGTH?
		min_pos[1] = 0;
		min_pos[2] = 0;
	
	  base_min_pos[0] = 0;
		base_min_pos[1] = 0;
		base_min_pos[2] = 0;
	
	  base_home_pos[0] = 0;
		base_home_pos[1] = 0;
		base_home_pos[2] = 0;//delta  z_max_pos
	
	
		max_length[0]	 = x_max_pos;
		max_length[1]	 = y_max_pos;
		max_length[2]	 = z_max_pos;

}

void Storage_variable_Int()
{
	unsigned char read_judge;
	u8 i;
  u16 read_color=0;
			FLASH_READ_VAR(FLASH_SET_STORE_OFFSET+151,read_judge);
      if(read_judge>=GUI_LANGUAGE_NUM) {gui_phy.language = 0;FLASH_WRITE_VAR(FLASH_SET_STORE_OFFSET+151,gui_phy.language);}
				else {FLASH_READ_VAR(FLASH_SET_STORE_OFFSET+151,gui_phy.language);}
			
//      gui_phy.language = 2;				
			
			
			FLASH_READ_VAR(FLASH_SET_STORE_OFFSET+152,read_judge);
      if((read_judge != 0)&&(read_judge != 1)) {emc_switch = 0;FLASH_WRITE_VAR(FLASH_SET_STORE_OFFSET+152,emc_switch);}
			 else {FLASH_READ_VAR(FLASH_SET_STORE_OFFSET+152,emc_switch);}
			
			FLASH_READ_VAR(FLASH_SET_STORE_OFFSET+153,read_judge);
      if((read_judge != 0)&&(read_judge != 1)) {temp_sensor_type = 1;FLASH_WRITE_VAR(FLASH_SET_STORE_OFFSET+153,temp_sensor_type);}
			 else {FLASH_READ_VAR(FLASH_SET_STORE_OFFSET+153,temp_sensor_type);}

			FLASH_READ_VAR(FLASH_SET_STORE_OFFSET+154,read_judge);
      if((read_judge != 0)&&(read_judge != 1)) {no_material_switch = 0;FLASH_WRITE_VAR(FLASH_SET_STORE_OFFSET+154,no_material_switch);}
			 else {FLASH_READ_VAR(FLASH_SET_STORE_OFFSET+154,no_material_switch);}	
				if(no_material_switch == 1)
					Material_EXIT_Set(ENABLE);
				else if(no_material_switch == 0)
					Material_EXIT_Set(DISABLE);

			FLASH_READ_VAR(FLASH_SET_STORE_OFFSET+155,read_judge);
      if((read_judge != 0)&&(read_judge != 1)) {brownout_switch = 0;FLASH_WRITE_VAR(FLASH_SET_STORE_OFFSET+155,brownout_switch);}
			 else {FLASH_READ_VAR(FLASH_SET_STORE_OFFSET+155,brownout_switch);}
				
			FLASH_READ_VAR(FLASH_SET_STORE_OFFSET+156,read_judge);
      if((read_judge != 0)&&(read_judge != 1)) {screen_saving_switch = 0;FLASH_WRITE_VAR(FLASH_SET_STORE_OFFSET+156,screen_saving_switch);}
			 else {FLASH_READ_VAR(FLASH_SET_STORE_OFFSET+156,screen_saving_switch);}

			FLASH_READ_VAR(FLASH_SET_STORE_OFFSET+157,read_judge);
      if((read_judge != 0)&&(read_judge != 1)) {auto_shutdown_switch = 0;FLASH_WRITE_VAR(FLASH_SET_STORE_OFFSET+157,auto_shutdown_switch);}
			 else {FLASH_READ_VAR(FLASH_SET_STORE_OFFSET+157,auto_shutdown_switch);}
			 
			 FLASH_READ_VAR(FLASH_SET_STORE_OFFSET+158,read_judge);
      if((read_judge != 0)&&(read_judge != 1)&&(read_judge != 2)) {Machine_type = 0;FLASH_WRITE_VAR(FLASH_SET_STORE_OFFSET+158,Machine_type);}
			 else {FLASH_READ_VAR(FLASH_SET_STORE_OFFSET+158,Machine_type);}
			 

			max_xy_jerk=DEFAULT_XYJERK;
			max_z_jerk=DEFAULT_ZJERK;
			max_e_jerk=DEFAULT_EJERK;

			 
			FLASH_READ_VAR(FLASH_SET_STORE_OFFSET+159,read_judge);
      if(read_judge == 0xFF) 
        {
					diagonal=219.7;
					smooth_rod=145;
					effector=26.5;
					carriage=20;
					FLASH_WRITE_VAR(FLASH_SET_STORE_OFFSET+159,diagonal);                                        //224-227    float 						4个字节   
					FLASH_WRITE_VAR(FLASH_SET_STORE_OFFSET+163,smooth_rod);                                  			//228-231    float 						4个字节     
					FLASH_WRITE_VAR(FLASH_SET_STORE_OFFSET+167,effector);                                  			//232-235    float 						4个字节    
					FLASH_WRITE_VAR(FLASH_SET_STORE_OFFSET+171,carriage);                                  			//232-235    float 						4个字节
				}
			else
       {
			   FLASH_READ_VAR(FLASH_SET_STORE_OFFSET+159,diagonal);                                        //224-227    float 						4个字节   
			   FLASH_READ_VAR(FLASH_SET_STORE_OFFSET+163,smooth_rod);                                  			//228-231    float 						4个字节     
			   FLASH_READ_VAR(FLASH_SET_STORE_OFFSET+167,effector);                                  			//232-235    float 						4个字节    
			   FLASH_READ_VAR(FLASH_SET_STORE_OFFSET+171,carriage);                                  			//232-235    float 						4个字节 
			 }
			 
		 FLASH_READ_VAR(FLASH_SET_STORE_OFFSET+175,read_judge);
      if((read_judge != 0)&&(read_judge != 1)) {level_endstops_inverting = 1;FLASH_WRITE_VAR(FLASH_SET_STORE_OFFSET+175,level_endstops_inverting);}
			 else {FLASH_READ_VAR(FLASH_SET_STORE_OFFSET+175,level_endstops_inverting);}
			 
		FLASH_READ_VAR(FLASH_SET_STORE_OFFSET+176,read_judge);
      if((read_judge != 0)&&(read_judge != 1)) {Password_switch = 0;FLASH_WRITE_VAR(FLASH_SET_STORE_OFFSET+176,Password_switch);}
			 else {FLASH_READ_VAR(FLASH_SET_STORE_OFFSET+176,Password_switch);}
			 
		FLASH_READ_VAR(FLASH_SET_STORE_OFFSET+177,read_judge);
      if(read_judge ==0xFF) 
				{
					for(i=0;i<6;i++)
					Password_read_buffer[i] = '0';
			   FLASH_WRITE_VAR(FLASH_SET_STORE_OFFSET+177,Password_read_buffer);
				}
			 else {FLASH_READ_VAR(FLASH_SET_STORE_OFFSET+177,Password_read_buffer);}
			 
			 
			 FLASH_READ_VAR(BED_LEVEL_STORE_OFFSET,read_judge);
      if(read_judge ==0xFF) 
				{
             
				}
			 else
			 {
			   FLASH_READ_VAR(BED_LEVEL_STORE_OFFSET,bed_level);
					  {
							int x,y;
							for (x = 0; x < ACCURATE_BED_LEVELING_POINTS; x++) 
							{
								for (y = 0; y < ACCURATE_BED_LEVELING_POINTS; y++) 
								{
									printf("%d,%d",x,y);
									printf("bed_level:%f\r\n",bed_level[x][y]);
								}
							}
					  }
			 }
			 
			 
			 FLASH_READ_VAR(FLASH_SET_STORE_OFFSET+183,read_judge);
      if((read_judge != 0)&&(read_judge != 1)) {bed_level_switch = 0;FLASH_WRITE_VAR(FLASH_SET_STORE_OFFSET+183,bed_level_switch);}
			 else {FLASH_READ_VAR(FLASH_SET_STORE_OFFSET+183,bed_level_switch);}
			 
			 FLASH_READ_VAR(FLASH_SET_STORE_OFFSET+184,read_judge);
      if((read_judge != 1)&&(read_judge != 2)) {temp_sensor_num = 1;FLASH_WRITE_VAR(FLASH_SET_STORE_OFFSET+184,temp_sensor_num);}
			 else {FLASH_READ_VAR(FLASH_SET_STORE_OFFSET+184,temp_sensor_num);}
			 
			 FLASH_READ_VAR(FLASH_SET_STORE_OFFSET+185,read_judge);
      if((read_judge != 0)&&(read_judge != 1)) {backup_power = 0;FLASH_WRITE_VAR(FLASH_SET_STORE_OFFSET+185,backup_power);}
			 else {FLASH_READ_VAR(FLASH_SET_STORE_OFFSET+185,backup_power);}
			 
			 FLASH_READ_VAR(FLASH_SET_STORE_OFFSET+186,read_judge);
      if((read_judge != 0)&&(read_judge != 1)) {e0_endstops_inverting = 0;FLASH_WRITE_VAR(FLASH_SET_STORE_OFFSET+186,e0_endstops_inverting);}
			 else {FLASH_READ_VAR(FLASH_SET_STORE_OFFSET+186,e0_endstops_inverting);}
			 
			 
			 FLASH_READ_VAR(FLASH_SET_STORE_OFFSET+187,read_judge);
      if(read_judge == 0xFF) 
        {
					pulse_width=6;//jee

					FLASH_WRITE_VAR(FLASH_SET_STORE_OFFSET+187,pulse_width);                                        //224-227    float 						4个字节   
        }
			else
       {
			   FLASH_READ_VAR(FLASH_SET_STORE_OFFSET+187,pulse_width);                                        //224-227    float 						4个字节   
			 }
			 
			 
			 
			 FLASH_READ_VAR(FLASH_SET_STORE_OFFSET+191,read_judge);
      if(read_judge == 0xFF) 
			{
				pause_length=15;

				FLASH_WRITE_VAR(FLASH_SET_STORE_OFFSET+191,pause_length);                                        //224-227    float 						4个字节   
			}
			else
       {
			   FLASH_READ_VAR(FLASH_SET_STORE_OFFSET+191,pause_length);                                        //224-227    float 						4个字节   
			 }
			 
			 
			 
			 
			 FLASH_READ_VAR(FLASH_SET_STORE_OFFSET+195,read_judge);
      if(read_judge == 0xFF) 
			{
				extrude_length_advance=8;

				FLASH_WRITE_VAR(FLASH_SET_STORE_OFFSET+195,extrude_length_advance);                                        //224-227    float 						4个字节   
			}
			else
       {
			   FLASH_READ_VAR(FLASH_SET_STORE_OFFSET+195,extrude_length_advance);                                        //224-227    float 						4个字节   
			 }
			 	 FLASH_READ_VAR(FLASH_SET_STORE_OFFSET+195,read_judge);
			 //JEE2018
//      if(read_judge == 0xFF) 
//			{
//				appoint_z_height=0;

//				FLASH_WRITE_VAR(BACKUP_Z_HEIGHT_ADDRESS,appoint_z_height);                                        //224-227    float 						4个字节   
//			}
//			else
//       {
//			   FLASH_READ_VAR(BACKUP_Z_HEIGHT_ADDRESS,appoint_z_height);                                        //224-227    float 						4个字节   
//			 }
			 
			 FLASH_READ_VAR(FLASH_SET_STORE_OFFSET+204,read_judge);
      if(read_judge == 0xFF) 
			{
				floor_height=0.2;

				FLASH_WRITE_VAR(FLASH_SET_STORE_OFFSET+204,floor_height);                                        //224-227    float 						4个字节   
			}
			else
       {
			   FLASH_READ_VAR(FLASH_SET_STORE_OFFSET+204,floor_height);                                        //224-227    float 						4个字节   
			 }
			 
			 FLASH_READ_VAR(FLASH_SET_STORE_OFFSET+208,read_judge);
      if(read_judge == 0xFF) 
			{
				add_homeing_ofsize[0]=0;
				add_homeing_ofsize[1]=0;
				add_homeing_ofsize[2]=0;
				FLASH_WRITE_VAR(FLASH_SET_STORE_OFFSET+208,add_homeing_ofsize);                                        //224-227    float 						4个字节   
			}
			else
       {
			   FLASH_READ_VAR(FLASH_SET_STORE_OFFSET+208,add_homeing_ofsize);                                        //224-227    float 						4个字节   
			 }
			 FLASH_READ_VAR(FLASH_SET_STORE_OFFSET+220,read_judge);
      if(read_judge == 0xFF) 
			{
				z_homeing_ofsize=0;

				FLASH_WRITE_VAR(FLASH_SET_STORE_OFFSET+220,z_homeing_ofsize);                                        //224-227    float 						4个字节   
			}
			else
       {
			   FLASH_READ_VAR(FLASH_SET_STORE_OFFSET+220,z_homeing_ofsize);                                        //224-227    float 						4个字节   
			 }
			 {
					 u8 read_dir[3];
					 FLASH_READ_VAR(FLASH_SET_STORE_OFFSET+224,read_dir);                                  //12 char
					if((read_dir[0] == 0xFF)&&(read_dir[1] == 0xFF)&&(read_dir[2] == 0xFF))
					{
						home_dir[0]=-1;
						home_dir[1]=-1;
						home_dir[2]=-1;
						FLASH_WRITE_VAR(FLASH_SET_STORE_OFFSET+224,home_dir);                                        //224-227    float 						4个字节   
					}
					else
					 {
						 FLASH_READ_VAR(FLASH_SET_STORE_OFFSET+224,home_dir);                                        //224-227    float 						4个字节   
					 }
		    }
			 //默认颜色
					 FLASH_READ_VAR(FLASH_SET_STORE_OFFSET+1000,title_fontcol);//可以为0XFFFF
       FLASH_READ_VAR(FLASH_SET_STORE_OFFSET+1002,read_color);//不能为0xffff
			 if(read_color==0xffff)
			 {
				  title_backgroundcol=gui_color_chg(0X888888);
				  FLASH_WRITE_VAR(FLASH_SET_STORE_OFFSET+1002,title_backgroundcol);
			 }else
        {			 
			  FLASH_READ_VAR(FLASH_SET_STORE_OFFSET+1002,title_backgroundcol);
		 		}
				
			  FLASH_READ_VAR(FLASH_SET_STORE_OFFSET+1004,read_color);//不能为0xffff
			  if(read_color==0xffff)
				{
					body_backgroundcol=gui_color_chg(0X999999);
				  FLASH_WRITE_VAR(FLASH_SET_STORE_OFFSET+1004,body_backgroundcol);
				}else{
				FLASH_READ_VAR(FLASH_SET_STORE_OFFSET+1004,body_backgroundcol);			 
				}			 
				 		
				FLASH_READ_VAR(FLASH_SET_STORE_OFFSET+1006,logo_hold_time);
			 if(logo_hold_time == 0xFF)	
			 {	
				 logo_hold_time=3;
			   FLASH_WRITE_VAR(FLASH_SET_STORE_OFFSET+1006,logo_hold_time);
			 }				 							 
}

void setup(void)
{ 
	u8 res=0xff;
  //printf("start");//启动
  // loads data from EEPROM if available else uses defaults (and resets step acceleration rate)
  Config_RetrieveSettings();//从Flash中读取设置
	Storage_variable_Int();
//	
	Print_parameter_init();
	
  st_init();    // Initialize stepper, this enables interrupts
  tp_init();    // Initialize 
  plan_init();  // Initialize planner;
 // watchdog_init()
 // setup_photpin();
 // servo_init();
	 FLASH_READ_VAR(FILE_GCODE_PATH_ADDRESS,fnameDirPath);//读取文件路径
	
	 res=f_open(&card.fgcode,(const TCHAR*)fnameDirPath,FA_READ);
	 if(res==FR_OK)
	 {
		 card_closefile();
		 currentMenu = Continue_Print_screen;
		 nextMenu = Continue_Print_screen;
	 }
//	 if(Password_switch == 1)
//	 {
//		 Password_set_flag = 0x02;
//	   currentMenu = Password_screen;
//		 nextMenu = Password_screen;
//	 }
		 
	 
 lcd_update();//4.上电初始化设置完 进主循环前显示
 }

 void loop(void)
{
 while(1)
{ 
		if(buflen < (BUFSIZE-1))
		 {
			 get_command();                                                                                                     //读取指令
		 }
		#ifdef SDSUPPORT
		 if(SD_CD)//无SD卡
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
							printf(MSG_OK);
						}
						else                                             //如果没有
						{
							card_closefile();
//							exfuns_init();							//为fatfs相关变量申请内存	
//							f_mount(0,fs[0]); 					 	//挂载SD卡 //20160412
//							card_initsd();//20160409遇到M29说明传输完成，然后关闭文件重新初始化SD卡
							printf(MSG_FILE_SAVED);
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
		 else 
		 Abnormal_Handling(Abnormal_Flag);
		//这个一定是要放到这个位置的，因为如果和显示他们放一起，那么在plan的while中点了暂停后，	 
		 //虽然退出了while，但是while后面还是会继续填充一个没用的block，所以在异常处理函数中也要加Discard_Buf_and_Block();
		 //在显示里点暂停加Discard_Buf_and_Block();是为了让机器立刻停下来，而停下来跳出plan中的while后还会还会再立刻填充一个没用的block，所以在异常处理函数中也要加Discard_Buf_and_Block();
	  
  //check heater every n milliseconds 每个ms 检测加热头
  manage_heater();
  manage_inactivity();
  checkHitEndstops();

	//checkDTR();//20160228
  lcd_update();//5.循环显示 主循环显示
	
 }
}
 
void get_command(void)
{ int16_t n;
  char time[30];
  unsigned long t;
  int hours, minutes;
  while( MYSERIAL_available() > 0  && buflen < BUFSIZE) //如果串口数据包里有数据且
  {	//	LCD_ShowString(5,5,240,320,12, ".1.");	
    serial_char = MYSERIAL_read();
//		printf(" serial_char: %c\n\r",serial_char);
    if(serial_char == '\n' ||serial_char == '\r' ||(serial_char == ':' && comment_mode == false) ||serial_count >= (MAX_CMD_SIZE - 1) )//sanse 冒号
    {
      if(!serial_count)  //if empty line
	    {
        comment_mode = false; //for new command 遇到;分号时才为true
        return;
      }
      cmdbuffer[bufindw][serial_count] = 0; //terminate string
      if(!comment_mode)
	    {
        comment_mode = false; //for new command
        fromsd[bufindw] = false;
        if(strchr(cmdbuffer[bufindw], 'N') != NULL)	 //只有有'N'和'*'并且校验和正确的才算是正确的命令                                                                    //如果字符串中有'N'
        {
          strchr_pointer = strchr(cmdbuffer[bufindw], 'N');//查找字符串s中首次出现字符N的位置。strchr_pointer是指向cmdbuffer[bufindw]中'N'的地址
          gcode_N = (strtol(&cmdbuffer[bufindw][strchr_pointer - cmdbuffer[bufindw] + 1], NULL, 10));//获取N 后面的参数   
          if(gcode_N != gcode_LastN+1 && (strstr(cmdbuffer[bufindw], PSTR("M110")) == NULL) )  //extern char *strstr(char *haystack, char *needle);从字符串haystack中寻找needle第一次出现的位置（不比较结束符NULL)。
		      {                                                                     //行码检测            
            SERIAL_ERROR_START;
            printf(MSG_ERR_LINE_NO);
            printf("%ld",gcode_LastN);
            //Serial.println(gcode_N);
            FlushSerialRequestResend();
            serial_count = 0;
            return;
          }

          if(strchr(cmdbuffer[bufindw], '*') != NULL) //上面检测到'N'开头,现在检测到'*'结尾开始校验命令                                                             //如果命令包里有'N'也有'*'
          {
            u8 checksum = 0;
            u8 count = 0;
            while(cmdbuffer[bufindw][count] != '*') checksum = checksum^cmdbuffer[bufindw][count++];
            strchr_pointer = strchr(cmdbuffer[bufindw], '*');
                                                         //如果本地计算出来的检验码和'*'后面的标记码不同，就要从新发送一遍
            if( (u8)(strtod(&cmdbuffer[bufindw][strchr_pointer - cmdbuffer[bufindw] + 1], NULL)) != checksum)//校验出来发现错误就重发一遍
							{                                                                
									SERIAL_ERROR_START;
									printf(MSG_ERR_CHECKSUM_MISMATCH);
									printf(" checksum: %d\n\r",checksum);
									count = 0;
								  printf(" '");
									while(cmdbuffer[bufindw][count] != '*')
									{
										printf("%c",cmdbuffer[bufindw][count++]);
									}
									printf(" '\n\r ");
									checksum = 0;
									count = 0;
									while(cmdbuffer[bufindw][count] != '*')
									{ 
										printf("cmdbuffer:%d;",cmdbuffer[bufindw][count]);
										checksum = checksum^cmdbuffer[bufindw][count++];
									  printf(" checksum:%d \n\r",checksum);
									}
									///	printf("\n\r ");
						
									printf("%ld",gcode_LastN);
									FlushSerialRequestResend();
									serial_count = 0;
									return;
							}
            //if no errors, continue parsing//没有错误就不处理，继续下面
          }
          else
          {
            SERIAL_ERROR_START;
            printf(MSG_ERR_NO_CHECKSUM);
            printf("%ld",gcode_LastN);
            FlushSerialRequestResend();
            serial_count = 0;
            return;
          }

          gcode_LastN = gcode_N;
          //if no errors, continue parsing//没有错误就不处理，继续下面
        }
        else  // if we don't receive 'N' but still see '*'
        {
          if((strchr(cmdbuffer[bufindw], '*') != NULL))                                                                 //如果没有'N'，但有'*'   错误
          {
            SERIAL_ERROR_START;
            printf(MSG_ERR_NO_LINENUMBER_WITH_CHECKSUM);
            printf("%ld",gcode_LastN);
            serial_count = 0;
            return;
          }
        }
        if((strchr(cmdbuffer[bufindw], 'G') != NULL))                                                                   //如果有'G'
		    {
          strchr_pointer = strchr(cmdbuffer[bufindw], 'G');//查找字符串s中首次出现字符G的位置?
          switch((int)((strtod(&cmdbuffer[bufindw][strchr_pointer - cmdbuffer[bufindw] + 1], NULL))))//获取G 后面的参数
						{
							case 0:
							case 1:
							case 2:
							case 3:
								if(Stopped == false) { // If printer is stopped by an error the G[0-3] codes are ignored.
							#ifdef SDSUPPORT
									if(card.saving)
										break;
							#endif //SDSUPPORT
								}
								else {
									printf(MSG_ERR_STOPPED);
								 // LCD_MESSAGEPGM(MSG_STOPPED);
								}
								break;
							default:
								break;
             }
        }
        bufindw = (bufindw + 1)%BUFSIZE;//命令个数+1//如果这条命令没有一点错误，就加1保存
        buflen += 1;//sanse
      }
      serial_count = 0; //clear buffer
    }
    else                                                 //如果串口数据中没有'\n'，'\r'，':'   就保存串口字节到buf
    {
      if(serial_char == ';') comment_mode = true;//遇到分号表示注释不作任何处理
      if(!comment_mode) cmdbuffer[bufindw][serial_count++] = serial_char;//如果也没有遇到';'，命令中的字节数就+1
    }
  }

  #ifdef SDSUPPORT   //sanse
  if((!card.sdprinting || serial_count!=0))//&&(Material_all_process ==0X00))//20160403
		{ 
 //	LCD_ShowString(20,5,240,320,12, ".2.");
    return;
   }
  while( !card_eof()  && buflen < BUFSIZE) 
  {	//LCD_ShowString(50,5,240,320,12, ".3.");
    n=card_get();
    serial_char = (BYTE)n;
    if(serial_char == '\n' ||
       serial_char == '\r' ||
       (serial_char == ':' && comment_mode == false) ||
       serial_count >= (MAX_CMD_SIZE - 1)||
			 n==-1)                                     //每行结束会有一个0x0D('\n'回车)和0x0A('\r'换行)
    { 
      if(card_eof())//sanse//查询SD卡GCODE文件是否到头了
			{
        printf(MSG_FILE_PRINTED);
	    	printf("\n");
        stoptime=millis();
        t=(stoptime-starttime)/1000;
        minutes=(t/60)%60;
        hours=t/60/60;
        sprintf(time, PSTR("%i hours %i minutes"),hours, minutes);
        SERIAL_ECHO_START;
        printf("%s",time);
     //   lcd_setstatus(time);
        card_printingHasFinished();
       // card_checkautostart(true);
				//beep_flag = 0x01;
				if((auto_shutdown_switch==1)&&(config_ok == 0x00))
				{
					if(config_ok == 0x00)
//						fgcode_fptr_flag = 0x00;
//						SPI_Flash_Erase_Sector(FGCODE_FPTR_ADDRESS/1024/4);//两个地方擦除，1.在发现不是断电前的文件时2.是断电前文件，在移动到断电点时
						POWER_KEY = 0;  						
				}
				else
				{
				  config_ok = 0x00;
				  //currentMenu = system_screen; /* function pointer to the currently active menu */
          nextMenu = home_screen;  //进入下一个界面
				}
      }
      if(!serial_count)
      {
        comment_mode = false; //for new command
       //return; //if empty line
				continue;
      }
      cmdbuffer[bufindw][serial_count] = 0; //terminate string
//      if(!comment_mode){
        fromsd[bufindw] = true;//sanse
        buflen += 1;
        bufindw = (bufindw + 1)%BUFSIZE;
//      }
      comment_mode = false; //for new command
      serial_count = 0; //clear buffer
    }
    else
    {
      if(serial_char == ';') 
				comment_mode = true;
      if(!comment_mode) 
				cmdbuffer[bufindw][serial_count++] = serial_char;
    }
  }
  #endif //SDSUPPORT

}


 float code_value(void)
 {

  return (strtod(&cmdbuffer[bufindr][strchr_pointer - cmdbuffer[bufindr] + 1], NULL));
}

long code_value_long(void)
{
  return (strtol(&cmdbuffer[bufindr][strchr_pointer - cmdbuffer[bufindr] + 1], NULL, 10));
}

bool code_seen(char code)
{
  strchr_pointer = strchr(cmdbuffer[bufindr], code);
  return (strchr_pointer != NULL);  //Return True if a character was found
}
//type: 0：图标和字体颜色 1:标题栏背景颜色，2:界面颜色 
u16 code_getcolor(u8 type)
{	
	//long strtol(const char *nptr,char **endptr,int base);
	//strtol函数会将参数nptr字符串根据参数base来转换成长整型数，参数base范围从2至36进制
	u32 temp_color = 0;
	//如果指令后没字符#,恢复默认值
	if(!code_seen('#'))
	{
		switch(type)
		{
			case 0:
				temp_color= GUI_ICON_FONTCOL;
				break;
			case 1:
				temp_color= GUI_TITLE_BACKCOL;
				break;
			case 2:
				temp_color= GUI_BODY_BACKCOL;
				break;
			default:break;
		   }
      }else
	     temp_color = strtol(&cmdbuffer[bufindr][strchr_pointer - cmdbuffer[bufindr] + 1],NULL,16);
  return gui_color_chg(temp_color);
}
//static const float  base_min_pos[3]   = { X_MIN_POS, Y_MIN_POS, Z_MIN_POS };  
//static const float  base_max_pos[3]   = {110,110,110};//{ X_MAX_POS, Y_MAX_POS, Z_MAX_POS };
//static const float  base_home_pos[3]  = { X_HOME_POS, Y_HOME_POS, Z_HOME_POS };
//static const float  max_length[3]	    = {110,110,110};//{ X_MAX_LENGTH, Y_MAX_LENGTH, Z_MAX_LENGTH };
//static const float  home_retract_mm[3]= { X_HOME_RETRACT_MM, Y_HOME_RETRACT_MM, Z_HOME_RETRACT_MM };
//static const signed char home_dir[3]  = { X_HOME_DIR, Y_HOME_DIR, Z_HOME_DIR };

void axis_is_at_home(int axis) 
{
  current_position[axis] = base_home_pos[axis] + add_homeing_ofsize[axis];
  min_pos[axis] =          base_min_pos[axis] + add_homeing_ofsize[axis];
  max_pos[axis] =          base_max_pos[axis] + add_homeing_ofsize[axis];
}
#define HOMEAXIS_DO(LETTER) (( LETTER##_HOME_DIR==-1) || (LETTER##_HOME_DIR==1))
static void homeaxis(int axis) 
{
  if (axis==X_AXIS ? HOMEAXIS_DO(X) :
      axis==Y_AXIS ? HOMEAXIS_DO(Y) :
      axis==Z_AXIS ? HOMEAXIS_DO(Z) :
      0) 	//
   {
    current_position[axis] =0;//jee2018
    plan_set_position(current_position[X_AXIS], current_position[Y_AXIS], current_position[Z_AXIS], current_position[E_AXIS]);

    destination[axis] = 1.5 * max_length[axis] * home_dir[axis];
    feedrate = homing_feedrate[axis];

		 plan_buffer_line(destination[X_AXIS], destination[Y_AXIS], destination[Z_AXIS], destination[E_AXIS], feedrate/60, active_extruder); 
			 st_synchronize();
			 
			 

    current_position[axis] = 0;
			 
    plan_set_position(current_position[X_AXIS], current_position[Y_AXIS], current_position[Z_AXIS], current_position[E_AXIS]);

		destination[axis] = -home_retract_mm[axis] * home_dir[axis];
    plan_buffer_line(destination[X_AXIS], destination[Y_AXIS], destination[Z_AXIS], destination[E_AXIS], feedrate/60, active_extruder);
		st_synchronize();

		//jee201804
		//3.归位的第三次运动	
		current_position[axis] = 0;
    plan_set_position(current_position[X_AXIS], current_position[Y_AXIS], current_position[Z_AXIS], current_position[E_AXIS]);

		feedrate = homing_feedrate[axis]/4;
		destination[axis] = 2*home_retract_mm[axis] * home_dir[axis];

    plan_buffer_line(destination[X_AXIS], destination[Y_AXIS], destination[Z_AXIS], destination[E_AXIS], feedrate/60, active_extruder);
    st_synchronize();
		 //jee201804
		//4.归位的第四次运动	
			if(home_dir[axis]==1)
			 {
				 switch(axis)
			  {
				 case X_AXIS:
				     current_position[axis]=x_max_pos;
						 destination[axis]=x_max_pos-add_homeing_ofsize[axis];
							break;
				 case Y_AXIS:
				     current_position[axis]=y_max_pos;
						 destination[axis]=y_max_pos-add_homeing_ofsize[axis];
							break;
				 case Z_AXIS:
				     current_position[axis]=z_max_pos;
						 destination[axis]=z_max_pos-add_homeing_ofsize[axis];
							break;
				}
						
			}
		 else
			 {
				 home_dir[axis]=-1;
				 current_position[axis]=0;
				destination[axis]=add_homeing_ofsize[axis];				 
			 }
		plan_set_position(current_position[X_AXIS], current_position[Y_AXIS], current_position[Z_AXIS], current_position[E_AXIS]);		
		plan_buffer_line(destination[X_AXIS], destination[Y_AXIS], destination[Z_AXIS], destination[E_AXIS],feedrate/60, active_extruder);
    st_synchronize();
		
    //axis_is_at_home(axis);
		current_position[axis] = destination[axis];
    //destination[axis] = current_position[axis];
    feedrate = 0.0;
    endstops_hit_on_purpose();
  }
}
#define HOMEAXIS(LETTER) homeaxis(LETTER##_AXIS)
void process_commands(void)
{ unsigned long codenum; //throw away variable
  char *starpos = NULL;
  int8_t i;
//	static u8 screen_shot_cnt=0;
//#ifdef ENABLE_AUTO_BED_LEVELING
//  float x_tmp, y_tmp, z_tmp, real_z;
//#endif

  if(code_seen('G'))//同时将strchr_pointer指向'G'的位置
  {
    switch((int)code_value())
    {
    case 0: //G0=G1
    case 1: // G1
      if(Stopped == false) 
				{
					 get_coordinates(); // For X Y Z E F
//          #ifdef FWRETRACT
//            if(autoretract_enabled)
//            if( !(code_seen('X') || code_seen('Y') || code_seen('Z')) && code_seen('E')) {
//              float echange=destination[E_AXIS]-current_position[E_AXIS];
//              if((echange<-MIN_RETRACT && !retracted) || (echange>MIN_RETRACT && retracted)) { //move appears to be an attempt to retract or recover
//                  current_position[E_AXIS] = destination[E_AXIS]; //hide the slicer-generated retract/recover from calculations
//                  plan_set_e_position(current_position[E_AXIS]); //AND from the planner
//                  retract(!retracted);
//                  return;
//              }
//            }
//          #endif //FWRETRACT					
						prepare_move();
						//ClearToSend();
      }
			//return;	
      break;
    case 2: // G2  - CW ARC
      if(Stopped == false) {
        get_arc_coordinates();
        prepare_arc_move(true);
        return;
      }
    case 3: // G3  - CCW ARC
      if(Stopped == false) {
        get_arc_coordinates();
        prepare_arc_move(false);
        return;
      }
    case 4: // G4 dwell
     // LCD_MESSAGEPGM(MSG_DWELL); ///////////////////////////////////////////////////////////
      codenum = 0;
      if(code_seen('P')) codenum = code_value(); // milliseconds to wait
      if(code_seen('S')) codenum = code_value() * 1000; // seconds to wait

      st_synchronize();
      codenum += millis();  // keep track of when we started waiting
      previous_millis_cmd = millis();
      while(millis()  < codenum ){
        manage_heater();
        manage_inactivity();
        lcd_update();	//6.G4里的现户籍
					
      }
      break;
    #ifdef FWRETRACT	   //ONLY PARTIALLY TESTED
      case 10: // G10 retract
      if(!retracted)
      {
        destination[X_AXIS]=current_position[X_AXIS];
        destination[Y_AXIS]=current_position[Y_AXIS];
        destination[Z_AXIS]=current_position[Z_AXIS];
        current_position[Z_AXIS]+=-retract_zlift;
        destination[E_AXIS]=current_position[E_AXIS]-retract_length;
        feedrate=retract_feedrate;
        retracted=true;
        prepare_move();
      }

      break;
      case 11: // G10 retract_recover
      if(!retracted)
      {
        destination[X_AXIS]=current_position[X_AXIS];
        destination[Y_AXIS]=current_position[Y_AXIS];
        destination[Z_AXIS]=current_position[Z_AXIS];

        current_position[Z_AXIS]+=retract_zlift;
        current_position[E_AXIS]+=-retract_recover_length;
        feedrate=retract_recover_feedrate;
        retracted=false;
        prepare_move();
      }
      break;
      #endif //FWRETRACT   //ONLY PARTIALLY TESTED
    case 28: //G28 Home all Axis one at a time
//			#ifdef ENABLE_AUTO_BED_LEVELING
//      plan_bed_level_matrix.set_to_identity();  //Reset the plane ("erase" all leveling data)
//      #endif
      saved_feedrate = feedrate;
      saved_feedmultiply = feedmultiply;
      feedmultiply = 100;
      previous_millis_cmd = millis();
      enable_endstops(true);

		for(i=0; i < NUM_AXIS; i++) {
        destination[i] = current_position[i];
      }
      feedrate = 0.0;
      home_all_axis = !((code_seen(axis_codes[0])) || (code_seen(axis_codes[1])) || (code_seen(axis_codes[2])));


	
			//Z上升z_homeing_ofsize
			if(!code_seen(axis_codes[Z_AXIS]))
			{
			 plan_set_position(current_position[X_AXIS], current_position[Y_AXIS], current_position[Z_AXIS], current_position[E_AXIS]);		
			 plan_buffer_line(destination[X_AXIS], destination[Y_AXIS], destination[Z_AXIS]+z_homeing_ofsize, destination[E_AXIS], 1000, active_extruder);
       st_synchronize();
//				current_position[Z_AXIS]=destination[Z_AXIS]+z_homeing_ofsize;
//				plan_set_position(current_position[X_AXIS], current_position[Y_AXIS], current_position[Z_AXIS], current_position[E_AXIS]);	
////			 //z_homeing_ofsize 可在其它地方临时改变
   		 FLASH_READ_VAR(FLASH_SET_STORE_OFFSET+220,z_homeing_ofsize);	//恢复z_homeing_ofsize
			}
      #if Z_HOME_DIR > 0                      // If homing away from BED do Z first  
      if((home_all_axis) || (code_seen(axis_codes[Z_AXIS]))){
        HOMEAXIS(Z);
      }
      #endif

      if((home_all_axis) || (code_seen(axis_codes[X_AXIS])))
      {
        HOMEAXIS(X);
//				current_position[X_AXIS] = 0;
//				plan_set_position(0, current_position[Y_AXIS], current_position[Z_AXIS], current_position[E_AXIS]);
//				plan_buffer_line(current_position[X_AXIS]+add_homeing_ofsize[0], current_position[Y_AXIS], current_position[Z_AXIS], destination[E_AXIS], 200, active_extruder);
//				st_synchronize();
//				current_position[0] = add_homeing_ofsize[0];
      }

      if((home_all_axis) || (code_seen(axis_codes[Y_AXIS]))) {
        HOMEAXIS(Y);
      }

      #if Z_HOME_DIR < 0                      // If homing towards BED do Z last
      if((home_all_axis) || (code_seen(axis_codes[Z_AXIS]))) {
        HOMEAXIS(Z);
      }
      #endif
      plan_set_position(current_position[X_AXIS], current_position[Y_AXIS], current_position[Z_AXIS], current_position[E_AXIS]);
  

      #ifdef ENDSTOPS_ONLY_FOR_HOMING
        enable_endstops(false);
      #endif

      feedrate = saved_feedrate;
      feedmultiply = saved_feedmultiply;
      previous_millis_cmd = millis();
      endstops_hit_on_purpose();
	
			
      break;
	
	
	
    case 29: // G29 Detailed Z-Probe, probes the bed at 3 or more points.
        {
			
					
				}
        break;

    case 30: // G30 Single Z Probe
        {
        }
        break;
	
    case 90: // G90
      relative_mode = false;
      break;
    case 91: // G91
      relative_mode = true;
//	  #ifdef DEBUG_PRINTF
//	  printf("relative_mode = true;");
//	  #endif
      break;
    case 92: // G92
      //if(!code_seen(axis_codes[E_AXIS]))
        st_synchronize();
      for(i=0; i < NUM_AXIS; i++) 
	  {
        if(code_seen(axis_codes[i])) 
	    {
           if(i == E_AXIS) 
		   {
             current_position[i] = code_value();
             plan_set_e_position(current_position[E_AXIS]);
           }
           else 
		   {
             current_position[i] = code_value()+add_homeing[i];
             plan_set_position(current_position[X_AXIS], current_position[Y_AXIS], current_position[Z_AXIS], current_position[E_AXIS]);
           }
        }
      }
      break;
    }
  }
  else if(code_seen('M'))//同时将strchr_pointer指向'M'的位置
  {	switch( (int)code_value() )//获取'M'后面的参数
    { 
  	 	 
	    case 0:// M0 - Unconditional stop
		 {
       if(residencyStart > -1)
			 {
				 card.sdprinting=false;
				 				 
				 Abnormal_Flag = 0x01;  													 				
				 Pause_flag = 0x01;				
		     }
		 }			 
             break;
		  // M0 - Unconditional stop - Wait for user button press on LCD
	    case 1: // M1 - Conditional stop - Wait for user button press on LCD
			{
				 #ifdef ULTIPANEL
				 // LCD_MESSAGEPGM(MSG_USERWAIT);	/////////////////////////////////////////////////
				  codenum = 0;
				  if(code_seen('P')) codenum = code_value(); // milliseconds to wait
				  if(code_seen('S')) codenum = code_value() * 1000; // seconds to wait
			
				  st_synchronize();
				  previous_millis_cmd = millis();
				  if (codenum > 0){
					codenum += millis();  // keep track of when we started waiting
					while(millis()  < codenum && !LCD_CLICKED){
					  manage_heater();
					  manage_inactivity();
					  lcd_update();	 //7.M1中的显示
									
					}
				  }else{
					while(!LCD_CLICKED){
					  manage_heater();
					  manage_inactivity();
					  lcd_update();	 //8.M1中的显示
									
					}
				  }
		   //   LCD_MESSAGEPGM(MSG_RESUMING); ////////////////////////////////
		   	  #endif
		      } 						
			     break;
			case 12:// M15  - update flash data (font data ; icon data and so on)	
			      for(i=0;i<6;i++)
							Password_read_buffer[i] = '0';
						FLASH_WRITE_VAR(FLASH_SET_STORE_OFFSET+177,Password_read_buffer);
						break;
			case 13:// M15  - update flash data (font data ; icon data and so on)	
			      printf("mem perused:%d%%\r\n",mem_perused(SRAMIN));			    
						break;
			case 14:// M14  - update flash data (font data ; icon data and so on)	
			      LCD_Clear(WHITE);//清屏				
						if(update_font(5,0,16,0)==0)
						{	
							delay_ms(500);						
						}
						nextMenu = home_screen; 		    
						break;
			case 15:// M15  - update flash data (font data ; icon data and so on)	
//				    nextMenu = system_screen;
						LCD_Clear(WHITE);//清屏				
						if(update_font(5,0,12,0)==0)
						{	LCD_ShowString(5,80,200,200,16, "UPDATE SUCCESS!"); 	
							delay_ms(500);						
						}
						update_theme(5,20,12);
					  nextMenu = home_screen;
//						LCD_Clear(WHITE);//清屏
//						 currentMenu = system_screen; /* function pointer to the currently active menu */
//              nextMenu = home_screen;  //进入下一个界面
//						__set_FAULTMASK(1); //关闭所有中断
//              NVIC_SystemReset();// 复位
						break;
			case 16:// M16  - screen_adjust
				      nextMenu = Tp_Adjust_screen;
		          printf("Adjust OK\r\n");	      
			
//						LCD_Clear(WHITE);//清屏
//						TP_Adjust();  //屏幕校准 
//						//TP_Save_Adjdata();	
//			      printf("Adjust OK\r\n");
//			        currentMenu = system_screen; /* function pointer to the currently active menu */
//              nextMenu = home_screen;  //进入下一个界面
						break;
	    case 17:
	       // LCD_MESSAGEPGM(MSG_NO_MOVE);	////////////////////////////////
	        enable_x();
	        enable_y();
	        enable_z();
	        enable_e0();
	       // enable_e1();
            break;
	#ifdef SDSUPPORT
//			case 19:// M19  截屏
//				//bmp_screen_shot(screen_shot_cnt);
//					screen_shot_cnt++;
//						break;
//	    case 20: // M20 - list SD card  
//		      printf(MSG_BEGIN_FILE_LIST);
//			  printf("\n");
//		      card_ls();
//		      printf(MSG_END_FILE_LIST);
//			  printf("\n");
//		      break;
//	    case 21: // M21 - init SD card	
//          f_mount(0,fs[0]); 					 	//挂载SD卡 //20160412		
//		      card_initsd();		
//		      break;
//	    case 22: //M22 - release SD card
//		      card_release();
//		      break;
//	    case 23: //M23 - Select file   选择文件开始打印
//		      starpos = (strchr(strchr_pointer + 4,'*'));
//		      if(starpos!=NULL)
//		        *(starpos)='\0';//   
//		      card_openFile(strchr_pointer + 4,true);//+4的意思是将指针指向'M23 '4个字节后的     
//					break;
//	    case 24: //M24 - Start SD print
//		      card.sdprinting = true;
			
//			    USART_ITConfig(USART3, USART_IT_RXNE, DISABLE);//20170213-8266
//					TIM_ITConfig(TIM7,TIM_IT_Update,DISABLE );//TIM_Cmd(TIM7, DISABLE);
			
//			    TIM_ITConfig(TIM3,TIM_IT_Update, ENABLE);//20160323
//		      starttime=millis();
//		      break;
//	    case 25: //M25 - Pause SD print
//		      card_pauseSDPrint();
//			    TIM_ITConfig(TIM3,TIM_IT_Update, DISABLE);//20160323
//		      break;
//	    case 26: //M26 - Set SD index
//		      if(card.cardOK && code_seen('S')) {
//		        card_setIndex(code_value_long());
//		      }
//		      break;
//	    case 27: //M27 - Get SD status
//		      card_getStatus();
//		      break;
//	    case 28: //M28 - Start SD write
//		      starpos = (strchr(strchr_pointer + 4,'*'));
//		     if(starpos != NULL)
//				  {//这一片都是还是让strchr_pointer指向'M',然后后面再+4指向文件名，没有用，因为没有这些，strchr_pointer也是指向'M'的
//		        char* npos = strchr(cmdbuffer[bufindr], 'N');
//		        strchr_pointer = strchr(npos,' ') + 1;
//		       *(starpos) = '\0';//*(starpos-1) = '\0';//20160401
//		      }
//		      card_openFile(strchr_pointer+4,false);//20160401
//		      break;
//	    case 29: //M29 - Stop SD write
//		      //processed in write to file routine above
//		      card.saving = false;
//		      break;
//	    case 30: //M30 <filename> Delete File
//		      if (card.cardOK){
//		        card_closefile();
//		        starpos = (strchr(strchr_pointer + 4,'*'));
//		        if(starpos != NULL){
//		          char* npos = strchr(cmdbuffer[bufindr], 'N');
//		          strchr_pointer = strchr(npos,' ') + 1;
//		          *(starpos-1) = '\0';
//		        }
//		        card_removeFile(strchr_pointer + 4);
//		      }
//		      break;
//		 case 32: //M32 - Select file and start SD print
//      {
//      if(card.sdprinting) {
//        st_synchronize();

//      }
//      starpos = (strchr(strchr_pointer + 4,'*'));

//      char* namestartpos = (strchr(strchr_pointer + 4,'!'));   //find ! to indicate filename string start.
//      if(namestartpos==NULL)
//      {
//        namestartpos=strchr_pointer + 4; //default name position, 4 letters after the M
//      }
//      else
//        namestartpos++; //to skip the '!'

//      if(starpos!=NULL)
//        *(starpos)='\0';

//      bool call_procedure=(code_seen('P'));

//      if(strchr_pointer>namestartpos)
//        call_procedure=false;  //false alert, 'P' found within filename

//      if( card.cardOK )
//      {
//        card.openFile(namestartpos,true,!call_procedure);
//        if(code_seen('S'))
//          if(strchr_pointer<namestartpos) //only if "S" is occuring _before_ the filename
//            card.setIndex(code_value_long());
//        card.startFileprint();
//        if(!call_procedure)
//          starttime=millis(); //procedure calls count as normal print time.
//      }
//     }
//			break;		
//	   case 928: //M928 - Start SD write
//      starpos = (strchr(strchr_pointer + 5,'*'));
//      if(starpos != NULL){
//        char* npos = strchr(cmdbuffer[bufindr], 'N');
//        strchr_pointer = strchr(npos,' ') + 1;
//        *(starpos) = '\0';
//      }
//      card.openLogFile(strchr_pointer+5);
//      break;
	#endif //SDSUPPORT
	    case 31: //M31 take time since the start of the SD print or an M109 command
		      {
		     // char time[30];
		      unsigned long t=(stoptime-starttime)/1000;
		      int sec,min;
					stoptime=millis();
		      min=t/60;
		      sec=t%60;
		      printf("%d min, %d sec", min, sec);
		      SERIAL_ECHO_START;
		     // printf("%s\n",time);
		     // lcd_setstatus(time); //////////////////////////////////////////
		      autotempShutdown();
		      }
		      break;
		case 42: //M42 -Change pin status via gcode
		     break;

		case 104: // M104
		      if(setTargetedHotend(104))
					{
		        break;
		      }
		      if (code_seen('S')) 
					{	
						setTargetHotend(code_value(), tmp_extruder);

							heater_temp[tmp_extruder]=target_temperature[tmp_extruder];//20161125  20170111

//						 heater_0_temp = target_temperature[tmp_extruder];
					}
		      setWatch();
		      break;
			case 112: //  M112 -Emergency Stop
          kill();	
			    break;
	    case 140: // M140 set bed temp
		      if (code_seen('S')) setTargetBed(code_value());
 		      	bed_temp = target_temperature_bed;
		      break;
	    case 105 : // M105
		      if(setTargetedHotend(105)){
		        break;
		      }
		      #if defined(TEMP_0_PIN)
			  		printf("ok T:%.1f /%.1f",degHotend(tmp_extruder),degTargetHotend(tmp_extruder));
		       // SERIAL_PROTOCOLPGM("ok T:");
		       // SERIAL_PROTOCOL_F(degHotend(tmp_extruder),1);
		       // SERIAL_PROTOCOLPGM(" /");
		      //  SERIAL_PROTOCOL_F(degTargetHotend(tmp_extruder),1);
		      	#if defined(TEMP_BED_PIN)
			  		printf(" B:%.1f /%.1f",degBed(),degTargetBed());
		        //  SERIAL_PROTOCOLPGM(" B:");
		        //  SERIAL_PROTOCOL_F(degBed(),1);
		        //  SERIAL_PROTOCOLPGM(" /");
		        //  SERIAL_PROTOCOL_F(degTargetBed(),1);
		       	#endif //TEMP_BED_PIN
		      #else
		        SERIAL_ERROR_START;
		        printf(MSG_ERR_NO_THERMISTORS);
		      #endif
		
		      //  SERIAL_PROTOCOLPGM(" @:");
		        printf(" @:%d",getHeaterPower(tmp_extruder));
				    printf(" B@:%d\n",getHeaterPower(-1));
		      //  SERIAL_PROTOCOLPGM(" B@:");
		      //  SERIAL_PROTOCOL(getHeaterPower(-1));
		      //  SERIAL_PROTOCOLLN("");
			  return;
		    //  break;
		case 109:
			{// M109 - Wait for extruder heater to reach target.
				bool target_direction;
//				long residencyStart;
				if(setTargetedHotend(109))break;
			 //  LCD_MESSAGEPGM(MSG_HEATING);	/////////////////////////////////////////
		        #ifdef AUTOTEMP
		        	 autotemp_enabled=false;
		        #endif
			    if (code_seen('S')) 
						setTargetHotend(code_value(), tmp_extruder);
					
							heater_temp[tmp_extruder]=target_temperature[tmp_extruder];//20161125  20170111
              FLASH_WRITE_VAR(BACKUP_TARGET_TEM_ADDRESS,target_temperature[0]);  //JEE2018
//				   heater_0_temp = target_temperature[0];
		        #ifdef AUTOTEMP
			        if (code_seen('S')) autotemp_min=code_value();
			        if (code_seen('B')) autotemp_max=code_value();	
			        if (code_seen('F'))
			        {
			          autotemp_factor=code_value();
			          autotemp_enabled=true;
			        }
			    #endif
				setWatch();
      			codenum = millis();
		      	/* See if we are heating up or cooling down */
      			target_direction = isHeatingHotend(tmp_extruder); // true if heating, false if cooling
				#ifdef TEMP_RESIDENCY_TIME
		        	residencyStart = -1;
		       /* continue to loop until we have reached the target temp
		          _and_ until TEMP_RESIDENCY_TIME hasn't passed since we reached it */                                       //20180628-->Abnormal_Flag = 0x06---打印完成
			        while(((residencyStart == -1) || (residencyStart >= 0 && (((unsigned int) (millis() - residencyStart)) < (TEMP_RESIDENCY_TIME * 1000UL))))&&(Outage_Flag == 0x00)&&(Abnormal_Flag != 0x03)&&(Abnormal_Flag != 0x06))
						  {//如果温度达到了目标温度的±3度认为是达到了目标温度，达到目标温度后再等10s稳定后再跳出循环
								 if( (millis() - codenum) > 1000UL )
							    { //Print Temp Reading and remaining time every 1 second while heating up/cooling down
							      //printf("T:%.1f E:%d",degHotend(tmp_extruder),tmp_extruder);
										if(tmp_extruder==0)
										  printf("T0:%.1f /%.1f",degHotend(tmp_extruder),degTargetHotend(tmp_extruder));
										else if(tmp_extruder==1)
										  printf("T1:%.1f /%.1f",degHotend(tmp_extruder),degTargetHotend(tmp_extruder));
			  			     
										
					          //  SERIAL_PROTOCOLPGM("T:");
					          //  SERIAL_PROTOCOL_F(degHotend(tmp_extruder),1);
					          //  SERIAL_PROTOCOLPGM(" E:");
					           // SERIAL_PROTOCOL((int)tmp_extruder);
					            #ifdef TEMP_RESIDENCY_TIME
					              printf(" W:");
					              if(residencyStart > -1)
					              {
					                 codenum = ((TEMP_RESIDENCY_TIME * 1000UL) - (millis() - residencyStart)) / 1000UL;
					                 printf("%ld\n", codenum );
					              }
					              else
					              {
					                 printf("?\n");
					              }
					            #else
					              printf("\n");
					            #endif
					            codenum = millis();
					         }
							  manage_heater();
          			manage_inactivity();
							// printf("%.1f",current_temperature[0]);//jee-----test
							  lcd_update();//9.循环显示 M109中等待打印头加热完成的显示
									 
					         #ifdef TEMP_RESIDENCY_TIME
					            /* start/restart the TEMP_RESIDENCY_TIME timer whenever we reach target temp for the first time
					              or when current temp falls outside the hysteresis after target temp was reached */
						         if ((residencyStart == -1 &&  target_direction && (degHotend(tmp_extruder) >= (degTargetHotend(tmp_extruder)-TEMP_WINDOW))) ||
						              (residencyStart == -1 && !target_direction && (degHotend(tmp_extruder) <= (degTargetHotend(tmp_extruder)+TEMP_WINDOW))) ||
						              (residencyStart > -1 && labs(degHotend(tmp_extruder) - degTargetHotend(tmp_extruder)) > TEMP_HYSTERESIS) )
						          {
						            residencyStart = millis();
						          }
					        #endif //TEMP_RESIDENCY_TIME
						  }
		     	#else
//		        	while ( target_direction ? (isHeatingHotend(tmp_extruder)) : (isCoolingHotend(tmp_extruder)&&(CooldownNoWait==false)) ) 
//						  { if( (millis() - codenum) > 1000UL )
//							  { //Print Temp Reading and remaining time every 1 second while heating up/cooling down
//							    printf("T:%.1f E:%d",degHotend(tmp_extruder),tmp_extruder);
//					          //  SERIAL_PROTOCOLPGM("T:");
//					          //  SERIAL_PROTOCOL_F(degHotend(tmp_extruder),1);
//					          //  SERIAL_PROTOCOLPGM(" E:");
//					           // SERIAL_PROTOCOL((int)tmp_extruder);
//					            #ifdef TEMP_RESIDENCY_TIME
//					              printf(" W:");
//					              if(residencyStart > -1)
//					              {
//					                 codenum = ((TEMP_RESIDENCY_TIME * 1000UL) - (millis() - residencyStart)) / 1000UL;
//					                 printf("%ld\n", codenum );
//					              }
//					              else
//					              {
//					                 printf("?\n");
//					              }
//					            #else
//					              printf("\n");
//					            #endif
//					            codenum = millis();
//					          }
//							  manage_heater();
//          					  manage_inactivity();
//							          //  lcd_update();////////////////////////////////////////////////
//					         #ifdef TEMP_RESIDENCY_TIME
//					            /* start/restart the TEMP_RESIDENCY_TIME timer whenever we reach target temp for the first time
//					              or when current temp falls outside the hysteresis after target temp was reached */
//						         if ((residencyStart == -1 &&  target_direction && (degHotend(tmp_extruder) >= (degTargetHotend(tmp_extruder)-TEMP_WINDOW))) ||
//						              (residencyStart == -1 && !target_direction && (degHotend(tmp_extruder) <= (degTargetHotend(tmp_extruder)+TEMP_WINDOW))) ||
//						              (residencyStart > -1 && labs(degHotend(tmp_extruder) - degTargetHotend(tmp_extruder)) > TEMP_HYSTERESIS) )
//						          {
//						            residencyStart = millis();
//						          }
//					        #endif //TEMP_RESIDENCY_TIME
//						  }
		      	#endif //TEMP_RESIDENCY_TIME
			    starttime=millis();//加热完重新计时
                previous_millis_cmd = millis();								
			}break;
		case 190: // M190 - Wait for bed heater to reach target.
		    #if defined(TEMP_BED_PIN)
		       // LCD_MESSAGEPGM(MSG_BED_HEATING);	   //////////////////////////////////////////////////////
		        if (code_seen('S')) setTargetBed(code_value());			
		        bed_temp = target_temperature_bed;
						FLASH_WRITE_VAR(BACKUP_BED_TARGET_TEM_ADDRESS,target_temperature_bed); //JEE2018
						    
		        codenum = millis();
		        residencyStart = -1;
		        while((isHeatingBed())&&(Outage_Flag == 0x00)&&(Abnormal_Flag != 0x03))//20170213
		        {
		          if(( millis() - codenum) > 1000 ) //Print Temp Reading every 1 second while heating up.
		          {
		            float tt=degHotend(tmp_extruder);//20170111
								printf("T:%.1f E:%d B:%.1f\n",tt,tmp_extruder,degBed());
		           // SERIAL_PROTOCOLPGM("T:");
		           // SERIAL_PROTOCOL(tt);
		           // SERIAL_PROTOCOLPGM(" E:");
		          //  SERIAL_PROTOCOL((int)active_extruder);
		          //  SERIAL_PROTOCOLPGM(" B:");
		          //  SERIAL_PROTOCOL_F(degBed(),1);
		           // SERIAL_PROTOCOLLN("");
		            codenum = millis();
		          }
		          manage_heater();
		          manage_inactivity();
		          lcd_update();//9.循环显示 M190中等待热床加热完成的显示
								
		        }
						residencyStart = millis();
		       // LCD_MESSAGEPGM(MSG_BED_DONE);	   //////////////////////////////////////////////////
		        previous_millis_cmd = millis();
			  #endif
		    break;

	      case 106: //M106 Fan On
	        break;
	      case 107: //M107 Fan Off
	        break;
		  case 126: //M126 valve open
			#ifdef BARICUDA
	        // PWM for HEATER_1_PIN
		        #if defined(HEATER_1_PIN) 
				    if (code_seen('S')){
			             ValvePressure=constrain(code_value(),0,255);
			          }
			          else {
			            ValvePressure=255;
			          }
			          
				#endif
			#endif
			break;
		  case 127: //M127 valve closed
			#ifdef BARICUDA
	        // PWM for HEATER_1_PIN
		        #if defined(HEATER_1_PIN) 
			          ValvePressure = 0;
				#endif
			#endif
			break;
		 case 128://M128 valve open
			#ifdef BARICUDA
	        // PWM for HEATER_1_PIN
		        #if defined(HEATER_2_PIN) 
			          if (code_seen('S')){
			             EtoPPressure=constrain(code_value(),0,255);
			          }
			          else {
			            EtoPPressure=255;
			          }
				#endif
			#endif
			break;
		  case 129: //M129 valve closed
			#ifdef BARICUDA
	        // PWM for HEATER_1_PIN
		        #if defined(HEATER_2_PIN) 
			          EtoPPressure = 0;
				#endif
			#endif
			break;
		 case 80: // M80 - ATX Power On
//	       SET_OUTPUT(PS_ON_PIN); //GND
//        WRITE(PS_ON_PIN, PS_ON_AWAKE);

//        // If you have a switch on suicide pin, this is useful
//        // if you want to start another print with suicide feature after
//        // a print without suicide...
//        #if defined SUICIDE_PIN && SUICIDE_PIN > -1
//            SET_OUTPUT(SUICIDE_PIN);
//            WRITE(SUICIDE_PIN, HIGH);
//        #endif

//        #ifdef ULTIPANEL
//          powersupply = true;
//          LCD_MESSAGEPGM(WELCOME_MSG);
//          lcd_update();
//        #endif WRITE(PS_ON_PIN, PS_ON_AWAKE);////////////////////////////////////////////////////
	        break;
	     case 81: // M81 - ATX Power Off
//				disable_heater();
//        st_synchronize();
//        disable_e0();
//        disable_e1();
//        disable_e2();
//        finishAndDisableSteppers();
//        LaserPower = 0;
//        delay(1000); // Wait a little before to switch off
//      #if defined(SUICIDE_PIN) && SUICIDE_PIN > -1
//        st_synchronize();
//        suicide();
//      #elif defined(PS_ON_PIN) && PS_ON_PIN > -1
//        SET_OUTPUT(PS_ON_PIN);
//        WRITE(PS_ON_PIN, PS_ON_ASLEEP);
//      #endif
//      #ifdef ULTIPANEL
//        powersupply = false;
//        LCD_MESSAGEPGM(MACHINE_NAME" "MSG_OFF".");
//        lcd_update();
//      #endif
		 	break;
	     case 82:
	        axis_relative_modes[3] = false;
	        break;
	     case 83:
	        axis_relative_modes[3] = true;
	        break;
	     case 18: //compatibility
	     case 84: // M84
	        if(code_seen('S'))
			{
	          stepper_inactive_time = code_value() * 1000;
	        }
			else
			{
		        bool all_axis = !((code_seen(axis_codes[0])) || (code_seen(axis_codes[1])) || (code_seen(axis_codes[2]))|| (code_seen(axis_codes[3])));
		        if(all_axis)
		        {
		          st_synchronize();
		          disable_e0();
//		          disable_e1();
		        //  disable_e2();
		          finishAndDisableSteppers();
		        }
		        else
		        {
		          st_synchronize();
		          if(code_seen('X')) disable_x();
		          if(code_seen('Y')) disable_y();
		          if(code_seen('Z')) disable_z();
		         // #if ((E0_ENABLE_PIN != X_ENABLE_PIN) && (E1_ENABLE_PIN != Y_ENABLE_PIN)) // Only enable on boards that have seperate ENABLE_PINS
		            if(code_seen('E')) {
		              disable_e0();
//		              disable_e1();
		            //  disable_e2();
		            }
		        //  #endif
		        }
		     }
			 break;
		  case 85: // M85
		     code_seen('S');
		     max_inactive_time = code_value() * 1000;
		     break;
		  case 92: // M92
		      for(i=0; i < NUM_AXIS; i++)
		      {
		        if(code_seen(axis_codes[i]))
		        {
							float value = code_value();
		          if(i == E_AXIS) 
							{ 
		            if(value < 20.0) {
		              float factor = axis_steps_per_unit[i] / value; // increase e constants if M92 E14 is given for netfab.
		              max_e_jerk *= factor;
		              max_feedrate[i] *= factor;
		              axis_steps_per_sqr_second[i] *= factor;
		            }  
		          }
							
						  if(i == X_AXIS)
							{
								printf("before:%f\r\n",x_offset_pos);
								x_offset_pos = x_offset_pos*axis_steps_per_unit[i]/value;
								printf("after:%f\r\n",x_offset_pos);
							}
							
						  if(i == Y_AXIS)
							{
								y_offset_pos = y_offset_pos*axis_steps_per_unit[i]/value;
							}

							axis_steps_per_unit[i] = value;
		        }
		       }
					reset_acceleration_rates();
//			   for(i=0; i < (NUM_AXIS-1); i++)
//		      {
//		        if(code_seen(axis_codes[i]))
//		        {
//		          axis_steps_per_unit[i] = code_value();
//		        }
//		      }
//					reset_acceleration_rates();//设置axis_steps_per_sqr_second[i] = max_acceleration_units_per_sq_second[i] * axis_steps_per_unit[i]
//					if(code_seen('E')) 
//						{ // E
//		            float value = code_value();
//		            if(value < 20.0) {
//		              float factor = axis_steps_per_unit[3] / value; // increase e constants if M92 E14 is given for netfab.
//		              max_e_jerk *= factor;
//		              max_feedrate[3] *= factor;
//		              axis_steps_per_sqr_second[3] *= factor;
//								}
//								else
//								{
//								  axis_steps_per_unit[3] = code_value();
//									reset_acceleration_rates();
//								}
//						}	
			    break; 
	      case 115: // M115
		      printf(MSG_M115_REPORT);
		      break;
	      case 117: // M117 display message		/////////////////////////////////////////////////////
		      starpos = (strchr(strchr_pointer + 5,'*'));
		      if(starpos!=NULL)
		        *(starpos-1)='\0';
		   //   lcd_setstatus(strchr_pointer + 5);	 //////////////////////////////////////////////////
		      break;
		  case 114: // M114	 
				  printf("X:%f Y:%f Z:%f E:%f",current_position[X_AXIS],current_position[Y_AXIS],current_position[Z_AXIS],current_position[E_AXIS]);			
			    printf(MSG_COUNT_X);
				  printf("%f Y:%f Z:%f\n",((float)st_get_position(X_AXIS))/axis_steps_per_unit[X_AXIS],((float)st_get_position(Y_AXIS))/axis_steps_per_unit[Y_AXIS],((float)st_get_position(Z_AXIS))/axis_steps_per_unit[Z_AXIS]);
		      break; 
		   case 120: // M120
		      enable_endstops(false) ;
		      break;
		   case 121: // M121
		      enable_endstops(true) ;
		      break;
		   case 119: // M119
		      printf(MSG_M119_REPORT);
			  printf("\n");
		      #if defined(X_MIN_PIN) 
		        printf(MSG_X_MIN);
				printf(((X_MIN_PIN==X_ENDSTOPS_INVERTING)?MSG_ENDSTOP_HIT:MSG_ENDSTOP_OPEN));
				printf("\n");
		       // (X_MIN_PIN==X_ENDSTOPS_INVERTING) ? (printf(MSG_ENDSTOP_HIT)) : (printf(MSG_ENDSTOP_OPEN));
		      #endif
		      #if defined(X_MAX_PIN)
		        printf(MSG_X_MAX);
		        printf(((X_MAX_PIN==X_ENDSTOPS_INVERTING)?MSG_ENDSTOP_HIT:MSG_ENDSTOP_OPEN));
				printf("\n");
		      #endif
		      #if defined(Y_MIN_PIN)
		        printf(MSG_Y_MIN);
		        printf(((Y_MIN_PIN^Y_ENDSTOPS_INVERTING)?MSG_ENDSTOP_HIT:MSG_ENDSTOP_OPEN));
				printf("\n");
		      #endif
		      #if defined(Y_MAX_PIN) 
		        printf(MSG_Y_MAX);
		        printf(((Y_MAX_PIN^Y_ENDSTOPS_INVERTING)?MSG_ENDSTOP_HIT:MSG_ENDSTOP_OPEN));
				printf("\n");
		      #endif
		      #if defined(Z_MIN_PIN) 
		        printf(MSG_Z_MIN);
		        printf(((Z_MIN_PIN^Z_ENDSTOPS_INVERTING)?MSG_ENDSTOP_HIT:MSG_ENDSTOP_OPEN));
				printf("\n");
		      #endif
		      #if defined(Z_MAX_PIN)
		        printf(MSG_Z_MAX);
		        printf(((Z_MAX_PIN^Z_ENDSTOPS_INVERTING)?MSG_ENDSTOP_HIT:MSG_ENDSTOP_OPEN));
				printf("\n");
		      #endif
		      break;
			  //TODO: update for all axis, use for loop
		  case 201: // M201	 
		      for( i=0; i < NUM_AXIS; i++)
		      {
		        if(code_seen(axis_codes[i]))
		        {
		          max_acceleration_units_per_sq_second[i] = code_value();
		        }
		      }
		      // steps per sq second need to be updated to agree with the units per sq second (as they are what is used in the planner)
		      reset_acceleration_rates();
		      break;
	      case 202: // M202
		  	#if 0 // Not used for Sprinter/grbl gen6
		      for(i=0; i < NUM_AXIS; i++) {
		        if(code_seen(axis_codes[i])) axis_travel_steps_per_sqr_second[i] = code_value() * axis_steps_per_unit[i];
		      }
			#endif
		      break;
	      case 203: // M203 max feedrate mm/sec
		      for( i=0; i < NUM_AXIS; i++) 
				  {
		        if(code_seen(axis_codes[i])) max_feedrate[i] = code_value();
		      }
		      break; 
	      case 204: // M204 acclereration S normal moves T filmanent only moves
		      {
		        if(code_seen('S')) acceleration = code_value();
		        if(code_seen('T')) retract_acceleration = code_value();
		      }
	          break;
	      case 205: //M205 advanced settings:  minimum travel speed S=while printing T=travel only,  B=minimum segment time X= maximum xy jerk, Z=maximum Z jerk
		    {
		      if(code_seen('S')) minimumfeedrate = code_value();
		      if(code_seen('T')) mintravelfeedrate = code_value();
		      if(code_seen('B')) minsegmenttime = code_value();
		      if(code_seen('X')) max_xy_jerk = code_value();
		      if(code_seen('Z')) max_z_jerk = code_value();
		      if(code_seen('E')) max_e_jerk = code_value();
					}
	    	break;
	      case 206: // M206 additional homeing offset
		      for( i=0; i < 3; i++)
		      {
		        if(code_seen(axis_codes[i])) add_homeing[i] = code_value();
		      }
		      break;
        #ifdef FWRETRACT
		    case 207: //M207 - set retract length S[positive mm] F[feedrate mm/sec] Z[additional zlift/hop]
		    {
		      if(code_seen('S'))
		      {
		        retract_length = code_value() ;
		      }
		      if(code_seen('F'))
		      {
		        retract_feedrate = code_value() ;
		      }
		      if(code_seen('Z'))
		      {
		        retract_zlift = code_value() ;
		      }
		    }break;
		    case 208: // M208 - set retract recover length S[positive mm surplus to the M207 S*] F[feedrate mm/sec]
		    {
		      if(code_seen('S'))
		      {
		        retract_recover_length = code_value() ;
		      }
		      if(code_seen('F'))
		      {
		        retract_recover_feedrate = code_value() ;
		      }
		    }break;
		    case 209: // M209 - S<1=true/0=false> enable automatic retract detect if the slicer did not support G10/11: every normal extrude-only move will be classified as retract depending on the direction.
		    {
		      if(code_seen('S'))
		      {
		        int t= code_value() ;
		        switch(t)
		        {
		          case 0: autoretract_enabled=false;retracted=false;break;
		          case 1: autoretract_enabled=true;retracted=false;break;
		          default:
		            SERIAL_ECHO_START;
		            printf(MSG_UNKNOWN_COMMAND);
		            printf("%d",cmdbuffer[bufindr]);
		            printf("\"");
		        }
		      }
		
		    }break;
   			#endif // FWRETRACT
		    #if EXTRUDERS > 1
			case 218: // M218 - set hotend offset (in mm), T<extruder_number> X<offset_on_X> Y<offset_on_Y>
			    {
			      if(setTargetedHotend(218)){
			        break;
			      }
			      if(code_seen('X'))
			      {
			        extruder_offset[X_AXIS][tmp_extruder] = code_value();
			      }
			      if(code_seen('Y'))
			      {
			        extruder_offset[Y_AXIS][tmp_extruder] = code_value();
			      }
			      SERIAL_ECHO_START;
			      printf(MSG_HOTEND_OFFSET);
			      for(tmp_extruder = 0; tmp_extruder < EXTRUDERS; tmp_extruder++)
			      {
			         //SERIAL_ECHO(" ");
			         printf(" %f,%f",extruder_offset[X_AXIS][tmp_extruder],extruder_offset[Y_AXIS][tmp_extruder]);
			        // SERIAL_ECHO(",");
			       //  SERIAL_ECHO(extruder_offset[Y_AXIS][tmp_extruder]);
			      }
			      printf("\n");
			    }break;
		   #endif
		   case 220: // M220 S<factor in percent>- set speed factor override percentage
			    {
			      if(code_seen('S'))
			      {
			        feedmultiply = code_value() ;
			      }
			    }
		      break;
		   case 221: // M221 S<factor in percent>- set extrude factor override percentage
			    {
			      if(code_seen('S'))
			      {
			        extrudemultiply = code_value() ;
			      }
			    }
		      break;
		   #if NUM_SERVOS > 0
		   case 280: // M280 - set servo position absolute. P: servo index, S: angle or microseconds
		      {
		        int servo_index = -1;
		        int servo_position = 0;
		        if (code_seen('P'))
		          servo_index = code_value();
		        if (code_seen('S')) {
		          servo_position = code_value();
		          if ((servo_index >= 0) && (servo_index < NUM_SERVOS)) {
		            servos[servo_index].write(servo_position);
		          }
		          else {
		            SERIAL_ECHO_START;
		            SERIAL_ECHO("Servo ");
		            SERIAL_ECHO(servo_index);
		            SERIAL_ECHOLN(" out of range");
		          }
		        }
		        else if (servo_index >= 0) {
		          SERIAL_PROTOCOL(MSG_OK);
		          SERIAL_PROTOCOL(" Servo ");
		          SERIAL_PROTOCOL(servo_index);
		          SERIAL_PROTOCOL(": ");
		          SERIAL_PROTOCOL(servos[servo_index].read());
		          SERIAL_PROTOCOLLN("");
		        }
		      }
		      break;
		    #endif // NUM_SERVOS > 0
		   // #if LARGE_FLASH == true && ( BEEPER > 0 || defined(ULTRALCD) )
		    case 300: // M300
		    {
//		    //  int beepS = 400;
//		      int beepP = 1000;
//		      //if(code_seen('S')) beepS = code_value();//喇叭频率
//		      if(code_seen('P')) beepP = code_value();//发声时间
//		   //   #if BEEPER > 0
//						BEEP=1;  
//						delay_ms(beepP);
//						BEEP=0;
		   //   #elif defined(ULTRALCD)
		    //    lcd_buzz(beepS, beepP);
		    //  #endif
		    }
		    break;
		 //   #endif // M300
       #ifdef PIDTEMP
	    case 301: // M301
	      {
	        if(code_seen('P'))  {Default_Kp = code_value(); Kp = code_value();}
	        if(code_seen('I')) {Default_Ki = code_value();Ki = scalePID_i(code_value());}
	        if(code_seen('D')) {Default_Kd = code_value();Kd = scalePID_d(code_value());}
	
	        #ifdef PID_ADD_EXTRUSION_RATE
	        if(code_seen('C')) Kc = code_value();
	        #endif
	
	        updatePID();
	        printf(MSG_OK);
					printf(" p:%f i:%f d:%f",Kp,unscalePID_i(Ki),unscalePID_d(Kd));
	       // SERIAL_PROTOCOL(" p:");
	       // SERIAL_PROTOCOL(Kp);
	       // SERIAL_PROTOCOL(" i:");
	       // SERIAL_PROTOCOL(unscalePID_i(Ki));
	       // SERIAL_PROTOCOL(" d:");
	       // SERIAL_PROTOCOL(unscalePID_d(Kd));
	        #ifdef PID_ADD_EXTRUSION_RATE
	       
	        //Kc does not have scaling applied above, or in resetting defaults
			printf(" c:%f",Kc);
	      //  SERIAL_PROTOCOL(Kc);
	        #endif
	        printf("\n");
	      }
	      break;
	    #endif //PIDTEMP
	    #ifdef PIDTEMPBED
	    case 304: // M304
	      {
	        if(code_seen('P')) bedKp = code_value();
	        if(code_seen('I')) bedKi = scalePID_i(code_value());
	        if(code_seen('D')) bedKd = scalePID_d(code_value());
	
	        updatePID();
					printf(MSG_OK);
					printf(" p:%f i:%f d:%f",Kp,unscalePID_i(bedKi,unscalePID_d(bedKd)));
					printf("\n");
	//         SERIAL_PROTOCOL(MSG_OK);
	//         SERIAL_PROTOCOL(" p:");
	//         SERIAL_PROTOCOL(bedKp);
	//         SERIAL_PROTOCOL(" i:");
	//         SERIAL_PROTOCOL(unscalePID_i(bedKi));
	//         SERIAL_PROTOCOL(" d:");
	//         SERIAL_PROTOCOL(unscalePID_d(bedKd));
	//         SERIAL_PROTOCOLLN("");
	      }
	      break;
	    #endif //PIDTEMP
	    case 240: // M240  Triggers a camera by emulating a Canon RC-1 : http://www.doc-diy.net/photo/rc-1_hacked/
		     {
		      #if defined(PHOTOGRAPH_PIN)
		//         const uint8_t NUM_PULSES=16;
		//         const float PULSE_LENGTH=0.01524;
		//         for(int i=0; i < NUM_PULSES; i++) {
		//           WRITE(PHOTOGRAPH_PIN, HIGH);
		//           _delay_ms(PULSE_LENGTH);
		//           WRITE(PHOTOGRAPH_PIN, LOW);
		//           _delay_ms(PULSE_LENGTH);
		//         }
		//         delay(7.33);
		//         for(int i=0; i < NUM_PULSES; i++) {
		//           WRITE(PHOTOGRAPH_PIN, HIGH);
		//           _delay_ms(PULSE_LENGTH);
		//           WRITE(PHOTOGRAPH_PIN, LOW);
		//           _delay_ms(PULSE_LENGTH);
		//         }
		      #endif
		     }
		    break;
		    case 303: // M303 PID autotune
		    {
		      float temp = 150.0;
		      int e=0;
		      int c=5;
		      if (code_seen('E')) e=code_value();
		        if (e<0)
		          temp=70;
		      if (code_seen('S')) temp=code_value();
		      if (code_seen('C')) c=code_value();
		      PID_autotune(temp, e, c);
		    }
		    break;
		    case 400: // M400 finish all moves
		    {
		      st_synchronize();
		    }
		    break;
		    case 500: // M500 Store settings in EEPROM
		    {
		        Config_StoreSettings();////////////////////////////////////////////////////////////////////////////////////////
		    }
		    break;
		    case 501: // M501 Read settings from EEPROM
		    {
		        Config_RetrieveSettings();////////////////////////////////////////////////////////////////////////////////////
		    }
		    break;
		    case 502: // M502 Revert to default settings
		    {
		        Read_Factor_Refault();//Config_ResetDefault();//////////////////////////////////////////////////////////////////////////////////////////
		    }
		    break;
		    case 503: // M503 print settings currently in memory
		    {
		        Config_PrintSettings();///////////////////////////////////////////////////////////////////////////////////////////
		    }
		    break;
	    #ifdef ABORT_ON_ENDSTOP_HIT_FEATURE_ENABLED
		 case 540:
		    {
		        if(code_seen('S')) abort_on_endstop_hit = code_value() > 0;
		    }
		    break;
		#endif

	    #ifdef FILAMENTCHANGEENABLE
	    case 600: //Pause for filament change X[pos] Y[pos] Z[relative lift] E[initial retract] L[later retract distance for removal]
	    {
	        float target[4];
	        float lastpos[4];
	        target[X_AXIS]=current_position[X_AXIS];
	        target[Y_AXIS]=current_position[Y_AXIS];
	        target[Z_AXIS]=current_position[Z_AXIS];
	        target[E_AXIS]=current_position[E_AXIS];
	        lastpos[X_AXIS]=current_position[X_AXIS];
	        lastpos[Y_AXIS]=current_position[Y_AXIS];
	        lastpos[Z_AXIS]=current_position[Z_AXIS];
	        lastpos[E_AXIS]=current_position[E_AXIS];
	        //retract by E
	        if(code_seen('E'))
	        {
	          target[E_AXIS]+= code_value();
	        }
	        else
	        {
	          #ifdef FILAMENTCHANGE_FIRSTRETRACT
	            target[E_AXIS]+= FILAMENTCHANGE_FIRSTRETRACT ;
	          #endif
	        }
	        plan_buffer_line(target[X_AXIS], target[Y_AXIS], target[Z_AXIS], target[E_AXIS], feedrate/60, active_extruder);
	
	        //lift Z
	        if(code_seen('Z'))
	        {
	          target[Z_AXIS]+= code_value();
	        }
	        else
	        {
	          #ifdef FILAMENTCHANGE_ZADD
	            target[Z_AXIS]+= FILAMENTCHANGE_ZADD ;
	          #endif
	        }
	        plan_buffer_line(target[X_AXIS], target[Y_AXIS], target[Z_AXIS], target[E_AXIS], feedrate/60, active_extruder);
	
	        //move xy
	        if(code_seen('X'))
	        {
	          target[X_AXIS]+= code_value();
	        }
	        else
	        {
	          #ifdef FILAMENTCHANGE_XPOS
	            target[X_AXIS]= FILAMENTCHANGE_XPOS ;
	          #endif
	        }
	        if(code_seen('Y'))
	        {
	          target[Y_AXIS]= code_value();
	        }
	        else
	        {
	          #ifdef FILAMENTCHANGE_YPOS
	            target[Y_AXIS]= FILAMENTCHANGE_YPOS ;
	          #endif
	        }
	
	        plan_buffer_line(target[X_AXIS], target[Y_AXIS], target[Z_AXIS], target[E_AXIS], feedrate/60, active_extruder);
	
	        if(code_seen('L'))
	        {
	          target[E_AXIS]+= code_value();
	        }
	        else
	        {
	          #ifdef FILAMENTCHANGE_FINALRETRACT
	            target[E_AXIS]+= FILAMENTCHANGE_FINALRETRACT ;
	          #endif
	        }
	
	        plan_buffer_line(target[X_AXIS], target[Y_AXIS], target[Z_AXIS], target[E_AXIS], feedrate/60, active_extruder);
	
	        //finish moves
	        st_synchronize();
	        //disable extruder steppers so filament can be removed
	        disable_e0();
	        disable_e1();
	        disable_e2();
	        delay_ms(100);
	      //  LCD_ALERTMESSAGEPGM(MSG_FILAMENTCHANGE);/////////////////////////////////////////////
	        cnt=0;
	         while(!lcd_clicked){
	          cnt++;
	           manage_heater();
	           manage_inactivity();
	           lcd_update();//M600中的显示
						 	
	           if(cnt==0)
	           {
				       beep();
	           }
	         }
	
	        //return to normal
	        if(code_seen('L'))
	        {
	          target[E_AXIS]+= -code_value();
	        }
	        else
	        {
	          #ifdef FILAMENTCHANGE_FINALRETRACT
	            target[E_AXIS]+=(-1)*FILAMENTCHANGE_FINALRETRACT ;
	          #endif
	        }
	        current_position[E_AXIS]=target[E_AXIS]; //the long retract of L is compensated by manual filament feeding
	        plan_set_e_position(current_position[E_AXIS]);
	        plan_buffer_line(target[X_AXIS], target[Y_AXIS], target[Z_AXIS], target[E_AXIS], feedrate/60, active_extruder); //should do nothing
	        plan_buffer_line(lastpos[X_AXIS], lastpos[Y_AXIS], target[Z_AXIS], target[E_AXIS], feedrate/60, active_extruder); //move xy back
	        plan_buffer_line(lastpos[X_AXIS], lastpos[Y_AXIS], lastpos[Z_AXIS], target[E_AXIS], feedrate/60, active_extruder); //move z back
	        plan_buffer_line(lastpos[X_AXIS], lastpos[Y_AXIS], lastpos[Z_AXIS], lastpos[E_AXIS], feedrate/60, active_extruder); //final untretract
	    }
	    break;
	    #endif //FILAMENTCHANGEENABLE
			case 700: //M700 Laser On
		   #if defined(LASER_PIN) 
		        if (code_seen('S'))	LaserPower=constrain(code_value(),0,255);
		        else	LaserPower=255;
		   #endif //LASER_PIN
	        break;
			case 701: //M701 Laser Off
		   #if defined(LASER_PIN) 
		        if (code_seen('S'))	LaserPower=constrain(code_value(),0,255);
		        else	LaserPower=0;
		   #endif //LASER_PIN
	        break;
			case 702: //M702 enter axis_adjust
			{
				axis_adjust = true;		
			}
	    break;		
			case 703: //M703 exit axis_adjust
			{
				axis_adjust = false;		
			}
	    break;					
			case 704: //M703 set  axis offset
			{

					if(code_seen('X')) x_offset_pos=code_value();
					if(code_seen('Y')) y_offset_pos=code_value();
						
			}
	    break;	
			case 705: //M705 center  galvo
			{	
					if(code_seen('X')) set_X_galvo(code_value_long());
					if(code_seen('Y')) set_Y_galvo(code_value_long());	
			}
	    break;				
			
	    case 907: // M907 Set digital trimpot motor current using axis codes.
	    {
	        for(i=0;i<NUM_AXIS;i++) if(code_seen(axis_codes[i])) digipot_current(i,code_value());
	        if(code_seen('B')) digipot_current(4,code_value());
	        if(code_seen('S')) for(i=0;i<=4;i++) digipot_current(i,code_value());
	    }
	    break;
	    case 908: // M908 Control digital trimpot directly.
	    {
	        uint8_t channel,current;
	        if(code_seen('P')) channel=code_value();
	        if(code_seen('S')) current=code_value();
	        digipot_current(channel, current);
	    }
	    break;
	    case 350: // M350 Set microstepping mode. Warning: Steps per unit remains unchanged. S code sets stepping mode for all drivers.
	    { 
	        if(code_seen('S')) for( i=0;i<=4;i++) microstep_mode(i,code_value());
	        for( i=0;i<NUM_AXIS;i++) if(code_seen(axis_codes[i])) microstep_mode(i,(uint8_t)code_value());
	        if(code_seen('B')) microstep_mode(4,code_value());
	        microstep_readings();
	    }
	    break;
	    case 351: // M351 Toggle MS1 MS2 pins directly, S# determines MS1 or MS2, X# sets the pin high/low.
	    {
	      if(code_seen('S')) switch((int)code_value())
	      {
	        case 1:
	          for(i=0;i<NUM_AXIS;i++) if(code_seen(axis_codes[i])) microstep_ms(i,code_value(),-1,-1);
	          if(code_seen('B')) microstep_ms(4,code_value(),-1,-1);
	          break;
	        case 2:
	          for(i=0;i<NUM_AXIS;i++) if(code_seen(axis_codes[i])) microstep_ms(i,-1,code_value(),-1);
	          if(code_seen('B')) microstep_ms(4,-1,code_value(),-1);
	          break;
					case 3:
	          for(i=0;i<NUM_AXIS;i++) if(code_seen(axis_codes[i])) microstep_ms(i,-1,-1,code_value());
	          if(code_seen('B')) microstep_ms(4,-1,-1,code_value());
	          break;
	      }
	      microstep_readings();
	
	    }
	    break;
////////////////////////////wifi esp8266
			 
			 
	case 999: // M999: Restart after being stopped
	      Stopped = false;
	    //  lcd_reset_alert_level();//////////////////////////////////////
	      gcode_LastN = Stopped_gcode_LastN;
	      FlushSerialRequestResend();
	    break;	
	
	case 1001: // M1001
	      if(code_seen('X')) x_max_pos = code_value() ;
	      if(code_seen('Y')) y_max_pos = code_value() ;
		    if(code_seen('Z')) z_max_pos = code_value() ;
	      Print_parameter_init();
	    break;
  case 1002: // M1002
	      if(code_seen('X')) max_feedrate[0] = code_value() ;
	      if(code_seen('Y')) max_feedrate[1] = code_value() ;
		    if(code_seen('Z')) max_feedrate[2] = code_value() ;
	      if(code_seen('E')) max_feedrate[3] = code_value() ;
	    break;	
  case 1003: // M1003
		    if(code_seen('X')) invert_x_dir = code_value();
	      if(code_seen('Y')) invert_y_dir = code_value();
		    if(code_seen('Z')) invert_z_dir = code_value();
	      if(code_seen('E')) invert_e0_dir = code_value();
	    break;	
  case 1004: // M1004
	      if(code_seen('X')) x_endstops_inverting = code_value();
	      if(code_seen('Y')) y_endstops_inverting = code_value();
		    if(code_seen('Z')) z_endstops_inverting = code_value();
	      if(code_seen('E')) e0_endstops_inverting = code_value();
	      if(code_seen('L')) level_endstops_inverting = code_value();
	    break;
  case 1005: 
	      if(code_seen('S')) heater_0_maxtemp = code_value();
				tp_parameter_init();//点击确定，就重新初始化最大值有关的变量
	    break;	
	case 1006: 
	      if(code_seen('S')) bed_maxtemp = code_value();
				tp_parameter_init();//点击确定，就重新初始化最大值有关的变量
	    break;	
	case 1007: 
	      if(code_seen('S')) heater_0_PLAtemp = code_value();
	    break;	
	case 1008: 
	      if(code_seen('S')) heater_0_ABStemp = code_value();
	    break;	
	case 1011: 
	      if(code_seen('T')) gui_phy.language = code_value();
	    break;
	case 1012: 
	      if(code_seen('T')) emc_switch = code_value();
	    break;
	case 1013: 
	      if(code_seen('T')) temp_sensor_type = code_value();
	    break;
	case 1014: 
	      if(code_seen('T')) no_material_switch = code_value();
	    break;
	case 1015: 
	      if(code_seen('T')) auto_shutdown_switch = code_value();
	    break;
	case 1017: // M1003
		    if(code_seen('D')) diagonal = code_value();
	      if(code_seen('S')) smooth_rod = code_value();
		    if(code_seen('E')) effector = code_value();
	      if(code_seen('C')) carriage = code_value();
	    break;	
	case 1018: 
	     
	    break;
	case 1019: 
	      if(code_seen('T')) bed_level_switch = code_value();
	    break;
	case 1020: 
	      if(code_seen('T')) temp_sensor_num = code_value();
	    break;
	case 1021: 
	      if(code_seen('T')) backup_power = code_value();
	    break;
	case 1022: 
	      if(code_seen('S')) pulse_width = code_value();
	    break;
	case 1023: 
	      if(code_seen('S')) pause_length = code_value();
	    break;
	case 1024: 
	      if(code_seen('S')) extrude_length_advance = code_value();
	    break;
	case 1025: 
//	      if(code_seen('S')) appoint_z_height = code_value();
	    break;
	case 1026:
	      if(code_seen('S')) floor_height = code_value();
	    break;
	case 1027:
	      if(code_seen('X')) add_homeing_ofsize[0] = code_value();
	      if(code_seen('Y')) add_homeing_ofsize[1] = code_value();
		    if(code_seen('Z')) add_homeing_ofsize[2] = code_value();
	    break;
	case 1028:
	      if(code_seen('S')) z_homeing_ofsize = code_value();
	    break;
	case 1029://设置各轴归位时坐标方向0?max
	      if(code_seen('X')) home_dir[0] = code_value();
	      if(code_seen('Y')) home_dir[1] = code_value();
		    if(code_seen('Z')) home_dir[2] = code_value();;
	    break;
	case 1030://设置打印头最小挤出温度
	      if(code_seen('S')) heater_0_mintemp = code_value();
	    break;
	
	
	
	
	
		//屏幕截屏
//	case 1990:
//		   if(code_seen('S'))bmp_screen_shot(code_value());
//	    break;
		/////颜色设置
	case 1993://标题栏前颜色	       
	         GUI_ICON_FONTCOL =  code_getcolor(0);         
	     break;
	case 1994://标题栏背颜色	       
	         GUI_TITLE_BACKCOL =  code_getcolor(1);         
	     break;
	case 1995://背颜色	       
//	         GUI_BODY_BACKCOL =  code_getcolor(2);         
	     break;
	case 1996://logo持续时间
        if(code_seen('L'))  
			  { 
	        logo_hold_time = code_value();
					if(logo_hold_time==0)
						logo_hold_time=1;
          else if(logo_hold_time>10)
					  logo_hold_time=10;
			  }					  
	     break;
	case 1997: //默认只支持横屏旋转180度	不等于0即认为要旋转屏幕 	
//         FLASH_READ_VAR(LCD_SCANDIR_ADRASS,LCDScanDir);		
//		     if(LCDScanDir!=L2R_U2D)
//					{
//					 LCDScanDir= L2R_U2D;											
//					}else LCDScanDir=R2L_D2U;
//				 FLASH_WRITE_VAR(LCD_SCANDIR_ADRASS,LCDScanDir);	
//			   LCD_Init();				//初始化LCD 
//         TP_Init();  //屏幕校准
		    break;
	case 1998: 
			   Write_Factor_Refault();//将数据写入恢复出厂区域
		break;
	case 1999: 
	      Config_StoreSettings();//将数据写入平时的存储区
	      
        __set_FAULTMASK(1); //关闭所有中断
        NVIC_SystemReset();// 复位	
	    break;
	}

  }
   else if(code_seen('T'))//没有G，也没有M，有T
   {
		 active_extruder= code_value();//20161218
		 if(temp_sensor_num==1)
			 tmp_extruder = 0;//20170111
			else if(temp_sensor_num==2)
			 tmp_extruder = active_extruder;//20170111
		 printf("active_extruder=%d",active_extruder);
    if(tmp_extruder >= EXTRUDERS) 
		{
      SERIAL_ECHO_START;
      printf("T%d",tmp_extruder);
     // SERIAL_ECHO(tmp_extruder);
      printf(MSG_INVALID_EXTRUDER);
    }
		else 
		{
			volatile bool make_move = false;
				if(code_seen('F')) 
			  {
					make_move = true;
					next_feedrate = code_value();
					if(next_feedrate > 0.0) {
						feedrate = next_feedrate;
					}
				}
//			#if EXTRUDERS > 1
//			if(tmp_extruder != active_extruder) 
//			{
//					// Save current position to return to after applying extruder offset
//					memcpy(destination, current_position, sizeof(destination));
//					// Offset extruder (only by XY)
//					for(i = 0; i < 2; i++) 
//						{
//						 current_position[i] = current_position[i] -\
//																	 extruder_offset[i][active_extruder] +\
//																	 extruder_offset[i][tmp_extruder];
//					}
//					// Set the new active extruder and position
//					active_extruder = tmp_extruder;
//					plan_set_position(current_position[X_AXIS], current_position[Y_AXIS], current_position[Z_AXIS], current_position[E_AXIS]);
//					// Move to the old position if 'F' was in the parameters
//					if(make_move && Stopped == false) {
//						 prepare_move();
//					}
//				}	//end   if(tmp_extruder != active_extruder) 
//			 #endif
			SERIAL_ECHO_START;
				printf(MSG_ACTIVE_EXTRUDER);
				printf("%d",active_extruder);
			printf("\n");
		}
  }//end else if(code_seen('T'))
  else
  { SERIAL_ECHO_START;
    printf(MSG_UNKNOWN_COMMAND);
    printf("%s",cmdbuffer[bufindr]);
    printf("\"");
  }
   ClearToSend();
}

void FlushSerialRequestResend()
{
  //char cmdbuffer[bufindr][100]="Resend:";
  MYSERIAL_flush();
  printf(MSG_RESEND);
  printf("%d\n",gcode_LastN + 1);
  ClearToSend();
}

void ClearToSend()
{
  previous_millis_cmd = millis();
  #ifdef SDSUPPORT
  if(fromsd[bufindr])
    return;
  #endif //SDSUPPORT
  
  printf(MSG_OK);
  printf("\n");
}

void get_coordinates()
{ int8_t i;
  volatile bool seen[4]={false,false,false,false};
  for( i=0; i < (NUM_AXIS-1); i++) 
	{
    if(code_seen(axis_codes[i]))
    {
      destination[i] = (float)code_value() + (axis_relative_modes[i] || relative_mode)*current_position[i];
      seen[i]=true;
    }
    else destination[i] = current_position[i]; //Are these else lines really needed?
		
		
				//		float d =  sqrt(150*150+(float)(count_position[Y_AXIS]*count_position[Y_AXIS])/(axis_steps_per_unit[Y_AXIS]*axis_steps_per_unit[Y_AXIS]))+ 5;
		//		long  x = atan((count_position[X_AXIS]/axis_steps_per_unit[X_AXIS])/d)*d*axis_steps_per_unit[X_AXIS];
			//	printf("D:%f\r\n",d);
			//	printf("-:%ld\r\n",count_position[X_AXIS]);
			//	printf("X:%ld\r\n",x);
		//		set_X_galvo(x+32767);
	#ifdef DEBUG_PRINTF 
	printf("%d：%f\n",i,destination[i]);
	#endif
  }
		
	if((code_seen('E'))||(code_seen('A')))
    {
      destination[E_AXIS] = (float)code_value() + (axis_relative_modes[E_AXIS] || relative_mode)*current_position[E_AXIS];
      seen[E_AXIS]=true;
    }
    else 
			destination[E_AXIS] = current_position[E_AXIS];
	
	
    if(code_seen('F')) 
		{
			next_feedrate = code_value();

			feedrate = next_feedrate;

		}

  	#ifdef DEBUG_PRINTF 
	printf("feedrate：%f\n",feedrate);
	#endif

  #ifdef FWRETRACT
  if(autoretract_enabled)
  if( !(seen[X_AXIS] || seen[Y_AXIS] || seen[Z_AXIS]) && seen[E_AXIS])
  {
    float echange=destination[E_AXIS]-current_position[E_AXIS];
    if(echange<-MIN_RETRACT) //retract
    {
      if(!retracted)
      {

      destination[Z_AXIS]+=retract_zlift; //not sure why chaninging current_position negatively does not work.
      //if slicer retracted by echange=-1mm and you want to retract 3mm, corrrectede=-2mm additionally
      float correctede=-echange-retract_length;
      //to generate the additional steps, not the destination is changed, but inversely the current position
      current_position[E_AXIS]+=-correctede;
      feedrate=retract_feedrate;
      retracted=true;
      }

    }
    else
      if(echange>MIN_RETRACT) //retract_recover
    {
      if(retracted)
      {
      //current_position[Z_AXIS]+=-retract_zlift;
      //if slicer retracted_recovered by echange=+1mm and you want to retract_recover 3mm, corrrectede=2mm additionally
      float correctede=-echange+1*retract_length+retract_recover_length; //total unretract=retract_length+retract_recover_length[surplus]
      current_position[E_AXIS]+=correctede; //to generate the additional steps, not the destination is changed, but inversely the current position
      feedrate=retract_recover_feedrate;
      retracted=false;
      }
    }

  }
  #endif //FWRETRACT
}
void get_arc_coordinates(void)
{
#ifdef SF_ARC_FIX
   bool relative_mode_backup = relative_mode;
   relative_mode = true;
#endif
   get_coordinates();
#ifdef SF_ARC_FIX
   relative_mode=relative_mode_backup;
#endif

   if(code_seen('I')) {
     offset[0] = code_value();
   }
   else {
     offset[0] = 0.0;
   }
   if(code_seen('J')) {
     offset[1] = code_value();
   }
   else {
     offset[1] = 0.0;
   }
}
void clamp_to_software_endstops(float target[3])
{
  if(min_software_endstops)
	{
    if (target[X_AXIS] < min_pos[X_AXIS]) target[X_AXIS] = min_pos[X_AXIS];
    if (target[Y_AXIS] < min_pos[Y_AXIS]) target[Y_AXIS] = min_pos[Y_AXIS];
    if (target[Z_AXIS] < min_pos[Z_AXIS]) target[Z_AXIS] = min_pos[Z_AXIS];
  }

  if (max_software_endstops) {
    if (target[X_AXIS] > max_pos[X_AXIS]) target[X_AXIS] = max_pos[X_AXIS];
    if (target[Y_AXIS] > max_pos[Y_AXIS]) target[Y_AXIS] = max_pos[Y_AXIS];
    if (target[Z_AXIS] > max_pos[Z_AXIS]) target[Z_AXIS] = max_pos[Z_AXIS];
  }
}

void calculate_delta(float cartesian[3])
{//219.7*219.7-(85.3035022727672-0.3125)*(-85.3035022727672-0.3125)-(-49.25)*(-49.25)=48268.09-7329.45735-2425.5625=38513.07015开根号196.2474717
	
  delta[X_AXIS] = sqrt(DELTA_DIAGONAL_ROD_2 - square(DELTA_TOWER1_X-cartesian[X_AXIS]) - square(DELTA_TOWER1_Y-cartesian[Y_AXIS])) + cartesian[Z_AXIS];//202+196.2474717=398.2474717
  delta[Y_AXIS] = sqrt(DELTA_DIAGONAL_ROD_2 - square(DELTA_TOWER2_X-cartesian[X_AXIS]) - square(DELTA_TOWER2_Y-cartesian[Y_AXIS])) + cartesian[Z_AXIS];//48268-7276-2425=40000----200
  delta[Z_AXIS] = sqrt(DELTA_DIAGONAL_ROD_2 - square(DELTA_TOWER3_X-cartesian[X_AXIS]) - square(DELTA_TOWER3_Y-cartesian[Y_AXIS])) + cartesian[Z_AXIS];//48268-100-9702=40000----200
}

// Adjust print surface height by linear interpolation over the bed_level array.
void adjust_delta(float cartesian[3])
{
  int half = (ACCURATE_BED_LEVELING_POINTS - 1) / 2;
  float grid_x = max(0.001-half, min(half-0.001, cartesian[X_AXIS] / ACCURATE_BED_LEVELING_GRID_X));
  float grid_y = max(0.001-half, min(half-0.001, cartesian[Y_AXIS] / ACCURATE_BED_LEVELING_GRID_Y));
  int floor_x = floor(grid_x);
  int floor_y = floor(grid_y);
  float ratio_x = grid_x - floor_x;
  float ratio_y = grid_y - floor_y;
  float z1 = bed_level[floor_x+half][floor_y+half];
  float z2 = bed_level[floor_x+half][floor_y+half+1];
  float z3 = bed_level[floor_x+half+1][floor_y+half];
  float z4 = bed_level[floor_x+half+1][floor_y+half+1];
  float left = (1-ratio_y)*z1 + ratio_y*z2;
  float right = (1-ratio_y)*z3 + ratio_y*z4;
  float offset = (1-ratio_x)*left + ratio_x*right;

  delta[X_AXIS] += offset;
  delta[Y_AXIS] += offset;
  delta[Z_AXIS] += offset;

}

void prepare_move_raw()
{
	int8_t i;
  previous_millis_cmd = millis();
  calculate_delta(destination);
	 if(bed_level_switch == 1)
			adjust_delta(current_position);
  plan_buffer_line(delta[X_AXIS], delta[Y_AXIS], delta[Z_AXIS],destination[E_AXIS], feedrate*feedmultiply/60/100.0,active_extruder);
  for(i=0; i < NUM_AXIS; i++) 
	{
    current_position[i] = destination[i];
  }
}


void prepare_move(void)
{  
	int8_t i;
  clamp_to_software_endstops(destination);
  previous_millis_cmd = millis();
	

		// Do not use feedmultiply for E or Z only moves
		if( (current_position[X_AXIS] == destination [X_AXIS]) && (current_position[Y_AXIS] == destination [Y_AXIS])) 
		{
			plan_buffer_line(destination[X_AXIS], destination[Y_AXIS], destination[Z_AXIS], destination[E_AXIS], feedrate/60, active_extruder);
		}
		else
		{
			plan_buffer_line(destination[X_AXIS], destination[Y_AXIS], destination[Z_AXIS], destination[E_AXIS], feedrate*feedmultiply/60/100.0, active_extruder);
		}

		for(i=0; i < NUM_AXIS; i++) 
		{
			current_position[i] = destination[i];
		}
}

void prepare_arc_move(u8 isclockwise) {
	int8_t i;
  float r = hypot(offset[X_AXIS], offset[Y_AXIS]); // Compute arc radius for mc_arc

  // Trace the arc
  mc_arc(current_position, destination, offset, X_AXIS, Y_AXIS, Z_AXIS, feedrate*feedmultiply/60/100.0, r, isclockwise, active_extruder);

  // As far as the parser is concerned, the position is now == target. In reality the
  // motion control system might still be processing the action and the real tool position
  // in any intermediate location.
  for(i=0; i < NUM_AXIS; i++) {
    current_position[i] = destination[i];
  }
  previous_millis_cmd = millis();
}
void manage_inactivity(void)
{
  if( (millis() - previous_millis_cmd) >  max_inactive_time )//因为max_inactive_time只有在M85中才会出现，所以基本max_inactive_time这个值一直为0，所以也不会kill
    if(max_inactive_time)
      kill();
  if(stepper_inactive_time)  {
    if( (millis() - previous_millis_cmd) >  stepper_inactive_time )
    {
      if(blocks_queued() == false) {
        disable_x();
        disable_y();
        disable_z();
        disable_e0();
//        disable_e1();
     //   disable_e2();
      }
    }
  }
	
	if(stop_brake_time)
	{
	  if((millis() - stop_brake_time) <  1000)
        quickStop();
    else
     stop_brake_time = 0;			
	}
	
	
  check_axes_activity();
	//beep_check();
//	Extract_Command();////wifi
//	HMI_Command();/////////
	
//  Outage_check();
	
}

void kill(void)
{
  CRITICAL_SECTION_START; // Stop interrupts
  disable_heater();

  disable_x();
  disable_y();
  disable_z();
  disable_e0();
  //disable_e1();
 // disable_e2();

//#if defined(PS_ON_PIN) && PS_ON_PIN > -1
//  pinMode(PS_ON_PIN,INPUT);
//#endif  
  SERIAL_ERROR_START;
  printf(MSG_ERR_KILLED);
 // LCD_ALERTMESSAGEPGM(MSG_KILLED);
//  suicide();
  while(1) { /* Intentionally left empty */ } // Wait for reset
}

void Stop(void)//#ifndef BOGUS_TEMPERATURE_FAILSAFE_OVERRIDE         在温度异常中调用
{
//	if((target_temperature[0] != 0) ||(target_temperature_bed != 0))
//	{
//	  nextMenu = Temperature_error_screen;
//	}
	//target_temperature_bed
  disable_heater();
  if(Stopped == false)
		{

    Stopped = true;
    Stopped_gcode_LastN = gcode_LastN; // Save last g_code for restart
    SERIAL_ERROR_START;
    printf(MSG_ERR_STOPPED);
//		nextMenu = Temperature_error_screen;
			
  //  LCD_MESSAGEPGM(MSG_STOPPED);
  }
//	  disable_heater();
}

bool IsStopped(void) 
{
	return Stopped; 
}
bool setTargetedHotend(int code)
{
	if(temp_sensor_num==1)
   tmp_extruder = 0;//20161218   20170711
  if(code_seen('T'))
	{
    active_extruder = code_value();//20161218
		if(temp_sensor_num==1)
		 tmp_extruder = 0;//20170111
		else if(temp_sensor_num==2)
		 tmp_extruder = active_extruder;//20170111
    if(tmp_extruder >= EXTRUDERS)
		{
      SERIAL_ECHO_START;
      switch(code)
			{
        case 104:
          printf(MSG_M104_INVALID_EXTRUDER);
          break;
        case 105:
          printf(MSG_M105_INVALID_EXTRUDER);
          break;
        case 109:
          printf(MSG_M109_INVALID_EXTRUDER);
          break;
        case 218:
          printf(MSG_M218_INVALID_EXTRUDER);
          break;
      }
      printf("%d",tmp_extruder);
      return true;
    }
  }
  return false ;
}




