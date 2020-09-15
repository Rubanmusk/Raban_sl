// Tonokip RepRap firmware rewrite based off of Hydra-mmm firmware.
// Licence: GPL

#ifndef MARLIN_H
#define MARLIN_H

//#define  FORCE_INLINE __attribute__((always_inline)) inline

#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "sys.h"
//#include <inttypes.h>

//#include <delay.h>
//#include <avr/pgmspace.h>
//#include <avr/eeprom.h>
//#include <avr/interrupt.h>


//#include "fastio.h"
#include "Configuration.h"
#include "pins.h"
#include "adc.h"


extern bool axis_adjust;

extern int buflen;
extern char cmdbuffer[BUFSIZE][MAX_CMD_SIZE];
extern int bufindr;
extern signed char home_dir[3];

extern long residencyStart;

extern u32 stop_brake_time;


extern u8 enable_level_sensor;

extern bool Stopped;


#define OK_KEY (Get_Adc_Average(ADC_Channel_15,10)>3250)



void reset_bed_level(void);

void Clear_cmdbuffer(void);//20160330
	
void get_command(void);
void process_commands(void);
void manage_inactivity(void);

#define  enable_x()  X_ENABLE_PIN=X_ENABLE_ON
#define disable_x()	 X_ENABLE_PIN=!X_ENABLE_ON

#define  enable_y()  Y_ENABLE_PIN=Y_ENABLE_ON
#define disable_y()	 Y_ENABLE_PIN=!Y_ENABLE_ON

#define  enable_z()  Z_ENABLE_PIN=Z_ENABLE_ON
#define disable_z()	 Z_ENABLE_PIN=!Z_ENABLE_ON

#define  enable_e0() E0_ENABLE_PIN=E_ENABLE_ON
#define disable_e0() E0_ENABLE_PIN=!E_ENABLE_ON

#define  enable_e1() E1_ENABLE_PIN=E_ENABLE_ON
#define disable_e1() E1_ENABLE_PIN=!E_ENABLE_ON

enum AxisEnum {X_AXIS=0, Y_AXIS=1, Z_AXIS=2, E_AXIS=3};

#define SERIAL_ERROR_START	printf("Error:")
#define SERIAL_ECHO_START	printf("echo:")
void loop(void);
void Print_parameter_init(void);
void setup(void);
void FlushSerialRequestResend(void);
void ClearToSend(void);

void get_coordinates(void);


void calculate_delta(float cartesian[3]);
//void adjust_delta(float cartesian[3]);
extern float delta[3];
//void enquecommand_P(const char *cmd); //put an ascii command at the end of the current buffer, read from flash
void prepare_arc_move(u8 isclockwise);

void adjust_delta(float cartesian[3]);
void prepare_move_raw(void);
void prepare_move(void);
void kill(void);
void Stop(void);

bool IsStopped(void);

void enquecommand(const char *cmd); //put an ascii command at the end of the current buffer.




void clamp_to_software_endstops(float target[3]);

#define CRITICAL_SECTION_START  __disable_irq();
#define CRITICAL_SECTION_END    __enable_irq();
extern float homing_feedrate[];
extern bool axis_relative_modes[];
extern int feedmultiply;
extern int extrudemultiply; // Sets extrude multiply factor (in percent)
extern float current_position[NUM_AXIS] ;

extern float destination[NUM_AXIS] ;

extern float add_homeing[3];
extern float min_pos[3];
extern float max_pos[3];
extern float  base_min_pos[3];  
extern float  base_max_pos[3];
extern float  base_home_pos[3];
extern float  max_length[3];


extern float LaserPower;
#ifdef BARICUDA
extern int ValvePressure;
extern int EtoPPressure;
#endif

#ifdef FWRETRACT
extern bool autoretract_enabled;
extern bool retracted;
extern float retract_length, retract_feedrate, retract_zlift;
extern float retract_recover_length, retract_recover_feedrate;
#endif

extern unsigned long starttime;
extern unsigned long stoptime;

// Handling multiple extruders pins
extern uint8_t active_extruder;
extern u8 tmp_extruder;//20161218
#endif
