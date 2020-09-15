#ifndef __ICON_CODE_H
#define __ICON_CODE_H

#include "sys.h" 

extern const u32 ICON_OFFSET[21];

//icon
#define  SYSTEM_BACK            (u8*)ICON_OFFSET+0
#define  SYSTEM_BED             (u8*)ICON_OFFSET+4
#define  SYSTEM_FOLDER          (u8*)ICON_OFFSET+8
#define  SYSTEM_GCODEFILE       (u8*)ICON_OFFSET+12
#define  SYSTEM_HEATER1         (u8*)ICON_OFFSET+16
#define  SYSTEM_HEATER2         (u8*)ICON_OFFSET+20
#define  SYSTEM_SD              (u8*)ICON_OFFSET+24
#define  SYSTEM_SHUTDOWN        (u8*)ICON_OFFSET+28
#define  SYSTEM_WIFI1           (u8*)ICON_OFFSET+32
#define  SYSTEM_WIFI2           (u8*)ICON_OFFSET+36
#define  SYSTEM_WIFI3           (u8*)ICON_OFFSET+40
#define  PRINTING_0      (u8*)ICON_OFFSET+44
#define  PRINTING_1      (u8*)ICON_OFFSET+48
#define  PRINTING_2      (u8*)ICON_OFFSET+52
#define  PRINTING_3      (u8*)ICON_OFFSET+56
#define  PRINTING_4      (u8*)ICON_OFFSET+60
#define  PRINTING_5      (u8*)ICON_OFFSET+64
#define  PRINTING_6      (u8*)ICON_OFFSET+68
#define  PRINTING_7      (u8*)ICON_OFFSET+72
#define  PRINTING_8      (u8*)ICON_OFFSET+76
#define  PRINTING_9      (u8*)ICON_OFFSET+80


extern const u32 THEMM_OFFSET[];
//////3.5´ç
#define UNLOAD0					            (u8*)THEMM_OFFSET
#define UNLOAD1					          	(u8*)THEMM_OFFSET+4
#define PRINT0								(u8*)THEMM_OFFSET+8
#define PRINT1     						    (u8*)THEMM_OFFSET+12
#define APPCATION0			                (u8*)THEMM_OFFSET+16
#define APPCATION1	                        (u8*)THEMM_OFFSET+20
#define SETTING0				        	(u8*)THEMM_OFFSET+24
#define SETTING1			        		(u8*)THEMM_OFFSET+28
#define ABOUT0				                (u8*)THEMM_OFFSET+32
#define ABOUT1				                (u8*)THEMM_OFFSET+36
#define SET_BACK0		                    (u8*)THEMM_OFFSET+40
#define SET_BACK1                           (u8*)THEMM_OFFSET+44
#define LANGUAGE0      					    (u8*)THEMM_OFFSET+48
#define LANGUAGE1						    (u8*)THEMM_OFFSET+52
#define RESTORE0					    	(u8*)THEMM_OFFSET+56
#define RESTORE1						    (u8*)THEMM_OFFSET+60
////////
#define APP_BED0		                    (u8*)THEMM_OFFSET+64
#define APP_BED1		                    (u8*)THEMM_OFFSET+68
#define APP_MOVING0		                    (u8*)THEMM_OFFSET+72
#define APP_MOVING1		                    (u8*)THEMM_OFFSET+76
#define APP_FAN0                            (u8*)THEMM_OFFSET+80     
#define APP_FAN1                            (u8*)THEMM_OFFSET+84 
#define APP_DELTA0		                    (u8*)THEMM_OFFSET+88
#define APP_DELTA1		                    (u8*)THEMM_OFFSET+92
#define APP_WIFI0		                    (u8*)THEMM_OFFSET+96
#define APP_WIFI1		                    (u8*)THEMM_OFFSET+100
//#define UPDATE0		                    (u8*)THEMM_OFFSET+120
//#define UPDATE1		                    (u8*)THEMM_OFFSET+124
//#define APP_BACK0		                    (u8*)THEMM_OFFSET+128
//#define APP_BACK1		                    (u8*)THEMM_OFFSET+132
///
#define PRINT_MORE0                         (u8*)THEMM_OFFSET+104
#define PRINT_MORE1                         (u8*)THEMM_OFFSET+108
#define MOVE_X0		                        (u8*)THEMM_OFFSET+112
#define MOVE_X1		                        (u8*)THEMM_OFFSET+116
#define MOVE_Y0		                        (u8*)THEMM_OFFSET+120
#define MOVE_Y1		                        (u8*)THEMM_OFFSET+124
#define MOVE_Z0		                        (u8*)THEMM_OFFSET+128
#define MOVE_Z1		                        (u8*)THEMM_OFFSET+132
#define MOVE_L0		                        (u8*)THEMM_OFFSET+136
#define MOVE_L1		                        (u8*)THEMM_OFFSET+140
#define MOVE_R0		                        (u8*)THEMM_OFFSET+144
#define MOVE_R1		                        (u8*)THEMM_OFFSET+148
#define HOMEZERO0		                    (u8*)THEMM_OFFSET+152
#define HOMEZERO1		                    (u8*)THEMM_OFFSET+156
#define MOVE_STOP0		                    (u8*)THEMM_OFFSET+160
#define MOVE_STOP1		                    (u8*)THEMM_OFFSET+164
#define E_DOWN0		                        (u8*)THEMM_OFFSET+168
#define E_DOWN1	                            (u8*)THEMM_OFFSET+172
#define E_UP0		                  	    (u8*)THEMM_OFFSET+176
#define E_UP1		                        (u8*)THEMM_OFFSET+180
#define HEAT_BACK0					        (u8*)THEMM_OFFSET+184
#define HEAT_BACK1							(u8*)THEMM_OFFSET+188
#define KEY_OFF0						    (u8*)THEMM_OFFSET+192
#define KEY_ON0				                (u8*)THEMM_OFFSET+196
#define DIS0_BLUE						    (u8*)THEMM_OFFSET+200
#define DIS1_GREEN					        (u8*)THEMM_OFFSET+204
#define DIS2_RED						    (u8*)THEMM_OFFSET+208
#define DIS3_ORANGE		                    (u8*)THEMM_OFFSET+212
////////
#define PRINT_HEAD10                        (u8*)THEMM_OFFSET+216
#define PRINT_HEAD11                        (u8*)THEMM_OFFSET+220
#define PRINT_HEAD20                        (u8*)THEMM_OFFSET+224
#define PRINT_HEAD21                        (u8*)THEMM_OFFSET+228
#define PRINT_SPEED0		                (u8*)THEMM_OFFSET+232 //
#define PRINT_SPEED1		                (u8*)THEMM_OFFSET+236
#define PRINT_PLAY		                    (u8*)THEMM_OFFSET+240
#define PRINT_PAUSE		                    (u8*)THEMM_OFFSET+244
#define FILE_BACK0		                    (u8*)THEMM_OFFSET+248///
#define FILE_BACK1		                    (u8*)THEMM_OFFSET+252
//#define PRINT_BED0                        (u8*)THEMM_OFFSET+256
//#define PRINT_BED1                        (u8*)THEMM_OFFSET+260
//#define PRINT_STOP0		                (u8*)THEMM_OFFSET+272
//#define PRINT_STOP1		                (u8*)THEMM_OFFSET+276
//////    
#define SET_ABOUT0		                  (u8*)THEMM_OFFSET+256
#define START_PIC0 		                 	(u8*)THEMM_OFFSET+260       
#define SET_ABOUT2		              		(u8*)THEMM_OFFSET+264     
#define START_PIC2		              		(u8*)THEMM_OFFSET+268       //2.8´ç
///////////////////
//////2.8´ç
#define UNLOAD2					            (u8*)THEMM_OFFSET+272
#define UNLOAD3					          	(u8*)THEMM_OFFSET+276
#define PRINT2								(u8*)THEMM_OFFSET+280
#define PRINT3     							(u8*)THEMM_OFFSET+284
#define APPCATION2			            	(u8*)THEMM_OFFSET+288
#define APPCATION3	                		(u8*)THEMM_OFFSET+292
#define SETTING2				        	(u8*)THEMM_OFFSET+296
#define SETTING3			        		(u8*)THEMM_OFFSET+300
#define ABOUT2				                (u8*)THEMM_OFFSET+304
#define ABOUT3				              	(u8*)THEMM_OFFSET+308
#define SET_BACK2		                	(u8*)THEMM_OFFSET+312
#define SET_BACK3                   		(u8*)THEMM_OFFSET+316
#define LANGUAGE2      						(u8*)THEMM_OFFSET+320
#define LANGUAGE3						    (u8*)THEMM_OFFSET+324
#define RESTORE2					    	(u8*)THEMM_OFFSET+328
#define RESTORE3						    (u8*)THEMM_OFFSET+332
////////
#define APP_BED2		                    (u8*)THEMM_OFFSET+336
#define APP_BED3		                    (u8*)THEMM_OFFSET+340
#define APP_MOVING2		               		(u8*)THEMM_OFFSET+344
#define APP_MOVING3		              		(u8*)THEMM_OFFSET+348
#define APP_FAN2		                	(u8*)THEMM_OFFSET+352
#define APP_FAN3		                	(u8*)THEMM_OFFSET+356
#define APP_DELTA2		              		(u8*)THEMM_OFFSET+360
#define APP_DELTA3		              		(u8*)THEMM_OFFSET+364
#define APP_WIFI2		                    (u8*)THEMM_OFFSET+368
#define APP_WIFI3		                    (u8*)THEMM_OFFSET+372
//#define UPDATE2		                  	(u8*)THEMM_OFFSET+392
//#define UPDATE3		                  	(u8*)THEMM_OFFSET+396
//#define APP_BACK2		                  	(u8*)THEMM_OFFSET+400
//#define APP_BACK3		                  	(u8*)THEMM_OFFSET+404
////MOVE
#define PRINT_MORE2                         (u8*)THEMM_OFFSET+376
#define PRINT_MORE3                         (u8*)THEMM_OFFSET+380
#define MOVE_X2		                  		(u8*)THEMM_OFFSET+384
#define MOVE_X3		                  		(u8*)THEMM_OFFSET+388
#define MOVE_Y2		                  		(u8*)THEMM_OFFSET+392
#define MOVE_Y3		                  		(u8*)THEMM_OFFSET+396
#define MOVE_Z2		                  		(u8*)THEMM_OFFSET+400
#define MOVE_Z3		                  		(u8*)THEMM_OFFSET+404
#define MOVE_L2		                  		(u8*)THEMM_OFFSET+408
#define MOVE_L3		                  		(u8*)THEMM_OFFSET+412
#define MOVE_R2		                  		(u8*)THEMM_OFFSET+416
#define MOVE_R3		                  		(u8*)THEMM_OFFSET+420
#define HOMEZERO2		                	(u8*)THEMM_OFFSET+424
#define HOMEZERO3		                	(u8*)THEMM_OFFSET+428
#define MOVE_STOP2		              		(u8*)THEMM_OFFSET+432
#define MOVE_STOP3		              		(u8*)THEMM_OFFSET+436
#define E_DOWN2		                  		(u8*)THEMM_OFFSET+440
#define E_DOWN3	                    		(u8*)THEMM_OFFSET+444
#define E_UP2		                  		(u8*)THEMM_OFFSET+448
#define E_UP3		                    	(u8*)THEMM_OFFSET+452
#define HEAT_BACK2					        (u8*)THEMM_OFFSET+456
#define HEAT_BACK3							(u8*)THEMM_OFFSET+460
#define KEY_OFF2						    (u8*)THEMM_OFFSET+464
#define KEY_ON2				              	(u8*)THEMM_OFFSET+468
#define DIS0_BLUE2						    (u8*)THEMM_OFFSET+472
#define DIS1_GREEN2					        (u8*)THEMM_OFFSET+476
#define DIS2_RED2						    (u8*)THEMM_OFFSET+480
#define DIS3_ORANGE2	              		(u8*)THEMM_OFFSET+484
////PRINT
#define PRINT_HEAD12                		(u8*)THEMM_OFFSET+488
#define PRINT_HEAD13                		(u8*)THEMM_OFFSET+492
#define PRINT_HEAD22                		(u8*)THEMM_OFFSET+496
#define PRINT_HEAD23                		(u8*)THEMM_OFFSET+500
#define PRINT_SPEED2		            	(u8*)THEMM_OFFSET+504 //
#define PRINT_SPEED3		            	(u8*)THEMM_OFFSET+508
#define PRINT_PLAY2		              		(u8*)THEMM_OFFSET+512
#define PRINT_PAUSE2		            	(u8*)THEMM_OFFSET+516
#define FILE_BACK2		              		(u8*)THEMM_OFFSET+520///
#define FILE_BACK3		              		(u8*)THEMM_OFFSET+524
//#define PRINT_BED2                  		(u8*)THEMM_OFFSET+528
//#define PRINT_BED3                  		(u8*)THEMM_OFFSET+532
//#define PRINT_STOP2		              	(u8*)THEMM_OFFSET+544
//#define PRINT_STOP3		              	(u8*)THEMM_OFFSET+548
//////
#endif



