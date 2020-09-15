#include "Marlin.h"
#include "planner.h"
#include "temperature.h"
//#include "ultralcd.h"
#include "ConfigurationStore.h"
//#include "flash.h" 
#include "usart.h"
#include "guix.h"//20160412
//#include "power_off.h"//20160506
#include "gcodeplayer.h"
#include "Abnormal_Handling.h"
//#include "apsta.h"
extern u8 logo_hold_time;
//void _EEPROM_writeData(u32* address, uint8_t* value, uint8_t size)
//{
//	SPI_Flash_Write(value,*address,size);
////	add += size;
//    /*do
//    {	SPI_Flash_Write(*value,pos,1);
//       // eeprom_write_byte((unsigned char*)pos, *value);
//        pos++;
//        value++;
//    }while(--size);
//	*/
//}
////#define FLASH_WRITE_VAR(address, value) _EEPROM_writeData(address, (uint8_t*)&value, sizeof(value))
//void _EEPROM_readData(u32* address, uint8_t* value, uint8_t size)
//{
////	u8 setup_buffer[size];
////	u8 i;
//	SPI_Flash_Read(value,*address,size);
////	for(i=0;i<size;i++)
////		{
////		  ((u8 *)value)[i]=setup_buffer[i];
////		}
////	address += size;
//	/*
//    do
//    {	SPI_Flash_Write(u8* pBuffer,u32 WriteAddr,1);
//       // *value = eeprom_read_byte((unsigned char*)pos);
//        pos++;
//        value++;
//    }while(--size);
//	*/
//}
////#define FLASH_READ_VAR(address, value) _EEPROM_readData(address, (uint8_t*)&value, sizeof(value))
////======================================================================================



//#define EEPROM_OFFSET 100


// IMPORTANT:  Whenever there are changes made to the variables stored in EEPROM
// in the functions below, also increment the version number. This makes sure that
// the default values are used whenever there is a change to the data, to prevent
// wrong data being written to the variables.
// ALSO:  always make sure the variables in the Store and retrieve sections are in the same order.
//#define EEPROM_VERSION "V07"

//#ifdef EEPROM_SETTINGS
void Config_StoreSettings()  //存储设置
{
 #ifndef ULTIPANEL
//  int plaPreheatHotendTemp = PLA_PREHEAT_HOTEND_TEMP;
//  int plaPreheatHPBTemp = PLA_PREHEAT_HPB_TEMP; 
//  int plaPreheatFanSpeed = PLA_PREHEAT_FAN_SPEED;
//  int absPreheatHotendTemp = ABS_PREHEAT_HOTEND_TEMP; 
//  int absPreheatHPBTemp = ABS_PREHEAT_HPB_TEMP; 
//  int absPreheatFanSpeed = ABS_PREHEAT_FAN_SPEED;
  #endif
  char ver[4]= "000";
  char ver2[4]=FLASH_SET_EEPROM_VERSION;
//  u32 i=EEPROM_OFFSET; 
   FLASH_WRITE_VAR(FLASH_SET_STORE_OFFSET,ver); // invalidate data first                       //100-103    char  						4个字节     版本号              
  FLASH_WRITE_VAR( FLASH_SET_STORE_OFFSET+4,axis_steps_per_unit);                            //104-119    float 						4*4个字节   4轴电机每mm的步数           M92    
  FLASH_WRITE_VAR(FLASH_SET_STORE_OFFSET+20,max_feedrate);                                          //120-135   float 						4*4个字节   4轴最大速度                 M202     
  FLASH_WRITE_VAR(FLASH_SET_STORE_OFFSET+36,max_acceleration_units_per_sq_second);               //136-151   unsigned long     4*4个字节   4轴最大加速度               M201  
  FLASH_WRITE_VAR(FLASH_SET_STORE_OFFSET+52,acceleration);                                       //152-155   float             4个字节     4轴统一默认打印加速度       M204  'S'
  FLASH_WRITE_VAR(FLASH_SET_STORE_OFFSET+56,retract_acceleration);                               //156-159   float 					  4个字节     4轴统一默认回抽加速度       M204  'T'
  FLASH_WRITE_VAR(FLASH_SET_STORE_OFFSET+60,minimumfeedrate);                                    //160-163   float 					  4个字节     4轴统一最小速度             M205  'S'
  FLASH_WRITE_VAR(FLASH_SET_STORE_OFFSET+64,mintravelfeedrate);                                  //164-167   float 					  4个字节     4轴统一最小行程速度         M205  'T'
  FLASH_WRITE_VAR(FLASH_SET_STORE_OFFSET+68,minsegmenttime);                                     //168-171   unsigned long 		4个字节     4轴统一最小时间us           M205  'B'
  FLASH_WRITE_VAR(FLASH_SET_STORE_OFFSET+72,max_xy_jerk);                                        //172-175   float 					  4个字节     XY轴速度变化率         M205  'X'
  FLASH_WRITE_VAR(FLASH_SET_STORE_OFFSET+76,max_z_jerk);                                         //176-179   float 					  4个字节     Z轴速度变化率           M205  'Z'
  FLASH_WRITE_VAR(FLASH_SET_STORE_OFFSET+80,max_e_jerk);                                         //180-183   float 					  4个字节     E轴速度变化率         M205  'E'
  FLASH_WRITE_VAR(FLASH_SET_STORE_OFFSET+84,add_homeing);                                        //184-187   float 					  4个字节     归位偏差                    M206

    #ifndef ULTIPANEL
//  FLASH_WRITE_VAR(FLASH_SET_STORE_OFFSET+84,plaPreheatHotendTemp);
//  FLASH_WRITE_VAR(FLASH_SET_STORE_OFFSET+84,plaPreheatHPBTemp);
//  FLASH_WRITE_VAR(FLASH_SET_STORE_OFFSET+84,plaPreheatFanSpeed);
//  FLASH_WRITE_VAR(FLASH_SET_STORE_OFFSET+84,absPreheatHotendTemp);
//  FLASH_WRITE_VAR(FLASH_SET_STORE_OFFSET+84,absPreheatHPBTemp);
//  FLASH_WRITE_VAR(FLASH_SET_STORE_OFFSET+84,absPreheatFanSpeed);
  #endif
  #ifdef PIDTEMP
    FLASH_WRITE_VAR(FLASH_SET_STORE_OFFSET+96,Default_Kp);                                              //188-191   float 					    4个字节     归位偏差                    M301   'P'
    FLASH_WRITE_VAR(FLASH_SET_STORE_OFFSET+100,Default_Ki);                                              //192-195   float 					    4个字节     归位偏差                    M301   'I'
    FLASH_WRITE_VAR(FLASH_SET_STORE_OFFSET+104,Default_Kd);                                              //196-199   float 					    4个字节     归位偏差                    M301   'D'
  #else
		float dummy = 3000.0f;
    FLASH_WRITE_VAR(i,dummy);
		dummy = 0.0f;
    FLASH_WRITE_VAR(i,dummy);
    FLASH_WRITE_VAR(i,dummy);
  #endif
	
	FLASH_WRITE_VAR(FLASH_SET_STORE_OFFSET+108,heater_0_maxtemp);                                 //200-203    int 						  4个字节   打印头0最高温度限制      
	FLASH_WRITE_VAR(FLASH_SET_STORE_OFFSET+112,bed_maxtemp);                                      //204-207    float 						4个字节   热床最高温度限制 
	FLASH_WRITE_VAR(FLASH_SET_STORE_OFFSET+116,heater_0_PLAtemp);                                 //208-211    float 						4个字节   打印头0-PLA材料目标温度 
	FLASH_WRITE_VAR(FLASH_SET_STORE_OFFSET+120,heater_0_ABStemp);                                 //212-215    float 						4个字节   打印头0-ABS材料目标温度   
	FLASH_WRITE_VAR(FLASH_SET_STORE_OFFSET+124,x_offset_pos);                                     //216-219    float 						4个字节   X轴原点偏移
	FLASH_WRITE_VAR(FLASH_SET_STORE_OFFSET+128,y_offset_pos);                                     //220-223    float 						4个字节   y轴原点偏移
	FLASH_WRITE_VAR(FLASH_SET_STORE_OFFSET+132,x_max_pos);                                        //224-227    float 						4个字节   打印尺寸  
	FLASH_WRITE_VAR(FLASH_SET_STORE_OFFSET+136,y_max_pos);                                  			//228-231    float 						4个字节     
	FLASH_WRITE_VAR(FLASH_SET_STORE_OFFSET+140,z_max_pos);                                  			//232-235    float 						4个字节     
	FLASH_WRITE_VAR(FLASH_SET_STORE_OFFSET+144,invert_x_dir);                                  		//236		     bool 						1个字节   电机方向 
	FLASH_WRITE_VAR(FLASH_SET_STORE_OFFSET+145,invert_y_dir);                                  		//237        bool 						1个字节     
	FLASH_WRITE_VAR(FLASH_SET_STORE_OFFSET+146,invert_z_dir);                                  		//238        bool 						1个字节     
	FLASH_WRITE_VAR(FLASH_SET_STORE_OFFSET+147,invert_e0_dir);                                    //239        bool 						1个字节     
	FLASH_WRITE_VAR(FLASH_SET_STORE_OFFSET+148,x_endstops_inverting);                             //240        bool 						1个字节   限位开关状态
	FLASH_WRITE_VAR(FLASH_SET_STORE_OFFSET+149,y_endstops_inverting);                             //241        bool 						1个字节    
	FLASH_WRITE_VAR(FLASH_SET_STORE_OFFSET+150,z_endstops_inverting);                             //242        bool 						1个字节  

  FLASH_WRITE_VAR(FLASH_SET_STORE_OFFSET+151,gui_phy.language);                                             //243        u8               1个字节   语言类型   //20160412     

   FLASH_WRITE_VAR(FLASH_SET_STORE_OFFSET+152,emc_switch);                                             //244        bool               1个字节   EMC开关   //20160412 
	 
	 FLASH_WRITE_VAR(FLASH_SET_STORE_OFFSET+153,temp_sensor_type);                                       //245        u8               4个字节   温度传感器类型选择
	 
	  FLASH_WRITE_VAR(FLASH_SET_STORE_OFFSET+154,no_material_switch);                                             //244        bool               1个字节      //20160412 
		 FLASH_WRITE_VAR(FLASH_SET_STORE_OFFSET+155,brownout_switch);                                             //244        bool               1个字节     //20160412 
		  FLASH_WRITE_VAR(FLASH_SET_STORE_OFFSET+156,screen_saving_switch);                                             //244        bool               1个字节      //20160412 
			 FLASH_WRITE_VAR(FLASH_SET_STORE_OFFSET+157,auto_shutdown_switch);                                             //244        bool               1个字节     //20160412
       
     FLASH_WRITE_VAR(FLASH_SET_STORE_OFFSET+158,Machine_type);	//u8		 


	FLASH_WRITE_VAR(FLASH_SET_STORE_OFFSET+159,diagonal);                                        //224-227    float 						4个字节   
	FLASH_WRITE_VAR(FLASH_SET_STORE_OFFSET+163,smooth_rod);                                  			//228-231    float 						4个字节     
	FLASH_WRITE_VAR(FLASH_SET_STORE_OFFSET+167,effector);                                  			//232-235    float 						4个字节    
	FLASH_WRITE_VAR(FLASH_SET_STORE_OFFSET+171,carriage);                                  			//232-235    float 						4个字节  	
	
	FLASH_WRITE_VAR(FLASH_SET_STORE_OFFSET+175,level_endstops_inverting);                                  			//232-235    bool 						1个字节 
	
	FLASH_WRITE_VAR(FLASH_SET_STORE_OFFSET+176,Password_switch);
	
	FLASH_WRITE_VAR(FLASH_SET_STORE_OFFSET+177,Password_read_buffer);  
	
	FLASH_WRITE_VAR(FLASH_SET_STORE_OFFSET+183,bed_level_switch);  
	
	FLASH_WRITE_VAR(FLASH_SET_STORE_OFFSET+184,temp_sensor_num); 
	
	FLASH_WRITE_VAR(FLASH_SET_STORE_OFFSET+185,backup_power); 
	
	FLASH_WRITE_VAR(FLASH_SET_STORE_OFFSET+186,e0_endstops_inverting); 
	
	FLASH_WRITE_VAR(FLASH_SET_STORE_OFFSET+187,pulse_width); 
	
	FLASH_WRITE_VAR(FLASH_SET_STORE_OFFSET+191,pause_length); 
	
	FLASH_WRITE_VAR(FLASH_SET_STORE_OFFSET+195,extrude_length_advance); 
  
	//指定高度
//	FLASH_WRITE_VAR(FLASH_SET_STORE_OFFSET+200,appoint_z_height);
  //	
	FLASH_WRITE_VAR(FLASH_SET_STORE_OFFSET+204,floor_height); 
	FLASH_WRITE_VAR(FLASH_SET_STORE_OFFSET+208,add_homeing_ofsize); 
  FLASH_WRITE_VAR(FLASH_SET_STORE_OFFSET+220,z_homeing_ofsize); 
	
	FLASH_WRITE_VAR(FLASH_SET_STORE_OFFSET+224,home_dir); //jee201804
	//1000~存储系统相关变量
	FLASH_WRITE_VAR(FLASH_SET_STORE_OFFSET+1000,title_fontcol);
	FLASH_WRITE_VAR(FLASH_SET_STORE_OFFSET+1002,title_backgroundcol);
	FLASH_WRITE_VAR(FLASH_SET_STORE_OFFSET+1004,body_backgroundcol);				
	FLASH_WRITE_VAR(FLASH_SET_STORE_OFFSET+1006,logo_hold_time);
//  i=EEPROM_OFFSET;
  FLASH_WRITE_VAR(FLASH_SET_STORE_OFFSET,ver2); // validate data                             //100-103    u8  						4个字节     新版本号
  SERIAL_ECHO_START;
  printf("Settings Stored\r\n");
}
//#endif //EEPROM_SETTINGS


//#ifdef EEPROM_CHITCHAT
void Config_PrintSettings()//获取设置
{  // Always have this function, even with EEPROM_SETTINGS disabled, the current values will be shown
	
		SERIAL_ECHO_START;
    printf("Max Positon (mm):");
    printf("  M1001 X%f",x_max_pos);
    printf(" Y%f",y_max_pos);
    printf(" Z%f\r\n",z_max_pos);

	
    SERIAL_ECHO_START;
    printf("Steps per unit:");
    printf("  M92 X%f",axis_steps_per_unit[0]);
    printf(" Y%f",axis_steps_per_unit[1]);
    printf(" Z%f",axis_steps_per_unit[2]);
    printf(" E%f\r\n",axis_steps_per_unit[3]);

      
    SERIAL_ECHO_START;
    printf("Maximum feedrates (mm/s):");
    printf("  M203 X%f",max_feedrate[0]);
    printf(" Y%f",max_feedrate[1] ); 
    printf(" Z%f", max_feedrate[2] ); 
    printf(" E%f\r\n", max_feedrate[3]);


    SERIAL_ECHO_START;
    printf("Maximum Acceleration (mm/s2):");
    printf("  M201 X%ld" ,max_acceleration_units_per_sq_second[0] ); 
    printf(" Y%ld" , max_acceleration_units_per_sq_second[1] ); 
    printf(" Z%ld" ,max_acceleration_units_per_sq_second[2] );
    printf(" E%ld\r\n" ,max_acceleration_units_per_sq_second[3]);
  //  SERIAL_ECHOLN("");
    SERIAL_ECHO_START;
    printf("Acceleration: S=acceleration, T=retract acceleration");
    printf("  M204 S%f",acceleration ); 
    printf(" T%f\r\n" ,retract_acceleration);
 ///   SERIAL_ECHOLN("");

    SERIAL_ECHO_START;
    printf("Advanced variables: S=Min feedrate (mm/s), T=Min travel feedrate (mm/s), B=minimum segment time (ms), X=maximum XY jerk (mm/s),  Z=maximum Z jerk (mm/s),  E=maximum E jerk (mm/s)");
    printf("  M205 S%f",minimumfeedrate ); 
    printf(" T%f" ,mintravelfeedrate ); 
    printf(" B%ld" ,minsegmenttime ); 
    printf(" X%f" ,max_xy_jerk ); 
    printf(" Z%f" ,max_z_jerk);
    printf(" E%f\r\n" ,max_e_jerk);
   // SERIAL_ECHOLN(""); 

    SERIAL_ECHO_START;
    printf("Home offset (mm):");
    printf("  M206 X%f",add_homeing[0] );
    printf(" Y%f" ,add_homeing[1] );
    printf(" Z%f\r\n" ,add_homeing[2] );
  //  SERIAL_ECHOLN("");
#ifdef PIDTEMP
    SERIAL_ECHO_START;
    printf("PID settings:");
    printf("   M301 P%f",Kp); 
    printf(" I%f" ,unscalePID_i(Ki)); 
    printf(" D%f\r\n" ,unscalePID_d(Kd));
  //  SERIAL_ECHOLN(""); 
#endif
} 
//#endif


//#ifdef EEPROM_SETTINGS
void Config_RetrieveSettings()//读取设置
{
	
    #ifndef ULTIPANEL
//    int plaPreheatHotendTemp, plaPreheatHPBTemp, plaPreheatFanSpeed;
//    int absPreheatHotendTemp, absPreheatHPBTemp, absPreheatFanSpeed;
    #endif
//    u32 i=EEPROM_OFFSET;
    u8 stored_ver[4];
    u8 ver[4]=FLASH_SET_EEPROM_VERSION;
	

	
	  SERIAL_ECHO_START;
	    {
		   char pname[32]="Firmware Version   :    ";
			 strcat(pname,(char *)FLASH_SET_EEPROM_VERSION);	//拷贝path到pname里面
			 strcat(pname,(char *)"\r\n");	//拷贝path到pname里面	
	     printf(pname);//20160401
			}
		
			
//		FLASH_READ_VAR(WIFI_ACCOUNT_STORE_OFFSET,wifi_account);
//			FLASH_READ_VAR(WIFI_PASSWORD_STORE_OFFSET,wifi_password);
//		
//		  FLASH_READ_VAR(SERVER_IP_STORE_OFFSET,server_IP);
//		  FLASH_READ_VAR(SERVER_PORTNUM_STORE_OFFSET,server_portnum);
//		  if(wifi_account[0]==0xFF)
//			{
//			  memset(wifi_account, 0, sizeof(wifi_account));
//				memset(wifi_password, 0, sizeof(wifi_password));
//				FLASH_WRITE_VAR(WIFI_ACCOUNT_STORE_OFFSET,wifi_account);
//				FLASH_WRITE_VAR(WIFI_PASSWORD_STORE_OFFSET,wifi_password);
//			}
//			if(server_IP[0]==0xFF)
//			{
//			  memset(server_IP, 0, sizeof(server_IP));
//				memset(server_portnum, 0, sizeof(server_portnum));
//				FLASH_WRITE_VAR(SERVER_IP_STORE_OFFSET,server_IP);
//				FLASH_WRITE_VAR(SERVER_PORTNUM_STORE_OFFSET,server_portnum);
//			}
			
			
//		FLASH_READ_VAR(FGCODE_FPTR_ADDRESS,buf1);
			{
			  unsigned char fgcode_fptr_buf[4];
				FLASH_READ_VAR(FGCODE_FPTR_ADDRESS,fgcode_fptr_buf);//20170317                                             //244        unsigned long      4个字节   掉电时的文件位置指针//20160506
				if((fgcode_fptr_buf[0] != 0xFF)&&(fgcode_fptr_buf[1] != 0xFF)&&(fgcode_fptr_buf[2] != 0xFF)&&(fgcode_fptr_buf[3] != 0xFF))
				{
				 fgcode_fptr_flag = 0x01;	
				}
	    }
		
		//    if (strncmp(ver,stored_ver,3) == 0)                     //如果ver和stored_ver的前3个字符相同，则返回0
    FLASH_READ_VAR(FLASH_SET_STORE_OFFSET,stored_ver); //read stored version                         //100-103    char  						4个字节     版本号                      
	 if((stored_ver[0] != 0xFF)&&(stored_ver[1] != 0xFF)&&(stored_ver[2] != 0xFF))//20160323
    {       // version number match
			  if (strncmp((char*)ver,(char*)stored_ver,3) != 0)                     //如果ver和stored_ver的前3个字符相同，则返回0  //20160323
				{
				 FLASH_WRITE_VAR(FLASH_SET_STORE_OFFSET,ver); // validate data                             //0-3    char  						4个字节     新版本号 
				}
        FLASH_READ_VAR( FLASH_SET_STORE_OFFSET+4,axis_steps_per_unit);                                //4-19    float 						4*4个字节   4轴电机每mm的步数           M92    
				FLASH_READ_VAR(FLASH_SET_STORE_OFFSET+20,max_feedrate);                                       //20-35   float 						4*4个字节   4轴最大速度                 M202     
				FLASH_READ_VAR(FLASH_SET_STORE_OFFSET+36,max_acceleration_units_per_sq_second);               //36-51   unsigned long     4*4个字节   4轴最大加速度               M201  
        
        // steps per sq second need to be updated to agree with the units per sq second (as they are what is used in the planner)
		reset_acceleration_rates();//设置axis_steps_per_sqr_second[i] = max_acceleration_units_per_sq_second[i] * axis_steps_per_unit[i]
        
        FLASH_READ_VAR(FLASH_SET_STORE_OFFSET+52,acceleration);                                       //52-55   float             4个字节     4轴统一默认打印加速度       M204  'S'
				FLASH_READ_VAR(FLASH_SET_STORE_OFFSET+56,retract_acceleration);                               //56-59   float 					  4个字节     4轴统一默认回抽加速度       M204  'T'
				FLASH_READ_VAR(FLASH_SET_STORE_OFFSET+60,minimumfeedrate);                                    //60-63   float 					  4个字节     4轴统一最小速度             M205  'S'
				FLASH_READ_VAR(FLASH_SET_STORE_OFFSET+64,mintravelfeedrate);                                  //64-67   float 					  4个字节     4轴统一最小行程速度         M205  'T'
				FLASH_READ_VAR(FLASH_SET_STORE_OFFSET+68,minsegmenttime);                                     //68-71   unsigned long 		4个字节     4轴统一最小时间us           M205  'B'
				FLASH_READ_VAR(FLASH_SET_STORE_OFFSET+72,max_xy_jerk);                                        //72-75   float 					  4个字节     XY轴不需加速的速度          M205  'X'
				FLASH_READ_VAR(FLASH_SET_STORE_OFFSET+76,max_z_jerk);                                         //76-79   float 					  4个字节     Z轴不需加速的速度           M205  'Z'
				FLASH_READ_VAR(FLASH_SET_STORE_OFFSET+80,max_e_jerk);                                         //80-83   float 					  4个字节     E轴不需加速的速度           M205  'E'
				FLASH_READ_VAR(FLASH_SET_STORE_OFFSET+84,add_homeing);                                        //84-87   float 					  4个字节     归位偏差                    M206

		#ifndef ULTIPANEL
        //  FLASH_READ_VAR(FLASH_SET_STORE_OFFSET+84,plaPreheatHotendTemp);
				//  FLASH_READ_VAR(FLASH_SET_STORE_OFFSET+84,plaPreheatHPBTemp);
				//  FLASH_READ_VAR(FLASH_SET_STORE_OFFSET+84,plaPreheatFanSpeed);
				//  FLASH_READ_VAR(FLASH_SET_STORE_OFFSET+84,absPreheatHotendTemp);
				//  FLASH_READ_VAR(FLASH_SET_STORE_OFFSET+84,absPreheatHPBTemp);
				//  FLASH_READ_VAR(FLASH_SET_STORE_OFFSET+84,absPreheatFanSpeed);
		#endif
        #ifndef PIDTEMP
        float Kp,Ki,Kd;
        #endif
        // do not need to scale PID values as the values in EEPROM are already scaled		
				 FLASH_READ_VAR(FLASH_SET_STORE_OFFSET+96,Default_Kp);                                              //88-91   float 					    4个字节     原始Kp                   M301   'P'
				 FLASH_READ_VAR(FLASH_SET_STORE_OFFSET+100,Default_Ki);                                              //92-95   float 					    4个字节     原始Ki                   M301   'I'
				 FLASH_READ_VAR(FLASH_SET_STORE_OFFSET+104,Default_Kd);                                              //96-99   float 					    4个字节     原始Kd                   M301   'D'
         Kp = Default_Kp;               //PID
			   Ki = scalePID_i(Default_Ki);
				 Kd = scalePID_d(Default_Kd);
	// Call updatePID (similar to when we have processed M301)
		updatePID();
		
		    FLASH_READ_VAR(FLASH_SET_STORE_OFFSET+108,heater_0_maxtemp);                                 //100-103    int 						  4个字节   打印头0最高温度限制      
				FLASH_READ_VAR(FLASH_SET_STORE_OFFSET+112,bed_maxtemp);                                      //104-107    float 						4个字节   热床最高温度限制 
				FLASH_READ_VAR(FLASH_SET_STORE_OFFSET+116,heater_0_PLAtemp);                                 //108-111    float 						4个字节   打印头0-PLA材料目标温度 
				FLASH_READ_VAR(FLASH_SET_STORE_OFFSET+120,heater_0_ABStemp);                                 //112-115    float 						4个字节   打印头0-ABS材料目标温度   
				FLASH_READ_VAR(FLASH_SET_STORE_OFFSET+124,x_offset_pos);                                      //116-119    float 						4个字节   X轴原点偏移
				FLASH_READ_VAR(FLASH_SET_STORE_OFFSET+128,y_offset_pos);                                      //120-123    float 						4个字节   y轴原点偏移
				FLASH_READ_VAR(FLASH_SET_STORE_OFFSET+132,x_max_pos);                                        //124-127    float 						4个字节   打印尺寸  
				FLASH_READ_VAR(FLASH_SET_STORE_OFFSET+136,y_max_pos);                                  			//128-131    float 						4个字节     
				FLASH_READ_VAR(FLASH_SET_STORE_OFFSET+140,z_max_pos);                                  			//132-135    float 						4个字节     
				FLASH_READ_VAR(FLASH_SET_STORE_OFFSET+144,invert_x_dir);                                  		//136		     bool 						1个字节   电机方向 
				FLASH_READ_VAR(FLASH_SET_STORE_OFFSET+145,invert_y_dir);                                  		//137        bool 						1个字节     
				FLASH_READ_VAR(FLASH_SET_STORE_OFFSET+146,invert_z_dir);                                  	                                                                                                                                                                                                                                                    	//238        bool 						1个字节     
				FLASH_READ_VAR(FLASH_SET_STORE_OFFSET+147,invert_e0_dir);                                    //139        bool 						1个字节     
				FLASH_READ_VAR(FLASH_SET_STORE_OFFSET+148,x_endstops_inverting);                             //140        bool 						1个字节   限位开关状态
				FLASH_READ_VAR(FLASH_SET_STORE_OFFSET+149,y_endstops_inverting);                             //141        bool 						1个字节    
				FLASH_READ_VAR(FLASH_SET_STORE_OFFSET+150,z_endstops_inverting);                             //142        bool 						1个字节 
				//断电续打相关变量
//				FLASH_READ_VAR(FLASH_SET_STORE_OFFSET+200,appoint_z_height);
				
			  FLASH_READ_VAR(FLASH_SET_STORE_OFFSET+1000,title_fontcol);
			  FLASH_READ_VAR(FLASH_SET_STORE_OFFSET+1002,title_backgroundcol);			
				FLASH_READ_VAR(FLASH_SET_STORE_OFFSET+1004,body_backgroundcol);				
			  FLASH_READ_VAR(FLASH_SET_STORE_OFFSET+1006,logo_hold_time);
			 
				SERIAL_ECHO_START;                            
		    printf("Stored settings retrieved\r\n");
    }
    else 
    {
        Config_ResetDefault();//如果第一次上电读版本号为0xFF，那么就说初始化并存储所有参数，这里不能用Read_Factor_Refault();  这里只用到一次
    }
    Config_PrintSettings();
}
//#endif

void Config_ResetDefault()//将设置恢复出厂设置
{	  u8 i;
    float tmp1[]=DEFAULT_AXIS_STEPS_PER_UNIT;
    float tmp2[]=DEFAULT_MAX_FEEDRATE;
    long tmp3[]=DEFAULT_MAX_ACCELERATION;
    for (i=0;i<4;i++) 
    {
        axis_steps_per_unit[i]=tmp1[i];  //每mm步数
        max_feedrate[i]=tmp2[i];         //最大速度
        max_acceleration_units_per_sq_second[i]=tmp3[i];//最大打印加速度
    }
    
    // steps per sq second need to be updated to agree with the units per sq second
    reset_acceleration_rates();//设置axis_steps_per_sqr_second[i] = max_acceleration_units_per_sq_second[i] * axis_steps_per_unit[i]
    
    acceleration=DEFAULT_ACCELERATION;//默认加速度
    retract_acceleration=DEFAULT_RETRACT_ACCELERATION;//默认回抽加速度
    minimumfeedrate=DEFAULT_MINIMUMFEEDRATE;
    minsegmenttime=DEFAULT_MINSEGMENTTIME;       
    mintravelfeedrate=DEFAULT_MINTRAVELFEEDRATE;
		  max_xy_jerk=DEFAULT_XYJERK;//速度变化率
			max_z_jerk=DEFAULT_ZJERK;
			max_e_jerk=DEFAULT_EJERK;

     add_homeing[0]  = 0;
		 add_homeing[1]  = 0;
		 add_homeing[2]  = 0;
#ifdef ULTIPANEL
    plaPreheatHotendTemp = PLA_PREHEAT_HOTEND_TEMP;
    plaPreheatHPBTemp = PLA_PREHEAT_HPB_TEMP;
    plaPreheatFanSpeed = PLA_PREHEAT_FAN_SPEED;
    absPreheatHotendTemp = ABS_PREHEAT_HOTEND_TEMP;
    absPreheatHPBTemp = ABS_PREHEAT_HPB_TEMP;
    absPreheatFanSpeed = ABS_PREHEAT_FAN_SPEED;
#endif
#ifdef PIDTEMP
         Default_Kp = 22.2;               //PID
         Default_Ki = 1.08;
         Default_Kd = 144;
       
         Kp = Default_Kp;               //PID
			   Ki = scalePID_i(Default_Ki);
				 Kd = scalePID_d(Default_Kd);
    
    // call updatePID (similar to when we have processed M301)
    updatePID();
    
#ifdef PID_ADD_EXTRUSION_RATE
    Kc = DEFAULT_Kc;
#endif//PID_ADD_EXTRUSION_RATE
#endif//PIDTEMP

    heater_0_maxtemp=275;//打印头0最高温度
		bed_maxtemp=150;//热床最高温度
		heater_0_PLAtemp=185;
		heater_0_ABStemp=230;
		x_offset_pos=0;
		y_offset_pos=0;
		  x_max_pos=X_MAX_POS;
			y_max_pos=Y_MAX_POS;
			z_max_pos=Z_MAX_POS;
		invert_x_dir=false;
		invert_y_dir=false; 
		invert_z_dir=true;
		invert_e0_dir=false;
		x_endstops_inverting=1;
		y_endstops_inverting=1; 
		z_endstops_inverting=1;
		
		
		gui_phy.language = 0;
		
		emc_switch = 0;

    temp_sensor_type = 1;
		
	 no_material_switch = 0;
   brownout_switch = 0;
   screen_saving_switch=0;
   auto_shutdown_switch=0;
		
	 Machine_type = 0;
		
   diagonal=219.7;
	 smooth_rod=145;
   effector=26.5;
   carriage=20;

  level_endstops_inverting = 1;
	Password_switch = 0;
	for (i=0;i<6;i++)
	Password_read_buffer[i] = '0';
	bed_level_switch = 0;

  
	
	temp_sensor_num=1;
	
	backup_power=0;
	
	e0_endstops_inverting=0;
	
	pulse_width=6;  //jee
  pause_length=15;
	extrude_length_advance=8;
	
		
//	appoint_z_height=0;
	floor_height=0.2;
	add_homeing_ofsize[0]=0;
	 add_homeing_ofsize[1]=0;
	 add_homeing_ofsize[2]=0;
	 z_homeing_ofsize=0;
	 home_dir[0]=-1;
	 home_dir[1]=-1;
	 home_dir[2]=-1;
	//颜色
	 title_fontcol=WHITE ; //标题栏前景色 白色
	 title_backgroundcol=gui_color_chg(0X888888); //
	 body_backgroundcol= gui_color_chg(0X999999);//主背景色
	 logo_hold_time=3;
	 
		Print_parameter_init();
		tp_parameter_init();
		
		Config_StoreSettings();//恢复出厂设置最后存储一下
		
		
		

SERIAL_ECHO_START;
printf("Hardcoded Default Settings Loaded\r\n");


        __set_FAULTMASK(1); //关闭所有中断
        NVIC_SystemReset();// 复位

}



void Write_Factor_Refault(void)
{
   FLASH_WRITE_VAR( FACTORY_STORE_OFFSET+4,axis_steps_per_unit);                            //104-119    float 						4*4个字节   4轴电机每mm的步数           M92    
  FLASH_WRITE_VAR(FACTORY_STORE_OFFSET+20,max_feedrate);                                          //120-135   float 						4*4个字节   4轴最大速度                 M202     
  FLASH_WRITE_VAR(FACTORY_STORE_OFFSET+36,max_acceleration_units_per_sq_second);               //136-151   unsigned long     4*4个字节   4轴最大加速度               M201  
  FLASH_WRITE_VAR(FACTORY_STORE_OFFSET+52,acceleration);                                       //152-155   float             4个字节     4轴统一默认打印加速度       M204  'S'
  FLASH_WRITE_VAR(FACTORY_STORE_OFFSET+56,retract_acceleration);                               //156-159   float 					  4个字节     4轴统一默认回抽加速度       M204  'T'
  FLASH_WRITE_VAR(FACTORY_STORE_OFFSET+60,minimumfeedrate);                                    //160-163   float 					  4个字节     4轴统一最小速度             M205  'S'
  FLASH_WRITE_VAR(FACTORY_STORE_OFFSET+64,mintravelfeedrate);                                  //164-167   float 					  4个字节     4轴统一最小行程速度         M205  'T'
  FLASH_WRITE_VAR(FACTORY_STORE_OFFSET+68,minsegmenttime);                                     //168-171   unsigned long 		4个字节     4轴统一最小时间us           M205  'B'
  FLASH_WRITE_VAR(FACTORY_STORE_OFFSET+72,max_xy_jerk);                                        //172-175   float 					  4个字节     XY轴速度变化率         M205  'X'
  FLASH_WRITE_VAR(FACTORY_STORE_OFFSET+76,max_z_jerk);                                         //176-179   float 					  4个字节     Z轴速度变化率           M205  'Z'
  FLASH_WRITE_VAR(FACTORY_STORE_OFFSET+80,max_e_jerk);                                         //180-183   float 					  4个字节     E轴速度变化率         M205  'E'
  FLASH_WRITE_VAR(FACTORY_STORE_OFFSET+84,add_homeing);                                        //184-187   float 					  4个字节     归位偏差                    M206

    #ifndef ULTIPANEL
//  FLASH_WRITE_VAR(FLASH_SET_STORE_OFFSET+84,plaPreheatHotendTemp);
//  FLASH_WRITE_VAR(FLASH_SET_STORE_OFFSET+84,plaPreheatHPBTemp);
//  FLASH_WRITE_VAR(FLASH_SET_STORE_OFFSET+84,plaPreheatFanSpeed);
//  FLASH_WRITE_VAR(FLASH_SET_STORE_OFFSET+84,absPreheatHotendTemp);
//  FLASH_WRITE_VAR(FLASH_SET_STORE_OFFSET+84,absPreheatHPBTemp);
//  FLASH_WRITE_VAR(FLASH_SET_STORE_OFFSET+84,absPreheatFanSpeed);
  #endif
  #ifdef PIDTEMP
    FLASH_WRITE_VAR(FACTORY_STORE_OFFSET+96,Default_Kp);                                              //188-191   float 					    4个字节     归位偏差                    M301   'P'
    FLASH_WRITE_VAR(FACTORY_STORE_OFFSET+100,Default_Ki);                                              //192-195   float 					    4个字节     归位偏差                    M301   'I'
    FLASH_WRITE_VAR(FACTORY_STORE_OFFSET+104,Default_Kd);                                              //196-199   float 					    4个字节     归位偏差                    M301   'D'
  #else
		float dummy = 3000.0f;
    FLASH_WRITE_VAR(i,dummy);
		dummy = 0.0f;
    FLASH_WRITE_VAR(i,dummy);
    FLASH_WRITE_VAR(i,dummy);
  #endif
	
	FLASH_WRITE_VAR(FACTORY_STORE_OFFSET+108,heater_0_maxtemp);                                 //200-203    int 						  4个字节   打印头0最高温度限制      
	FLASH_WRITE_VAR(FACTORY_STORE_OFFSET+112,bed_maxtemp);                                      //204-207    float 						4个字节   热床最高温度限制 
	FLASH_WRITE_VAR(FACTORY_STORE_OFFSET+116,heater_0_PLAtemp);                                 //208-211    float 						4个字节   打印头0-PLA材料目标温度 
	FLASH_WRITE_VAR(FACTORY_STORE_OFFSET+120,heater_0_ABStemp);                                 //212-215    float 						4个字节   打印头0-ABS材料目标温度   
	FLASH_WRITE_VAR(FACTORY_STORE_OFFSET+124,x_offset_pos);                                     //216-219    float 						4个字节   x轴原点偏移
	FLASH_WRITE_VAR(FACTORY_STORE_OFFSET+128,y_offset_pos);                                     //220-223    float 						4个字节   y轴原点偏移
	FLASH_WRITE_VAR(FACTORY_STORE_OFFSET+132,x_max_pos);                                        //224-227    float 						4个字节   打印尺寸  
	FLASH_WRITE_VAR(FACTORY_STORE_OFFSET+136,y_max_pos);                                  			//228-231    float 						4个字节     
	FLASH_WRITE_VAR(FACTORY_STORE_OFFSET+140,z_max_pos);                                  			//232-235    float 						4个字节     
	FLASH_WRITE_VAR(FACTORY_STORE_OFFSET+144,invert_x_dir);                                  		//236		     bool 						1个字节   电机方向 
	FLASH_WRITE_VAR(FACTORY_STORE_OFFSET+145,invert_y_dir);                                  		//237        bool 						1个字节     
	FLASH_WRITE_VAR(FACTORY_STORE_OFFSET+146,invert_z_dir);                                  		//238        bool 						1个字节     
	FLASH_WRITE_VAR(FACTORY_STORE_OFFSET+147,invert_e0_dir);                                    //239        bool 						1个字节     
	FLASH_WRITE_VAR(FACTORY_STORE_OFFSET+148,x_endstops_inverting);                             //240        bool 						1个字节   限位开关状态
	FLASH_WRITE_VAR(FACTORY_STORE_OFFSET+149,y_endstops_inverting);                             //241        bool 						1个字节    
	FLASH_WRITE_VAR(FACTORY_STORE_OFFSET+150,z_endstops_inverting);                             //242        bool 						1个字节  

  FLASH_WRITE_VAR(FACTORY_STORE_OFFSET+151,gui_phy.language);                                             //243        u8               1个字节   语言类型   //20160412     

   FLASH_WRITE_VAR(FACTORY_STORE_OFFSET+152,emc_switch);                                             //244        bool               1个字节   EMC开关   //20160412 
	 
	 FLASH_WRITE_VAR(FACTORY_STORE_OFFSET+153,temp_sensor_type);                                       //245        u8               4个字节   温度传感器类型选择
	 
	  FLASH_WRITE_VAR(FACTORY_STORE_OFFSET+154,no_material_switch);                                             //244        bool               1个字节      //20160412 
		 FLASH_WRITE_VAR(FACTORY_STORE_OFFSET+155,brownout_switch);                                             //244        bool               1个字节     //20160412 
		  FLASH_WRITE_VAR(FACTORY_STORE_OFFSET+156,screen_saving_switch);                                             //244        bool               1个字节      //20160412 
			 FLASH_WRITE_VAR(FACTORY_STORE_OFFSET+157,auto_shutdown_switch);                                             //244        bool               1个字节     //20160412
       
     FLASH_WRITE_VAR(FACTORY_STORE_OFFSET+158,Machine_type);	//u8		 


	FLASH_WRITE_VAR(FACTORY_STORE_OFFSET+159,diagonal);                                        //224-227    float 						4个字节   
	FLASH_WRITE_VAR(FACTORY_STORE_OFFSET+163,smooth_rod);                                  			//228-231    float 						4个字节     
	FLASH_WRITE_VAR(FACTORY_STORE_OFFSET+167,effector);                                  			//232-235    float 						4个字节    
	FLASH_WRITE_VAR(FACTORY_STORE_OFFSET+171,carriage);                                  			//232-235    float 						4个字节  	
	
	FLASH_WRITE_VAR(FACTORY_STORE_OFFSET+175,level_endstops_inverting);                                  			//232-235    bool 						1个字节 
	
	FLASH_WRITE_VAR(FACTORY_STORE_OFFSET+176,Password_switch);
	
	FLASH_WRITE_VAR(FACTORY_STORE_OFFSET+177,Password_read_buffer);  
	
	FLASH_WRITE_VAR(FACTORY_STORE_OFFSET+183,bed_level_switch);  
	
	FLASH_WRITE_VAR(FACTORY_STORE_OFFSET+184,temp_sensor_num); 
	
	FLASH_WRITE_VAR(FACTORY_STORE_OFFSET+185,backup_power); 
	
	FLASH_WRITE_VAR(FACTORY_STORE_OFFSET+186,e0_endstops_inverting); 
	
	FLASH_WRITE_VAR(FACTORY_STORE_OFFSET+187,pulse_width); 
	
	FLASH_WRITE_VAR(FACTORY_STORE_OFFSET+191,pause_length);  
  
	FLASH_WRITE_VAR(FACTORY_STORE_OFFSET+195,extrude_length_advance);  

//	FLASH_WRITE_VAR(FACTORY_STORE_OFFSET+200,appoint_z_height);
	
	FLASH_WRITE_VAR(FACTORY_STORE_OFFSET+204,floor_height);
	
	FLASH_WRITE_VAR(FACTORY_STORE_OFFSET+208,add_homeing_ofsize); 
	FLASH_WRITE_VAR(FACTORY_STORE_OFFSET+220,z_homeing_ofsize); 
	FLASH_WRITE_VAR(FACTORY_STORE_OFFSET+228,heater_0_mintemp); //jee201805 
	//颜色
 
	FLASH_WRITE_VAR(FACTORY_STORE_OFFSET+1000,title_fontcol);
	FLASH_WRITE_VAR(FACTORY_STORE_OFFSET+1002,title_backgroundcol);
	FLASH_WRITE_VAR(FACTORY_STORE_OFFSET+1004,body_backgroundcol);				
  FLASH_WRITE_VAR(FACTORY_STORE_OFFSET+1006,logo_hold_time);
				
}



void Read_Factor_Refault(void)
{
	u8 stored_data;
	FLASH_READ_VAR(FACTORY_STORE_OFFSET+4,stored_data);
	if(stored_data != 0xFF)//20160323
    {
	
        FLASH_READ_VAR( FACTORY_STORE_OFFSET+4,axis_steps_per_unit);                                //4-19    float 						4*4个字节   4轴电机每mm的步数           M92    
				FLASH_READ_VAR(FACTORY_STORE_OFFSET+20,max_feedrate);                                       //20-35   float 						4*4个字节   4轴最大速度                 M202     
				FLASH_READ_VAR(FACTORY_STORE_OFFSET+36,max_acceleration_units_per_sq_second);               //36-51   unsigned long     4*4个字节   4轴最大加速度               M201  
        
        // steps per sq second need to be updated to agree with the units per sq second (as they are what is used in the planner)
		    reset_acceleration_rates();//设置axis_steps_per_sqr_second[i] = max_acceleration_units_per_sq_second[i] * axis_steps_per_unit[i]
        
        FLASH_READ_VAR(FACTORY_STORE_OFFSET+52,acceleration);                                       //52-55   float             4个字节     4轴统一默认打印加速度       M204  'S'
				FLASH_READ_VAR(FACTORY_STORE_OFFSET+56,retract_acceleration);                               //56-59   float 					  4个字节     4轴统一默认回抽加速度       M204  'T'
				FLASH_READ_VAR(FACTORY_STORE_OFFSET+60,minimumfeedrate);                                    //60-63   float 					  4个字节     4轴统一最小速度             M205  'S'
				FLASH_READ_VAR(FACTORY_STORE_OFFSET+64,mintravelfeedrate);                                  //64-67   float 					  4个字节     4轴统一最小行程速度         M205  'T'
				FLASH_READ_VAR(FACTORY_STORE_OFFSET+68,minsegmenttime);                                     //68-71   unsigned long 		4个字节     4轴统一最小时间us           M205  'B'
				FLASH_READ_VAR(FACTORY_STORE_OFFSET+72,max_xy_jerk);                                        //72-75   float 					  4个字节     XY轴不需加速的速度          M205  'X'
				FLASH_READ_VAR(FACTORY_STORE_OFFSET+76,max_z_jerk);                                         //76-79   float 					  4个字节     Z轴不需加速的速度           M205  'Z'
				FLASH_READ_VAR(FACTORY_STORE_OFFSET+80,max_e_jerk);                                         //80-83   float 					  4个字节     E轴不需加速的速度           M205  'E'
				FLASH_READ_VAR(FACTORY_STORE_OFFSET+84,add_homeing);                                        //84-87   float 					  4个字节     归位偏差                    M206

		#ifndef ULTIPANEL
        //  FLASH_READ_VAR(FLASH_SET_STORE_OFFSET+84,plaPreheatHotendTemp);
				//  FLASH_READ_VAR(FLASH_SET_STORE_OFFSET+84,plaPreheatHPBTemp);
				//  FLASH_READ_VAR(FLASH_SET_STORE_OFFSET+84,plaPreheatFanSpeed);
				//  FLASH_READ_VAR(FLASH_SET_STORE_OFFSET+84,absPreheatHotendTemp);
				//  FLASH_READ_VAR(FLASH_SET_STORE_OFFSET+84,absPreheatHPBTemp);
				//  FLASH_READ_VAR(FLASH_SET_STORE_OFFSET+84,absPreheatFanSpeed);
		#endif
        #ifndef PIDTEMP
        float Kp,Ki,Kd;
        #endif
        // do not need to scale PID values as the values in EEPROM are already scaled		
				 FLASH_READ_VAR(FACTORY_STORE_OFFSET+96,Default_Kp);                                              //88-91   float 					    4个字节     原始Kp                   M301   'P'
				 FLASH_READ_VAR(FACTORY_STORE_OFFSET+100,Default_Ki);                                              //92-95   float 					    4个字节     原始Ki                   M301   'I'
				 FLASH_READ_VAR(FACTORY_STORE_OFFSET+104,Default_Kd);                                              //96-99   float 					    4个字节     原始Kd                   M301   'D'
         Kp = Default_Kp;               //PID
			   Ki = scalePID_i(Default_Ki);
				 Kd = scalePID_d(Default_Kd);
	// Call updatePID (similar to when we have processed M301)
		   updatePID();
		
		    FLASH_READ_VAR(FACTORY_STORE_OFFSET+108,heater_0_maxtemp);                                 //100-103    int 						  4个字节   打印头0最高温度限制      
				FLASH_READ_VAR(FACTORY_STORE_OFFSET+112,bed_maxtemp);                                      //104-107    float 						4个字节   热床最高温度限制 
				FLASH_READ_VAR(FACTORY_STORE_OFFSET+116,heater_0_PLAtemp);                                 //108-111    float 						4个字节   打印头0-PLA材料目标温度 
				FLASH_READ_VAR(FACTORY_STORE_OFFSET+120,heater_0_ABStemp);                                 //112-115    float 						4个字节   打印头0-ABS材料目标温度   
				FLASH_READ_VAR(FACTORY_STORE_OFFSET+124,x_offset_pos);                                     //116-119    float 						4个字节   x轴原点偏移
				FLASH_READ_VAR(FACTORY_STORE_OFFSET+128,y_offset_pos);                                     //120-123    float 						4个字节   y轴原点偏移
				FLASH_READ_VAR(FACTORY_STORE_OFFSET+132,x_max_pos);                                        //124-127    float 						4个字节   打印尺寸  
				FLASH_READ_VAR(FACTORY_STORE_OFFSET+136,y_max_pos);                                  			//128-131    float 						4个字节     
				FLASH_READ_VAR(FACTORY_STORE_OFFSET+140,z_max_pos);                                  			//132-135    float 						4个字节     
				FLASH_READ_VAR(FACTORY_STORE_OFFSET+144,invert_x_dir);                                  		//136		     bool 						1个字节   电机方向 
				FLASH_READ_VAR(FACTORY_STORE_OFFSET+145,invert_y_dir);                                  		//137        bool 						1个字节     
				FLASH_READ_VAR(FACTORY_STORE_OFFSET+146,invert_z_dir);                                  	                                                                                                                                                                                                                                                    	//238        bool 						1个字节     
				FLASH_READ_VAR(FACTORY_STORE_OFFSET+147,invert_e0_dir);                                    //139        bool 						1个字节     
				FLASH_READ_VAR(FACTORY_STORE_OFFSET+148,x_endstops_inverting);                             //140        bool 						1个字节   限位开关状态
				FLASH_READ_VAR(FACTORY_STORE_OFFSET+149,y_endstops_inverting);                             //141        bool 						1个字节    
				FLASH_READ_VAR(FACTORY_STORE_OFFSET+150,z_endstops_inverting);                             //142        bool 						1个字节 
				
				
				FLASH_READ_VAR(FACTORY_STORE_OFFSET+151,gui_phy.language);
						
			FLASH_READ_VAR(FACTORY_STORE_OFFSET+152,emc_switch);
			
			FLASH_READ_VAR(FACTORY_STORE_OFFSET+153,temp_sensor_type);

			FLASH_READ_VAR(FACTORY_STORE_OFFSET+154,no_material_switch);
				if(no_material_switch == 1)
					Material_EXIT_Set(ENABLE);
				else if(no_material_switch == 0)
					Material_EXIT_Set(DISABLE);

			FLASH_READ_VAR(FACTORY_STORE_OFFSET+155,brownout_switch);
				
			FLASH_READ_VAR(FACTORY_STORE_OFFSET+156,screen_saving_switch);

			FLASH_READ_VAR(FACTORY_STORE_OFFSET+157,auto_shutdown_switch);
			 
			 FLASH_READ_VAR(FACTORY_STORE_OFFSET+158,Machine_type);
			 

			max_xy_jerk=DEFAULT_XYJERK;
			max_z_jerk=DEFAULT_ZJERK;
			max_e_jerk=DEFAULT_EJERK;


		 FLASH_READ_VAR(FACTORY_STORE_OFFSET+159,diagonal);                                        //224-227    float 						4个字节   
		 FLASH_READ_VAR(FACTORY_STORE_OFFSET+163,smooth_rod);                                  			//228-231    float 						4个字节     
		 FLASH_READ_VAR(FACTORY_STORE_OFFSET+167,effector);                                  			//232-235    float 						4个字节    
		 FLASH_READ_VAR(FACTORY_STORE_OFFSET+171,carriage);                                  			//232-235    float 						4个字节 

		FLASH_READ_VAR(FACTORY_STORE_OFFSET+175,level_endstops_inverting);
			 
		FLASH_READ_VAR(FACTORY_STORE_OFFSET+176,Password_switch);
			 
		FLASH_READ_VAR(FACTORY_STORE_OFFSET+177,Password_read_buffer);
			 
			 FLASH_READ_VAR(FACTORY_STORE_OFFSET+183,bed_level_switch);
			 FLASH_READ_VAR(FACTORY_STORE_OFFSET+184,temp_sensor_num);
			 FLASH_READ_VAR(FACTORY_STORE_OFFSET+185,backup_power);
			 FLASH_READ_VAR(FACTORY_STORE_OFFSET+186,e0_endstops_inverting);
			 FLASH_READ_VAR(FACTORY_STORE_OFFSET+187,pulse_width);                                        //224-227    float 						4个字节   
			 FLASH_READ_VAR(FACTORY_STORE_OFFSET+191,pause_length);                                        //224-227    float 						4个字节   
			 FLASH_READ_VAR(FACTORY_STORE_OFFSET+195,extrude_length_advance);
				
//			 FLASH_READ_VAR(FACTORY_STORE_OFFSET+200,appoint_z_height);
			 FLASH_READ_VAR(FACTORY_STORE_OFFSET+204,floor_height);
			 FLASH_READ_VAR(FACTORY_STORE_OFFSET+208,add_homeing_ofsize);
				 FLASH_READ_VAR(FACTORY_STORE_OFFSET+220,z_homeing_ofsize);
			 FLASH_READ_VAR(FACTORY_STORE_OFFSET+224,home_dir);
				//颜色				
			  FLASH_READ_VAR(FACTORY_STORE_OFFSET+1000,title_fontcol);
			  FLASH_READ_VAR(FACTORY_STORE_OFFSET+1002,title_backgroundcol);			 
				FLASH_READ_VAR(FACTORY_STORE_OFFSET+1004,body_backgroundcol);				
	      FLASH_READ_VAR(FACTORY_STORE_OFFSET+1006,logo_hold_time);
				reset_bed_level();
				
				Print_parameter_init();
		    tp_parameter_init();
				
				Config_StoreSettings();//恢复出厂设置最后存储一下
		}
//		else 
//    {
//        Config_ResetDefault();//
//    }
				
				__set_FAULTMASK(1); //关闭所有中断
        NVIC_SystemReset();// 复位	
				
}

