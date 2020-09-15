/*
  temperature.c - temperature control
  Part of Marlin
  
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

#include "temperature.h"
#include "Marlin.h"
//#include "ultralcd.h"
#include "delay.h"
#include "watchdog.h"
#include "adc.h"
#include "usart.h"
#include "planner.h"
#include "thermistortables.h"
#include "lcdmenu.h"
#include "spi.h"
#include "ConfigurationStore.h"


//===========================================================================
//=============================public variables============================
//===========================================================================
int target_temperature[EXTRUDERS] = { 0 };
int target_temperature_bed = 0;
int current_temperature_raw[EXTRUDERS] = { 0 };
float current_temperature[EXTRUDERS] = { 0 };
int current_temperature_bed_raw = 0;
float current_temperature_bed = 0;

#ifdef PIDTEMP
  
  float Default_Kp=22.2;
  float Default_Ki=1.08;
  float Default_Kd=144;

  float Kp=22.2;
  float Ki=(1.08*PID_dT);
  float Kd=(144/PID_dT);
  #ifdef PID_ADD_EXTRUSION_RATE
    float Kc=DEFAULT_Kc;
  #endif
#endif //PIDTEMP

#ifdef PIDTEMPBED
  float bedKp=DEFAULT_bedKp;
  float bedKi=(DEFAULT_bedKi*PID_dT);
  float bedKd=(DEFAULT_bedKd/PID_dT);
#endif //PIDTEMPBED
  
  
//===========================================================================
//=============================private variables============================
//===========================================================================
static volatile bool temp_meas_ready = false;

#ifdef PIDTEMP
  //static cannot be external:
  static float temp_iState[EXTRUDERS] = { 0 };
  static float temp_dState[EXTRUDERS] = { 0 };
  static float pTerm[EXTRUDERS];
  static float iTerm[EXTRUDERS];
  static float dTerm[EXTRUDERS];
  //int output;
  static float pid_error[EXTRUDERS];
  static float temp_iState_min[EXTRUDERS];
  static float temp_iState_max[EXTRUDERS];
  // static float pid_input[EXTRUDERS];
  // static float pid_output[EXTRUDERS];
  static bool pid_reset[EXTRUDERS];
#endif //PIDTEMP
#ifdef PIDTEMPBED
  //static cannot be external:
  static float temp_iState_bed = { 0 };
  static float temp_dState_bed = { 0 };
  static float pTerm_bed;
  static float iTerm_bed;
  static float dTerm_bed;
  //int output;
  static float pid_error_bed;
  static float temp_iState_min_bed;
  static float temp_iState_max_bed;
#else //PIDTEMPBED
	static unsigned long  previous_millis_bed_heater;
#endif //PIDTEMPBED
  static unsigned char soft_pwm[EXTRUDERS];
  static unsigned char soft_pwm_bed;

#if (defined(EXTRUDER_0_AUTO_FAN_PIN) && EXTRUDER_0_AUTO_FAN_PIN > -1) || \
    (defined(EXTRUDER_1_AUTO_FAN_PIN) && EXTRUDER_1_AUTO_FAN_PIN > -1) || \
    (defined(EXTRUDER_2_AUTO_FAN_PIN) && EXTRUDER_2_AUTO_FAN_PIN > -1)
  static unsigned long extruder_autofan_last_check;
#endif  
  
#if EXTRUDERS > 3
# error Unsupported number of extruders
#elif EXTRUDERS > 2
#define ARRAY_BY_EXTRUDERS(v1, v2, v3) { v1, v2, v3 }
#elif EXTRUDERS > 1
#define ARRAY_BY_EXTRUDERS(v1, v2, v3) { v1, v2 }
#else
#define ARRAY_BY_EXTRUDERS(v1, v2, v3) { v1 }
#endif

// Init min and max temp with extreme values to prevent false errors during startup
static int minttemp_raw[EXTRUDERS] = ARRAY_BY_EXTRUDERS( HEATER_0_RAW_LO_TEMP , HEATER_1_RAW_LO_TEMP , HEATER_2_RAW_LO_TEMP );
static int maxttemp_raw[EXTRUDERS] = ARRAY_BY_EXTRUDERS( HEATER_0_RAW_HI_TEMP , HEATER_1_RAW_HI_TEMP , HEATER_2_RAW_HI_TEMP );
static int minttemp[EXTRUDERS] = ARRAY_BY_EXTRUDERS( 0, 0, 0 );
static int maxttemp[EXTRUDERS] = ARRAY_BY_EXTRUDERS( 16383, 16383, 16383 );
//static int bed_minttemp_raw = HEATER_BED_RAW_LO_TEMP; /* No bed mintemp error implemented?!? */
#ifdef BED_MAXTEMP
static int bed_maxttemp_raw = HEATER_BED_RAW_HI_TEMP;
#endif
static void *heater_ttbl_map[EXTRUDERS] = ARRAY_BY_EXTRUDERS( (void *)HEATER_0_TEMPTABLE, (void *)HEATER_1_TEMPTABLE, (void *)HEATER_2_TEMPTABLE );
static uint8_t heater_ttbllen_map[EXTRUDERS] = ARRAY_BY_EXTRUDERS( HEATER_0_TEMPTABLE_LEN, HEATER_1_TEMPTABLE_LEN, HEATER_2_TEMPTABLE_LEN );

static float analog2temp(int raw, uint8_t e);
static float analog2tempBed(int raw);
static void updateTemperaturesFromRawValues(void);

#ifdef WATCH_TEMP_PERIOD
int watch_start_temp[EXTRUDERS] = ARRAY_BY_EXTRUDERS(0,0,0);
unsigned long watchmillis[EXTRUDERS] = ARRAY_BY_EXTRUDERS(0,0,0);
#endif //WATCH_TEMP_PERIOD


//===========================================================================
//=============================   functions      ============================
//===========================================================================

void PID_autotune(float temp, int extruder, int ncycles)
{
  float input = 0.0;
  int cycles=0;
  bool heating = true;

  unsigned long temp_millis = millis();
  unsigned long t1=temp_millis;
  unsigned long t2=temp_millis;
  long t_high = 0;
  long t_low = 0;

  long bias, d;
  float Ku, Tu;
  float Kp, Ki, Kd;
  float max = 0, min = 10000;

  if (extruder > EXTRUDERS)
  {
	printf("PID Autotune failed. Bad extruder number.");
	return;
  }	
  printf("PID Autotune start");
  
  disable_heater(); // switch off all heaters.

  if (extruder<0)
	{
	 	soft_pwm_bed = (MAX_BED_POWER)/2;
		bias = d = (MAX_BED_POWER)/2;
  	}
	else
	{
	  soft_pwm[extruder] = (PID_MAX)/2;
		bias = d = (PID_MAX)/2;
  }




 for(;;) {

    if(temp_meas_ready == true)	  // temp sample ready
	 { 
	      updateTemperaturesFromRawValues();  //将AD 值转换为 实际温度值
	
	      input = (extruder<0)?current_temperature_bed:current_temperature[extruder]; //获取当前温度值
	
	      max=max(max,input);   //取最大值
	      min=min(min,input);	//取最小值
	
	      if(heating == true && input > temp) //加热状态 当前温度比目标温度大 
		   {	 
		        if(millis() - t2 > 5000) 	 //	加热时间大于5000ms
					{ 
			          heating=false;	 //停止加热		并减小pwm
					  if (extruder<0)
						 soft_pwm_bed = (bias - d) >> 1;
					  else
						 soft_pwm[extruder] = (bias - d) >> 1;
			          t1=millis();
			          t_high=t1 - t2;
			          max=temp;
			        }
		   }
	      if(heating == false && input < temp)	//非加热状态 目标温度大于当前温度
		  {
	        if(millis() - t1 > 5000) //	没加热时间大于5000ms
			{
	          heating=true;		   //开始加热
	          t2=millis();
	          t_low=t2 - t1;
	          if(cycles > 0) 
			  {
		            bias += (d*(t_high - t_low))/(t_low + t_high);
		            bias = constrain(bias, 20 ,(extruder<0?(MAX_BED_POWER):(PID_MAX))-20);
		            if(bias > (extruder<0?(MAX_BED_POWER):(PID_MAX))/2) d = (extruder<0?(MAX_BED_POWER):(PID_MAX)) - 1 - bias;
		            else d = bias;
		
		            printf(" bias: %ld",bias); 
		            printf(" d: %ld",d); 
		            printf(" min: %f",min);
		            printf(" max: %f",max);
		            if(cycles > 2) 
					{
			              Ku = (4.0*d)/(3.14159*(max-min)/2.0);
			              Tu = ((float)(t_low + t_high)/1000.0);
			              printf(" Ku: %f",Ku);
			              printf(" Tu: %f",Tu); 
			              Kp = 0.6*Ku;
			              Ki = 2*Kp/Tu;
			              Kd = Kp*Tu/8;
			              printf(" Clasic PID ");
			              printf(" Kp: %f",Kp); 
			              printf(" Ki: %f",Ki);
			              printf(" Kd: %f",Kd); 
			              /*
			              Kp = 0.33*Ku;
			              Ki = Kp/Tu;
			              Kd = Kp*Tu/3;
			              SERIAL_PROTOCOLLNPGM(" Some overshoot ")
			              SERIAL_PROTOCOLPGM(" Kp: "); SERIAL_PROTOCOLLN(Kp);
			              SERIAL_PROTOCOLPGM(" Ki: "); SERIAL_PROTOCOLLN(Ki);
			              SERIAL_PROTOCOLPGM(" Kd: "); SERIAL_PROTOCOLLN(Kd);
			              Kp = 0.2*Ku;
			              Ki = 2*Kp/Tu;
			              Kd = Kp*Tu/3;
			              SERIAL_PROTOCOLLNPGM(" No overshoot ")
			              SERIAL_PROTOCOLPGM(" Kp: "); SERIAL_PROTOCOLLN(Kp);
			              SERIAL_PROTOCOLPGM(" Ki: "); SERIAL_PROTOCOLLN(Ki);
			              SERIAL_PROTOCOLPGM(" Kd: "); SERIAL_PROTOCOLLN(Kd);
			              */
		            }
	           }
			 if (extruder<0)
				soft_pwm_bed = (bias + d) >> 1;
			 else
				soft_pwm[extruder] = (bias + d) >> 1;
		     cycles++;
		     min=temp;
		   }
		  } 
    }
    if(input > (temp + 20)) 
	{
      printf("PID Autotune failed! Temperature to high");
      return;
    }
    if(millis() - temp_millis > 2000) 
	{
		int p;
		if (extruder<0)
		{
		   p=soft_pwm_bed;       
		   printf("ok B:");
		}
		else
		{
		    p=soft_pwm[extruder];       
		    printf("ok T:");
		}
			
	     printf("%f @:%d",input,p);          
      	 temp_millis = millis();
    }
    if(((millis() - t1) + (millis() - t2)) > (10L*60L*1000L*2L)) {
      printf("PID Autotune failed! timeout");
      return;
    }
    if(cycles > ncycles) {
      printf("PID Autotune finished ! Place the Kp, Ki and Kd constants in the configuration.h");
      return;
    }
    lcd_update();  
  }
}

void updatePID(void)
{
#ifdef PIDTEMP
  int e; 
  for(e = 0; e < EXTRUDERS; e++) { 
     temp_iState_max[e] = PID_INTEGRAL_DRIVE_MAX / Ki;  
  }
#endif
#ifdef PIDTEMPBED
  temp_iState_max_bed = PID_INTEGRAL_DRIVE_MAX / bedKi;  
#endif
}
  
int getHeaterPower(int heater) {  //获取pwm值
	if (heater<0)
		return soft_pwm_bed;
  return soft_pwm[heater];
}
void manage_heater(void)
{
  float pid_input;
  float pid_output;
  int e;
  if(temp_meas_ready != true)   //better readability
    return; 

  updateTemperaturesFromRawValues();

  for(e = 0; e < EXTRUDERS; e++) 
  {
	  #ifdef PIDTEMP
		    pid_input = current_temperature[e];
		
		    #ifndef PID_OPENLOOP
		        pid_error[e] = target_temperature[e] - pid_input;
		        if(pid_error[e] > PID_FUNCTIONAL_RANGE) {
		          pid_output = BANG_MAX;
		          pid_reset[e] = true;
		        }
		        else if(pid_error[e] < -PID_FUNCTIONAL_RANGE || target_temperature[e] == 0) {
		          pid_output = 0;
		          pid_reset[e] = true;
		        }
		        else {
		          if(pid_reset[e] == true) {
		            temp_iState[e] = 0.0;
		            pid_reset[e] = false;
		          }
		          pTerm[e] = Kp * pid_error[e];
		          temp_iState[e] += pid_error[e];
		          temp_iState[e] = constrain(temp_iState[e], temp_iState_min[e], temp_iState_max[e]);
		          iTerm[e] = Ki * temp_iState[e];
		
		          //K1 defined in Configuration.h in the PID settings
		          #define K2 (1.0-K1)
		          dTerm[e] = (Kd * (pid_input - temp_dState[e]))*K2 + (K1 * dTerm[e]);
		          temp_dState[e] = pid_input;
		
		          pid_output = constrain(pTerm[e] + iTerm[e] - dTerm[e], 0, PID_MAX);
		        }
		    #else 
		          pid_output = constrain(target_temperature[e], 0, PID_MAX);
		    #endif //PID_OPENLOOP
	
	    #ifdef PID_DEBUG
		    printf(" PIDDEBUG %d",e);
		    printf(": Input %f",pid_input);
		  //  printf(pid_input);
		    printf(" Output %f",pid_output);
		  //  printf(pid_output);
		    printf(" pTerm %f",pTerm[e]);
		  //  printf(pTerm[e]);
		    printf(" iTerm %f",iTerm[e]);
		  //  printf(iTerm[e]);
		    printf(" dTerm %f",dTerm[e]);
		  //  printf(dTerm[e]); 
		  printf("\n\r"); 
	    #endif //PID_DEBUG

	  #else /* PID off */
	    pid_output = 0;
	    if(current_temperature[e] < target_temperature[e]) {
	      pid_output = PID_MAX;
	    }
	  #endif
	
	    // Check if temperature is within the correct range	   
	    if((current_temperature[e] > minttemp[e]) && (current_temperature[e] < maxttemp[e])) 
	    {
	      soft_pwm[e] = (int)pid_output >> 1;
	    }
	    else {
	      soft_pwm[e] = 0;
	    }
	
	    #ifdef WATCH_TEMP_PERIOD	 // 加热前检查
	    if(watchmillis[e] && millis() - watchmillis[e] > WATCH_TEMP_PERIOD)
	    {
	        if(degHotend(e) < watch_start_temp[e] + WATCH_TEMP_INCREASE)
	        {
	            setTargetHotend(0, e);
	            LCD_MESSAGEPGM("Heating failed");
	            SERIAL_ECHO_START;
	            SERIAL_ECHOLN("Heating failed");
	        }else{
	            watchmillis[e] = 0;
	        }
	    }
	    #endif

  } // End extruder for loop
 /*
  #if (defined(EXTRUDER_0_AUTO_FAN_PIN) && EXTRUDER_0_AUTO_FAN_PIN > -1) ||
      (defined(EXTRUDER_1_AUTO_FAN_PIN) && EXTRUDER_1_AUTO_FAN_PIN > -1) || 
      (defined(EXTRUDER_2_AUTO_FAN_PIN) && EXTRUDER_2_AUTO_FAN_PIN > -1)
		  if(millis() - extruder_autofan_last_check > 2500)  // only need to check fan state very infrequently
		  {
		    checkExtruderAutoFans();
		    extruder_autofan_last_check = millis();
		  }  
  #endif       
  */
  #ifndef PIDTEMPBED
	  if(millis() - previous_millis_bed_heater < BED_CHECK_INTERVAL)
	    return;
	  previous_millis_bed_heater = millis();
  #endif

  #if TEMP_SENSOR_BED != 0
	  
	  #ifdef PIDTEMPBED
	    pid_input = current_temperature_bed;
	    #ifndef PID_OPENLOOP
			  pid_error_bed = target_temperature_bed - pid_input;
			  pTerm_bed = bedKp * pid_error_bed;
			  temp_iState_bed += pid_error_bed;
			  temp_iState_bed = constrain(temp_iState_bed, temp_iState_min_bed, temp_iState_max_bed);
			  iTerm_bed = bedKi * temp_iState_bed;
	
			  //K1 defined in Configuration.h in the PID settings
			  #define K2 (1.0-K1)
			  dTerm_bed= (bedKd * (pid_input - temp_dState_bed))*K2 + (K1 * dTerm_bed);
			  temp_dState_bed = pid_input;
	
			  pid_output = constrain(pTerm_bed + iTerm_bed - dTerm_bed, 0, MAX_BED_POWER);
	
	    #else 
	     	  pid_output = constrain(target_temperature_bed, 0, MAX_BED_POWER);
	    #endif //PID_OPENLOOP
	
		  if((current_temperature_bed > BED_MINTEMP) && (current_temperature_bed < BED_MAXTEMP)) 
		  {
		    soft_pwm_bed = (int)pid_output >> 1;
		  }
		  else {
		    soft_pwm_bed = 0;
		  }
	
	    #elif !defined(BED_LIMIT_SWITCHING)
	      // Check if temperature is within the correct range
	      if((current_temperature_bed > BED_MINTEMP) && (current_temperature_bed < BED_MAXTEMP))
	      {
	        if(current_temperature_bed >= target_temperature_bed)
	        {
	          soft_pwm_bed = 0;
	        }
	        else 
	        {
	          soft_pwm_bed = MAX_BED_POWER>>1;
	        }
	      }
	      else
	      {
	        soft_pwm_bed = 0;
	        HEATER_BED_PIN = 0;
	      }
	    #else //#ifdef BED_LIMIT_SWITCHING
	      // Check if temperature is within the correct band
	      if((current_temperature_bed > BED_MINTEMP) && (current_temperature_bed < BED_MAXTEMP))
	      {
	        if(current_temperature_bed > target_temperature_bed + BED_HYSTERESIS)
	        {
	          soft_pwm_bed = 0;
	        }
	        else if(current_temperature_bed <= target_temperature_bed - BED_HYSTERESIS)
	        {
	          soft_pwm_bed = MAX_BED_POWER>>1;
	        }
	      }
	      else
	      {
	        soft_pwm_bed = 0;
	         HEATER_BED_PIN = 0;
	      }

	    #endif
  #endif
}

//#define PGM_RD_W(x)   (short)pgm_read_word(&x)
// Derived from RepRap FiveD extruder::getTemperature()
// For hot end temperature measurement.
static float analog2temp(int raw, uint8_t e) 
{
  if(e >= EXTRUDERS)
  {
      SERIAL_ERROR_START;
      printf("%d",e);
      printf(" - Invalid extruder number !");
      kill();
  } 
//  #ifdef HEATER_0_USES_MAX6675
//    if (e == 0)
//    {
//      return 0.25 * raw;
//    }
//  #endif

  if(heater_ttbl_map[e] != NULL)
  {
    float celsius = 0;
    uint8_t i;
    short (*tt)[][2] = (short (*)[][2])(heater_ttbl_map[e]);

    for (i=1; i<heater_ttbllen_map[e]; i++)
    {
      if ((*tt)[i][0] > raw)
      {
        celsius = (*tt)[i-1][1] + 
          (raw - (*tt)[i-1][0]) * 
          (float)((*tt)[i][1] - (*tt)[i-1][1]) /
          (float)((*tt)[i][0] - (*tt)[i-1][0]);
        break;
      }
    }

    // Overflow: Set to last value in the table
    if (i == heater_ttbllen_map[e]) celsius = (*tt)[i-1][1];

    return celsius;
  }
  return 0;
  //return ((raw * ((5.0 * 100.0) / 1024.0) / OVERSAMPLENR) * TEMP_SENSOR_AD595_GAIN) + TEMP_SENSOR_AD595_OFFSET;
}

// Derived from RepRap FiveD extruder::getTemperature()
// For bed temperature measurement.
static float analog2tempBed(int raw) {
  #ifdef BED_USES_THERMISTOR
    float celsius = 0;
    u8 i;

    for (i=1; i<BEDTEMPTABLE_LEN; i++)
    {
      if (BEDTEMPTABLE[i][0] > raw)
      {
        celsius  = BEDTEMPTABLE[i-1][1] + 
          (raw - BEDTEMPTABLE[i-1][0]) * 
          (float)(BEDTEMPTABLE[i][1] - BEDTEMPTABLE[i-1][1]) /
          (float)(BEDTEMPTABLE[i][0] - BEDTEMPTABLE[i-1][0]);
        break;
      }
    }

    // Overflow: Set to last value in the table
    if (i == BEDTEMPTABLE_LEN) celsius = BEDTEMPTABLE[i-1][1];

    return celsius;
  #elif defined BED_USES_AD595
    return ((raw * ((5.0 * 100.0) / 1024.0) / OVERSAMPLENR) * TEMP_SENSOR_AD595_GAIN) + TEMP_SENSOR_AD595_OFFSET;
  #else
    return 0;
  #endif
}

/* Called to get the raw values into the the actual temperatures. The raw values are created in interrupt context,
    and this function is called from normal context as it is too slow to run in interrupts and will block the stepper routine otherwise */
static void updateTemperaturesFromRawValues(void)
{	 u8 e;
    for(e=0;e<EXTRUDERS;e++)
    {
			 if(temp_sensor_type == 1)
        current_temperature[e] = analog2temp(current_temperature_raw[e], e);
			 else if(temp_sensor_type == 0)
				 current_temperature[e] = current_temperature_raw[e];
    }
    current_temperature_bed = analog2tempBed(current_temperature_bed_raw);

    //Reset the watchdog after we know we have a temperature measurement.
    watchdog_reset();

    CRITICAL_SECTION_START;
    temp_meas_ready = false;
    CRITICAL_SECTION_END;
}


//TIM1 CH1 PWM输出设置 
//PWM输出初始化
//arr：自动重装值
//psc：时钟预分频数
void TIM1_PWM_Init(u16 arr,u16 psc)
{		 					 
	
 	
	GPIO_InitTypeDef GPIO_InitStructure;
	TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
	TIM_OCInitTypeDef  TIM_OCInitStructure;
	

	RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM1, ENABLE); //使能TIMx外设
 	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);  //使能GPIOA外设时钟使能
	
  //MINI版改
  //设置该引脚为复用输出功能,输出TIM1 CH4的PWM脉冲波形
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_11; //TIM1_CH1
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;  //复用功能输出
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOA, &GPIO_InitStructure); //初始化GPIO
 
	TIM_TimeBaseStructure.TIM_Period = arr; //设置自动重装载周期值
	TIM_TimeBaseStructure.TIM_Prescaler =psc; //设置预分频值 不分频
	TIM_TimeBaseStructure.TIM_ClockDivision = 0; //设置时钟分割:TDTS = Tck_tim
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;  //TIM向上计数模式
	TIM_TimeBaseInit(TIM1, &TIM_TimeBaseStructure); //根据指定的参数初始化TIMx
	
	 
	TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM2; //CH1 PWM2模式	
	TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable; //比较输出使能
	TIM_OCInitStructure.TIM_Pulse = 0; //设置待装入捕获比较寄存器的脉冲值
	TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_Low; //OC1 低电平有效 
	TIM_OC4Init(TIM1, &TIM_OCInitStructure);  //根据指定的参数初始化外设TIMx

	TIM_OC4PreloadConfig(TIM1, TIM_OCPreload_Enable);  //CH1 预装载使能
	
	TIM_ARRPreloadConfig(TIM1, ENABLE); //使能TIMx在ARR上的预装载寄存器
	
	TIM_CtrlPWMOutputs(TIM1,ENABLE);	//MOE 主输出使能,高级定时器必须开启这个 
	
	TIM_Cmd(TIM1, ENABLE);  //使能TIMx
	   										  
} 

/*
*MINi版ADC3----ch12,ch13
*
*/
void tp_init()
{	GPIO_InitTypeDef GPIO_InitStructure;
	TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
	TIM_OCInitTypeDef  TIM_OCInitStructure;
	NVIC_InitTypeDef NVIC_InitStructure;
  // Finish init of mult extruder arrays 
  int e;
  //	printf("tpint!\n\r");
  for( e = 0; e < EXTRUDERS; e++) 
  {   // populate with the first value 
	    maxttemp[e] = maxttemp[0];
	  #ifdef PIDTEMP
	    temp_iState_min[e] = 0.0;
	    temp_iState_max[e] = PID_INTEGRAL_DRIVE_MAX / Ki;
	  #endif //PIDTEMP
	  #ifdef PIDTEMPBED
	    temp_iState_min_bed = 0.0;
	    temp_iState_max_bed = PID_INTEGRAL_DRIVE_MAX / bedKi;
	  #endif //PIDTEMPBED
  }
	//使能定时器5 AFIO复用功能模块 GPIOB端口时钟	GPIOA端口时钟
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM5, ENABLE);	
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM1|RCC_APB2Periph_AFIO|RCC_APB2Periph_GPIOA|RCC_APB2Periph_GPIOB|RCC_APB2Periph_GPIOC, ENABLE);
 ///MINI版TIM8没用到
 GPIO_InitStructure.GPIO_Pin = GPIO_Pin_3;				 //E_Fan-----PA3
 GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP; 		 //推挽输出
 GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;	 //速度为50MHz
 GPIO_Init(GPIOA, &GPIO_InitStructure);	 //根据参数初始化GPIOA3                                                  //初始化E0风扇  PA1  不带PWM调节的 所以不用复用
 GPIO_ResetBits(GPIOA,GPIO_Pin_3);//输出0                                                                         //初始化拉低关闭风扇  
	//GPIO_SetBits(GPIOA,GPIO_Pin_3);//输出0
	
// GPIO_InitStructure.GPIO_Pin = GPIO_Pin_12;				 //BEEP-->PA.1 端口配置
// GPIO_Init(GPIOA, &GPIO_InitStructure);	 //根据参数初始化GPIOA.1                                                  //初始化E1风扇  PA12  不带PWM调节的 所以不用复用
// GPIO_ResetBits(GPIOA,GPIO_Pin_1);//输出0                                                                         //初始化拉低关闭风扇 

	
	//BED_PWM--PWM输出TIM5_CH1 -->PA0
	//E_PWM  --PWM输出TIM5_CH2 -->PA1
	//BED_FAN--PWM输出TIM5_CH3 -->PA2
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2|GPIO_Pin_1|GPIO_Pin_0;//BED_FAN--PWM输出TIM5_CH3
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;  //复用推挽输出
	GPIO_Init(GPIOA, &GPIO_InitStructure);//初始化GPIO                                              //初始化BED风扇   使用PA0复用的TIM5-CH1功能   PA0-TIM5-CH1   带PWM调节

                                     
	
//	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_11;
//	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;  //复用推挽输出
//	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
//	GPIO_Init(GPIOA, &GPIO_InitStructure);//初始化GPIO                                              //初始化打印头1和热床的加热   使用PA11  PA11复用的TIM1-CH4功能     带PWM调节
	
 //	TIM_CtrlPWMOutputs(TIM8, ENABLE);
	
   //初始化TIM 5                          																																		 //初始化TIM8 TIM5
	TIM_TimeBaseStructure.TIM_Period = 127; //设置在下一个更新事件装入活动的自动重装载寄存器周期的值
	TIM_TimeBaseStructure.TIM_Prescaler =19; //设置用来作为TIMx时钟频率除数的预分频值  72000000/30=10K
	TIM_TimeBaseStructure.TIM_ClockDivision = 0; //设置时钟分割:TDTS = Tck_tim
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;  //TIM向上计数模式

	TIM_TimeBaseInit(TIM5, &TIM_TimeBaseStructure); //根据TIM_TimeBaseInitStruct中指定的参数初始化TIMx的时间基数单位
	
	//TIM_TimeBaseInit(TIM1, &TIM_TimeBaseStructure);

//	TIM_TimeBaseStructure.TIM_RepetitionCounter=0;//重复寄存器，用于自动更新
//	TIM_TimeBaseInit(TIM8, &TIM_TimeBaseStructure);
	
	//TIM_TimeBaseInit(TIM1, &TIM_TimeBaseStructure);
	
	//初始化TIM5 8 Channel5 8 PWM模式	                                                                           ////初始化TIM5 8 Channel5 8 PWM模式	   不用中断
	TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM2; //选择定时器模式:TIM脉冲宽度调制模式2
 	TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable; //比较输出使能
	TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_Low; //输出极性:TIM输出比较极性高	
		
    #if defined(LASER_PIN)	//T5_1 //               //热床风扇PWM配置
		TIM_OC1Init(TIM5, &TIM_OCInitStructure);  //根据T指定的参数初始化外设TIM5 OC1
		TIM_OC1PreloadConfig(TIM5, TIM_OCPreload_Enable);  //使能TIM5在CCR1上的预装载寄存器
    #endif 
   	TIM_Cmd(TIM5, ENABLE);  //使能TIM5
		
		
//	TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM2; //选择定时器模式:TIM脉冲宽度调制模式2
// 	TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable; //比较输出使能
//	TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_Low; //输出极性:TIM输出比较极性高	
//		
//    #if defined(HEATER_1_PIN)	//T5_3 //设置该引脚为复用输出功能,输出TIM5 CH3的PWM脉冲波形	GPIOA.2               //E1加热PWM配置
//		TIM_OC4Init(TIM1, &TIM_OCInitStructure);  //根据T指定的参数初始化外设TIM5 OC3
//		TIM_OC4PreloadConfig(TIM1, TIM_OCPreload_Enable);  //使能TIM5在CCR3上的预装载寄存器
//    #endif 

//   	TIM_Cmd(TIM1, ENABLE);  //使能TIM1
		
	///TIM8未使用----MINI版
//	 TIM_OCInitStructure.TIM_OCMode=TIM_OCMode_PWM1; //设置为pwm1输出模式
//	 TIM_OCInitStructure.TIM_OCPolarity=TIM_OCPolarity_High; //设置输出极性
//	 TIM_OCInitStructure.TIM_OutputState=TIM_OutputState_Disable; //禁止该通道输出
//	 //下面几个参数是高级定时器才会用到，通用定时器不用配置
//	 TIM_OCInitStructure.TIM_OCNPolarity=TIM_OCPolarity_High; //设置互补端输出极性
//	 TIM_OCInitStructure.TIM_OutputNState=TIM_OutputNState_Enable;//使能互补端输出
//	 TIM_OCInitStructure.TIM_OCIdleState=TIM_OCIdleState_Reset; //死区后输出状态
//	 TIM_OCInitStructure.TIM_OCNIdleState=TIM_OCNIdleState_Reset;//死区后互补端输出状态
	// TIM_OC1Init(TIM8,&TIM_OCInitStructure); //按照指定参数初始化

		#if defined(HEATER_0_PIN)  //PA1	//T8_2 //设置该引脚为复用输出功能,输出TIM8 CH2的PWM脉冲波形	GPIOB.0             //E0加热PWM配置
		TIM_OC2Init(TIM5, &TIM_OCInitStructure);  //根据T指定的参数初始化外设TIM8 OC2
		TIM_OC2PreloadConfig(TIM5, TIM_OCPreload_Enable);  //使能TIM8在CCR2上的预装载寄存器
    #endif 
    #if defined(HEATER_BED_PIN) //PA0 	//T8_3 //设置该引脚为复用输出功能,输出TIM8 CH3的PWM脉冲波形	GPIOB.1           //热床加热PWM配置
		TIM_OC3Init(TIM5, &TIM_OCInitStructure);  //根据T指定的参数初始化外设TIM8 OC3
		TIM_OC3PreloadConfig(TIM5, TIM_OCPreload_Enable);  //使能TIM8在CCR3上的预装载寄存器   
    #endif 
//	TIM_CtrlPWMOutputs(TIM8, ENABLE); //pwm输出使能，一定要记得打开
//	TIM_Cmd(TIM8, ENABLE);  //使能TIM8
	
	
	
	//TIM_OCInitStructure.TIM_OCMode=TIM_OCMode_PWM1; //设置为pwm1输出模式
//	 TIM_OCInitStructure.TIM_OCPolarity=TIM_OCPolarity_High; //设置输出极性
//	 TIM_OCInitStructure.TIM_OutputState=TIM_OutputState_Disable; //禁止该通道输出
//	 //下面几个参数是高级定时器才会用到，通用定时器不用配置
//	 TIM_OCInitStructure.TIM_OCNPolarity=TIM_OCPolarity_High; //设置互补端输出极性
//	 TIM_OCInitStructure.TIM_OutputNState=TIM_OutputNState_Enable;//使能互补端输出
//	 TIM_OCInitStructure.TIM_OCIdleState=TIM_OCIdleState_Reset; //死区后输出状态
//	 TIM_OCInitStructure.TIM_OCNIdleState=TIM_OCNIdleState_Reset;//死区后互补端输出状态
//	// TIM_OC1Init(TIM8,&TIM_OCInitStructure); //按照指定参数初始化

//		#if defined(HEATER_1_PIN)  	//T8_2 //设置该引脚为复用输出功能,输出TIM8 CH2的PWM脉冲波形	GPIOB.0             //E0加热PWM配置
//		TIM_OC4Init(TIM1, &TIM_OCInitStructure);  //根据T指定的参数初始化外设TIM8 OC2
//		TIM_OC4PreloadConfig(TIM1, TIM_OCPreload_Enable);  //使能TIM8在CCR2上的预装载寄存器
//    #endif 
//	TIM_CtrlPWMOutputs(TIM1, ENABLE); //pwm输出使能，一定要记得打开
//	TIM_Cmd(TIM1, ENABLE);  //使能TIM8
	
	//mini版无E1
//	TIM1_PWM_Init(127,7199);	//TIM1 PWM初始化, Fpwm=72M/256=281.25Khz.
//  TIM_SetCompare4(TIM1,0);//初始值为0	

    // Set analog inputs  PC2---CH12,PC3---CH13
  //见adc.c 文件 void Adc_Init(void)
		Adc_Init();		  		//ADC初始化	
  	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AIN;		                 //设置为模拟输入引脚
  	#if defined(TEMP_0_PIN) 
		GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2;
		GPIO_Init(GPIOC, &GPIO_InitStructure);	                         //PC2   E0的AD温度采集脚
  	#endif
//  	#if defined(TEMP_1_PIN) 
//		GPIO_InitStructure.GPIO_Pin = GPIO_Pin_3;
//		GPIO_Init(GPIOC, &GPIO_InitStructure);	                        //PF8   E1的AD温度采集脚
//  	#endif
  	#if defined(TEMP_BED_PIN)
		GPIO_InitStructure.GPIO_Pin = GPIO_Pin_3;
		GPIO_Init(GPIOC, &GPIO_InitStructure);	                        //PC3  BED的AD温度采集脚
  	#endif



  // Use timer6 for temperature measurement
  // Interleave temperature interrupt with millies interrupt
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM6, ENABLE); //时钟使能

	//定时器TIM6初始化
	TIM_TimeBaseStructure.TIM_Period = 9; //设置在下一个更新事件装入活动的自动重装载寄存器周期的值	计数到10为1ms
	TIM_TimeBaseStructure.TIM_Prescaler =7199; //设置用来作为TIMx时钟频率除数的预分频值	 10Khz的计数频率 72MHz/7200	 100uS
	TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1; //设置时钟分割:TDTS = Tck_tim
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;  //TIM向上计数模式
	TIM_TimeBaseInit(TIM6, &TIM_TimeBaseStructure); //根据指定的参数初始化TIMx的时间基数单位

	TIM_ITConfig(TIM6,TIM_IT_Update,ENABLE ); //使能指定的TIM6中断,允许更新中断

	//中断优先级NVIC设置
	NVIC_InitStructure.NVIC_IRQChannel = TIM6_IRQn;  //
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;  //先占优先级0级
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;  //从优先级3级
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE; //IRQ通道被使能

	NVIC_Init(&NVIC_InitStructure);  //初始化NVIC寄存器

	TIM_Cmd(TIM6, ENABLE);  //使能
  // Wait for temperature measurement to settle
  	delay_ms(250);
		
		tp_parameter_init();
		

//	printf("end!\n\r");
}

void tp_parameter_init()
{
	 	#ifdef HEATER_0_MINTEMP
	  minttemp[0] = HEATER_0_MINTEMP;//最小温度限制
		if(temp_sensor_type == 1)//热敏电阻
		{
			while(analog2temp(minttemp_raw[0], 0) < HEATER_0_MINTEMP) {
		#if HEATER_0_RAW_LO_TEMP < HEATER_0_RAW_HI_TEMP
				minttemp_raw[0] += OVERSAMPLENR;
		#else
				minttemp_raw[0] -= OVERSAMPLENR;//最大16383=1024*16，//从16383开始查表，直到查到表格中倒数第二个 1004*OVERSAMPLENR=16064（报错机制最大值）对应的是5度
		#endif
			}
	 }
		else if(temp_sensor_type == 0)//热电偶
		{
		  minttemp_raw[0] =  HEATER_0_MAXTEMP;//如果热电偶就把最大值赋给minttemp_raw来作为报错机制的最大值
		}
			
	#endif //MINTEMP
	
	#ifdef HEATER_0_MAXTEMP
	  maxttemp[0] = HEATER_0_MAXTEMP;//最大温度限制
		if(temp_sensor_type == 1)
		{
				while(analog2temp(maxttemp_raw[0], 0) > HEATER_0_MAXTEMP) {
			#if HEATER_0_RAW_LO_TEMP < HEATER_0_RAW_HI_TEMP
					maxttemp_raw[0] -= OVERSAMPLENR;
			#else
					maxttemp_raw[0] += OVERSAMPLENR;////最0=0*16，//从0开始查表，直到查到表格中第一个 23*OVERSAMPLENR=368对应的是5度
			#endif
				}
	 }
		else if(temp_sensor_type == 0)
		{
		  maxttemp_raw[0] = HEATER_0_MINTEMP;
		}
	#endif //MAXTEMP
		
		
		
		
		
		
		#ifdef HEATER_1_MINTEMP
	  minttemp[1] = HEATER_1_MINTEMP;//最小温度限制
		if(temp_sensor_type == 1)//热敏电阻
		{
			while(analog2temp(minttemp_raw[1], 1) < HEATER_1_MINTEMP) {
		#if HEATER_1_RAW_LO_TEMP < HEATER_1_RAW_HI_TEMP
				minttemp_raw[1] += OVERSAMPLENR;
		#else
				minttemp_raw[1] -= OVERSAMPLENR;//最大16383=1024*16，//从16383开始查表，直到查到表格中倒数第二个 1004*OVERSAMPLENR=16064（报错机制最大值）对应的是5度
		#endif
			}
	 }
		else if(temp_sensor_type == 0)//热电偶
		{
		  minttemp_raw[1] =  HEATER_1_MAXTEMP;//如果热电偶就把最大值赋给minttemp_raw来作为报错机制的最大值
		}
			
	#endif //MINTEMP
	
	#ifdef HEATER_1_MAXTEMP
	  maxttemp[1] = HEATER_1_MAXTEMP;//最大温度限制
		if(temp_sensor_type == 1)
		{
				while(analog2temp(maxttemp_raw[1], 1) > HEATER_1_MAXTEMP) {
			#if HEATER_1_RAW_LO_TEMP < HEATER_1_RAW_HI_TEMP
					maxttemp_raw[1] -= OVERSAMPLENR;
			#else
					maxttemp_raw[1] += OVERSAMPLENR;////最0=0*16，//从0开始查表，直到查到表格中第一个 23*OVERSAMPLENR=368对应的是5度
			#endif
				}
	 }
		else if(temp_sensor_type == 0)
		{
		  maxttemp_raw[1] = HEATER_1_MINTEMP;
		}
	#endif //MAXTEMP
	
//	#if (EXTRUDERS > 1) && defined(HEATER_1_MINTEMP)
//	  minttemp[1] = HEATER_1_MINTEMP;
//	  while(analog2temp(minttemp_raw[1], 1) < HEATER_1_MINTEMP) {
//	#if HEATER_1_RAW_LO_TEMP < HEATER_1_RAW_HI_TEMP
//	    minttemp_raw[1] += OVERSAMPLENR;
//	#else
//	    minttemp_raw[1] -= OVERSAMPLENR;
//	#endif
//	  }
//	#endif // MINTEMP 1
//	#if (EXTRUDERS > 1) && defined(HEATER_1_MAXTEMP)
//	  maxttemp[1] = HEATER_1_MAXTEMP;
//	  while(analog2temp(maxttemp_raw[1], 1) > HEATER_1_MAXTEMP) {
//	#if HEATER_1_RAW_LO_TEMP < HEATER_1_RAW_HI_TEMP
//	    maxttemp_raw[1] -= OVERSAMPLENR;
//	#else
//	    maxttemp_raw[1] += OVERSAMPLENR;
//	#endif
//	  }
//	#endif //MAXTEMP 1
	
	#if (EXTRUDERS > 2) && defined(HEATER_2_MINTEMP)
	  minttemp[2] = HEATER_2_MINTEMP;
	  while(analog2temp(minttemp_raw[2], 2) < HEATER_2_MINTEMP) {
	#if HEATER_2_RAW_LO_TEMP < HEATER_2_RAW_HI_TEMP
	    minttemp_raw[2] += OVERSAMPLENR;
	#else
	    minttemp_raw[2] -= OVERSAMPLENR;
	#endif
	  }
	#endif //MINTEMP 2
	#if (EXTRUDERS > 2) && defined(HEATER_2_MAXTEMP)
	  maxttemp[2] = HEATER_2_MAXTEMP;
	  while(analog2temp(maxttemp_raw[2], 2) > HEATER_2_MAXTEMP) {
	#if HEATER_2_RAW_LO_TEMP < HEATER_2_RAW_HI_TEMP
	    maxttemp_raw[2] -= OVERSAMPLENR;
	#else
	    maxttemp_raw[2] += OVERSAMPLENR;
	#endif
	  }
	#endif //MAXTEMP 2
	
	#ifdef BED_MINTEMP
	  /* No bed MINTEMP error implemented?!? */ /*
	  while(analog2tempBed(bed_minttemp_raw) < BED_MINTEMP) {
	#if HEATER_BED_RAW_LO_TEMP < HEATER_BED_RAW_HI_TEMP
	    bed_minttemp_raw += OVERSAMPLENR;
	#else
	    bed_minttemp_raw -= OVERSAMPLENR;
	#endif
	  }
	  */
	#endif //BED_MINTEMP
	#ifdef BED_MAXTEMP
	  while(analog2tempBed(bed_maxttemp_raw) > BED_MAXTEMP) 
		{
	#if HEATER_BED_RAW_LO_TEMP < HEATER_BED_RAW_HI_TEMP
	    bed_maxttemp_raw -= OVERSAMPLENR;
	#else
	    bed_maxttemp_raw += OVERSAMPLENR;
	#endif
	  }
	#endif //BED_MAXTEMP
}

void setWatch(void) 
{  
#ifdef WATCH_TEMP_PERIOD
  for (int e = 0; e < EXTRUDERS; e++)
  {
    if(degHotend(e) < degTargetHotend(e) - (WATCH_TEMP_INCREASE * 2))
    {
      watch_start_temp[e] = degHotend(e);
      watchmillis[e] = millis();
    } 
  }
#endif 
}


void disable_heater(void)//只在温度传感器没有插上，检测错误的地方才会调用，kill()函数基本也不会调用
{ int i;
	E0_FAN=0;
	//E1_FAN=0;
  for(i=0;i<EXTRUDERS;i++)
	setTargetHotend(0,i);
  	setTargetBed(0);
	
	{//20170111
		int e; 
		for(e = 0; e < EXTRUDERS; e++) 
		heater_temp[e]=0;//20161125  20170111
	 }
//	heater_0_temp = 0;//20160409
  bed_temp = 0;
  #if defined(TEMP_0_PIN) 
  	target_temperature[0]=0;
  	soft_pwm[0]=0;
 	#if defined(HEATER_0_PIN)  
     	HEATER_0_PIN = 0;
  	#endif
  #endif
     
  #if defined(TEMP_1_PIN) 
    target_temperature[1]=0;
    soft_pwm[1]=0;
    #if defined(HEATER_1_PIN)  
        HEATER_1_PIN = 0;
    #endif
  #endif
      
  #if defined(TEMP_2_PIN) 
    target_temperature[2]=0;
    soft_pwm[2]=0;
    #if defined(HEATER_2_PIN)  
        HEATER_2_PIN = 0;
    #endif
  #endif 

  #if defined(TEMP_BED_PIN) 
    target_temperature_bed=0;
    soft_pwm_bed=0;
    #if defined(HEATER_BED_PIN) 
        HEATER_BED_PIN = 0;
    #endif
  #endif 
}




void max_temp_error(uint8_t e) 
{
  disable_heater();
  if(IsStopped() == false) {
    SERIAL_ERROR_START;
    printf("%d",e);
    printf(": Extruder switched off. MAXTEMP triggered !");
   // LCD_ALERTMESSAGEPGM("Err: MAXTEMP");	 /////////////////////////////////
  }
  #ifndef BOGUS_TEMPERATURE_FAILSAFE_OVERRIDE
  Stop();
  #endif
}

void min_temp_error(uint8_t e) 
{
//  disable_heater();
//  if(IsStopped() == false) {
//    SERIAL_ERROR_START;
//    printf("%d",e);
//    printf(": Extruder switched off. MINTEMP triggered !");
//    //LCD_ALERTMESSAGEPGM("Err: MINTEMP"); //////////////////////////////////
//  }
//  #ifndef BOGUS_TEMPERATURE_FAILSAFE_OVERRIDE
//  Stop();
//  #endif
}

void bed_max_temp_error(void)
{
	#ifdef HEATER_BED_PIN 
			HEATER_BED_PIN=0;
	#endif
		if(IsStopped() == false) {
			SERIAL_ERROR_START;
			printf("Temperature heated bed switched off. MAXTEMP triggered !!");
		//  LCD_ALERTMESSAGEPGM("Err: MAXTEMP BED");///////////////////////////////////////////
		}
		#ifndef BOGUS_TEMPERATURE_FAILSAFE_OVERRIDE
		Stop();
		#endif
}
 /*
#ifdef HEATER_0_USES_MAX6675
#define MAX6675_HEAT_INTERVAL 250
long max6675_previous_millis = -HEAT_INTERVAL;
int max6675_temp = 2000;

int read_max6675()
{
  if (millis() - max6675_previous_millis < MAX6675_HEAT_INTERVAL) 
    return max6675_temp;
  
  max6675_previous_millis = millis();
  max6675_temp = 0;
    
  #ifdef	PRR
    PRR &= ~(1<<PRSPI);
  #elif defined PRR0
    PRR0 &= ~(1<<PRSPI);
  #endif
  
  SPCR = (1<<MSTR) | (1<<SPE) | (1<<SPR0);
  
  // enable TT_MAX6675
  WRITE(MAX6675_SS, 0);
  
  // ensure 100ns delay - a bit extra is fine
  asm("nop");//50ns on 20Mhz, 62.5ns on 16Mhz
  asm("nop");//50ns on 20Mhz, 62.5ns on 16Mhz
  
  // read MSB
  SPDR = 0;
  for (;(SPSR & (1<<SPIF)) == 0;);
  max6675_temp = SPDR;
  max6675_temp <<= 8;
  
  // read LSB
  SPDR = 0;
  for (;(SPSR & (1<<SPIF)) == 0;);
  max6675_temp |= SPDR;
  
  // disable TT_MAX6675
  WRITE(MAX6675_SS, 1);

  if (max6675_temp & 4) 
  {
    // thermocouple open
    max6675_temp = 2000;
  }
  else 
  {
    max6675_temp = max6675_temp >> 3;
  }

  return max6675_temp;
}
#endif
*/


float MAX6675_0_ReadByte(void)
{
	unsigned int t,i;
	unsigned char c;
//	unsigned char flag;
	float temprature;
	   PBout(5)=0;
//	PBout(9)=1;
		c = SPI3_ReadWriteByte(0xFF);
		i = c;
		i = i<<8;
		c = SPI3_ReadWriteByte(0xFF);
		PBout(5)=1;
//	  PBout(9)=1;
		
		i = i|((unsigned int)c);			//i是读出来的原始数据
//		flag = i&0x04;						//flag保存了热电偶的连接状态
		t = i<<1;
		t = t>>4;
		temprature = t*0.25;
//	 if(i!=0)							//max6675有数据返回
//		{
//			if(flag==0)						//热电偶已连接
//			{
//				printf("原始数据是：%04X,  当前温度是：%4.2f。\r\n",i,temprature);
//			}	
//			else							//热电偶掉线
//			{
//				printf("未检测到热电偶，请检查。\r\n");
//			}
//		
//		}
//		else								//max6675没有数据返回
//		{
//			printf("max6675没有数据返回，请检查max6675连接。\r\n");
//		}
	return temprature;
}

float MAX6675_1_ReadByte(void)
{
	unsigned int t,i;
	unsigned char c;
//	unsigned char flag;
	float temprature;
//	PBout(5)=1;
	PBout(9)=0;
		c = SPI3_ReadWriteByte(0xFF);
		i = c;
		i = i<<8;
		c = SPI3_ReadWriteByte(0xFF);
//		PBout(5)=1;
	  PBout(9)=1;
		
		i = i|((unsigned int)c);			//i是读出来的原始数据
//		flag = i&0x04;						//flag保存了热电偶的连接状态
		t = i<<1;
		t = t>>4;
		temprature = t*0.25;
//	 if(i!=0)							//max6675有数据返回
//		{
//			if(flag==0)						//热电偶已连接
//			{
//				printf("原始数据是：%04X,  当前温度是：%4.2f。\r\n",i,temprature);
//			}	
//			else							//热电偶掉线
//			{
//				printf("未检测到热电偶，请检查。\r\n");
//			}
//		
//		}
//		else								//max6675没有数据返回
//		{
//			printf("max6675没有数据返回，请检查max6675连接。\r\n");
//		}
	return temprature;
}

float MAX31855_0_ReadByte(void)
{
	u8 buffer[4];
	u8 n;
	
	u32 Thermocouple;
//	unsigned char flag;
	float temprature;
	
	
	PBout(5)=0;
	PBout(8)=1;
	for(n=0;n<4;n++)
	{
	  buffer[n]=SPI3_ReadWriteByte(0xFF);    //buff 存储测量结果			 
  }	 
	PBout(5)=1;
	PBout(8)=1;
		
	    Thermocouple =buffer[0];
			Thermocouple = (Thermocouple<<6);
		  Thermocouple =(Thermocouple | (buffer[1]>>2));
			temprature = Thermocouple *0.25;//热端温度//Thermocouple = Thermocouple *0.25;//热端温度
	if(temprature>=2047)
				temprature = 0;
//		temprature =(Thermocouple*1.407);//-566.85);

	return temprature;
}


float MAX31855_1_ReadByte(void)
{
	u8 buffer[4];
	u8 n;
	
	u32 Thermocouple;
//	unsigned char flag;
	float temprature;
	
	
	PBout(5)=1;
	PBout(8)=0;
	for(n=0;n<4;n++)
	{
	  buffer[n]=SPI3_ReadWriteByte(0xFF);    //buff 存储测量结果			 
  }	 
	PBout(5)=1;
	PBout(8)=1;
		
	    Thermocouple =buffer[0];
			Thermocouple = (Thermocouple<<6);
		  Thermocouple =(Thermocouple | (buffer[1]>>2));
			temprature = Thermocouple *0.25;//热端温度//Thermocouple = Thermocouple *0.25;//热端温度
	if(temprature>=2047)
				temprature = 0;
//		temprature =(Thermocouple*1.407);//-566.85);

	return temprature;
}


// Timer 6 is shared with millies
void TIM6_IRQHandler(void)
{
  //these variables are only accesible from the ISR, but static, so they don't loose their value
  static unsigned char temp_count = 0;
  static unsigned long raw_temp_0_value = 0;
  #if EXTRUDERS > 1
  static unsigned long raw_temp_1_value = 0;
  #endif
  #if EXTRUDERS > 2
  static unsigned long raw_temp_2_value = 0;
  #endif
  static unsigned long raw_temp_bed_value = 0;
  static unsigned char temp_state = 0;
	
	
	static unsigned char max6675_temp_count = 0;
	
	static int Backup_current_temperature_raw = 0;
	static int Backup_current_temperature_bed_raw = 0;
	static u8 temp_error_flag = 0x00;
	static u8 temp_error_cnt = 0;
	static u8 bed_temp_error_flag = 0x00;
	static u8 bed_temp_error_cnt = 0;
	
 if (TIM_GetITStatus(TIM6, TIM_IT_Update) != RESET)
 { 
 	TIM_ClearITPendingBit(TIM6, TIM_IT_Update);
  //调整PWM 占空比  
	HEATER_0_PIN=(uint16_t)soft_pwm[0];  	// TIME8_CH3
//    #if EXTRUDERS > 1
//   	HEATER_1_PIN=(uint16_t)soft_pwm[1];		// TIME8_CH2
//    #endif
//    #if EXTRUDERS > 2
//   	HEATER_2_PIN=(uint16_t)soft_pwm[2];		// TIME5_CH4
//    #endif

    #ifdef HEATER_BED_PIN
   	HEATER_BED_PIN=(uint16_t)soft_pwm_bed;	// TIME5_CH3
    #endif
	 
    #ifdef LASER_PIN
    LASER_PIN=(uint16_t)LaserPower;				// TIME5_CH4
    #endif


//			switch(temp_state) {
//				case 0: // Prepare TEMP_0
//					#if defined(TEMP_0_PIN)	
//				  if(temp_sensor_type == 1)
//					{
//					  raw_temp_0_value +=TEMP_0_PIN;//热敏电阻1ms读取一次AD值，相加16次
//					}
//					else if(temp_sensor_type == 0)
//						{ 
//							max6675_temp_count++;
//							if(max6675_temp_count>=200)//热电偶每隔200ms读取一次，采集频率不能过高
//							{
//							 raw_temp_0_value = MAX6675_0_ReadByte();//MAX31855_ReadByte();//MAX6675_ReadByte();
//							}
//						}
//					#endif
//					temp_state = 1;
//					break;
//				case 1: // Measure TEMP_0
//					#if defined(TEMP_1_PIN) 
//						   if(temp_sensor_type == 1)
//							  {
//									raw_temp_1_value +=TEMP_1_PIN;//热敏电阻1ms读取一次AD值，相加16次
//								}
//						   else if(temp_sensor_type == 0)
//								{ 
//									max6675_temp_count++;
//									if(max6675_temp_count>=200)//热电偶每隔200ms读取一次，采集频率不能过高
//									{
//									 raw_temp_1_value = MAX6675_1_ReadByte();//MAX31855_ReadByte();//MAX6675_ReadByte();
//									}
//								}
//					#endif
//					temp_state = 2;
//					break;
//				case 2: // Prepare TEMP_BED
//					#if defined(TEMP_2_PIN) 
//					 raw_temp_2_value +=TEMP_2_PIN;
//					#endif
//					temp_state = 3;
//					break;
//				case 3: // Measure TEMP_BED
//					#if defined(TEMP_BED_PIN) 
//					 raw_temp_bed_value +=TEMP_BED_PIN; //28ms
//					#endif
//				 //  printf("%ld\r\n", millis());
//					temp_state = 0;
//					temp_count++;
//					break;
//				
//			  default: break;
//			}
//  
			switch(temp_state) {
				case 0: // Prepare TEMP_0
					#if defined(TEMP_0_PIN)	
				  if(temp_sensor_type == 1)
					{
					  raw_temp_0_value +=TEMP_0_PIN;//热敏电阻1ms读取一次AD值，相加16次
					}
					else if(temp_sensor_type == 0)
						{ 
							max6675_temp_count++;
							if(max6675_temp_count>=200)//热电偶每隔200ms读取一次，采集频率不能过高
							{
							 raw_temp_0_value = MAX6675_0_ReadByte();//MAX31855_ReadByte();//MAX6675_ReadByte();
							}
						}
					#endif
					temp_state = 1;
					break;
				case 1: // Measure TEMP_0
					#if defined(TEMP_1_PIN) 
						   if(temp_sensor_type == 1)
							  {
									raw_temp_1_value +=TEMP_1_PIN;//热敏电阻1ms读取一次AD值，相加16次
								}
						   else if(temp_sensor_type == 0)
								{ 
									max6675_temp_count++;
									if(max6675_temp_count>=200)//热电偶每隔200ms读取一次，采集频率不能过高
									{
									 raw_temp_1_value = MAX6675_1_ReadByte();//MAX31855_ReadByte();//MAX6675_ReadByte();
									}
								}
					#endif
					temp_state = 2;
					break;
				case 2: // Prepare TEMP_BED
					#if defined(TEMP_2_PIN) 
					 raw_temp_2_value +=TEMP_2_PIN;
					#endif
					temp_state = 3;
					break;
				case 3: // Measure TEMP_BED
					#if defined(TEMP_BED_PIN) 
					 raw_temp_bed_value +=TEMP_BED_PIN; //28ms
					#endif
				 //  printf("%ld\r\n", millis());
					temp_state = 0;
					temp_count++;
					break;
				
			  default: break;
			}
  
  if(temp_count >= 16) // 4 ms * 16 = 64ms.
  {			 //
    if(emc_switch == 1)//开启了EMC，就备份当前的温度
		{
			if((temp_sensor_type == 1) ||((temp_sensor_type == 0)&&(max6675_temp_count>=200)))
			   Backup_current_temperature_raw = current_temperature_raw[0];

		  Backup_current_temperature_bed_raw = current_temperature_bed_raw;
		}
		
    if (!temp_meas_ready) //Only update the raw values if they have been read. Else we could be updating them during reading.
    {
			if((temp_sensor_type == 1) ||((temp_sensor_type == 0)&&(max6675_temp_count>=200)))
			{
				current_temperature_raw[0] = raw_temp_0_value;
				  
		#if EXTRUDERS > 1
		      current_temperature_raw[1] = raw_temp_1_value;
		#endif
		#if EXTRUDERS > 2
		      current_temperature_raw[2] = raw_temp_2_value;
		#endif
//		      current_temperature_bed_raw = raw_temp_bed_value;
				
				//temp_meas_ready = true;
			}
			
			 current_temperature_bed_raw = raw_temp_bed_value;
			  //    printf("\r\n ex0:%d\r\n",current_temperature_raw[0]);
			//	  printf("\r\n bed:%d\r\n",current_temperature_bed_raw);
    }
    temp_meas_ready = true;
    temp_count = 0;
    raw_temp_0_value = 0;


		
	#if EXTRUDERS > 1
   		raw_temp_1_value = 0;
	#endif
    #if EXTRUDERS > 2
    	raw_temp_2_value = 0;
	#endif
    raw_temp_bed_value = 0;
	
	 if((temp_sensor_type == 1) ||((temp_sensor_type == 0)&&(max6675_temp_count>=200)))
   {
		 max6675_temp_count = 0;
		if(emc_switch == 1)
		{
			if((temp_error_flag == 0x01)&&(current_temperature_raw[0] > maxttemp_raw[0])&&(current_temperature_raw[0] < minttemp_raw[0]))//只要一有正常温度出现就将错误标志清零
			{
				temp_error_flag = 0x00;
				temp_error_cnt = 0;
			}
		}
	#if HEATER_0_RAW_LO_TEMP > HEATER_0_RAW_HI_TEMP
	    if(current_temperature_raw[0] <= maxttemp_raw[0]) 
			{
				if(emc_switch == 1)
				{
						current_temperature_raw[0] = Backup_current_temperature_raw;
							if(temp_error_flag == 0x00)//第一次检测到有异常就置异常标志位下面并开始计数
							{
								temp_error_flag = 0x01;
							}
							else
							{
								temp_error_cnt++;
								if(temp_error_cnt>=30)//64ms*30=2s//计够30次2s钟时才报错
								{
									temp_error_cnt = 0;
									max_temp_error(0);
								}
							}  
			 }
        else
				{
				  max_temp_error(0);
				}
	    }
	#else
	    if(current_temperature_raw[0] >= maxttemp_raw[0]) {
		        max_temp_error(0);
	    }
	#endif

	#if HEATER_0_RAW_LO_TEMP > HEATER_0_RAW_HI_TEMP
	    if(current_temperature_raw[0] >= minttemp_raw[0]) 
			{
				if(emc_switch == 1)
				{
						current_temperature_raw[0] = Backup_current_temperature_raw;
						if(temp_error_flag == 0x00)
							{
								temp_error_flag = 0x01;			
							}
							else
							{
								temp_error_cnt++;
								if(temp_error_cnt>=30)
								{
									temp_error_cnt = 0;
									min_temp_error(0);
								}
							}
				 }
				else
				{
				  min_temp_error(0);
				}
		        
	    }
	#else
	    if(current_temperature_raw[0] <= minttemp_raw[0]) {
		        min_temp_error(0);
	    }
	#endif
			
		}
	
	
	#if EXTRUDERS > 1	//第二个挤出机	
   if(temp_sensor_num>1)	
	 {		 
			#if HEATER_1_RAW_LO_TEMP > HEATER_1_RAW_HI_TEMP
					if(current_temperature_raw[1] <= maxttemp_raw[1]) {
								max_temp_error(1);
					}
			#else
					if(current_temperature_raw[1] >= maxttemp_raw[1]) {
								max_temp_error(1);
					}
			#endif	
			#if HEATER_1_RAW_LO_TEMP > HEATER_1_RAW_HI_TEMP
					if(current_temperature_raw[1] >= minttemp_raw[1]) {
								min_temp_error(1);
					}
			#else
					if(current_temperature_raw[1] <= minttemp_raw[1]) {
								min_temp_error(1);
					}
			#endif	
		}
	#endif
	
	
	#if EXTRUDERS > 2  //第三个挤出机	
		 if(temp_sensor_num>2)
		 {
			#if HEATER_2_RAW_LO_TEMP > HEATER_2_RAW_HI_TEMP
					if(current_temperature_raw[2] <= maxttemp_raw[2]) {
								max_temp_error(2);
					}
			#else
					if(current_temperature_raw[2] >= maxttemp_raw[2]) {
								max_temp_error(2);
					}
			#endif
				
			#if HEATER_2_RAW_LO_TEMP > HEATER_2_RAW_HI_TEMP
					if(current_temperature_raw[2] >= minttemp_raw[2]) {
								min_temp_error(2);
					}
			#else
					if(current_temperature_raw[2] <= minttemp_raw[2]) {
								min_temp_error(2);
					}
		#endif
			}
	
	#endif
	 
	  
	  /* No bed MINTEMP error? */
	#if defined(BED_MAXTEMP) && (TEMP_SENSOR_BED != 0)
		# if HEATER_BED_RAW_LO_TEMP > HEATER_BED_RAW_HI_TEMP
		    if(current_temperature_bed_raw <= bed_maxttemp_raw) 
				{
					if(emc_switch == 1)
					{
							current_temperature_bed_raw = Backup_current_temperature_bed_raw;
							if(bed_temp_error_flag == 0x00)
								{
									bed_temp_error_flag = 0x01;			
								}
								else
								{
									bed_temp_error_cnt++;
									if(bed_temp_error_cnt>=30)
									{
										bed_temp_error_cnt = 0;
										target_temperature_bed = 0;
					 	        bed_max_temp_error();
									}
								}
					 }
					 else
					 {
						target_temperature_bed = 0;
					 	bed_max_temp_error();
					 }
		    }
		#else
		    if(current_temperature_bed_raw >= bed_maxttemp_raw) {
			       target_temperature_bed = 0;
		           bed_max_temp_error();
		    }
		#endif
	#endif
  }  
}

}

#ifdef PIDTEMP
// Apply the scale factors to the PID values


float scalePID_i(float i)
{
	return i*PID_dT;
}

float unscalePID_i(float i)
{
	return i/PID_dT;
}

float scalePID_d(float d)
{
    return d/PID_dT;
}

float unscalePID_d(float d)
{
	return d*PID_dT;
}

#endif //PIDTEMP
float degHotend(u8 extruder) 
{  
  return current_temperature[extruder];
}

float degBed(void) 
{
  return current_temperature_bed;
}


float degTargetHotend(u8 extruder) 
{  
  return target_temperature[extruder];
}

float degTargetBed(void) 
{   
  return target_temperature_bed;
}
void setTargetHotend(const float celsius, uint8_t extruder)
{
	if (celsius==0) 
	{
//		if(extruder==0)
		 E0_FAN=0;
//		else if(extruder==1)
//		 E1_FAN=0;
	}
	else 
	{
//		if(extruder==0)
		 E0_FAN=1;
//		else if(extruder==1)
//		 E1_FAN=1;
	}
	if(celsius>HEATER_0_MAXTEMP)
	{
		target_temperature[extruder] = HEATER_0_MAXTEMP;
	}
	else
	{
		target_temperature[extruder] = celsius;
	}
}

void setTargetBed(const float celsius) 
{ if(celsius>BED_MAXTEMP)
	{target_temperature_bed=BED_MAXTEMP;
	}
	else
	{
  target_temperature_bed = celsius;
	}
}

bool isHeatingHotend(u8 extruder)
{  
  return target_temperature[extruder] > current_temperature[extruder];
}

bool isHeatingBed(void) 
{
  return target_temperature_bed > current_temperature_bed;
}

bool isCoolingHotend(u8 extruder) 
{  
  return target_temperature[extruder] < current_temperature[extruder];
}

bool isCoolingBed(void) 
{
  return target_temperature_bed < current_temperature_bed;
}
void autotempShutdown(void)//没用，在取消打印里有将温度置为0
{
 #ifdef AUTOTEMP
 if(autotemp_enabled)
 {
  autotemp_enabled=false;
  if(degTargetHotend(tmp_extruder)>autotemp_min)//20161218
	{
    //setTargetHotend(0,tmp_extruder);
		 {//20170111
			int e; 
			for(e = 0; e < EXTRUDERS; e++) 
			setTargetHotend(0,e);//20161125  20170111
		 }
	}
 }
 #endif
}
