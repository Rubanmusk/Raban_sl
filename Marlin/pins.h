#ifndef PINS_H
#define PINS_H

//All these generations of Gen7 supply thermistor power
//via PS_ON, so ignore bad thermistor readings
//#define BOGUS_TEMPERATURE_FAILSAFE_OVERRIDE

//#define X_DIR_PIN     PBout(12) 
//#define X_STEP_PIN    PBout(2)     
#define X_ENABLE_PIN  PCout(6) 
#define X_MIN_PIN     PCin(7) 
//#define X_MAX_PIN     PGin(15)    
//#define X_MS1_PIN 	   PAout(14)
//#define X_MS2_PIN 	   PAout(13)
//#define X_MS3_PIN 	   PAout(9)

//#define Y_DIR_PIN      PBout(0)
//#define Y_STEP_PIN     PCout(5) 
#define Y_ENABLE_PIN   PBout(1)
#define Y_MIN_PIN      PAin(8) 
//#define Y_MAX_PIN      PGin(14)  
//#define Y_MS1_PIN      PGout(8) 
//#define Y_MS2_PIN      PGout(7) 
//#define Y_MS3_PIN      PGout(6) 

#define Z_DIR_PIN      PCout(1) 
#define Z_STEP_PIN     PCout(0) 
#define Z_ENABLE_PIN   PCout(4)
#define Z_MIN_PIN      PAin(11) 
//#define Z_MAX_PIN      PBin(13)   //调平
//#define Z_MS1_PIN      PCout(4)
//#define Z_MS2_PIN      PCout(3)
//#define Z_MS3_PIN      PCout(2)


#define E0_DIR_PIN     PCout(14) 
#define E0_STEP_PIN    PCout(15)         
#define E0_ENABLE_PIN  PCout(13)     
//#define E0_MS1_PIN     PFout(9)
//#define E0_MS2_PIN 	   PFout(8)
//#define E0_MS3_PIN 	   PFout(7)
#define E0_MAX_PIN      PAin(12) 

#define UART_DTR_PIN     PAin(8) 
//#define E1_DIR_PIN	   PAout(13) 
//#define E1_STEP_PIN    PAout(14)	
//#define E1_ENABLE_PIN   PAout(15) 
////#define E1_MS1_PIN      PEout(5) 
////#define E1_MS2_PIN      PEout(4) 
////#define E1_MS3_PIN      PEout(3)
//#define E1_MAX_PIN      PDin(3) 

#define  HEATER_0_PIN   TIM5->CCR2	  //E0_PWM T8_2            E0加热-PWM
//#define  HEATER_1_PIN   TIM1->CCR4	  //E1_PWM T5_3            E1加热-PWM
#define  HEATER_BED_PIN TIM5->CCR1	  //BED_PWM T8_3           BED加热-PWM

#define  LASER_PIN        TIM5->CCR3	  //激光接口

#define  E0_FAN       PAout(3) // TIM5->CCR2  	//E0_FAN T5_2   E0风扇，只有开关没有PWM
//#define  E1_FAN       PAout(12) // TIM5->CCR2  	//E0_FAN T5_2   E0风扇，只有开关没有PWM
 
#define TEMP_0_PIN	   (Get_Adc(ADC_Channel_12)>>2)   //PC2						// AD3_12	E0_TEMP  
//#define TEMP_1_PIN	 (Get_Adc(ADC_Channel_13)>>2)   						// AD3_6	E1_TEMP
#define TEMP_BED_PIN   (Get_Adc(ADC_Channel_13)>>2)   //PC3						// AD3_13  BED_TEMP


#define GALVO_SS_PIN  PBout(14)  	//PA4  CS  

#endif
