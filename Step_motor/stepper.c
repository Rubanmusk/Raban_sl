#include "delay.h"
#include "Marlin.h"
#include "stepper.h"
#include "Configuration_adv.h"
#include "Configuration.h"
#include "speed_lookuptable.h"
#include "usart.h"	
#include "spi.h"
#include "language.h"
#include "temperature.h"
#include "lcdmenu.h"
//#include "beep.h"


#define DIGIPOT_CHANNELS {4,1,0,2,3} // X Y Z E0 E1 digipot channels to stepper driver mapping

static u8 subsection_x_value=1;
static u8 subsection_y_value=1;
static u8 subsection_z_value=1;
static u8 subsection_e0_value=1;
static u8 subsection_e1_value=1;


#define ENABLE_STEPPER_DRIVER_INTERRUPT()  TIM_ITConfig(TIM3,TIM_IT_Update, ENABLE);  //使能TIMx
#define DISABLE_STEPPER_DRIVER_INTERRUPT() TIM_ITConfig(TIM3,TIM_IT_Update, DISABLE); 
/*
  stepper.c - stepper motor driver: executes motion plans using stepper motors
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

/* The timer calculations of this module informed by the 'RepRap cartesian firmware' by Zack Smith
   and Philipp Tiefenbacher. */


//#include "planner.h"
//
//#include "ultralcd.h"
//
//#include "cardreader.h"



//===========================================================================
//=============================public variables  ============================
//===========================================================================
block_t *current_block;  // A pointer to the block currently being traced


//===========================================================================
//=============================private variables ============================
//===========================================================================
//static makes it inpossible to be called from outside of this file by extern.!

// Variables used by The Stepper Driver Interrupt
static unsigned char out_bits;        // The next stepping-bits to be output
static long counter_x,       // Counter variables for the bresenham line tracer
            counter_y, 
            counter_z,       
            counter_e;
volatile static unsigned long step_events_completed; // The number of step events executed in the current block
volatile static unsigned long x_step_events_completed; // The number of step events executed in the current block
volatile static unsigned long y_step_events_completed; // The number of step events executed in the current block

static long acceleration_time, deceleration_time;
//static unsigned long accelerate_until, decelerate_after, acceleration_rate, initial_rate, final_rate, nominal_rate;
static unsigned long acc_step_rate; // needed for deccelaration start point
static unsigned short TIME3_nominal;
static unsigned short  step_loops;
static unsigned short step_loops_nominal;
static unsigned short  x_step_loops;
static unsigned short  y_step_loops;
static float  x_step_loops_float;
static float  y_step_loops_float;

volatile long endstops_trigsteps[3]={0,0,0};
volatile long endstops_stepsTotal,endstops_stepsDone;
static volatile bool endstop_x_hit=false;
static volatile bool endstop_y_hit=false;
static volatile bool endstop_z_hit=false;
#ifdef ABORT_ON_ENDSTOP_HIT_FEATURE_ENABLED
bool abort_on_endstop_hit = false;
#endif
#if defined X_MIN_PIN
//static bool old_x_min_endstop=true;
#endif
#if defined X_MAX_PIN
//static bool old_x_max_endstop=true;
#endif
#if defined Y_MIN_PIN
//static bool old_y_min_endstop=true;
#endif
#if defined Y_MAX_PIN
//static bool old_y_max_endstop=true;
#endif
#if defined Z_MIN_PIN
static bool old_z_min_endstop=true;
#endif
#if defined Z_MAX_PIN
static bool old_z_max_endstop=true;
#endif
static bool check_endstops = true;

volatile long count_position[NUM_AXIS] = { 0, 0, 0, 0};
volatile signed char count_direction[NUM_AXIS] = { 1, 1, 1, 1};

//===========================================================================
//=============================functions         ============================
//===========================================================================

#define CHECK_ENDSTOPS  if(check_endstops)

#define MultiU24X24toH16(intRes, longIn1, longIn2) intRes= ((uint64_t)(longIn1) * (longIn2)) >> 24
#define MultiU16X8toH16(intRes, charIn1, intIn2) intRes = ((charIn1) * (intIn2)) >> 16
void checkHitEndstops(void)
{
 if( endstop_x_hit || endstop_y_hit || endstop_z_hit) {
   SERIAL_ECHO_START;
   printf(MSG_ENDSTOPS_HIT);
   if(endstop_x_hit) {
     printf(" X:%f",(float)endstops_trigsteps[X_AXIS]/axis_steps_per_unit[X_AXIS]);
    // LCD_MESSAGEPGM(MSG_ENDSTOPS_HIT "X");  ////////////////////////////////////////////////////
   }
   if(endstop_y_hit) {
     printf(" Y:%f",(float)endstops_trigsteps[Y_AXIS]/axis_steps_per_unit[Y_AXIS]);
   //  LCD_MESSAGEPGM(MSG_ENDSTOPS_HIT "Y");   ////////////////////////////////////////////////////////////
   }
   if(endstop_z_hit) {
     printf(" Z:%f",(float)endstops_trigsteps[Z_AXIS]/axis_steps_per_unit[Z_AXIS]);
   //  LCD_MESSAGEPGM(MSG_ENDSTOPS_HIT "Z");  ////////////////////////////////////////////
   }
   printf("\n");
   endstop_x_hit=false;
   endstop_y_hit=false;
   endstop_z_hit=false;
#ifdef ABORT_ON_ENDSTOP_HIT_FEATURE_ENABLED
   if (abort_on_endstop_hit)
   {
     card.sdprinting = false;
     card.closefile();
     quickStop();
     setTargetHotend0(0);
     setTargetHotend1(0);
     setTargetHotend2(0);
   }
#endif
 }
}

void endstops_hit_on_purpose(void)
{
  endstop_x_hit=false;
  endstop_y_hit=false;
  endstop_z_hit=false;
}

void enable_endstops(bool check)
{
  check_endstops = check;
}

//         __________________________
//        /|                        |\     _________________         ^
//       / |                        | \   /|               |\        |
//      /  |                        |  \ / |               | \       s
//     /   |                        |   |  |               |  \      p
//    /    |                        |   |  |               |   \     e
//   +-----+------------------------+---+--+---------------+----+    e
//   |               BLOCK 1            |      BLOCK 2          |    d
//
//                           time ----->
// 
//  The trapezoid is the shape the speed curve over time. It starts at block->initial_rate, accelerates 
//  first block->accelerate_until step_events_completed, then keeps going at constant speed until 
//  step_events_completed reaches block->decelerate_after after which it decelerates until the trapezoid generator is reset.
//  The slope of acceleration is calculated with the leib ramp alghorithm.

void st_wake_up(void) {
  ENABLE_STEPPER_DRIVER_INTERRUPT();  
}



  

unsigned short calc_timer(unsigned 	long step_rate) 
{
  unsigned short timer;

  if(step_rate > MAX_STEP_FREQUENCY) step_rate = MAX_STEP_FREQUENCY;

  if(step_rate > 1280000) { // If steprate > 320kHz >> step 16 times
    step_rate = step_rate >> 8;//&0x07ff;
    step_loops = 256;
  }		
  else if(step_rate > 640000) { // If steprate > 320kHz >> step 16 times
    step_rate = step_rate >> 7;//&0x07ff;
    step_loops = 128;
  }	
  else if(step_rate > 320000) { // If steprate > 320kHz >> step 16 times
    step_rate = step_rate >> 6;//&0x07ff;
    step_loops = 64;
  }
	else if(step_rate > 160000) { // If steprate > 160kHz >> step 16 times
    step_rate = step_rate >> 5;//&0x07ff;
    step_loops = 32;
  }
	else if(step_rate > 80000) { // If steprate > 80kHz >> step 16 times
    step_rate = step_rate >> 4;//&0x0fff;
    step_loops = 16;
  }	
	else if(step_rate > 40000) { // If steprate > 40kHz >> step 8 times
    step_rate = step_rate >> 3;//&0x1fff;
    step_loops = 8;
  }
  else if(step_rate > 20000) { // If steprate > 20kHz >> step 4 times
    step_rate = step_rate >> 2;//&0x3fff;
    step_loops = 4;
  }
  else if(step_rate > 10000) { // If steprate > 10kHz >> step 2 times
    step_rate = step_rate >> 1;//&0x7fff;
    step_loops = 2;
  }
  else {
    step_loops = 1;
  } 
  
  if(step_rate < 100) step_rate = 100;

  timer = 2000000/step_rate - 1;
  if(timer < 99)  timer = 99;//(20kHz this should never happen)
	
  return timer;
}

// Initializes the trapezoid generator from the current block. Called whenever a new 
// block begins.
void trapezoid_generator_reset(void)
{
	float  step_loops_float;
  deceleration_time = 0;
  // step_rate to timer interval
  TIME3_nominal = calc_timer(current_block->nominal_rate);
	
	#ifdef DEBUG_PRINTF 
	printf("trapezoid_generator_reset:block->nominal_rate:%d \r\n",current_block->nominal_rate);
	printf("trapezoid_generator_reset:TIME3_nominal:%d \r\n",TIME3_nominal);
	#endif  
  // make a note of the number of step loops required at nominal speed
	
	
	if((current_block->steps_e == 0) && (current_block->steps_z == 0)) 
	{

		if( current_block->steps_x > current_block->steps_y )
		{
			step_loops_float = step_loops*((float)current_block->steps_y/current_block->steps_x) ;
			x_step_loops =  step_loops;
			x_step_loops_float = 0;			
			y_step_loops =  floor(step_loops_float);	
			y_step_loops_float = step_loops_float-y_step_loops;
		}
		else
		{
			step_loops_float = step_loops*((float)current_block->steps_x/current_block->steps_y) ;
			y_step_loops =  step_loops;
			y_step_loops_float = 0;
			x_step_loops =  floor(step_loops_float);
			x_step_loops_float = step_loops_float-x_step_loops;
		}
	}

  step_loops_nominal = step_loops;
  acc_step_rate = current_block->initial_rate;
  acceleration_time = calc_timer(acc_step_rate);
  TIM_SetAutoreload(TIM3, acceleration_time-1);

    
}

// "The Stepper Driver Interrupt" - This timer interrupt is the workhorse.  
// It pops blocks from the block_buffer and executes them by pulsing the stepper pins appropriately. 
//定时器3中断服务程序
void TIM3_IRQHandler(void)   //TIM3中断
{ 
if(TIM_GetITStatus(TIM3, TIM_IT_Update) != RESET) //检查指定的TIM中断发生与否:TIM 中断源 
{	
	TIM_ClearITPendingBit(TIM3, TIM_IT_Update );  //清除TIMx的中断待处理位:TIM 中断源  
//printf("T3\n");  
  // If there is no current block, attempt to pop one from the buffer
 
	
  if (current_block == NULL) 
	{
    // Anything in the buffer?
    current_block = plan_get_current_block();
    if (current_block != NULL) 
			{
				current_block->busy = true;
				trapezoid_generator_reset();
				counter_x = -(current_block->step_event_count >> 1);
				counter_y = counter_x;
				counter_z = counter_x;
				counter_e = counter_x;
				step_events_completed = 0; 
				x_step_events_completed = 0;
				y_step_events_completed = 0;
      
        #ifdef Z_LATE_ENABLE 
        if(current_block->steps_z > 0) 
				{
          enable_z();
          TIM_SetAutoreload(TIM3, 2000-1);//1ms wait
          return;
        }
       #endif     
				
				
     }
     else
			{
        TIM_SetAutoreload(TIM3, 2000-1);
      }    
  }

  if (current_block != NULL) 
	{
		
    // Set directions TO DO This should be done once during init of trapezoid. Endstops -> interrupt
    out_bits = current_block->direction_bits;
		
		// Set direction en check limit switches
		if((out_bits & (1<<X_AXIS))!=0){
			count_direction[X_AXIS]=-1;
		}
		else{
			count_direction[X_AXIS]=1;
		}
		if((out_bits & (1<<Y_AXIS))!=0){
			count_direction[Y_AXIS]=-1;
		 }
		else {
			count_direction[Y_AXIS]=1;
		}

	
	// Set direction en check limit switches
     
    if ((out_bits & (1<<Z_AXIS)) != 0)  // -direction
		{   // -direction

				Z_DIR_PIN=INVERT_Z_DIR;
				count_direction[Z_AXIS]=-1;
	
      CHECK_ENDSTOPS
      {
        #if defined(Z_MIN_PIN)
          bool z_min_endstop= Z_MIN_PIN != Z_ENDSTOPS_INVERTING;
          if(z_min_endstop && old_z_min_endstop && (current_block->steps_z > 0)) {
            endstops_trigsteps[Z_AXIS] = count_position[Z_AXIS];
            endstop_z_hit=true;
            step_events_completed = current_block->step_event_count;
          }
          old_z_min_endstop = z_min_endstop;
        #endif
      }
    }
    else { // +direction

				Z_DIR_PIN=!INVERT_Z_DIR;
				count_direction[Z_AXIS]=1;

    }
	
		if ((out_bits & (1<<E_AXIS)) != 0) {  // -direction
			REV_E_DIR();
			count_direction[E_AXIS]=-1;
		}
		else { // +direction
			NORM_E_DIR();
			count_direction[E_AXIS]=1;
		} 
		
    {
			int i;
		  int count_x = 0 ;
			int count_y = 0 ;
			
			if((current_block->steps_z == 0)&& (current_block->steps_e == 0))
			{		
					step_events_completed += step_loops;
					if(step_events_completed >= current_block->step_event_count) 
					{
						count_x = current_block->steps_x-x_step_events_completed;
						count_y	=	current_block->steps_y-y_step_events_completed;				
					}
					else
					{
						count_x = x_step_loops;
						count_y = y_step_loops;
						if(x_step_loops_float>1)
						{
							count_x++;
							x_step_loops_float -= 1;
						}
						else
						{
							x_step_loops_float +=x_step_loops_float;
						}
						
						if(y_step_loops_float>1)
						{
							count_y++;
							y_step_loops_float -= 1;
						}
						else
						{
							y_step_loops_float +=y_step_loops_float;
						}
					}
			}
			else
			{
				for(i=0; i < step_loops; i++) 
				{
					counter_x += current_block->steps_x;
					counter_y += current_block->steps_y;
					counter_z += current_block->steps_z;
					counter_e += current_block->steps_e;
					
					if((counter_x > 0)||(counter_y > 0)||(counter_z > 0)||(counter_e > 0))
					{
						if(counter_z > 0) 
						 Z_STEP_PIN = !INVERT_Z_STEP_PIN;
						if (counter_e > 0)
						 E0_STEP_PIN = !INVERT_E_STEP_PIN;
						
						checkRx();
						if(counter_x > 0)
						{
							counter_x -= current_block->step_event_count;
							count_x +=count_direction[X_AXIS];   
						}
						if (counter_y > 0) 
						{
							counter_y -= current_block->step_event_count; 
							count_y +=count_direction[Y_AXIS];  

						}
						
						if (counter_z > 0)
						{
							 counter_z -= current_block->step_event_count;
							 count_position[Z_AXIS]+=count_direction[Z_AXIS];
							 Z_STEP_PIN= INVERT_Z_STEP_PIN;
						}
						if (counter_e > 0)
						{
							counter_e -= current_block->step_event_count;
							count_position[E_AXIS]+=count_direction[E_AXIS];
							WRITE_E_STEP(INVERT_E_STEP_PIN);
						}
					}
					
					step_events_completed += 1;  
					if(step_events_completed >= current_block->step_event_count) break;
				}		
			
			}
			
			if(count_x != 0)
			{	
				
				x_step_events_completed +=	 count_x; 
				count_position[X_AXIS] +=  count_direction[X_AXIS]*count_x;							
			//	move_X_galvo();
			//	printf("---%d\r\n",count_position[X_AXIS]);
				
				
			}
			if(count_y != 0)
			{
				y_step_events_completed +=	 count_y;
				count_position[Y_AXIS] +=  count_direction[Y_AXIS]*count_y;							
			//	move_Y_galvo();
				
			}
			{
				set_X_galvo(count_position[X_AXIS]+32767);
				set_Y_galvo(count_position[Y_AXIS]+32767);
			}
	
		}
		if((current_block->steps_z == 0)&& (current_block->steps_e == 0))
		{
			TIM_SetAutoreload(TIM3,TIME3_nominal-1);
		}
		else
		{    // Calculare new timer value

			unsigned short timer;
			unsigned long step_rate;

			if (step_events_completed <= (unsigned long int)current_block->accelerate_until)  // 加速区
			{
				MultiU24X24toH16(acc_step_rate, acceleration_time, current_block->acceleration_rate);
				acc_step_rate += current_block->initial_rate;
				
				// upper limit
				if(acc_step_rate > current_block->nominal_rate)
					acc_step_rate = current_block->nominal_rate;

				// step_rate to timer interval
				
				timer = calc_timer(acc_step_rate);
				#ifdef DEBUG_PRINTF 
				printf("1 acc_step_rate:%d TIME3_nominal:%d\r\n",acc_step_rate,timer);
				#endif
			//	printf("1 timer:%d \r\n",timer);
				TIM_SetAutoreload(TIM3, timer-1);
				acceleration_time += timer;

			} 
			else if (step_events_completed > (unsigned long int)current_block->decelerate_after) //减速区
			{
					MultiU24X24toH16(step_rate, deceleration_time, current_block->acceleration_rate);
					#ifdef DEBUG_PRINTF 
					printf("MultiU24X24toH16 step_rate:%d  acc_step_rate:%d current_block->final_rate:%d\r\n",step_rate,acc_step_rate,current_block->final_rate);
					#endif	      
					if(step_rate > acc_step_rate) { // Check step_rate stays positive
						step_rate = current_block->final_rate;
					}
					else {
						step_rate = acc_step_rate - step_rate; // Decelerate from aceleration end point.
					}
		
					// lower limit
					if(step_rate < current_block->final_rate)
						step_rate = current_block->final_rate;
		
					// step_rate to timer interval
					timer = calc_timer(step_rate);
					#ifdef DEBUG_PRINTF 
					printf("3 current_block->acceleration_rate:%d deceleration_time:%d step_rate:%d TIME3_nominal:%d\r\n",current_block->acceleration_rate,deceleration_time,step_rate,timer);
					#endif
				//	printf("3 timer:%d \r\n",timer);
					TIM_SetAutoreload(TIM3, timer-1);
					deceleration_time += timer;
			}
			else 				//匀速
			{
					#ifdef DEBUG_PRINTF 
					printf("2 TIME3_nominal:%d\r\n",TIME3_nominal);
					#endif
					//	printf("2 timer:%d \r\n",TIME3_nominal);
					TIM_SetAutoreload(TIM3,TIME3_nominal-1);
					// ensure we're running at the correct step rate, even if we just came off an acceleration
					step_loops = step_loops_nominal;
			}
		}     // Calculare new timer value
    // If current block is finished, reset pointer 
    if (step_events_completed >= current_block->step_event_count) 
		{
				current_block = NULL;
				plan_discard_current_block();
		}  
		
	 } 
		
  }

}



void st_init(void)
{	 
	 GPIO_InitTypeDef  GPIO_InitStructure;
	 TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
	 NVIC_InitTypeDef NVIC_InitStructure;
 	 //步进电机驱动引脚初始化---MINI版
	// GPIO_PinRemapConfig(GPIO_Remap_SWJ_NoJTRST, ENABLE);
   //GPIO_PinRemapConfig(GPIO_Remap_SWJ_Disable, ENABLE);//20161218
	 	GPIO_PinRemapConfig(GPIO_Remap_SWJ_JTAGDisable, ENABLE);//关闭JTAG,可用SWD模式
	 RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA|RCC_APB2Periph_GPIOB|RCC_APB2Periph_GPIOC, ENABLE);	 
	
	
	 GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP; 		 //推挽输出
	 GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;	 //速度为50MHz

	 GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0|GPIO_Pin_1|GPIO_Pin_2|GPIO_Pin_12;		
	 GPIO_Init(GPIOB, &GPIO_InitStructure);	

	 GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0|GPIO_Pin_1|GPIO_Pin_4|GPIO_Pin_5|GPIO_Pin_6|GPIO_Pin_13|GPIO_Pin_14|GPIO_Pin_15;		
	 GPIO_Init(GPIOC, &GPIO_InitStructure);	
 
//	 GPIO_InitStructure.GPIO_Pin = GPIO_Pin_12;		//E_DEF
//	 GPIO_Init(GPIOA, &GPIO_InitStructure);	
	//  E0_DET-->GPIOA12,		

	//初始化限位引脚
	//		  XMIN-->GPIOC7
	//      YMIN-->GPIOA8
	//      ZMIN-->GPIOA11	 
	GPIO_InitStructure.GPIO_Pin  = GPIO_Pin_7;//PC7
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU; //设置成上拉输入
 	GPIO_Init(GPIOC, &GPIO_InitStructure);//
  
	GPIO_InitStructure.GPIO_Pin  = GPIO_Pin_8; //PA8
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU; //设置成上拉输入 
	GPIO_Init(GPIOA, &GPIO_InitStructure);//初始化
	

	
	 //电机驱动器使能   steppers default to disabled.
	 disable_x();	  
	 disable_y();	  
	 disable_z();
	 disable_e0();	  
	 //disable_e1();	  

	 //初始化step pin
	 Z_STEP_PIN=INVERT_Z_STEP_PIN;
	 E0_STEP_PIN=INVERT_E_STEP_PIN;
	// E1_STEP_PIN=INVERT_E_STEP_PIN;


  // digipot_init(); //Initialize Digipot Motor Current
  //microstep_init(); //Initialize Microstepping Pins

  // Set the timer pre-scaler
  // Generally we use a divider of 8, resulting in a 2MHz timer
  // frequency on a 16MHz MCU. If you are going to change this, be
  // sure to regenerate speed_lookuptable.h with
  // create_speed_lookuptable.py

	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3, ENABLE); //时钟使能

	TIM_TimeBaseStructure.TIM_Period = 0x4000; //设置在下一个更新事件装入活动的自动重装载寄存器周期的值  8ms
	TIM_TimeBaseStructure.TIM_Prescaler =35; //设置用来作为TIMx时钟频率除数的预分频值  2Mhz的计数频率  
	TIM_TimeBaseStructure.TIM_ClockDivision = 0; //设置时钟分割:TDTS = Tck_tim
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;  //TIM向上计数模式
	TIM_TimeBaseInit(TIM3, &TIM_TimeBaseStructure); //根据TIM_TimeBaseInitStruct中指定的参数初始化TIMx的时间基数单位
 
	TIM_ITConfig(TIM3,TIM_IT_Update,ENABLE ); //使能指定的TIM3中断,允许更新中断

	NVIC_InitStructure.NVIC_IRQChannel = TIM3_IRQn;  //TIM3中断
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;  //先占优先级0级
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 3;  //从优先级3级
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE; //IRQ通道被使能
	NVIC_Init(&NVIC_InitStructure);  //根据NVIC_InitStruct中指定的参数初始化外设NVIC寄存器
	TIM_Cmd(TIM3, ENABLE);  //使能TIM3 	
  ENABLE_STEPPER_DRIVER_INTERRUPT();  

  
  enable_endstops(1); // Start with endstops active. After homing they can be disabled
 // sei();开启全局中断
}


// Block until all buffered steps are executed
void st_synchronize(void)
{
    while( blocks_queued()) //当头和尾相等时才能跳出这个循环，也就是当所有block都执行完时
	 {
    manage_heater();
    manage_inactivity();
    lcd_update();	 //2.循环显示   等待当前的block都执行完毕       
  }
}

void st_set_position(const long x, const long y, const long z, const long e)
{
  CRITICAL_SECTION_START;
  count_position[X_AXIS] = x;
  count_position[Y_AXIS] = y;
  count_position[Z_AXIS] = z;
  count_position[E_AXIS] = e;
  CRITICAL_SECTION_END; 
}

void st_set_e_position(const long e)
{
  CRITICAL_SECTION_START;
  count_position[E_AXIS] = e;
  CRITICAL_SECTION_END;
}

long st_get_position(uint8_t axis)
{
  long count_pos;
  CRITICAL_SECTION_START;
  count_pos = count_position[axis];
  CRITICAL_SECTION_END;
  return count_pos;
}

void finishAndDisableSteppers(void)
{
  st_synchronize(); 
  disable_x(); 
  disable_y(); 
  disable_z();
  disable_e0(); 
  //disable_e1(); 
}

void quickStop(void)
{
  DISABLE_STEPPER_DRIVER_INTERRUPT();
  while(blocks_queued())
    plan_discard_current_block();
  current_block = NULL;
	
	      disable_x();
        disable_y();
        disable_z();
        disable_e0();
       // disable_e1();
	
  ENABLE_STEPPER_DRIVER_INTERRUPT();
}


void digipot_init(void) //Initialize Digipot Motor Current
{   const uint8_t digipot_motor_current[] = DIGIPOT_MOTOR_CURRENT;  
	int i;     
    for(i=0;i<=4;i++) 
      digipot_current(i,digipot_motor_current[i]);
}

void digipot_current(uint8_t driver, uint8_t current)
{
    const uint8_t digipot_ch[] = DIGIPOT_CHANNELS;
//	printf("%d:%d\r\n",digipot_ch[driver],current);
    digitalPotWrite(digipot_ch[driver], (uint8_t)current);
}

void digitalPotWrite(uint8_t address, uint8_t value) // From Arduino DigitalPotControl example
{
/*
    DIGIPOTSS_PIN=1; // take the SS pin low to select the chip
    SPI1_ReadWriteByte(address); //  send in the address and value via SPI:
    SPI1_ReadWriteByte(value);
    DIGIPOTSS_PIN=0; // take the SS pin high to de-select the chip:
	*/
}

void microstep_init(void)
{ int i;
  for(i=0;i<=4;i++) microstep_mode(i,8);//16细分
 // for(i=3;i<=4;i++) microstep_mode(i,16);//16细分microstep_mode(4,8);//16细分
}

void microstep_ms(uint8_t driver, int8_t ms1, int8_t ms2, int8_t ms3)
{/*
  if(ms1 > -1) switch(driver)
  {
    case 0:X_MS1_PIN=ms1 ; break;
    case 1:Y_MS1_PIN=ms1 ; break;
    case 2:Z_MS1_PIN=ms1 ; break;
    case 3:E0_MS1_PIN=ms1 ; break;
    case 4:E1_MS1_PIN=ms1 ; break;
	default:  break;
  }
  if(ms2 > -1) 
  switch(driver)
  {
    case 0:X_MS2_PIN=ms2 ; break;
    case 1:Y_MS2_PIN=ms2 ; break;
    case 2:Z_MS2_PIN=ms2 ; break;
    case 3:E0_MS2_PIN=ms2 ; break;
    case 4:E1_MS2_PIN=ms2 ; break;
	default:  break;
  }
    if(ms3 > -1) switch(driver)
  {
    case 0:X_MS3_PIN=ms3 ; break;
    case 1:Y_MS3_PIN=ms3 ; break;
    case 2:Z_MS3_PIN=ms3 ; break;
    case 3:E0_MS3_PIN=ms3 ; break;
    case 4:E1_MS3_PIN=ms3 ; break;
	default:  break;
  }
	*/
}


void microstep_mode(uint8_t driver, uint8_t stepping_mode)
{ switch(driver)
  {
    case 0: subsection_x_value=stepping_mode; break;
    case 1: subsection_y_value=stepping_mode; break;
    case 2: subsection_z_value=stepping_mode; break;
    case 3: subsection_e0_value=stepping_mode; break;
    case 4: subsection_e1_value=stepping_mode; break;
	default:  break;
  }
  switch(stepping_mode)
  {
    case 1: microstep_ms(driver,MICROSTEP1); break;
    case 2: microstep_ms(driver,MICROSTEP2); break;
    case 4: microstep_ms(driver,MICROSTEP4); break;
    case 8: microstep_ms(driver,MICROSTEP8); break;
    case 16: microstep_ms(driver,MICROSTEP16); break;
    case 32: microstep_ms(driver,MICROSTEP32); break;
    case 64: microstep_ms(driver,MICROSTEP64); break;
    case 128: microstep_ms(driver,MICROSTEP128); break;
	default:  break;
  }
}
void microstep_readings(void)
{
	printf("Motor_Subsection \n");
	printf("X: %d\n",subsection_x_value);
	printf("Y: %d\n",subsection_y_value);
	printf("Z: %d\n",subsection_z_value);
	printf("E0: %d\n",subsection_e0_value);
	printf("E1: %d\n",subsection_e1_value);
}

void move_X_galvo(void)
{
	u8 txData[3];
	u32 inputShiftData;
	u32 position = 0x0000ffff & count_position[X_AXIS];
	GALVO_SS_PIN = 0 ;
	if(position>0x0000ffff)  position =  0x0000ffff;
	inputShiftData = 0x00180000+ position;

	txData[0]=inputShiftData>>16;
	txData[1]=inputShiftData>>8;
	txData[2]=inputShiftData;
		
	SPI2_WriteByte(txData[0]);
	SPI2_WriteByte(txData[1]);	 
	SPI2_WriteByte(txData[2]);	
	GALVO_SS_PIN = 1 ;
}

void move_Y_galvo(void)
{
	u8 txData[3];
	u32 inputShiftData;
	u32 position = 0x0000ffff & count_position[Y_AXIS];
	GALVO_SS_PIN = 0;
	if(position>0x0000ffff)  position =  0x0000ffff;
	inputShiftData = 0x00190000+ position;

	txData[0]=inputShiftData>>16;
	txData[1]=inputShiftData>>8;
	txData[2]=inputShiftData;
		
	SPI2_WriteByte(txData[0]);
	SPI2_WriteByte(txData[1]);	 
	SPI2_WriteByte(txData[2]);	
	GALVO_SS_PIN = 1;
}

void set_X_galvo(u32 position)
{
	u8 txData[3];
	u32 inputShiftData;
	GALVO_SS_PIN = 0 ;
	if(position>0x0000ffff)  position =  0x0000ffff;
	inputShiftData = 0x00180000+ position;

	txData[0]=inputShiftData>>16;
	txData[1]=inputShiftData>>8;
	txData[2]=inputShiftData;
		
	SPI2_WriteByte(txData[0]);
	SPI2_WriteByte(txData[1]);	 
	SPI2_WriteByte(txData[2]);	
	GALVO_SS_PIN = 1 ;
}

void set_Y_galvo(u32 position)
{
	u8 txData[3];
	u32 inputShiftData;
	GALVO_SS_PIN = 0;
	if(position>0x0000ffff)  position =  0x0000ffff;
	inputShiftData = 0x00190000+ position;

	txData[0]=inputShiftData>>16;
	txData[1]=inputShiftData>>8;
	txData[2]=inputShiftData;
		
	SPI2_WriteByte(txData[0]);
	SPI2_WriteByte(txData[1]);	 
	SPI2_WriteByte(txData[2]);	
	GALVO_SS_PIN = 1;
}

