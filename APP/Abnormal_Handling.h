#ifndef __ABNORMAL_HANDLING_H//Abnormal_Handling
#define __ABNORMAL_HANDLING_H	 
#include "sys.h"
#include "ff.h"


#define POWER_KEY PEout(5)	// GPIO_ResetBits(GPIOE,GPIO_Pin_5); 


extern volatile u8 Abnormal_Flag;

extern u8 Pause_flag;

extern u8 fgcode_fptr_flag;
//extern unsigned long fgcode_fptr;//20160506
//extern unsigned long fgcode_clust;
//extern unsigned long fgcode_dsect;
//extern FIL fgcode_tail;//20160403
//extern unsigned long stop_steps[4];
//extern u32 Backup_feedrate;
//extern char Backup_pname[50];//20160315
extern u16 Store_selindex;//20160507
extern u16 Read_selindex;

//extern u8 Button_continue_flag;


//extern volatile u8 Start_move_flag;//从开始打印到开始走完第一个Blcok(打印模型的第一个点)的标志位
//extern volatile u8 Power_Status;
//extern volatile u8 Print_all_process;
//extern volatile u8 Power_on_process;//20160403


extern volatile u8 Outage_Flag;


void Abnormal_Init(void); //异常处理：目前包括断电、断料和暂停
//void Outage_Pin_Init(void); 
//void Outage_Set_IRQ_Rising(void); 
//void Outage_Set_IRQ_Falling(void); 
//void Outage_EXIT_Set(FunctionalState NewState);  
void Material_Pin_Init(void); //IO初始化
void Material_EXIT_Set(FunctionalState NewState);
void Material_IRQ_Init(void);////PG11外中断使用EXTI9_5_IRQn中断通道，抢占优先级0，响应优先级0  
void Abnormal_Handling(u8 state);
//void Outage_Occur_Handling(void); 
//void Outage_Ok_Handling(void); 
void Material_Occur_Handling(void); 
void Material_Ok_Handling(void);
void Cancle_Print_Handling(void);
void Abnormal_Gcode_seek(void); 
bool Gcode_seek_valid(void);
float Gcode_seek_XYZE(u32 cur_fptr,int axis,bool direction);//前提是该文件已经打开


//void Check_Outage_Init(void);
//void Auto_Check_Outage(void);
	
void Discard_Buf_and_Block(void);//清除所有的Block和cmdbuffer
void Abnormal_Store(void);//20160507
void Abnormal_Read(void);//20160507

void Abnormal_Z_Height(void);


//void Outage_check(void);
//void Power_off(void);
#endif


























