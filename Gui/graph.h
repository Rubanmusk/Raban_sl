#ifndef __GRAPH_H
#define __GRAPH_H	 
#include "guix.h" 	
//////////////////////////////////////////////////////////////////////////////////	 
//本程序只供学习使用，未经作者许可，不得用于其它任何用途
//ALIENTEK战舰STM32开发板
//GUI-进度条 代码	   
//正点原子@ALIENTEK
//技术论坛:www.openedv.com
//修改日期:2012/10/4
//版本：V1.0
//版权所有，盗版必究。
//Copyright(C) 广州市星翼电子科技有限公司 2009-2019
//All rights reserved									  
//////////////////////////////////////////////////////////////////////////////////
extern const u32 ORDINZTEE_UNIT_TABL[];
//曲线图结构体定义
__packed typedef struct 
{
	u16 top; 				  		//按钮顶端坐标
	u16 left;             //按钮左端坐标
	u16 width; 				  	//宽度
	u16 height;						//高度
	
	u8 scanf_time; 				//已刷新次数
	
	u8 ordinate_unit_num; //纵坐标单位数量   
	u8 abscissa_unit_num; //横坐标单位数数量 
	
	u32 ordinate_unit_value; //纵坐标单位值 
//	float abscissa_unit_value; //横坐标单位值 
	
  int ordinate_start_value; //纵坐标坐标原点值
//	float abscissa_start_value; //横坐标坐标原点值
	
	float max_value;			//
	float min_value;			//

	float data_0_temp; 	//更新data_0 缓冲
	float data_1_temp;	//更新data_1 缓冲
	
	float *data_0; 	
	float *data_1;
	
	u16 data_0_color;
	u16 data_1_color;
	
}_graph_obj;


_graph_obj * graph_creat(u16 left,u16 top,u16 width,u16 height,u8 abscissa_unit_num,u8 ordinate_unit_num);
void graph_delete(_graph_obj * graph_del);
void graph_draw(_graph_obj * graph);					//画曲线图
void graph_updata_array(float *array,u8 size,float data,u8  position);

#endif





