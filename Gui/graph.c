#include "graph.h"
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
const u32 ORDINZTEE_UNIT_TABL[11]=
{	1,2,5,10,20,25,50,100,150,200,250
};

//	u16 top; 				  		//graph顶端坐标
//	u16 left;             //graph左端坐标
//	u16 width; 				  	//宽度
//	u16 height;						//高度
//	u8 abscissa_unit_num; //横坐标单位数数量 		
//	u8 ordinate_unit_num; //纵坐标单位数量   

//创建曲线图
_graph_obj * graph_creat(u16 left,u16 top,u16 width,u16 height,u8 abscissa_unit_num,u8 ordinate_unit_num)
{	
	_graph_obj *_temp_graph;
	
	_temp_graph=(_graph_obj*)gui_memin_malloc(sizeof(_graph_obj));//分配内存
	if(_temp_graph==NULL)return NULL;//内存分配不够.
	
	_temp_graph->top=top; 				 	//graph顶端坐标
	_temp_graph->left=left;         //graph左端坐标
	_temp_graph->width=width; 				  	//宽度
	_temp_graph->height=height;						//高度
	
	_temp_graph->scanf_time=0; 				//已刷新次数
	
	_temp_graph->max_value=0;			//
	_temp_graph->min_value=0;			//
	
	_temp_graph->ordinate_unit_num=ordinate_unit_num; //纵坐标单位格数
	_temp_graph->abscissa_unit_num=abscissa_unit_num; //横坐标单位格数

	_temp_graph->ordinate_unit_value=0;
	_temp_graph->ordinate_start_value=0;
	
	_temp_graph->data_0=(float*)gui_memin_malloc(sizeof(float)*_temp_graph->abscissa_unit_num+1);//为横坐标点数申请内存
	_temp_graph->data_1=(float*)gui_memin_malloc(sizeof(float)*_temp_graph->abscissa_unit_num+1);

	_temp_graph->data_0_color=RED;
	_temp_graph->data_1_color=LIGHTGREEN;
	
	return _temp_graph;
}
//画曲线图
void graph_draw(_graph_obj * graph)	
{ u8 i;
	int res;
	float temp;
	
	gui_fill_rectangle(graph->left,graph->top,graph->width,graph->height,WHITE );//填充背景色
	
	gui_draw_vline(graph->left-1,graph->top,graph->height+1,BLACK);//垂直线
	gui_draw_hline(graph->left-1,graph->top+graph->height,graph->width+1,BLACK);//水平线
	
	for(i=1;i<graph->abscissa_unit_num;i++)
	{	gui_draw_vline(graph->left+i*(graph->width/graph->abscissa_unit_num)-1,graph->top,graph->height,LIGHTGRAY);//垂直线
	}
	for(i=1;i<graph->ordinate_unit_num;i++)
	{	gui_draw_hline(graph->left,graph->top+i*(graph->height/graph->ordinate_unit_num)-1,graph->width,LIGHTGRAY);//水平线
	}


	
	graph_updata_array(graph->data_0,graph->abscissa_unit_num,graph->data_0_temp,graph->scanf_time);		  //更新当前温度
	graph_updata_array(graph->data_1,graph->abscissa_unit_num,graph->data_1_temp,graph->scanf_time);			//更新目标温度
	
	if(*graph->data_0>*graph->data_1)
	{	graph->max_value=*graph->data_0;
		graph->min_value=*graph->data_1;
	}
	else
	{	graph->max_value=*graph->data_1;
		graph->min_value=*graph->data_0;
	}

	for(i=1;i<graph->scanf_time;i++)
	{	if(graph->scanf_time>0)
		{	if(graph->min_value>*(graph->data_0+i))
			{	graph->min_value=*(graph->data_0+i);
			}
			if(graph->min_value>*(graph->data_1+i))
			{	graph->min_value=*(graph->data_1+i);
			}
			
			if(graph->max_value<*(graph->data_0+i))
			{	graph->max_value=*(graph->data_0+i);
			}
			if(graph->max_value<*(graph->data_1+i))
			{	graph->max_value=*(graph->data_1+i);
			}
			
		}
	}
	
	//超出显示范围
	if(	(graph->ordinate_start_value>graph->min_value)||
			((graph->ordinate_start_value+ORDINZTEE_UNIT_TABL[graph->ordinate_unit_value]*graph->ordinate_unit_num)<graph->max_value)
		)
	{		
		if((((ORDINZTEE_UNIT_TABL[graph->ordinate_unit_value]*graph->ordinate_unit_num)-(graph->max_value-graph->min_value))>
				(ORDINZTEE_UNIT_TABL[graph->ordinate_unit_value]*1)))//差值大于一个单位 说明只是一边超出范围 只需要调整纵轴原点
			{ if(graph->ordinate_start_value>graph->min_value) //最小值超出纵轴原点
				{ while(graph->ordinate_start_value>graph->min_value)		
						{graph->ordinate_start_value=graph->ordinate_start_value-ORDINZTEE_UNIT_TABL[graph->ordinate_unit_value];
						}
				}
				else	//最大值超出纵轴最大值
				{ while((graph->ordinate_start_value+ORDINZTEE_UNIT_TABL[graph->ordinate_unit_value]*graph->ordinate_unit_num)<graph->max_value)
						{	graph->ordinate_start_value=graph->ordinate_start_value+ORDINZTEE_UNIT_TABL[graph->ordinate_unit_value];
						}
				}	
			}
		else	//差值小于一个单位  需要调整单位值 
			{							
				while(	
							 (((ORDINZTEE_UNIT_TABL[graph->ordinate_unit_value]*graph->ordinate_unit_num)-(graph->max_value-graph->min_value))<
							 (ORDINZTEE_UNIT_TABL[graph->ordinate_unit_value]*1))
							)
					{ ++graph->ordinate_unit_value;//增大单位值
						if(graph->ordinate_unit_value>10) 
						{graph->ordinate_unit_value=10;
						 break;
						}
					}
					if(graph->ordinate_unit_value<10)
					{
							res=((ORDINZTEE_UNIT_TABL[graph->ordinate_unit_value]*graph->ordinate_unit_num)-(graph->max_value-graph->min_value))/
								 (ORDINZTEE_UNIT_TABL[graph->ordinate_unit_value]*2);//+graph->min_value;
							graph->ordinate_start_value=graph->min_value/ORDINZTEE_UNIT_TABL[graph->ordinate_unit_value];	
							graph->ordinate_start_value=graph->ordinate_start_value*ORDINZTEE_UNIT_TABL[graph->ordinate_unit_value] - ORDINZTEE_UNIT_TABL[graph->ordinate_unit_value]*res;
					
							while(graph->ordinate_start_value < 0)
							{graph->ordinate_start_value=graph->ordinate_start_value+ORDINZTEE_UNIT_TABL[graph->ordinate_unit_value];
							}
					}
					else
					{ graph->ordinate_start_value=0;
					}
			}
		
	}
	else
	{
		if(graph->ordinate_unit_value>0)//最大值与最小值之差大于一定值  单位降低一级 
		{		
				if((((ORDINZTEE_UNIT_TABL[graph->ordinate_unit_value-1]*graph->ordinate_unit_num)-(graph->max_value-graph->min_value))>
							(ORDINZTEE_UNIT_TABL[graph->ordinate_unit_value-1]*1)))
				{		--graph->ordinate_unit_value;
						res=((ORDINZTEE_UNIT_TABL[graph->ordinate_unit_value]*graph->ordinate_unit_num)-(graph->max_value-graph->min_value))/
							 (ORDINZTEE_UNIT_TABL[graph->ordinate_unit_value]*2);//+graph->min_value;
						graph->ordinate_start_value=graph->min_value/ORDINZTEE_UNIT_TABL[graph->ordinate_unit_value];	
						graph->ordinate_start_value=graph->ordinate_start_value*ORDINZTEE_UNIT_TABL[graph->ordinate_unit_value] - ORDINZTEE_UNIT_TABL[graph->ordinate_unit_value]*res;
				
						while(graph->ordinate_start_value < 0)
						{graph->ordinate_start_value=graph->ordinate_start_value+ORDINZTEE_UNIT_TABL[graph->ordinate_unit_value];
						}
				}			
		}
		
		res=((ORDINZTEE_UNIT_TABL[graph->ordinate_unit_value]*graph->ordinate_unit_num)-(graph->max_value-graph->min_value))/(ORDINZTEE_UNIT_TABL[graph->ordinate_unit_value]*2) ;
		res=res-(graph->min_value-graph->ordinate_start_value)/ORDINZTEE_UNIT_TABL[graph->ordinate_unit_value];
		graph->ordinate_start_value=graph->ordinate_start_value-ORDINZTEE_UNIT_TABL[graph->ordinate_unit_value]*res;
		while(graph->ordinate_start_value < 0)
		{ 	graph->ordinate_start_value=graph->ordinate_start_value+ORDINZTEE_UNIT_TABL[graph->ordinate_unit_value];
		}			
	}

		temp=(float)ORDINZTEE_UNIT_TABL[graph->ordinate_unit_value]/(float)(graph->height/graph->ordinate_unit_num);//每个像素表示的值

		if(graph->scanf_time<1)
		{ 
			_gui_draw_point(graph->left-1,graph->top+graph->height-((float)(*(graph->data_0)-graph->ordinate_start_value)/temp),graph->data_0_color);//绘制data_0
			_gui_draw_point(graph->left-1,graph->top+graph->height-((float)(*(graph->data_1)-graph->ordinate_start_value)/temp),graph->data_1_color);//绘制data_1
		}
		else
		{	for(i=0;i<graph->scanf_time-1;i++)
			{
			gui_draw_line(graph->left+(graph->width/graph->abscissa_unit_num)*i-1,
										graph->top+graph->height-((float)(*(graph->data_1+i)-graph->ordinate_start_value)/temp),
										graph->left+(graph->width/graph->abscissa_unit_num)*(i+1)-1,
										graph->top+graph->height-((float)(*(graph->data_1+i+1)-graph->ordinate_start_value)/temp),
										graph->data_1_color);//绘制data_1
										
			gui_draw_line(graph->left+(u16)((graph->width/graph->abscissa_unit_num)*i)-1,
										graph->top+graph->height-(u16)((float)(*(graph->data_0+i)-graph->ordinate_start_value)/temp),
										graph->left+(u16)((graph->width/graph->abscissa_unit_num)*(i+1))-1,
										graph->top+graph->height-(u16)((float)(*(graph->data_0+i+1)-graph->ordinate_start_value)/temp),
										graph->data_0_color);//绘制data_0
			}
		}
		
		if(graph->scanf_time<=graph->abscissa_unit_num)
		{graph->scanf_time++;
		}

	//	printf("最小值：%f\r\n",graph->min_value);	
	//	printf("最大值：%f\r\n",graph->max_value);				
	//	printf("纵轴单位值：%d\r\n",graph->ordinate_unit_value);			
	//	printf("纵轴原点数值：%d\r\n\r\n",graph->ordinate_start_value);
		//调整纵轴零点值
		//重新标注纵轴



}
//删除曲线图
void graph_delete(_graph_obj * graph_del)
{ if(graph_del==NULL)return;//非法的地址,直接退出
	
	gui_memin_free(graph_del->data_0);
	gui_memin_free(graph_del->data_1);
	gui_memin_free(graph_del);
}
//图表数据更新
//array 数组首地址
//size 数组大小 不能小于2
//data 待更新的数据
//position 更新的位置
void graph_updata_array(float *array,u8 size,float data,u8  position)
{	u8 i;
	if(position<size+1)//
	{ *(array+position)=data;
	}
	else
	{	
		
		for(i=0;i<position-1;i++)
		{
			*(array+i)=*(array+i+1);
		}
		*(array+i)=data;	
		
	}
}







