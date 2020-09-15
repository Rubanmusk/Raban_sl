#include "graph.h"
//////////////////////////////////////////////////////////////////////////////////	 
//������ֻ��ѧϰʹ�ã�δ��������ɣ��������������κ���;
//ALIENTEKս��STM32������
//GUI-������ ����	   
//����ԭ��@ALIENTEK
//������̳:www.openedv.com
//�޸�����:2012/10/4
//�汾��V1.0
//��Ȩ���У�����ؾ���
//Copyright(C) ������������ӿƼ����޹�˾ 2009-2019
//All rights reserved									  
//////////////////////////////////////////////////////////////////////////////////
const u32 ORDINZTEE_UNIT_TABL[11]=
{	1,2,5,10,20,25,50,100,150,200,250
};

//	u16 top; 				  		//graph��������
//	u16 left;             //graph�������
//	u16 width; 				  	//���
//	u16 height;						//�߶�
//	u8 abscissa_unit_num; //�����굥λ������ 		
//	u8 ordinate_unit_num; //�����굥λ����   

//��������ͼ
_graph_obj * graph_creat(u16 left,u16 top,u16 width,u16 height,u8 abscissa_unit_num,u8 ordinate_unit_num)
{	
	_graph_obj *_temp_graph;
	
	_temp_graph=(_graph_obj*)gui_memin_malloc(sizeof(_graph_obj));//�����ڴ�
	if(_temp_graph==NULL)return NULL;//�ڴ���䲻��.
	
	_temp_graph->top=top; 				 	//graph��������
	_temp_graph->left=left;         //graph�������
	_temp_graph->width=width; 				  	//���
	_temp_graph->height=height;						//�߶�
	
	_temp_graph->scanf_time=0; 				//��ˢ�´���
	
	_temp_graph->max_value=0;			//
	_temp_graph->min_value=0;			//
	
	_temp_graph->ordinate_unit_num=ordinate_unit_num; //�����굥λ����
	_temp_graph->abscissa_unit_num=abscissa_unit_num; //�����굥λ����

	_temp_graph->ordinate_unit_value=0;
	_temp_graph->ordinate_start_value=0;
	
	_temp_graph->data_0=(float*)gui_memin_malloc(sizeof(float)*_temp_graph->abscissa_unit_num+1);//Ϊ��������������ڴ�
	_temp_graph->data_1=(float*)gui_memin_malloc(sizeof(float)*_temp_graph->abscissa_unit_num+1);

	_temp_graph->data_0_color=RED;
	_temp_graph->data_1_color=LIGHTGREEN;
	
	return _temp_graph;
}
//������ͼ
void graph_draw(_graph_obj * graph)	
{ u8 i;
	int res;
	float temp;
	
	gui_fill_rectangle(graph->left,graph->top,graph->width,graph->height,WHITE );//��䱳��ɫ
	
	gui_draw_vline(graph->left-1,graph->top,graph->height+1,BLACK);//��ֱ��
	gui_draw_hline(graph->left-1,graph->top+graph->height,graph->width+1,BLACK);//ˮƽ��
	
	for(i=1;i<graph->abscissa_unit_num;i++)
	{	gui_draw_vline(graph->left+i*(graph->width/graph->abscissa_unit_num)-1,graph->top,graph->height,LIGHTGRAY);//��ֱ��
	}
	for(i=1;i<graph->ordinate_unit_num;i++)
	{	gui_draw_hline(graph->left,graph->top+i*(graph->height/graph->ordinate_unit_num)-1,graph->width,LIGHTGRAY);//ˮƽ��
	}


	
	graph_updata_array(graph->data_0,graph->abscissa_unit_num,graph->data_0_temp,graph->scanf_time);		  //���µ�ǰ�¶�
	graph_updata_array(graph->data_1,graph->abscissa_unit_num,graph->data_1_temp,graph->scanf_time);			//����Ŀ���¶�
	
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
	
	//������ʾ��Χ
	if(	(graph->ordinate_start_value>graph->min_value)||
			((graph->ordinate_start_value+ORDINZTEE_UNIT_TABL[graph->ordinate_unit_value]*graph->ordinate_unit_num)<graph->max_value)
		)
	{		
		if((((ORDINZTEE_UNIT_TABL[graph->ordinate_unit_value]*graph->ordinate_unit_num)-(graph->max_value-graph->min_value))>
				(ORDINZTEE_UNIT_TABL[graph->ordinate_unit_value]*1)))//��ֵ����һ����λ ˵��ֻ��һ�߳�����Χ ֻ��Ҫ��������ԭ��
			{ if(graph->ordinate_start_value>graph->min_value) //��Сֵ��������ԭ��
				{ while(graph->ordinate_start_value>graph->min_value)		
						{graph->ordinate_start_value=graph->ordinate_start_value-ORDINZTEE_UNIT_TABL[graph->ordinate_unit_value];
						}
				}
				else	//���ֵ�����������ֵ
				{ while((graph->ordinate_start_value+ORDINZTEE_UNIT_TABL[graph->ordinate_unit_value]*graph->ordinate_unit_num)<graph->max_value)
						{	graph->ordinate_start_value=graph->ordinate_start_value+ORDINZTEE_UNIT_TABL[graph->ordinate_unit_value];
						}
				}	
			}
		else	//��ֵС��һ����λ  ��Ҫ������λֵ 
			{							
				while(	
							 (((ORDINZTEE_UNIT_TABL[graph->ordinate_unit_value]*graph->ordinate_unit_num)-(graph->max_value-graph->min_value))<
							 (ORDINZTEE_UNIT_TABL[graph->ordinate_unit_value]*1))
							)
					{ ++graph->ordinate_unit_value;//����λֵ
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
		if(graph->ordinate_unit_value>0)//���ֵ����Сֵ֮�����һ��ֵ  ��λ����һ�� 
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

		temp=(float)ORDINZTEE_UNIT_TABL[graph->ordinate_unit_value]/(float)(graph->height/graph->ordinate_unit_num);//ÿ�����ر�ʾ��ֵ

		if(graph->scanf_time<1)
		{ 
			_gui_draw_point(graph->left-1,graph->top+graph->height-((float)(*(graph->data_0)-graph->ordinate_start_value)/temp),graph->data_0_color);//����data_0
			_gui_draw_point(graph->left-1,graph->top+graph->height-((float)(*(graph->data_1)-graph->ordinate_start_value)/temp),graph->data_1_color);//����data_1
		}
		else
		{	for(i=0;i<graph->scanf_time-1;i++)
			{
			gui_draw_line(graph->left+(graph->width/graph->abscissa_unit_num)*i-1,
										graph->top+graph->height-((float)(*(graph->data_1+i)-graph->ordinate_start_value)/temp),
										graph->left+(graph->width/graph->abscissa_unit_num)*(i+1)-1,
										graph->top+graph->height-((float)(*(graph->data_1+i+1)-graph->ordinate_start_value)/temp),
										graph->data_1_color);//����data_1
										
			gui_draw_line(graph->left+(u16)((graph->width/graph->abscissa_unit_num)*i)-1,
										graph->top+graph->height-(u16)((float)(*(graph->data_0+i)-graph->ordinate_start_value)/temp),
										graph->left+(u16)((graph->width/graph->abscissa_unit_num)*(i+1))-1,
										graph->top+graph->height-(u16)((float)(*(graph->data_0+i+1)-graph->ordinate_start_value)/temp),
										graph->data_0_color);//����data_0
			}
		}
		
		if(graph->scanf_time<=graph->abscissa_unit_num)
		{graph->scanf_time++;
		}

	//	printf("��Сֵ��%f\r\n",graph->min_value);	
	//	printf("���ֵ��%f\r\n",graph->max_value);				
	//	printf("���ᵥλֵ��%d\r\n",graph->ordinate_unit_value);			
	//	printf("����ԭ����ֵ��%d\r\n\r\n",graph->ordinate_start_value);
		//�����������ֵ
		//���±�ע����



}
//ɾ������ͼ
void graph_delete(_graph_obj * graph_del)
{ if(graph_del==NULL)return;//�Ƿ��ĵ�ַ,ֱ���˳�
	
	gui_memin_free(graph_del->data_0);
	gui_memin_free(graph_del->data_1);
	gui_memin_free(graph_del);
}
//ͼ�����ݸ���
//array �����׵�ַ
//size �����С ����С��2
//data �����µ�����
//position ���µ�λ��
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







