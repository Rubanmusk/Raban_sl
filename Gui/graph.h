#ifndef __GRAPH_H
#define __GRAPH_H	 
#include "guix.h" 	
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
extern const u32 ORDINZTEE_UNIT_TABL[];
//����ͼ�ṹ�嶨��
__packed typedef struct 
{
	u16 top; 				  		//��ť��������
	u16 left;             //��ť�������
	u16 width; 				  	//���
	u16 height;						//�߶�
	
	u8 scanf_time; 				//��ˢ�´���
	
	u8 ordinate_unit_num; //�����굥λ����   
	u8 abscissa_unit_num; //�����굥λ������ 
	
	u32 ordinate_unit_value; //�����굥λֵ 
//	float abscissa_unit_value; //�����굥λֵ 
	
  int ordinate_start_value; //����������ԭ��ֵ
//	float abscissa_start_value; //����������ԭ��ֵ
	
	float max_value;			//
	float min_value;			//

	float data_0_temp; 	//����data_0 ����
	float data_1_temp;	//����data_1 ����
	
	float *data_0; 	
	float *data_1;
	
	u16 data_0_color;
	u16 data_1_color;
	
}_graph_obj;


_graph_obj * graph_creat(u16 left,u16 top,u16 width,u16 height,u8 abscissa_unit_num,u8 ordinate_unit_num);
void graph_delete(_graph_obj * graph_del);
void graph_draw(_graph_obj * graph);					//������ͼ
void graph_updata_array(float *array,u8 size,float data,u8  position);

#endif





