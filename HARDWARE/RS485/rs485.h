#ifndef __RS485_H
#define __RS485_H			 
#include "sys.h"	 								  

extern u8 RS485_RX_BUF[2][2000]; 		//���ջ���,���1000���ֽ�
extern uint16_t RS485_RX_CNT;   			//���յ������ݳ���
extern u8 Rs485BufferFinishNumber;
extern u8 Rs485Meteor_Status;
//ģʽ����
#define RS485_TX_EN		PAout(2)	//485ģʽ����.0,����;1,����.
//����봮���жϽ��գ�����EN_USART2_RXΪ1����������Ϊ0
#define RS485_USART_RX 	1			//0,������;1,����.

														 
void RS485_Init(u32 bound);
void RS485_Send_Data(u8 *buf,u8 len);
void RS485_Receive_Data(u8 *buf,uint16_t *len);		

void RS485_Receive_BufferLen(uint16_t *len);
#endif	   
















