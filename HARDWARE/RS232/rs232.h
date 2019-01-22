#ifndef __RS232_H
#define __RS232_H			 
#include "sys.h"	 								  

extern u8 RS232_RX_BUF[2][2000]; 		//���ջ���,���1000���ֽ�
extern uint16_t RS232_RX_CNT;   			//���յ������ݳ���
extern u8 Meteor_Status;
extern u8 BufferFinishNumber;
//ģʽ����
#define RS232_TX_EN		PGout(8)	//485ģʽ����.0,����;1,����.
//����봮���жϽ��գ�����EN_USART2_RXΪ1����������Ϊ0
#define RS232_RX 	1			//0,������;1,����.

														 
void RS232_Init(u32 bound);
void RS232_Send_Data(u8 *buf,u8 len);
void RS232_Receive_Data(u8 *buf,uint16_t *len);	
void RS232_Receive_BufferLen(uint16_t *len);
#endif	   
















