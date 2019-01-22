#include "sys.h"		    
#include "rs232.h"	 
#include "delay.h"
#include "stdbool.h"
#include "fifo.h"
#include "Meteorological.h"

#if RS232_RX   		//���ʹ���˽���   	  
//���ջ����� 	
u8 RS232_RX_BUF[2][2000];  	//���ջ���,���2000���ֽ�.
//���յ������ݳ���
uint16_t RS232_RX_CNT=0;   
u8 Meteor_Status=0;
u8 BufferNumber0=0;
u8 BufferNumber1=1;
u8 BufferFinishNumber=0;
u8 BufferWorkNumber=0;
void USART2_IRQHandler(void)
{
	u8 res;	    
		if(USART_GetITStatus(USART2, USART_IT_RXNE) != RESET)//���յ�����
		{	 	
			USART_ClearITPendingBit(USART2, USART_IT_RXNE);
			res = USART_ReceiveData(USART2);//;��ȡ���յ�������USART2->DR

				if(BufferWorkNumber==BufferNumber1)
				{
					RS232_RX_BUF[BufferNumber1][RS232_RX_CNT] = res;
				}
				else 
				{
					RS232_RX_BUF[BufferNumber0][RS232_RX_CNT] = res;
				}
				
				RS232_RX_CNT++;

		} 
		if(USART_GetITStatus(USART2, USART_IT_IDLE) != RESET)
		{
			USART_ClearITPendingBit(USART2, USART_IT_IDLE);
			USART2->DR;
			Meteor_Status = 1;
			MeteorBufCounter = RS232_RX_CNT;
			RS232_RX_CNT = 0;
			/*��¼��ǰ������*/
			BufferFinishNumber = BufferWorkNumber;
			if(BufferWorkNumber==BufferNumber1)
				 BufferWorkNumber = BufferNumber0;
			else 
				 BufferWorkNumber = BufferNumber1;
		}		
}

#endif										 
//��ʼ��IO ����2
//bound:������	  
void RS232_Init(u32 bound)
{  	 
	
  GPIO_InitTypeDef GPIO_InitStructure;
	USART_InitTypeDef USART_InitStructure;
	NVIC_InitTypeDef NVIC_InitStructure;
	
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOD,ENABLE); //ʹ��GPIODʱ��
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART2,ENABLE);//ʹ��USART2ʱ��
	
  //����2���Ÿ���ӳ��
	GPIO_PinAFConfig(GPIOD,GPIO_PinSource5,GPIO_AF_USART2); //GPIOD5����ΪUSART2
	GPIO_PinAFConfig(GPIOD,GPIO_PinSource6,GPIO_AF_USART2); //GPIOD6����ΪUSART2
	
	//USART2    
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_5 | GPIO_Pin_6; //GPIOD5��GPIOD6
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;//���ù���
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;	//�ٶ�100MHz
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP; //���츴�����
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP; //����
	GPIO_Init(GPIOD,&GPIO_InitStructure); //��ʼ��PA2��PA3
	
   //USART2 ��ʼ������
	USART_InitStructure.USART_BaudRate = bound;//����������
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;//�ֳ�Ϊ8λ���ݸ�ʽ
	USART_InitStructure.USART_StopBits = USART_StopBits_1;//һ��ֹͣλ
	USART_InitStructure.USART_Parity = USART_Parity_No;//����żУ��λ
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;//��Ӳ������������
	USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;	//�շ�ģʽ
  USART_Init(USART2, &USART_InitStructure); //��ʼ������2
	
  USART_Cmd(USART2, ENABLE);  //ʹ�ܴ��� 2
	
	USART_ClearFlag(USART2, USART_FLAG_TC);
	
#if RS232_RX	
	USART_ITConfig(USART2, USART_IT_RXNE, ENABLE);//���������ж�
	USART_ITConfig(USART2,USART_IT_IDLE,ENABLE);	//���������ж�
	
	//Usart2 NVIC ����
  NVIC_InitStructure.NVIC_IRQChannel = USART2_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority=3;//��ռ���ȼ�3
	NVIC_InitStructure.NVIC_IRQChannelSubPriority =3;		//�����ȼ�3
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;			//IRQͨ��ʹ��
	NVIC_Init(&NVIC_InitStructure);	//����ָ���Ĳ�����ʼ��VIC�Ĵ�����

#endif	
	
	RS232_TX_EN=0;				//Ĭ��Ϊ����ģʽ	
}

////RS232��ѯ���յ�������
////buf:���ջ����׵�ַ
////len:���������ݳ���
//void RS232_Receive_Data(u8 *buf,uint16_t *len)
//{
//	u8 rxlen=RS232_RX_CNT;
//	u8 i=0;
//	*len=0;				//Ĭ��Ϊ0
//	//delay_ms(10);		//�ȴ�10ms,��������10msû�н��յ�һ������,����Ϊ���ս���
//	if(rxlen==RS232_RX_CNT&&rxlen)//���յ�������,�ҽ��������
//	{
//		for(i=0;i<rxlen;i++)
//		{
//			buf[i]=RS232_RX_BUF[i];	
//		}		
//		*len=RS232_RX_CNT;	//��¼�������ݳ���
//		RS232_RX_CNT=0;		//����
//	}
//}

////RS232��ѯ���յ������ݳ���

//void RS232_Receive_BufferLen(uint16_t *len)
//{
//	u8 rxlen=0;
//	*len=0;				//Ĭ��Ϊ0
//	//delay_ms(10);		//�ȴ�10ms,��������10msû�н��յ�һ������,����Ϊ���ս���
//	rxlen=RS232_RX_CNT;
//	if(rxlen==RS232_RX_CNT&&rxlen)//���յ�������,�ҽ��������
//	{
//		*len=RS232_RX_CNT;	//��¼�������ݳ���
//		RS232_RX_CNT=0;		//����
//	}
//}


