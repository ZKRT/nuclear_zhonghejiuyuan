#include "sys.h"		    
#include "rs485.h"	 
#include "delay.h"
#include "stdbool.h"
#include "Meteorological.h"

#define RS485_USART  UART4
extern uint8_t ReadyFlag;
#if RS485_USART_RX   		//���ʹ���˽���   	  
//���ջ����� 	
u8 RS485_RX_BUF[2][2000];  	//���ջ���,���2000���ֽ�.
//���յ������ݳ���
uint16_t RS485_RX_CNT=0;   

u8 Rs485Meteor_Status=0;
u8 Rs485BufferNumber0=0;
u8 Rs485BufferNumber1=1;
u8 Rs485BufferFinishNumber=0;
u8 Rs485BufferWorkNumber=0;
void UART4_IRQHandler(void)
{
	u8 res;	    
	if(USART_GetITStatus(RS485_USART, USART_IT_RXNE) != RESET)//���յ�����
	{	 
		USART_ClearITPendingBit(RS485_USART, USART_IT_RXNE);
	  res =USART_ReceiveData(RS485_USART);//;��ȡ���յ�������USART2->DR
		if(ReadyFlag)
			fifo_in(&NuclearFIFOBuffer,&res,1);

		if(Rs485BufferWorkNumber==Rs485BufferNumber1)
		{
			RS485_RX_BUF[Rs485BufferNumber1][RS485_RX_CNT] = res;
		}
		else 
		{
			RS485_RX_BUF[Rs485BufferNumber0][RS485_RX_CNT] = res;
		}
		RS485_RX_CNT++;
	}
	
	if(USART_GetITStatus(RS485_USART, USART_IT_IDLE) != RESET)
	{
		USART_ClearITPendingBit(RS485_USART, USART_IT_IDLE);
		RS485_USART->DR;
		Rs485Meteor_Status = 1;
		NuclearBufCounter = RS485_RX_CNT;
		RS485_RX_CNT = 0;
		/*��¼��ǰ������*/
		Rs485BufferFinishNumber = Rs485BufferWorkNumber;
		if(Rs485BufferWorkNumber==Rs485BufferNumber1)
			 Rs485BufferWorkNumber = Rs485BufferNumber0;
		else 
			 Rs485BufferWorkNumber = Rs485BufferNumber1;
	}	
} 
#endif										 
//��ʼ��IO ����2
//bound:������	  
void RS485_Init(u32 bound)
{  	 
	
  GPIO_InitTypeDef GPIO_InitStructure;
	USART_InitTypeDef USART_InitStructure;
	NVIC_InitTypeDef NVIC_InitStructure;
	
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA,ENABLE); //ʹ��GPIOAʱ��
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_UART4,ENABLE);//ʹ��USART2ʱ��
	
  //����2���Ÿ���ӳ��
	GPIO_PinAFConfig(GPIOA,GPIO_PinSource0,GPIO_AF_UART4); //GPIOA0����ΪUART4
	GPIO_PinAFConfig(GPIOA,GPIO_PinSource1,GPIO_AF_UART4); //GPIOA1����ΪUART4
	
	//USART2    
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0 | GPIO_Pin_1; //GPIOA0��GPIOA1
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;//���ù���
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;	//�ٶ�100MHz
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP; //���츴�����
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP; //����
	GPIO_Init(GPIOA,&GPIO_InitStructure); //��ʼ��PA0��PA1
	
	//PG8���������485ģʽ����  
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2; //GPIOA2
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;//���
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;	//�ٶ�100MHz
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP; //�������
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP; //����
	GPIO_Init(GPIOA,&GPIO_InitStructure); //��ʼ��PA2
	

   //USART2 ��ʼ������
	USART_InitStructure.USART_BaudRate = bound;//����������
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;//�ֳ�Ϊ8λ���ݸ�ʽ
	USART_InitStructure.USART_StopBits = USART_StopBits_1;//һ��ֹͣλ
	USART_InitStructure.USART_Parity = USART_Parity_No;//����żУ��λ
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;//��Ӳ������������
	USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;	//�շ�ģʽ
  USART_Init(RS485_USART, &USART_InitStructure); //��ʼ������2
	
  USART_Cmd(RS485_USART, ENABLE);  //ʹ�ܴ��� 2
	
	USART_ClearFlag(RS485_USART, USART_FLAG_TC);
	
#if RS485_USART_RX	
	USART_ITConfig(RS485_USART, USART_IT_RXNE, ENABLE);//���������ж�
	USART_ITConfig(RS485_USART, USART_IT_IDLE,ENABLE);	//���������ж�
	//Usart2 NVIC ����
  NVIC_InitStructure.NVIC_IRQChannel = UART4_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority=10;//��ռ���ȼ�10
	NVIC_InitStructure.NVIC_IRQChannelSubPriority =10;		//�����ȼ�10
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;			//IRQͨ��ʹ��
	NVIC_Init(&NVIC_InitStructure);	//����ָ���Ĳ�����ʼ��VIC�Ĵ�����

#endif	
	
	RS485_TX_EN=0;				//Ĭ��Ϊ����ģʽ	
}

//RS485����len���ֽ�.
//buf:�������׵�ַ
//len:���͵��ֽ���(Ϊ�˺ͱ�����Ľ���ƥ��,���ｨ�鲻Ҫ����64���ֽ�)
void RS485_Send_Data(u8 *buf,u8 len)
{
	u8 t;
	RS485_TX_EN=1;			//����Ϊ����ģʽ
	delay_ms(2);
  for(t=0;t<len;t++)		//ѭ����������
	{
	  while(USART_GetFlagStatus(RS485_USART,USART_FLAG_TC)==RESET)
		{}			//�ȴ����ͽ���		
    USART_SendData(RS485_USART,buf[t]); //��������
	}
	while(USART_GetFlagStatus(RS485_USART,USART_FLAG_TC)==RESET); //�ȴ����ͽ���		
	//RS485_RX_CNT=0;	
	RS485_TX_EN=0;				//����Ϊ����ģʽ	
}
////RS485��ѯ���յ�������
////buf:���ջ����׵�ַ
////len:���������ݳ���
//void RS485_Receive_Data(u8 *buf,uint16_t *len)
//{
//	u8 rxlen=0;
//	u8 i=0;
//	*len=0;				//Ĭ��Ϊ0
//	delay_ms(10);		//�ȴ�10ms,��������10msû�н��յ�һ������,����Ϊ���ս���
//	rxlen = RS485_RX_CNT;
//	if(rxlen==RS485_RX_CNT&&rxlen)//���յ�������,�ҽ��������
//	{
//		for(i=0;i<rxlen;i++)
//		{
//			buf[i]=RS485_RX_BUF[i];	
//		}		
//		*len=RS485_RX_CNT;	//��¼�������ݳ���
//		RS485_RX_CNT=0;		//����
//	}
//}

////RS485��ѯ���յ������ݳ���
//void RS485_Receive_BufferLen(uint16_t *len)
//{
//	u8 rxlen=0;
//	*len=0;				//Ĭ��Ϊ0
//	delay_ms(26);		//�ȴ�10ms,��������10msû�н��յ�һ������,����Ϊ���ս���
//	rxlen=RS485_RX_CNT;
//	if(rxlen==RS485_RX_CNT&&rxlen)//���յ�������,�ҽ��������
//	{
//		*len=RS485_RX_CNT;	//��¼�������ݳ���
//		RS485_RX_CNT=0;		//����
//	}
//}

