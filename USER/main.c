#include "sys.h"
#include "delay.h"
#include "usart.h"
#include "led.h"
#include "lcd.h"
#include "key.h"  
#include "sram.h"   
#include "malloc.h" 
#include "usmart.h"  
#include "sdio_sdcard.h"    
#include "malloc.h" 
#include "w25qxx.h"    
#include "ff.h"  
#include "exfuns.h"    
#include "rs485.h" 
#include <Meteorological.h>
#include "usart3.h"
#include "gps.h"
#include "fifo.h"
#include "rs232.h"

 

FIL   	fileTXT;
FIL* 		fp;
BYTE 			buffer[]="hello world!";//	д������
BYTE 			buffer2[]="";//	д������
BYTE			TXTdata2UartBuffer[2000];

void PutTXTdata2Uart(void);
void PutTXTdata2TFCardTest(void);

void PutData2TXT(u8 *databuffer,uint16_t length);



uint8_t ReadyFlag=0;
uint32_t TimeFlag=0;
uint32_t TimeCounter=1000;
uint32_t SetVotalgeValue=1;
uint16_t BatteryAllowance=0;
u8 ReturnFlag=0;

u8 USART1_TX_BUF[USART3_MAX_RECV_LEN]; 					//����1,���ͻ�����
nmea_msg gpsx; 											//GPS��Ϣ
__align(4) u8 dtbuf[50];   								//��ӡ������
const u8*fixmode_tbl[4]={"Fail","Fail"," 2D "," 3D "};	//fix mode�ַ��� 

extern u16 USART6_RX_STA;

/*
1-����
2-��ʵ����
3-����
4-��ʵ����
5-�¶�
6-ʪ��
7-����ѹ
8-¶���¶�
*/
u8 mem_perused=0;
u8 Usart1rxlen;
u8 Usart6rxlen;
u8 ComaBackCommand[]={0x5A,0x04,0xFF,0xFF,0x0D,0x0A};
int main(void)
{
	u16 i=0;
	u32 lenx;
	u32 TIME_Check=0;
 	u32 total,free;
	u8 keypress=0;
	u8 res=0;	
	u32 sumCounter=0;
	u8 upload=0; 
	fp = &fileTXT;
	u8 GPS_Save_File=0;
	FRESULT res_sd;
	
	
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);//����ϵͳ�ж����ȼ�����2
	delay_init(168);  //��ʼ����ʱ����
	API_Uart6_init(115200);
	uart_init(115200);		//��ʼ�����ڲ�����Ϊ115200
	LED_Init();					//��ʼ��LED 
	//usmart_dev.init(84);		//��ʼ��USMART
 	//KEY_Init();					//������ʼ�� 
	RS232_Init(4800);				//��ʼ��232
	RS485_Init(115200);				//��ʼ��485
	TIM7_Int_Init(1000,84-1);
	my_mem_init(SRAMIN);		//��ʼ���ڲ��ڴ�� 
	my_mem_init(SRAMCCM);		//��ʼ��CCM�ڴ��
	
 	while(SD_Init())//��ⲻ��SD��
	{
		delay_ms(50);					
		LED0=!LED0;//DS0��˸
	}
 	exfuns_init();							//Ϊfatfs��ر��������ڴ�				 
  f_mount(fs[0],"0:",1); 					//����SD�� 
 	res=f_mount(fs[1],"1:",1); 				//����FLASH.	
										    
	while(exf_getfree("0",&total,&free))	//�õ�SD������������ʣ������
	{
		delay_ms(200);
		LED0=!LED0;//DS0��˸
	}													  			    

	res_sd = f_open(fp, "0:/ReceiveData.txt", FA_READ|FA_WRITE|FA_OPEN_ALWAYS);
	if (res_sd==FR_OK) {
//		printf("Open file successfully\r\n");
//		printf("FATFS OK!\r\n");
//		printf("SD Total Size:%d 	MB\r\n",total>>10);
//		printf("SD  Free Size:%d	MB\r\n",free>>10);
	}
	else
	{
		printf("Open file failed\r\n");
	}

	res_sd = f_lseek(fp,fp->fsize);  
	
	LED0 = 0;
	fifo_alloc(&NuclearFIFOBuffer,8*1024);
	mem_perused = my_mem_perused(SRAMIN);
	
	delay_ms(1000);
	
	ReadyFlag = 0; //modify by yanly190220, this init value must be 0
	
	while(1)
	{
		/*****************�˷����豸ͨѶ---����*******************/
		if(ReadyFlag==0)   
		{
			if(CheckCommunication()==true)
			{
				printf("Communication is OK\r\n");
				printf("Ready to read...");
				StartMea();
				delay_ms(10);
				ReadyFlag = 1;
				RS485_RX_CNT = 0;
			}
			delay_ms(500);
			LED0 = 1;
			delay_ms(500);
		}
		/*****************�˷����豸����---��ȡ*******************/
		else if(ReadyFlag==1)
		{
			/*************************��λ��*************************/
			if(USART_RX_STA&0x8000)
			{
				Usart1rxlen=USART_RX_STA&0x3fff;	//�õ��˴ν��յ������ݳ���
				if(Usart1rxlen==4)
				{
					if(USART_RX_BUF[1]==0x01)  //���õ�ѹ
					{
						SetVotalgeValue = (USART_RX_BUF[3]<<8) + USART_RX_BUF[2];
						if(SetVotalgeValue>=999) SetVotalgeValue=999;
						if(SetVotalgeValue<=0)  SetVotalgeValue =0;
						SetVoltage(SetVotalgeValue);
					}
					if(USART_RX_BUF[1]==0x02)  //����ʱ��
					{
						TimeCounter = (USART_RX_BUF[3]<<8) + USART_RX_BUF[2];
						if(TimeCounter>9999) TimeCounter = 9999;
						if(TimeCounter<=0) TimeCounter = 0;
					}
					if(USART_RX_BUF[1]==0x04)  //����ָ��
					{
							APIUsart6_Send_Data(ComaBackCommand,6);
					}
				}
				USART_RX_STA=0;//״̬�Ĵ������
			}
			/**************API**************/
			if(USART6_RX_STA&0x8000)
			{
				Usart6rxlen=USART6_RX_STA&0x3fff;	//�õ��˴ν��յ������ݳ���
				if(Usart6rxlen==6)
				{
					if(USART6_RX_BUF[1]==0x03)  //��ȡ������
					{
							BatteryAllowance = (USART6_RX_BUF[5]<<24) + (USART6_RX_BUF[4]<<16)+(USART6_RX_BUF[3]<<8)+USART6_RX_BUF[2];
					}
				}
				if(Usart6rxlen==4)
				{
					if(USART6_RX_BUF[1]==0x14)  //�ظ������ɹ�
					{
							ReturnFlag = 1;
					}
				}
				USART6_RX_STA=0;//״̬�Ĵ������
			}
			/****************�˷�������****************/
			if(TimeFlag)
			{				
				TimeFlag =0;
				LED0=1;
				/*�ȴ�����ģ���ȡ�����ݺ���һ��ת����ȥ*/
				//PutData2TXT(RS232_RX_BUF[BufferFinishNumber],MeteorBufCounter);
				for(i=0;i<MeteorBufCounter;i++)
				{
						printf("%c",RS232_RX_BUF[BufferFinishNumber][i]);
				}
				//printf("\r\n");
				/*��ȡ�˷���ģ������*/
				ReadMeteorVal();
				/*ת���˷�������*/
				delay_ms(100);
				//NuclearGetData(ReceiveBuf,&NuclearBufCounter);
				//PutData2TXT(ReceiveBuf,NuclearBufCounter);
				if(NuclearBufCounter>0)
				{
					//PutData2TXT(RS485_RX_BUF[Rs485BufferFinishNumber],NuclearBufCounter);
					printf("$HFSSJ,");
					for(i=0;i<NuclearBufCounter;i++)
					{
						printf("%c",RS485_RX_BUF[Rs485BufferFinishNumber][i]);
					}
					NuclearBufCounter = 0;
					printf("\r\n");
				}
				/*���ͷɻ�״̬*/
				printf("$WRJSJ,Battery:%d,ReturnFlag:%d\r\n",BatteryAllowance,ReturnFlag);
				printf("\r\n");
			}
			else
			{
				LED0=0;
			}			
		}
	} 
}

/*******************************************
����ȡ�������ݱ��浽SD����
*******************************************/
uint32_t ByteCounter=0;
void PutData2TXT(u8 *databuffer,uint16_t length)
{
	FRESULT res_sd;

	res_sd = f_write(fp, databuffer, length, &bw);
	if (res_sd == FR_OK)
	{
		f_sync(fp);
		if (res_sd==FR_OK) 
		{
				ByteCounter = ByteCounter+bw;
				//printf("%8d\r\n",ByteCounter);
		}
	}	
}

/*******************************************
��SD���TXT��ȡ������ͨ�����ڴ�ӡ��ȥ�����ֻ�ܴ�ӡ1000���ֽ�
*******************************************/
void PutTXTdata2Uart(void)
{
	FRESULT res_sd;
	UINT fnum;
	f_close(fp);
	printf("\r\n******************Read TXT data*****************\r\n");
	res_sd = f_open(fp, "0:/ReceiveData.txt", FA_READ|FA_OPEN_EXISTING);
	if (res_sd == FR_OK) {
		printf("Open file successfully,ready to read...\r\n");
	}
	res_sd = f_read(fp, TXTdata2UartBuffer, sizeof(TXTdata2UartBuffer), &fnum);
	if (res_sd==FR_OK) {
		printf("Reveive Data Counter:%d\r\n",fnum);
		printf("Data:\r\n %s \r\n", TXTdata2UartBuffer);
	}
	else
	{
		printf("Read file failed\r\n");
	}
	f_close(fp);
	if (res_sd==FR_OK) {
		printf("close file successfully\r\n");
	}
	else
	{
		printf("close file failed\r\n");
	}
}

/*******************************************
����SD������SD������д��helloworld ����
*******************************************/
void PutTXTdata2TFCardTest(void)
{
	FRESULT res_sd;
	u32 total,free;
	int fputsCounter;
	
	exf_getfree("0",&total,&free);
	
	delay_ms(200);
	res_sd = f_open(fp, "0:/ReceiveData.txt", FA_READ|FA_WRITE|FA_OPEN_ALWAYS);
	if (res_sd==FR_OK) {
		printf("Open file successfully\r\n");
		printf("FATFS OK!\r\n");
		printf("SD Total Size:%d 	MB\r\n",total>>10);
		printf("SD  Free Size:%d	MB\r\n",free>>10);
	}
	else
	{
		printf("Open file failed\r\n");
	}
	delay_ms(200);
	fputsCounter = f_puts((char *)buffer2,fp); 
	
	printf("write data %d \r\n",fputsCounter);

	res_sd = f_close(fp);
	if (res_sd==FR_OK) {
		printf("close file successfully\r\n");
	}
	else
	{
		printf("close file failed\r\n");
	}
}



