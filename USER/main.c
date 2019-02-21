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
#include "osqtmr.h"
#include "ostmr.h"
#include "osusart.h"
#include "datalink_osdk_handle.h"

//#define DEBUG_TESTDATA_OPEN //Debug open //zkrt_debug
#ifdef DEBUG_TESTDATA_OPEN
#define SOFTWARE_VERSION  "DEBUG-V02"
#else
#define SOFTWARE_VERSION  "RELEASE-V02"
#endif

FIL fileTXT;
FIL *fp;
BYTE buffer[] = "hello world!"; //	д������
BYTE buffer2[] = "";			//	д������
BYTE TXTdata2UartBuffer[2000];

void PutTXTdata2Uart(void);
void PutTXTdata2TFCardTest(void);

void PutData2TXT(u8 *databuffer, uint16_t length);

uint8_t ReadyFlag = 0;
uint32_t TimeFlag = 0;
uint32_t TimeCounter = 1000;
uint32_t SetVotalgeValue = 1;
uint32_t BatteryAllowance = 0;
u8 ReturnFlag = 0;

u8 USART1_TX_BUF[USART3_MAX_RECV_LEN];						 //����1,���ͻ�����
nmea_msg gpsx;												 //GPS��Ϣ
__align(4) u8 dtbuf[50];									 //��ӡ������
const u8 *fixmode_tbl[4] = {"Fail", "Fail", " 2D ", " 3D "}; //fix mode�ַ���

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
u8 mem_perused = 0;
u8 Usart1rxlen;
u8 Usart6rxlen;
u8 ComaBackCommand[] = {0x5A, 0x04, 0xFF, 0xFF, 0x0D, 0x0A};
int main(void)
{
	u16 i = 0;
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);
	delay_init(168);
	b_systmr_init();
	b_ostmr_init();
	os_usart_init();
	usart_config();
	LED_Init();
	RS232_Init(4800);
	RS485_Init(115200);
	TIM7_Int_Init(1000, 84 - 1);
	my_mem_init(SRAMIN);
	my_mem_init(SRAMCCM);

	LED0 = 0;
	fifo_alloc(&NuclearFIFOBuffer, 8 * 1024);
	mem_perused = my_mem_perused(SRAMIN);

	delay_ms(1000);
#ifdef DEBUG_TESTDATA_OPEN
	ReadyFlag = 1; 
#else
	ReadyFlag = 0; //release������Ϊ0
#endif

	while (1)
	{
		/*****************�˷����豸ͨѶ---����*******************/
		if (ReadyFlag == 0)
		{
			if (CheckCommunication() == true)
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
		else if (ReadyFlag == 1)
		{
			usart6_handle(); //for onboard sdk mainboard
			usart1_handle(); //for datalink transport
			/****************�˷�������****************/
			if (TimeFlag)
			{
				TimeFlag = 0;
				LED0 = 1;
#ifdef DEBUG_TESTDATA_OPEN
				{
					char meteor[] =    "$GPGGA,062052.40,2309.5015,N,11329.6260,E,2,9,1.0,29.7,M,-4.4,M,,*72\r\n\
$GPVTG,320.2,T,322.8,M,0.0,N,0.0,K,D*2E\r\n\
$WIMDA,29.8991,I,1.0125,B,27.9,C,,,48.8,,16.1,C,29.4,T,32.0,M,1.6,N,0.8,M*63\r\n\
$TIROT,-20.9,A*2D\r\n\
$YXXDR,C,,C,WCHR,C,,C,WCHT,C,28.2,C,HINX,P,1.0090,B,STNP*55\r\n\
$WIMWV,56.2,R,1.6,N,A*15\r\n\
$GPZDA,062053.00,02,04,2018,00,00*69\r\n\
$YXXDR,A,0.0,D,PTCH,A,1.3,D,ROLL*5F\r\n";
					uint8_t nuclear[] = {31,32,33,34,35,36,37,38,39,40,31,32,33,34,35,36,37,38,39,40,
					31,32,33,34,35,36,37,38,39,40,31,32,33,34,35,36,37,38,39,40,
					31,32,33,34,35,36,37,38,39,40,31,32,33,34,35,36,37,38,39,40,
					31,32,33,34,35,36,37,38,39,40,31,32,33,34,35,36,37,38,39,40,
					31,32,33,34,35,36,37,38,39,40,31,32,33,34,35,36,37,38,39,40};
					printf("%s", meteor);
					delay_ms(15);
					delay_ms(100);
					printf("$HFSSJ,");
					for (i = 0; i < sizeof(nuclear); i++)
					{
						printf("%c", nuclear[i]);
					}
					printf("\r\n");
					printf("$WRJSJ,Battery:%f,ReturnFlag:%d\r\n", (double)BatteryAllowance/1000, ReturnFlag);
					printf("\r\n");
				}
#else  				
				/*�ȴ�����ģ���ȡ�����ݺ���һ��ת����ȥ*/  
				for (i = 0; i < MeteorBufCounter; i++)
				{
					printf("%c", RS232_RX_BUF[BufferFinishNumber][i]);
				}
				//printf("\r\n");
				/*��ȡ�˷���ģ������*/
				ReadMeteorVal();
				/*ת���˷�������*/
				delay_ms(100);
				if (NuclearBufCounter > 0)
				{

					printf("$HFSSJ,");
					for (i = 0; i < NuclearBufCounter; i++)
					{
						printf("%c", RS485_RX_BUF[Rs485BufferFinishNumber][i]);
					}
					NuclearBufCounter = 0;
					printf("\r\n");
				}
				/*���ͷɻ�״̬*/
				printf("$WRJSJ,Battery:%f,ReturnFlag:%d\r\n", (double)BatteryAllowance/1000, ReturnFlag);
				printf("\r\n");
#endif				
			}
			else
			{
				LED0 = 0;
			}
		}
	}
}
