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

#define SOFTWARE_VERSION  "RELEASE-V02"

FIL fileTXT;
FIL *fp;
BYTE buffer[] = "hello world!"; //	写入数据
BYTE buffer2[] = "";			//	写入数据
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

u8 USART1_TX_BUF[USART3_MAX_RECV_LEN];						 //串口1,发送缓存区
nmea_msg gpsx;												 //GPS信息
__align(4) u8 dtbuf[50];									 //打印缓存器
const u8 *fixmode_tbl[4] = {"Fail", "Fail", " 2D ", " 3D "}; //fix mode字符串

extern u16 USART6_RX_STA;

/*
1-风向
2-真实风向
3-风速
4-真实风速
5-温度
6-湿度
7-大气压
8-露点温度
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

	ReadyFlag = 0; //zkrt_debug 需设置为0

	while (1)
	{
		/*****************核辐射设备通讯---测试*******************/
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
		/*****************核辐射设备数据---读取*******************/
		else if (ReadyFlag == 1)
		{
			usart6_handle(); //for onboard sdk mainboard
			usart1_handle(); //for datalink transport
			/****************核辐射气象****************/
			if (TimeFlag)
			{
				TimeFlag = 0;
				LED0 = 1;
				/*等待气象模块读取到数据后，再一起转发出去*/  
				for (i = 0; i < MeteorBufCounter; i++)
				{
					printf("%c", RS232_RX_BUF[BufferFinishNumber][i]);
				}
				//printf("\r\n");
				/*读取核辐射模块数据*/
				ReadMeteorVal();
				/*转发核辐射数据*/
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
				/*发送飞机状态*/
				printf("$WRJSJ,Battery:%f,ReturnFlag:%d\r\n", (double)BatteryAllowance/1000, ReturnFlag);
				printf("\r\n");
			}
			else
			{
				LED0 = 0;
			}
		}
	}
}
