/**
  ******************************************************************************
  * @file    
  * @author  
  * @version 
  * @date    
  * @brief   
  ******************************************************************************
  * @COPYRIGHT
  */

/* Includes ------------------------------------------------------------------*/
#include "datalink_osdk_handle.h"
#include "osusart.h"
#include "Meteorological.h"
/* static functions define------------------------------------------------------------------*/
static uint8_t cmd_len_of_pc_control_data(uint8_t cmd);
static uint8_t cmd_len_of_respond_pc_control_data(uint8_t cmd);
static bool cmd_valided_pc_control_data(uint8_t cmd);
/* varibles ------------------------------------------------------------------*/
uint8_t u1_buf[200];
uint8_t u6_buf[200];
uint8_t respond_datalink_buf[200 + sizeof(RES_PC_CONTROL_TYPE)];
//extern
extern uint32_t TimeCounter;
extern u8 ReturnFlag;
extern uint32_t BatteryAllowance;
/* functions ------------------------------------------------------------------*/

void usart1_handle(void)
{
	PcContrlDatas pc_control_datas;
	uint8_t *buf = u1_buf;
	uint16_t buflen;
	uint8_t cmdlen;
	uint8_t need_transmit = 0;
	memset(buf, 0, sizeof(u1_buf));
	if (t_osscomm_ReceiveMessage(buf, &buflen, USART1) != SCOMM_RET_OK)
	{
		return;
	}
	pc_control_datas.start_code = buf[0];
	pc_control_datas.cmd = buf[1];
	pc_control_datas.data = &buf[2];
	if (pc_control_datas.start_code != START_CODE)
	{ //check startcode
		return;
	}
	cmdlen = cmd_len_of_pc_control_data(pc_control_datas.cmd);
	if (cmdlen + FIXED_FRAME_LEN != buflen)
	{ //check len
		return;
	}
	memcpy(&pc_control_datas.check_code, buf + cmdlen + FRAME_HEAD_LEN, 2);
	memcpy(&pc_control_datas.end_code, buf + cmdlen + FRAME_HEAD_LEN + 2, 2);
	if (pc_control_datas.end_code != END_CODE)
	{ //check endcode
		return;
	}
	if (crc_calculate(buf, cmdlen + FRAME_HEAD_LEN) != pc_control_datas.check_code)
	{ //check crc
		return;
	}
	switch (pc_control_datas.cmd)
	{
	case CMD_HIGH_PRESURE:
	{
		uint16_t SetVotalgeValue;
		HighPresure *hp = (HighPresure *)(pc_control_datas.data);
		SetVotalgeValue = hp->value;
		if (SetVotalgeValue >= 999)
			SetVotalgeValue = 999;
		if (SetVotalgeValue <= 0)
			SetVotalgeValue = 0;
		SetVoltage(SetVotalgeValue);
	}
	break;
	case CMD_NUCLEAR_UPLOAD_TIME:
	{
		NuclearUploadTime *nut = (NuclearUploadTime *)pc_control_datas.data;
		TimeCounter = nut->value;
		if (TimeCounter > 9999)
			TimeCounter = 9999;
		if (TimeCounter <= 0)
			TimeCounter = 0;
	}
	break;
	default:
		need_transmit = 1;
		break;
	}
	if (need_transmit)
	{
		t_osscomm_sendMessage(buf, buflen, USART6);
	}
}
//void usart1_handle(void)
//{
//	PcContrlDatas pc_control_datas;
//	uint8_t *buf = u1_buf;
//	uint16_t buflen;
//	uint8_t cmdlen;
//	uint8_t need_transmit = 0;
//	memset(buf, 0, sizeof(u1_buf));
//	if (t_osscomm_ReceiveMessage(buf, &buflen, USART1) != SCOMM_RET_OK)
//	{
//		return;
//	}
////	pc_control_datas.start_code = buf[0];
////	pc_control_datas.cmd = buf[1];
////	pc_control_datas.data = &buf[2];
////	if (pc_control_datas.start_code != START_CODE)
////	{ //check startcode
////		return;
////	}
////	cmdlen = cmd_len_of_pc_control_data(pc_control_datas.cmd);
////	if (cmdlen + FIXED_FRAME_LEN != buflen)
////	{ //check len
////		return;
////	}
////	memcpy(&pc_control_datas.check_code, buf + cmdlen + FRAME_HEAD_LEN, 2);
////	memcpy(&pc_control_datas.end_code, buf + cmdlen + FRAME_HEAD_LEN + 2, 2);
////	if (pc_control_datas.end_code != END_CODE)
////	{ //check endcode
////		return;
////	}
////	if (crc_calculate(buf, cmdlen + FRAME_HEAD_LEN) != pc_control_datas.check_code)
////	{ //check crc
////		return;
////	}
//	char test[] = "112#####################";
//	if(buflen ==180)
//		t_osscomm_sendMessage(buf, buflen, USART1);
////	else
////		t_osscomm_sendMessage((uint8_t*)test, sizeof(test), USART1);
//}
//
void usart6_handle(void)
{
	PcContrlDatas r_pc_control_datas;
	uint8_t *buf = u6_buf;
	uint16_t buflen;
	uint8_t cmdlen;
	memset(buf, 0, sizeof(u6_buf));
	if (t_osscomm_ReceiveMessage(buf, &buflen, USART6) != SCOMM_RET_OK)
	{
		return;
	}
	r_pc_control_datas.start_code = buf[0];
	r_pc_control_datas.cmd = buf[1];
	r_pc_control_datas.data = &buf[2];
	if (r_pc_control_datas.start_code != START_CODE)
	{ //check startcode
		return;
	}
	cmdlen = cmd_len_of_respond_pc_control_data(r_pc_control_datas.cmd);
	if (cmdlen + FIXED_FRAME_LEN != buflen)
	{ //check len
		return;
	}
	memcpy(&r_pc_control_datas.check_code, buf + cmdlen + FRAME_HEAD_LEN, 2);
	memcpy(&r_pc_control_datas.end_code, buf + cmdlen + FRAME_HEAD_LEN + 2, 2);
	if (r_pc_control_datas.end_code != END_CODE)
	{ //check endcode
		return;
	}
	if (crc_calculate(buf, cmdlen + FRAME_HEAD_LEN) != r_pc_control_datas.check_code)
	{ //check crc
		return;
	}
	//send respond data and heartbeat data to usart1
	memcpy(respond_datalink_buf, RES_PC_CONTROL_TYPE, sizeof(RES_PC_CONTROL_TYPE)-1); //add header string
	memcpy(respond_datalink_buf + sizeof(RES_PC_CONTROL_TYPE)-1, buf, buflen);
	if(cmd_valided_pc_control_data(r_pc_control_datas.cmd)==true)
		t_osscomm_sendMessage(respond_datalink_buf, buflen + sizeof(RES_PC_CONTROL_TYPE)-1, USART1);
	
	//special handle: zkrt_notice
	if((r_pc_control_datas.cmd ==CMD_SET_GOHOME)&&(r_pc_control_datas.data[0] ==ACK_OK))
	{
		ReturnFlag = 1;
	}
	if((r_pc_control_datas.cmd ==CMD_UPLOAD_GPS_DATA)&&(r_pc_control_datas.data[0] ==ACK_OK))
	{
		GpsData *gpsdata = (GpsData *)&r_pc_control_datas.data[1];
		BatteryAllowance = gpsdata->b.voltage;
	}	
}
//get cmd length 0-invalid, >0 valid
static uint8_t cmd_len_of_pc_control_data(uint8_t cmd)
{
	uint8_t len = 2;
	switch (cmd)
	{
	case CMD_HIGH_PRESURE:
		len = 2;
		break;
	case CMD_NUCLEAR_UPLOAD_TIME:
		len = 2;
		break;
	case CMD_READ_BATTORY:
		len = 2;
		break;
	case CMD_SET_GOHOME:
		len = 2;
		break;
	case CMD_WAYPOINT_INIT:
		len = sizeof(WayPointInitSettings);
		break;
	case CMD_SINGLE_WAYPOINT:
		len = sizeof(WayPointSettings);
		break;
	case CMD_START_WAYPOINT:
		len = 1;
		break;
	case CMD_PAUSE_WAYPOINT:
		len = 1;
		break;
	case CMD_READ_WAYPOINT_INIT_STATUS:
		len = 1;
		break;
	case CMD_READ_SINGLE_WAYPOINT:
		len = 1;
		break;
	case CMD_SET_WAYPOINT_IDLE_VEL:
		len = 4;
		break;
	case CMD_GET_WYAPOINT_IDLE_VEL:
		len = 1;
		break;
	// case CMD_UPLOAD_GPS_DATA:
	// 	break;
	default:
		len = 0;
		break;
	}
	return len;
}
//get cmd length respond 0-invalid, >0 valid
static uint8_t cmd_len_of_respond_pc_control_data(uint8_t cmd)
{
	uint8_t len = 2;
	switch (cmd)
	{
	case CMD_HIGH_PRESURE:
		len = 1;
		break;
	case CMD_NUCLEAR_UPLOAD_TIME:
		len = 1;
		break;
	case CMD_READ_BATTORY:
		len = 1;
		break;
	case CMD_SET_GOHOME:
		len = 1;
		break;
	case CMD_WAYPOINT_INIT:
		len = 1;
		break;
	case CMD_SINGLE_WAYPOINT:
		len = 2 + sizeof(WayPointSettings);
		break;
	case CMD_START_WAYPOINT:
		len = 1;
		break;
	case CMD_PAUSE_WAYPOINT:
		len = 1;
		break;
	case CMD_READ_WAYPOINT_INIT_STATUS:
		len = 1 + sizeof(WayPointInitSettings);
		break;
	case CMD_READ_SINGLE_WAYPOINT:
		len = 2 + sizeof(WayPointSettings);
		;
		break;
	case CMD_SET_WAYPOINT_IDLE_VEL:
		len = 5;
		break;
	case CMD_GET_WYAPOINT_IDLE_VEL:
		len = 5;
		break;
	case CMD_UPLOAD_GPS_DATA:
		len = 1 + sizeof(GpsData);
		break;
	default:
		len = 0;
		break;
	}
	return len;
}
//get cmd length respond 0-invalid, >0 valid
static bool cmd_valided_pc_control_data(uint8_t cmd)
{
//	if(cmd ==CMD_UPLOAD_GPS_DATA)
//		return false;
	return true;
}
