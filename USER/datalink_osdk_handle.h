/**
  ******************************************************************************
  * @file    
  * @author  
  * @version 
  * @date    
  * @brief  
  ******************************************************************************
  * @copy
  */ 
  
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __DATALINK_OSDK_HANDLE_H
#define __DATALINK_OSDK_HANDLE_H
#include "sys.h"

/////////////////////////////////////////////////
/*
协议结构体 需要1字节对齐
*/
#pragma pack(1)
/////////////////////////////////////////////////
//控制类型数据结构
typedef struct{
	uint8_t start_code;
	uint8_t cmd;
	uint8_t *data;
	uint16_t check_code;
	uint16_t end_code;
}PcContrlDatas;
//回复给PC的类型数据结构
#define RES_PC_CONTROL_TYPE "$RESSJ,"
typedef struct{
	char type_string[sizeof(RES_PC_CONTROL_TYPE)-1];
	PcContrlDatas r_data;
}ResPcContrlDatas;
//航线规划数据结构
/**
 * @brief Waypoint Mission Initialization settings
 * @details This is one of the few structs in the OSDK codebase that
 * is used in both a sending and a receiving API.
 */
typedef struct WayPointInitSettings
{
  uint8_t   indexNumber; /*!< Total number of waypoints <br>*/
  float32_t maxVelocity; /*!< Maximum speed joystick input(2~15m) <br>*/
  float32_t idleVelocity; /*!< Cruising Speed */
  /*!< (without joystick input, no more than vel_cmd_range) */
  uint8_t finishAction; /*!< Action on finish <br>*/
  /*!< 0: no action <br>*/
  /*!< 1: return to home <br>*/
  /*!< 2: auto landing <br>*/
  /*!< 3: return to point 0 <br>*/
  /*!< 4: infinite mode, no exit <br>*/
  uint8_t executiveTimes; /*!< Function execution times <br>*/
  /*!< 1: once <br>*/
  /*!< 2: twice <br>*/
  uint8_t yawMode; /*!< Yaw mode <br>*/
  /*!< 0: auto mode(point to next waypoint) <br>*/
  /*!< 1: lock as an initial value <br>*/
  /*!< 2: controlled by RC <br>*/
  /*!< 3: use waypoint's yaw(tgt_yaw) */
  uint8_t traceMode; /*!< Trace mode <br>*/
  /*!< 0: point to point, after reaching the target waypoint hover, 
   * complete waypoints action (if any), 
   * then fly to the next waypoint <br>
   * 1: Coordinated turn mode, smooth transition between waypoints,
   * no waypoints task <br>
   */
  uint8_t RCLostAction; /*!< Action on rc lost <br>*/
  /*!< 0: exit waypoint and failsafe <br>*/
  /*!< 1: continue the waypoint <br>*/
  uint8_t gimbalPitch; /*!< Gimbal pitch mode <br>*/
  /*!< 0: free mode, no control on gimbal <br>*/
  /*!< 1: auto mode, Smooth transition between waypoints <br>*/
  float64_t latitude;     /*!< Focus latitude (radian) */
  float64_t longitude;    /*!< Focus longitude (radian) */
  float32_t altitude;     /*!< Focus altitude (relative takeoff point height) */
  uint8_t   reserved[16]; /*!< Reserved, must be set to 0 */

} WayPointInitSettings; // pack(1)

/**
 * @brief Waypoint settings for individual waypoints being added to the mission
 * @details This is one of the few structs in the OSDK codebase that
 * is used in both a sending and a receiving API.
 */
typedef struct WayPointSettings
{
  uint8_t   index;     /*!< Index to be uploaded */
  float64_t latitude;  /*!< Latitude (radian) */
  float64_t longitude; /*!< Longitude (radian) */
  float32_t altitude;  /*!< Altitude (relative altitude from takeoff point) */
  float32_t damping; /*!< Bend length (effective coordinated turn mode only) */
  int16_t   yaw;     /*!< Yaw (degree) */
  int16_t   gimbalPitch; /*!< Gimbal pitch */
  uint8_t   turnMode;    /*!< Turn mode <br> */
  /*!< 0: clockwise <br>*/
  /*!< 1: counter-clockwise <br>*/
  uint8_t reserved[8]; /*!< Reserved */
  uint8_t hasAction;   /*!< Action flag <br>*/
  /*!< 0: no action <br>*/
  /*!< 1: has action <br>*/
  uint16_t actionTimeLimit;      /*!< Action time limit */
  uint8_t  actionNumber : 4;     /*!< Total number of actions */
  uint8_t  actionRepeat : 4;     /*!< Total running times */
  uint8_t  commandList[16];      /*!< Command list */
  uint16_t commandParameter[16]; /*!< Command parameters */
} WayPointSettings;              // pack(1)
//飞控状态数据
typedef struct Quaternion
{
  float32_t q0; /*!< w */
  float32_t q1; /*!< x */
  float32_t q2; /*!< y */
  float32_t q3; /*!< z */
} Quaternion;   // pack(1)
typedef struct GlobalPosition
{
  float64_t latitude;  /*!< unit: rad */
  float64_t longitude; /*!< unit: rad */
  float32_t altitude;  /*!< Measured by barometer: WGS 84 reference ellipsoid */
  float32_t height;    /*!< Ultrasonic height in meters */
  uint8_t   health;    /*!< scale from 0 - 5 signifying gps signal strength <br>
                        * greater than 3 for strong signal */
} GlobalPosition;      // pack(1)
typedef struct Battery
{
  uint32_t capacity;
  int32_t  voltage;
  int32_t  current;
  uint8_t  percentage;
} Battery; // pack(1)
//user define
typedef struct
{
    Quaternion q;
    GlobalPosition g;
    Battery b;
}GpsData;
typedef struct{
    uint16_t value;
}HighPresure;
typedef struct{
    uint16_t value;
}NuclearUploadTime;
typedef struct{
    uint16_t value;
}ReservedPccd;
/////////////////////////////////////////////////
#pragma pack()

//start code
#define START_CODE 0x5a
#define END_CODE 0x0a0d
//cmd code
#define CMD_HIGH_PRESURE                     0x01
#define CMD_NUCLEAR_UPLOAD_TIME              0x02
#define CMD_READ_BATTORY                     0x03
#define CMD_SET_GOHOME                       0x04
#define CMD_WAYPOINT_INIT                    0x05
#define CMD_SINGLE_WAYPOINT                  0x06
#define CMD_START_WAYPOINT                   0x07
#define CMD_PAUSE_WAYPOINT                   0x08
#define CMD_READ_WAYPOINT_INIT_STATUS        0x09
#define CMD_READ_SINGLE_WAYPOINT             0x0a
#define CMD_SET_WAYPOINT_IDLE_VEL            0x0b
#define CMD_GET_WYAPOINT_IDLE_VEL            0x0c
#define CMD_UPLOAD_GPS_DATA                  0xFE

//len
#define FIXED_FRAME_LEN 6
#define FRAME_HEAD_LEN  2
#define FRAME_TAIL_LEN 4

//ack
#define ACK_OK	0

/*
协议结构体 需要1字节对齐
*/

void usart1_handle(void);
void usart6_handle(void);
#endif 
