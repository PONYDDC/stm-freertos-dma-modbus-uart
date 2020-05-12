
#ifndef _MODBUS_H_
#define _MODBUS_H_

#include "MODBUS_CRC.H"

 
 
//typedef struct 
//{
//  uint8_t  myadd;        //本设备从机地址
//	uint8_t  rcbuf[100];   //modbus接受缓冲区
//	uint8_t  timout;       //modbus数据持续时间
//	uint8_t  recount;      //modbus端口接收到的数据个数
//	uint8_t  timrun;       //modbus定时器是否计时标志
//	uint8_t  reflag;       //modbus一帧数据接受完成标志位
//	uint8_t  sendbuf[100]; //modbus接发送缓冲区
//	
//}MODBUS;
 
//extern MODBUS modbus;
extern uint8_t Reg[];
void Modbus_Init(void);
void Modbus_Func3(uint8_t *);
void Modbus_Func6(uint8_t *);

 
#endif


