
#ifndef _MODBUS_H_
#define _MODBUS_H_

#include "MODBUS_CRC.H"

 
 
//typedef struct 
//{
//  uint8_t  myadd;        //���豸�ӻ���ַ
//	uint8_t  rcbuf[100];   //modbus���ܻ�����
//	uint8_t  timout;       //modbus���ݳ���ʱ��
//	uint8_t  recount;      //modbus�˿ڽ��յ������ݸ���
//	uint8_t  timrun;       //modbus��ʱ���Ƿ��ʱ��־
//	uint8_t  reflag;       //modbusһ֡���ݽ�����ɱ�־λ
//	uint8_t  sendbuf[100]; //modbus�ӷ��ͻ�����
//	
//}MODBUS;
 
//extern MODBUS modbus;
extern uint8_t Reg[];
void Modbus_Init(void);
void Modbus_Func3(uint8_t *);
void Modbus_Func6(uint8_t *);

 
#endif

