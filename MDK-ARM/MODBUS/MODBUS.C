
#include "MODBUS.H"

uint8_t  myadd=1;

uint8_t  Reg[64] ;//Modbus寄存器组，地址为0x00~0x04
 
 uint16_t Get_Buff(uint8_t *Rx_Data);
uint8_t sendbuf[64];
void Send_Bytes(uint8_t *D,uint16_t Len);		 
// Modbus事件处理函数
uint16_t MODBUS_Ansy_Data(uint8_t *Fdata,uint16_t Len){
  	
	uint16_t crc,rccrc;
	//没有收到数据包
  if(Len == 0)  
	{
	   return 0;
	}
	//收到数据包
	//通过读到的数据帧计算CRC
	crc = Modbus_CRC16(Fdata,Len-2); 
	// 读取数据帧的CRC
	rccrc = Fdata[Len-2]*256+Fdata[Len-1];
	if((crc == rccrc) && (Fdata[0] == myadd)) //CRC检验成功 开始分析包
	{	
	   // 检查地址是否时自己的地址

		   switch(Fdata[1])   //分析modbus功能码
			 {
			   case 0:             break;
				 case 1:             break;
				 case 2:             break;
				 case 3:      Modbus_Func3(Fdata);      break;
				 case 4:             break;
				 case 5:             break;
         case 6:      Modbus_Func6(Fdata);      break;
				 case 7:             break;
				 case 8:             break;
				 case 9:             break;			 
			 }


	}	
	return 0;
}
 
void MODBUS_Ansy(void){
uint8_t RxBuff[128];
uint16_t Len=0;	
	Len=Get_Buff(RxBuff);
	MODBUS_Ansy_Data(RxBuff,Len);
}
// Modbus 3号功能码函数
// Modbus 主机读取寄存器值
void Modbus_Func3(uint8_t *FData)
{ 


  uint16_t Regadd,Reglen,crc;
	uint8_t i,j;

	//得到要读取寄存器的首地址
	Regadd = FData[2]*256+FData[3];

	//得到要读取寄存器的数据长度
	Reglen = FData[4]*256+FData[5];
	//发送回应数据包
	i = 0;
sendbuf[i++] = myadd;      //发送本机设备地址
sendbuf[i++] = 0x03;              //发送功能码
sendbuf[i++] = ((Reglen*2)%256);   //返回字节个数
	for(j=0;j<Reglen;j++)                    //返回数据
	{
sendbuf[i++] = Reg[Regadd+j]/256;
sendbuf[i++] = Reg[Regadd+j]%256;

	}
	crc = Modbus_CRC16(sendbuf,i);
      	//计算要返回数据的CRC

	sendbuf[i++] = crc/256;
	sendbuf[i++] = crc%256;
	// 开始返回Modbus数据
  
	Send_Bytes(sendbuf,i);	

}
//// Modbus 6号功能码函数
//// Modbus 主机写入单个寄存器值
void Modbus_Func6(uint8_t *FData)  
{
  uint16_t Regadd;
	uint16_t val;
	uint16_t j,crc;
	j=0;
  Regadd=FData[2]*256+FData[3];  //得到要修改的地址 
	val=FData[4]*256+FData[5];     //修改后的值
	Reg[Regadd]=val;  //修改本设备相应的寄存器
	
	//以下为回应主机    地址  功能码 寄存器地址  单个寄存器的值  crc校验
	sendbuf[j++]=myadd;        //本设备地址
  sendbuf[j++]=0x06;        //功能码 
  sendbuf[j++]=Regadd/256;
	sendbuf[j++]=Regadd%256;
	sendbuf[j++]=val/256;
	sendbuf[j++]=val%256;
	crc=Modbus_CRC16(sendbuf,j);
	sendbuf[j++]=crc/256;  //
	sendbuf[j++]=crc%256;

	 Send_Bytes(sendbuf,j);

}

