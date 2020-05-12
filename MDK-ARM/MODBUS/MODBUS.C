
#include "MODBUS.H"

uint8_t  myadd=1;

uint8_t  Reg[64] ;//Modbus�Ĵ����飬��ַΪ0x00~0x04
 
 uint16_t Get_Buff(uint8_t *Rx_Data);
uint8_t sendbuf[64];
void Send_Bytes(uint8_t *D,uint16_t Len);		 
// Modbus�¼���������
uint16_t MODBUS_Ansy_Data(uint8_t *Fdata,uint16_t Len){
  	
	uint16_t crc,rccrc;
	//û���յ����ݰ�
  if(Len == 0)  
	{
	   return 0;
	}
	//�յ����ݰ�
	//ͨ������������֡����CRC
	crc = Modbus_CRC16(Fdata,Len-2); 
	// ��ȡ����֡��CRC
	rccrc = Fdata[Len-2]*256+Fdata[Len-1];
	if((crc == rccrc) && (Fdata[0] == myadd)) //CRC����ɹ� ��ʼ������
	{	
	   // ����ַ�Ƿ�ʱ�Լ��ĵ�ַ

		   switch(Fdata[1])   //����modbus������
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
// Modbus 3�Ź����뺯��
// Modbus ������ȡ�Ĵ���ֵ
void Modbus_Func3(uint8_t *FData)
{ 


  uint16_t Regadd,Reglen,crc;
	uint8_t i,j;

	//�õ�Ҫ��ȡ�Ĵ������׵�ַ
	Regadd = FData[2]*256+FData[3];

	//�õ�Ҫ��ȡ�Ĵ��������ݳ���
	Reglen = FData[4]*256+FData[5];
	//���ͻ�Ӧ���ݰ�
	i = 0;
sendbuf[i++] = myadd;      //���ͱ����豸��ַ
sendbuf[i++] = 0x03;              //���͹�����
sendbuf[i++] = ((Reglen*2)%256);   //�����ֽڸ���
	for(j=0;j<Reglen;j++)                    //��������
	{
sendbuf[i++] = Reg[Regadd+j]/256;
sendbuf[i++] = Reg[Regadd+j]%256;

	}
	crc = Modbus_CRC16(sendbuf,i);
      	//����Ҫ�������ݵ�CRC

	sendbuf[i++] = crc/256;
	sendbuf[i++] = crc%256;
	// ��ʼ����Modbus����
  
	Send_Bytes(sendbuf,i);	

}
//// Modbus 6�Ź����뺯��
//// Modbus ����д�뵥���Ĵ���ֵ
void Modbus_Func6(uint8_t *FData)  
{
  uint16_t Regadd;
	uint16_t val;
	uint16_t j,crc;
	j=0;
  Regadd=FData[2]*256+FData[3];  //�õ�Ҫ�޸ĵĵ�ַ 
	val=FData[4]*256+FData[5];     //�޸ĺ��ֵ
	Reg[Regadd]=val;  //�޸ı��豸��Ӧ�ļĴ���
	
	//����Ϊ��Ӧ����    ��ַ  ������ �Ĵ�����ַ  �����Ĵ�����ֵ  crcУ��
	sendbuf[j++]=myadd;        //���豸��ַ
  sendbuf[j++]=0x06;        //������ 
  sendbuf[j++]=Regadd/256;
	sendbuf[j++]=Regadd%256;
	sendbuf[j++]=val/256;
	sendbuf[j++]=val%256;
	crc=Modbus_CRC16(sendbuf,j);
	sendbuf[j++]=crc/256;  //
	sendbuf[j++]=crc%256;

	 Send_Bytes(sendbuf,j);

}
