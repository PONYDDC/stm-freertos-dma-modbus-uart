
#include "main.h"
#include "cmsis_os.h"
#include "Wavefrom.h"
#include "math.h"
#include "D:\SW_JJQ1925\SW_JJQ1925\MDK-ARM\MODBUS\MODBUS.H"
extern DAC_HandleTypeDef hdac1;
extern DAC_HandleTypeDef hdac2;
extern 	uint32_t Test_D1;
extern  uint32_t Test_D2;
extern TIM_HandleTypeDef htim6;
//extern modbus_data_t  modbus_data;
void  Set_DAC(uint8_t Chip);
void  square_wave( uint16_t tongdao);
void  square_wave2(uint8_t T,uint8_t i,uint8_t dianjiA,uint8_t dianjiB);
uint8_t wave_cut[]={0x00,0x01,0x02,0x03,0x04,0x05,0x06,0x07,0x08,0x09};
extern uint8_t dianji;
void GPIO_Port_Pulse(uint8_t Pulse,unsigned char State);
/*********************************************/
//波形：提插补泻——补法
//mun:一串波形个数
//T:脉宽
//Power:强度值(0~100)
//Chip_choice：通道选择
/*********************************************/
void Tichabuxie_bufa(uint8_t num,uint8_t T,uint16_t Power,uint8_t Chip_choice)
{
//  static  uint8_t wave_cut[]={0x01,0x02,0x03,0x04,};
//	uint16_t Dac_Val=0;
	uint8_t k;	
	GPIO_Port_Pulse(1,SET);
//	      HAL_GPIO_WritePin(GPIOB,SW_LT1_Pin,GPIO_PIN_SET);
//				 HAL_GPIO_WritePin(GPIOB,SW_LB1_Pin ,GPIO_PIN_SET);
//	       HAL_GPIO_WritePin(GPIOB,SW_LB2_Pin ,GPIO_PIN_SET);
		for(k=0;k<num;k++)		
		{	
			Set_DAC(Chip_choice);	
//			Dac_Val=900;
//			Dac_Val=(uint16_t)(1200*(Power/100.00));	
//			Dac_Val%=1200;				
//			Set_DAC(Chip_choice);
//			osDelay(5);
			HAL_TIM_Base_Start_IT(&htim6);   //启动定时器使能中断	
//			  for(i=0;i<24;i++)	
//					{			
//							 Dac_Val%=1200;				
//      Set_DAC(Chip_choice);											
//					}	
//			
//			
//			 Dac_Val=1200;	 
//			 Dac_Val=(uint16_t)Dac_Val*(Power/100.00);
//				 for(j=0;j<52;j++)
//				{	
//							Dac_Val-=(uint16_t)23*((Power/100.00));		 
//							Dac_Val%=1200;	
//             Set_DAC(Chip_choice);					
//				}	
		osDelay (200);	
		}
		HAL_TIM_Base_Stop_IT(&htim6);   //关闭定时器
        Set_DAC(Chip_choice); 
		osDelay (200);	
	}
/*********************************************/
//波形：提插补泻——泻法
//mun:一串波形个数
//T:脉宽
//Power:强度值(0~99)
//Chip_choice：通道  （1或0）
/*********************************************/
void Tichabuxie_xiefa(uint8_t num,uint8_t T,uint16_t Power,uint8_t Chip_choice)
{
	uint16_t Dac_Val=0;
	uint8_t i,j,k;	
		for(k=0;k<num;k++)		
		{
			 Dac_Val=0;	
Set_DAC(Chip_choice);
			 osDelay(1);
					 for(j=0;j<50;j++)
					{	
             Set_DAC(Chip_choice); 	
						Dac_Val+=(uint16_t)24*((Power/100.00));		 
						Dac_Val%=1200;				
					}	
			Dac_Val=(uint16_t)(1200*(Power/100.00));	
					for(i=0;i<25;i++)	
					{			
					 Dac_Val%=1200;				
			Set_DAC(Chip_choice);
						uint16_t  Dac_Val= (uint16_t)Dac_Val*Power*0.01;		
					 }	
		}
Set_DAC(Chip_choice);
	osDelay (200);
	}
	

/*********************************************/
//波形：捻转补泻——补法
//mun:一串波形个数
//T:脉宽
//Power:强度值(0~90)
//Chip_choice：通道  （1或0）
//波形要求：  20HZ   3波 	
	
/*********************************************/
void nianzhuanbuxie_bufa(uint8_t num,uint8_t T,uint16_t Power,uint8_t Chip_choice)
{
	uint16_t Dac_Val=0;
	uint8_t j,k;	
			for(k=0;k<num;k++)		
			{	
				 Dac_Val=0;	
			 Set_DAC(Chip_choice);
				 osDelay(1);
						 for(j=0;j<200;j++)
						{	
					     Set_DAC(Chip_choice);			 
							Dac_Val=(uint16_t)((-0.12)*j*j+24*j)*(Power/100.00);	
							Dac_Val%=1200;			
							osDelay (2);		
						}	
			}
	osDelay (2000);
	}	
	
/*********************************************/
	//波形：捻转补泻——泻法
//mun:一串波形个数
//T:脉宽
//Power:强度值(0~90)
//Chip_choice：通道  （1或0）
/*********************************************/
void nianzhuanbuxie_xiefa(uint8_t num,uint8_t T,uint16_t Power,uint8_t Chip_choice)
{
	uint16_t Dac_Val=0;
	uint16_t j,k;	
		for(k=0;k<num;k++)		
		{	
			 Dac_Val=0;	
		 Set_DAC(Chip_choice);
			 osDelay(1);
					 for(j=0;j<400;j++)
					{	
						Set_DAC(Chip_choice);				 
							Dac_Val=(uint16_t)((-0.03)*j*j+12*j)*(Power/100.00);	
							Dac_Val%=1200;	
							Dac_Val=(uint16_t)(Dac_Val*(Power/100.00));									
					}				
		}
	osDelay (2000);
	}

/*********************************************/
//波形：平补平泻
//mun:一串波形个数
//T:脉宽
//Power:强度值(0~90)
//Chip_choice：通道  （1或0）
	//20hz  3波 ， 100HZ  1波
/*********************************************/
void pingbupingxie(uint8_t num,uint8_t T,uint16_t Power,uint8_t Chip_choice)
{
	uint16_t Dac_Val=0;
	uint16_t j,k;	
for(k=0;k<num;k++)		
{	
	 Dac_Val=1199;	
   Dac_Val= (uint16_t)Dac_Val*Power*0.01;	
	 for(j=0;j<80;j++)
	{	
       Set_DAC(Chip_choice);;			 
		Dac_Val%=1200;
    uint16_t    Dac_Val= (uint16_t)Dac_Val*Power*0.01;			
		osDelay (1);
	}	
	Dac_Val=1199;
  Dac_Val= (uint16_t)Dac_Val*Power*0.01;		
		 for(j=0;j<220;j++)
	{	
	Set_DAC(Chip_choice);			 
		Dac_Val%=1200;
    uint16_t Dac_Val= (uint16_t)Dac_Val*Power*0.01;		
		osDelay (1);
	}		
}
	osDelay (2000);
	}	

	/*********************************************/
//波形：徐疾补泻
//mun:一串波形个数
//T:脉宽
//Power:强度值(0~90)
//Chip_choice：通道  （1或0）
//10-100HZ  单波
/*********************************************/
	void xujibuxie_bufa(uint8_t num,uint8_t T, uint16_t  Power,uint8_t Chip_choice)
{
  uint16_t Dac_Val;
	uint16_t j,k ,t=90;	
			for(k=0;k<num;k++)		
			{	
				 Dac_Val=1110;	
				 Dac_Val= (uint16_t)Dac_Val*Power*0.01;	
						 for(j=0;j<100;j++)
								{	
                Set_DAC(Chip_choice);		 
								Dac_Val%=1200;
								uint16_t  Dac_Val= (uint16_t)Dac_Val*Power*0.01;
								osDelay (t);
								if(t>10)
									t=t-1;
								else
								 t=10;
								}		
             Set_DAC(Chip_choice);
			}
	osDelay (2000);
	}	
	
	/*********************************************/
	//波形：徐疾补泻-泻法
//mun:一串波形个数
//T:脉宽
//Power:强度值(0~90)
//Chip_choice：通道  （1或0）
//10-100HZ  单波
/*********************************************/
	void xujibuxie_xiefa(uint8_t num,uint8_t T, uint16_t  Power,uint8_t Chip_choice)
{
  uint16_t Dac_Val;
	uint16_t j,k ,t=10;	
		for(k=0;k<num;k++)		
		{	
			 Dac_Val=1110;	
			 Dac_Val= (uint16_t)Dac_Val*Power*0.01;	
					 for(j=0;j<87;j++)
					{	
            Set_DAC(Chip_choice);		 
						Dac_Val%=1200;
						uint16_t  Dac_Val= (uint16_t)Dac_Val*Power*0.01;
						osDelay (t);
						if(t<=90)
							    t++;
						else
						      t=90;
					}		
			Set_DAC(Chip_choice);	
		}
	osDelay (2000);
	}	


	



	
	
	void  square_wave2(uint8_t T,uint8_t i,uint8_t dianjiA,uint8_t dianjiB)
{ 
   
	static uint8_t flag=9;	
	GPIO_Port_Pulse(1,RESET );
  GPIO_Port_Pulse(2,RESET );
	GPIO_Port_Pulse(3,RESET );
	GPIO_Port_Pulse(4,RESET );
	GPIO_Port_Pulse(5,RESET );
	GPIO_Port_Pulse(6,RESET );
	GPIO_Port_Pulse(7,RESET );
	GPIO_Port_Pulse(8,RESET );
	
	flag++;flag%=10;
	
if(dianjiA==1&&dianjiB==0)   //开通A通道电极
{
switch(wave_cut[flag])
	{
		case 0:
			for(uint8_t j=0;j<i;j++){
				GPIO_Port_Pulse(1,SET );
			}
	  case 1:
			for(uint8_t j=0;j<i;j++){
				GPIO_Port_Pulse(2,SET );
			}
		case 2:
			for(uint8_t j=0;j<i;j++){
				GPIO_Port_Pulse(3,SET );
			}
		case 3:
		for(uint8_t j=0;j<i;j++){
       GPIO_Port_Pulse(4,SET );
		}
		default :	
		break ;
	} 
}
if(dianjiA==0&&dianjiB ==1)//开启B通道电极
{
	switch(wave_cut[flag])
	{
		case 0:
			{		
				GPIO_Port_Pulse(5,SET );
			}
			break;
	  case 1:
			{
				GPIO_Port_Pulse(6,SET );
			}
			break;
		case 2:
			{
				GPIO_Port_Pulse(7,SET );
			}
			break;
		case 3:
		{
			GPIO_Port_Pulse(8,SET );
		}
		default :
		break ;
	}
}
	
if(dianjiA==1&&dianjiB==1)
	{
	switch(wave_cut[flag])
	{
		case 0:
			{
				GPIO_Port_Pulse(1,SET );
				GPIO_Port_Pulse(5,SET );
			}
		break;
	  case 1:
		{
			GPIO_Port_Pulse(2,SET );
		GPIO_Port_Pulse(6,SET );
			}
		break;
		case 2:
		{
			GPIO_Port_Pulse(3,SET );
			GPIO_Port_Pulse(7,SET );
			}
		break;
		case 3:
			{
			GPIO_Port_Pulse(4,SET );
			GPIO_Port_Pulse(7,SET );
			}
		default :
		break ;

	
		}	
	}
}  
	 
void GPIO_Port_Pulse(uint8_t Pulse,unsigned char State)
{
if(Pulse==1)
{
	Set_DAC(1);
if(State==SET)
{
	     HAL_GPIO_WritePin(GPIOB,SW_LT1_Pin,GPIO_PIN_SET);
				 HAL_GPIO_WritePin(GPIOB,SW_LB1_Pin ,GPIO_PIN_SET);
	       HAL_GPIO_WritePin(GPIOB,SW_LB2_Pin ,GPIO_PIN_SET);
}
else 
{
  HAL_GPIO_WritePin(GPIOB,SW_LT1_Pin,GPIO_PIN_RESET);
				 HAL_GPIO_WritePin(GPIOB,SW_LB1_Pin ,GPIO_PIN_RESET);
	       HAL_GPIO_WritePin(GPIOB,SW_LB2_Pin ,GPIO_PIN_RESET);
}
}

if(Pulse==2)
{
	Set_DAC(1);
if(State==SET)
{
		HAL_GPIO_WritePin(GPIOA,SW_RT1_Pin,GPIO_PIN_SET);
				  HAL_GPIO_WritePin(GPIOB,SW_RB1_Pin,GPIO_PIN_SET);
					HAL_GPIO_WritePin(GPIOB,SW_RB2_Pin,GPIO_PIN_SET);
}
	
else 
{
	       HAL_GPIO_WritePin(GPIOA,SW_RT1_Pin,GPIO_PIN_RESET);
				  HAL_GPIO_WritePin(GPIOB,SW_RB1_Pin,GPIO_PIN_RESET);
					HAL_GPIO_WritePin(GPIOB,SW_RB2_Pin,GPIO_PIN_RESET);
}

}

if(Pulse==3)
{
	Set_DAC(1);
if(State==SET)
{
		    HAL_GPIO_WritePin(GPIOB,SW_LT2_Pin,GPIO_PIN_SET);
				HAL_GPIO_WritePin(GPIOB,SW_LB1_Pin ,GPIO_PIN_SET);
				HAL_GPIO_WritePin(GPIOB,SW_LB2_Pin,GPIO_PIN_SET);	
}
else 
{
		    HAL_GPIO_WritePin(GPIOB,SW_LT2_Pin,GPIO_PIN_RESET);
				HAL_GPIO_WritePin(GPIOB,SW_LB1_Pin ,GPIO_PIN_RESET);
				HAL_GPIO_WritePin(GPIOB,SW_LB2_Pin,GPIO_PIN_RESET);	
}

}

if(Pulse==4)
{
	Set_DAC(1);
if(State==SET )
{ 	    HAL_GPIO_WritePin(GPIOB,SW_RT2_Pin,GPIO_PIN_SET);
				HAL_GPIO_WritePin(GPIOB,SW_RB2_Pin,GPIO_PIN_SET);	
        HAL_GPIO_WritePin(GPIOB,SW_RB1_Pin,GPIO_PIN_SET);
}
else 
{
	    	HAL_GPIO_WritePin(GPIOB,SW_RT2_Pin,GPIO_PIN_RESET);
				HAL_GPIO_WritePin(GPIOB,SW_RB2_Pin,GPIO_PIN_RESET);	
        HAL_GPIO_WritePin(GPIOB,SW_RB1_Pin,GPIO_PIN_RESET);
}
}

if(Pulse==5)
{
	Set_DAC(2);
if(State==SET )
{
	       HAL_GPIO_WritePin(GPIOB,SW_LT1_Pin,GPIO_PIN_SET);
				 HAL_GPIO_WritePin(GPIOB,SW_LB1_Pin ,GPIO_PIN_SET);
	       HAL_GPIO_WritePin(GPIOB,SW_LB2_Pin ,GPIO_PIN_SET);
}
else 
{        HAL_GPIO_WritePin(GPIOB,SW_LT1_Pin,GPIO_PIN_RESET);
				 HAL_GPIO_WritePin(GPIOB,SW_LB1_Pin ,GPIO_PIN_RESET);
	       HAL_GPIO_WritePin(GPIOB,SW_LB2_Pin ,GPIO_PIN_RESET);
}
}

if(Pulse==6)
{
		Set_DAC(2);
if(State==SET)
{
	      	HAL_GPIO_WritePin(GPIOA,SW_RT1_Pin,GPIO_PIN_SET);
				  HAL_GPIO_WritePin(GPIOB,SW_RB1_Pin,GPIO_PIN_SET);
					HAL_GPIO_WritePin(GPIOB,SW_RB2_Pin,GPIO_PIN_SET);
}
	
else 
{
         	HAL_GPIO_WritePin(GPIOA,SW_RT1_Pin,GPIO_PIN_RESET);
				  HAL_GPIO_WritePin(GPIOB,SW_RB1_Pin,GPIO_PIN_RESET);
					HAL_GPIO_WritePin(GPIOB,SW_RB2_Pin,GPIO_PIN_RESET);
}
}

if(Pulse==7)
{
		Set_DAC(2);
if(State==SET )
{
		    HAL_GPIO_WritePin(GPIOB,SW_LT2_Pin,GPIO_PIN_SET);
				HAL_GPIO_WritePin(GPIOB,SW_LB1_Pin ,GPIO_PIN_SET);
				HAL_GPIO_WritePin(GPIOB,SW_LB2_Pin,GPIO_PIN_SET);	
}
else 
{
		    HAL_GPIO_WritePin(GPIOB,SW_LT2_Pin,GPIO_PIN_RESET);
				HAL_GPIO_WritePin(GPIOB,SW_LB1_Pin ,GPIO_PIN_RESET);
				HAL_GPIO_WritePin(GPIOB,SW_LB2_Pin,GPIO_PIN_RESET);	
}
}
if(Pulse==8)
{
		Set_DAC(2);
if(State==SET )
{ 	    HAL_GPIO_WritePin(GPIOB,SW_RT2_Pin,GPIO_PIN_SET);
				HAL_GPIO_WritePin(GPIOB,SW_RB2_Pin,GPIO_PIN_SET);	
        HAL_GPIO_WritePin(GPIOB,SW_RB1_Pin,GPIO_PIN_SET);
}
else 
{
		    HAL_GPIO_WritePin(GPIOB,SW_RT2_Pin,GPIO_PIN_RESET);
				HAL_GPIO_WritePin(GPIOB,SW_RB2_Pin,GPIO_PIN_RESET);	
        HAL_GPIO_WritePin(GPIOB,SW_RB1_Pin,GPIO_PIN_RESET);
}

}
}
void  Set_DAC(uint8_t Chip)
{
	if(Chip==1)
	{
		HAL_DAC_Start(&hdac1, DAC_CHANNEL_1);
			HAL_TIM_Base_Start_IT(&htim6);
	for(uint16_t i=0;i<64;i++)
		{	
		HAL_DAC_SetValue(&hdac1, DAC_CHANNEL_1,DAC_ALIGN_12B_R,Test_D1);
			osDelay(20);
		  if(Test_D1<4096)
			{
				Test_D1+=64;
			 osDelay(20);
			
			}
				else  Test_D1=0;
	  }
		HAL_TIM_Base_Stop_IT(&htim6);	
	}
	if(Chip==2)
	{
	   HAL_DAC_Start(&hdac2, DAC_CHANNEL_2);
for(uint16_t j=0;j<512;j++)
{		
		HAL_DAC_SetValue(&hdac2, DAC_CHANNEL_2,DAC_ALIGN_12B_R,Test_D2);
		if(Test_D2>4)  Test_D2-=8;else Test_D2=4095;
}		
	}
}




