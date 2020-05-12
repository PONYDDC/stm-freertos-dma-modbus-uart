
#include "main.h"
#include "cmsis_os.h"
#include "string.h"
# include  "URAT.h"
extern  UART_HandleTypeDef huart1; 
extern DMA_HandleTypeDef hdma_usart1_rx;
extern DMA_HandleTypeDef hdma_usart1_tx;
extern  uint8_t  Rxbuff[128];
extern  uint8_t  Txbuff[8];
extern osThreadId defaultTaskHandle;
extern osThreadId myTask02Handle;
extern osThreadId myTask03Handle;
extern osThreadId myTask04Handle;
extern osTimerId myTimer03Handle;
extern  uint16_t usart1_buffer_index;
uint32_t Read_Index=0;    //头
 uint8_t ModBus_Data[128];
uint16_t ModBus_Data_Len=0;
uint16_t mSTick;

void ModBus_IncTick(void)
{
	if(mSTick<10000)
		mSTick ++;
	if(mSTick>=4){
	;
	}
}

//判断是否接受完成
uint16_t Receive_Changed(void){
	static uint32_t Last_CNDTR;
	uint32_t Next_CNDTR=hdma_usart1_rx.Instance->CNDTR;
	if(Last_CNDTR == Next_CNDTR)
		return 0;
	else{
		Last_CNDTR = Next_CNDTR;
		return 1;
	}
}
uint16_t Get_Buff(uint8_t *Rx_Data)
{
	uint32_t Next_CNDTR=0;
	uint16_t Len;
	uint16_t Result=0;
	Next_CNDTR=hdma_usart1_rx.Instance->CNDTR;
	Len=sizeof(Rxbuff)-Next_CNDTR;//CNDTR 为DMA数量寄存器，由高到低
	Len+=sizeof(Rxbuff);		
	Len-=Read_Index;
	Len%=sizeof(Rxbuff);		
	memcpy(Rx_Data, Rxbuff+Read_Index,Len);	
	Result=Len;
	Read_Index+=Len;
	Read_Index%=sizeof(Rxbuff); //修正地址
	return Result;
}
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
if(huart->Instance == USART1)   
{	
// HAL_UART_Receive_IT(&huart1,Rxbuff,128);
//      if(usart1_buffer_index==0)  
//				{
//        __HAL_TIM_SET_COUNTER(&htim3, 0);
//          HAL_TIM_Base_Start();
//        }
//         HAL_TIM_SET_COUNTER(&htim3);
//             ModbusOvertime=0; //
//        usart2_buffer_index++; //????
//        if(usart1_buffer_index>128)
//        {
//            usart1_buffer_index=0;    //1-3
 }
//}
}


void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
{
if(huart->Instance == USART1)   
{


}

}

void Send_Bytes(uint8_t *D,uint16_t Len)
{

HAL_UART_Transmit_DMA(&huart1,D, Len);	
	
huart1.gState = HAL_UART_STATE_READY;
}


//void  Send_Bytes(uint8_t *D,uint16_t Len)
//	{ 
//	
//	while(huart1.gState==HAL_UART_STATE_READY)
//	 {
//		 
//	HAL_UART_Transmit_DMA(&huart1,D, Len);
//		 
//	 }

//	 
//	 
//}

	





	









