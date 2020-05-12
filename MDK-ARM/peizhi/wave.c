#include "main.h"
#include "cmsis_os.h"
# include "wave.h"
# include "math.h"
extern TIM_HandleTypeDef htim2;
uint16_t t;
void  waveinit(void)
{ 
	 HAL_GPIO_WritePin(Power_EN_GPIO_Port, Power_EN_Pin, GPIO_PIN_RESET);
   HAL_GPIO_WritePin(GPIOB, SW_RT2_Pin|SW_LB2_Pin|SW_LT2_Pin|SW_RB1_Pin 
                          |SW_LB1_Pin|SW_LT1_Pin|PAD_Det_Pin, GPIO_PIN_RESET);
   HAL_GPIO_WritePin(GPIOA, SW_RT1_Pin|SW_RB2_Pin, GPIO_PIN_RESET); 
}
void  startwork(void)
{
	  HAL_GPIO_WritePin(Power_EN_GPIO_Port,Power_EN_Pin,GPIO_PIN_SET);
}
void  wavechan1()
{
     	HAL_TIM_Base_Start(&htim2);
      __HAL_TIM_SET_COUNTER(&htim2, 0);
      t= __HAL_TIM_GET_COUNTER(&htim2);
//	    if(t<1000)
//			{
//			
//				if(t%2==1)
//			{
//        
				 HAL_GPIO_WritePin(GPIOB,SW_LT1_Pin|SW_LB1_Pin|SW_LB2_Pin,GPIO_PIN_SET);//一脚
//			}
//	     if(t%2==0)
//			{ 
//				  HAL_GPIO_WritePin(GPIOA,SW_RT1_Pin,GPIO_PIN_SET);
//				  HAL_GPIO_WritePin(GPIOB,SW_RB1_Pin|SW_RB2_Pin,GPIO_PIN_SET);
//				//二脚
//			} 

//}
					HAL_TIM_Base_Stop(&htim2);
}
void  wavechan2()
{
	   	HAL_TIM_Base_Start(&htim2);
      __HAL_TIM_SET_COUNTER(&htim2, 0);
      t= __HAL_TIM_GET_COUNTER(&htim2);
	    if(t<1000&&t%2==0)
			{
				HAL_GPIO_WritePin(GPIOB,SW_LT2_Pin,GPIO_PIN_SET);
				HAL_GPIO_WritePin(GPIOB,SW_LB1_Pin ,GPIO_PIN_SET);
				HAL_GPIO_WritePin(GPIOB,SW_LB2_Pin,GPIO_PIN_SET);		//三脚	 
			}
	    if(t<1000&&t%2==1)
			{ 
				HAL_GPIO_WritePin(GPIOB,SW_RT2_Pin,GPIO_PIN_SET);
				HAL_GPIO_WritePin(GPIOB,SW_RB2_Pin,GPIO_PIN_SET);	
        HAL_GPIO_WritePin(GPIOB,SW_RB1_Pin,GPIO_PIN_SET);		//四脚		
			} 
	HAL_TIM_Base_Stop(&htim2);
}
	
