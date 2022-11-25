#include "sys.h"
#include "delay.h"
#include "usart.h"
#include "led.h"
#include "dht11.h"

  
int main(void)
{ 
	u8 t=0;			    
	u8 temperature;  	    
	u8 humidity;  
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);//����ϵͳ�ж����ȼ�����2
	delay_init(168);  //��ʼ����ʱ����
	uart_init(38400);		//��ʼ�����ڲ�����Ϊ115200
	
	LED_Init();					//��ʼ��LED 
 	while(DHT11_Init())	//DHT11��ʼ��	
	{
		
 		delay_ms(200);
	}								   

	while(1)
	{	    	    
 		if(t%10==0)//ÿ100ms��ȡһ��
		{									  
			DHT11_Read_Data(&temperature,&humidity);		//��ȡ��ʪ��ֵ				
      printf("temperature= %d\r\n",temperature)	;
			
		  printf("humidity=%d\r\n",humidity); 	   
			delay_ms(3000);
		}				   
	 	delay_ms(10);
		t++;
		if(t==20)
		{
			t=0;
			LED0=!LED0;
		}
	}
}






