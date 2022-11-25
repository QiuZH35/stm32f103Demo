#include "led.h"
#include "delay.h"
#include "key.h"
#include "sys.h"
#include "usart.h"
 

 int main(void)
 {		
  u8 a;
	delay_init();	    	 //��ʱ������ʼ��	  
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2); //����NVIC�жϷ���2:2λ��ռ���ȼ���2λ��Ӧ���ȼ�
	uart_init(115200);	 //���ڳ�ʼ��Ϊ115200
 	LED_Init();			     //LED�˿ڳ�ʼ��
	KEY_Init();          //��ʼ���밴�����ӵ�Ӳ���ӿ�
	 
 	while(1)
	{
		if(USART_GetFlagStatus(USART1,USART_FLAG_RXNE) != RESET){  //��ѯ���ڴ������־λ
			a =USART_ReceiveData(USART1);//��ȡ���յ�������
//			switch (a){
//				case '0':
//				printf("hello ");
//					break;
//				case '1':
//					printf("hello111");
//					break;
//				case '2':		
//					printf("BUZZER "); //���յ������ݷ��ͻص���
//					break;
//				default:
//					break;
//			}		  
			printf("message: %d\r\n",a);
		
		//while ѭ����demoģ��
	}	 
 }

}
 