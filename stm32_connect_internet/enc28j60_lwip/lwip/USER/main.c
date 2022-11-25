#include "led.h"
#include "delay.h"
#include "key.h"
#include "sys.h"
#include "usart.h"	 
#include "usmart.h"
#include "sram.h"
#include "malloc.h"
#include "enc28j60.h" 	 
#include "lwip/netif.h"
#include "lwip_comm.h"
#include "lwipopts.h"
#include "timer.h"
#include "tcp_server_demo.h"
#include "cjson.h"

extern u8 tcp_server_flag;	 //TCP Server ����ȫ��״̬��Ǳ���
//����UI
//mode:
//bit0:0,������;1,����ǰ�벿��UI
//bit1:0,������;1,���غ�벿��UI
void lwip_test_ui(u8 mode)
{
	u8 buf[30]; 

	if(mode&1<<0)
	{
		
	}
	if(mode&1<<1)
	{

		if(lwipdev.dhcpstatus==2)sprintf((char*)buf,"DHCP IP:%d.%d.%d.%d",lwipdev.ip[0],lwipdev.ip[1],lwipdev.ip[2],lwipdev.ip[3]);//��ӡ��̬IP��ַ
		else sprintf((char*)buf,"Static IP:%d.%d.%d.%d",lwipdev.ip[0],lwipdev.ip[1],lwipdev.ip[2],lwipdev.ip[3]);//��ӡ��̬IP��ַ
	
	}
}

int main(void)
{	 
	u8 key;
	delay_init();	    	//��ʱ������ʼ��	  
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);		//����NVIC�жϷ���2:2λ��ռ���ȼ���2λ��Ӧ���ȼ�
	uart_init(115200);	 	//���ڳ�ʼ��Ϊ115200
 	LED_Init();			    //LED�˿ڳ�ʼ��

	KEY_Init();	 			//��ʼ������
	TIM3_Int_Init(1000,719);//��ʱ��3Ƶ��Ϊ100hz
 	usmart_dev.init(72);	//��ʼ��USMART		
 	FSMC_SRAM_Init();		//��ʼ���ⲿSRAM
	my_mem_init(SRAMIN);	//��ʼ���ڲ��ڴ��
	my_mem_init(SRAMEX);	//��ʼ���ⲿ�ڴ��
		
	lwip_test_ui(1);		//����ǰ�벿��UI		    
	while(lwip_comm_init()) //lwip��ʼ��
	{

		delay_ms(1200);
	 
	}
	
#if LWIP_DHCP   //ʹ��DHCP
	while((lwipdev.dhcpstatus!=2)&&(lwipdev.dhcpstatus!=0XFF))//�ȴ�DHCP��ȡ�ɹ�/��ʱ���
	{
		lwip_periodic_handle();	//LWIP�ں���Ҫ��ʱ����ĺ���
	}
#endif
	lwip_test_ui(2);		//���غ�벿��UI 
	delay_ms(500);			//��ʱ1s
	delay_ms(500);
	tcp_server_test();  	//TCP Serverģʽ
  	while(1)
	{	
		key = KEY_Scan(0);
		if(key == KEY1_PRES)		//��KEY1����������
		{
			if((tcp_server_flag & 1<<6)) printf("TCP�����Ѿ�����,�����ظ�����\r\n");	//������ӳɹ�,�����κδ���
			else tcp_server_test();		//���Ͽ����Ӻ�,����tcp_server_test()����
		}
		delay_ms(10);
	}
}

