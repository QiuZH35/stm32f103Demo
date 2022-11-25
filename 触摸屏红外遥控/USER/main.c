#include "delay.h"
#include "sys.h"
#include "lcd.h"
#include "touch.h"
#include "gui.h"
#include "test.h"
#include "usart.h"
#include "remote.h"
#include  "key1.h"
#include "led.h"

//��һ��С���⣬��Ļˢ�²���ʾ����ң������bug,�����
int main(void)
{	
	u8 key;
	u8 t=0;	
 	u8 *str=0;
	SystemInit();//��ʼ��RCC ����ϵͳ��ƵΪ72MHZ
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);//�����ж����ȼ�����Ϊ��2��2λ��ռ���ȼ���2λ��Ӧ���ȼ�
  uart_init(115200);
	LED_Init();			     //LED�˿ڳ�ʼ��
	delay_init();
	LCD_Init();	   //Һ������ʼ��
	Remote_Init();			//������ճ�ʼ��	
  KEY1_Init();	 		
  //ѭ������
	DrawTestPage("����ң����");
	Show_Str(50,60,BLUE,YELLOW,"test",12,0);	
	Show_Str(50,100,BLUE,YELLOW,"action",12,0);		
	while(1)
	{
		key=Remote_Scan();	
		 
		if(key)
		{	 
			//LCD_ShowNum(86,130,key,3,16);		//��ʾ��ֵ
			//LCD_ShowNum(86,150,RmtCnt,3,16);	//��ʾ��������	
			
			switch(key)
			{
				case 0:str="ERROR";	break;			   
				case 162:str="POWER";break;	    
				case 98:str="UP";break;	    
				case 2:str="PLAY";break;		 
				case 226:str="ALIENTEK";break;		  
				case 194:str="RIGHT";break;	   
				case 34:str="LEFT";break;		  
				case 224:str="VOL-";break;		  
				case 168:str="DOWN";break;		   
				case 144:str="VOL+";break;		    
				case 104:str="1";break;		  
				case 152:str="2";break;	   
				case 176:str="3";break;	    
				case 48:str="4";break;		    
				case 24:str="5";break;		    
				case 122:str="6";break;		  
				case 16:str="7";break;			   					
				case 56:str="8";break;	 
				case 90:str="9";break;
				case 66:str="0";break;
				case 82:str="DELETE";break;		 
			}
		 printf("str: %s\r\n",str);
		
	  //	Touch_Test();		//��������д����  
		//Button_Test();
			Show_Str(80,100,BLUE,YELLOW,str,12,0);		
			delay_ms(5000);
		}else		
			delay_ms(10);	  
		   t++;
		if(t==20)
		{
			t=0;
			LED0=!LED0;
		}
		delay_ms(60000);
	}
		
}



