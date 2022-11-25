#include "lcd.h"
#include "delay.h"
#include "gui.h"
#include "test.h"
#include "touch.h"
#include "key.h" 
#include "led.h"
#include "pic.h"
#include "usart.h"
#include "rgb.h"



u16 ColorTab[5]={RED,GREEN,BLUE,YELLOW,BRED};//������ɫ����


void DrawTestPage(u8 *str)
{
//���ƹ̶���up
LCD_Clear(WHITE);
LCD_Fill(0,0,lcddev.width,20,BLUE);
//���ƹ̶���down
LCD_Fill(0,lcddev.height-20,lcddev.width,lcddev.height,BLUE);
POINT_COLOR=WHITE;
Gui_StrCenter(0,2,WHITE,BLUE,str,16,1);//������ʾ
Gui_StrCenter(0,lcddev.height-18,WHITE,BLUE,"hello world !",16,1);//������ʾ
//���Ʋ�������
//LCD_Fill(0,20,lcddev.width,lcddev.height-20,WHITE);
}



void Touch_Test(void)
{
	u8 key;
	u16 i=0;
	u16 j=0;
	u16 colorTemp=0;
	TP_Init();
	KEY_Init();
	LED_Init();
	DrawTestPage("����9:Touch����(��KEY0��У׼)  ");
	LCD_ShowString(lcddev.width-24,0,16,"RST",1);//��bʾ��������
	POINT_COLOR=RED;
	LCD_Fill(lcddev.width-52,2,lcddev.width-50+20,18,RED); 
		while(1)
	{
	 	key=KEY_Scan();
		tp_dev.scan(0); 		 
		if(tp_dev.sta&TP_PRES_DOWN)			//������������
		{	
		
		 	if(tp_dev.x<lcddev.width&&tp_dev.y<lcddev.height)
			{	
					printf("x= %d\r\n",tp_dev.x);
				if(tp_dev.x>(lcddev.width-24)&&tp_dev.y<16)
				{
					DrawTestPage("����9:Touch����(��KEY0��У׼)  ");//���
					LCD_ShowString(lcddev.width-24,0,16,"RST",1);//��ʾ��������
					POINT_COLOR=colorTemp;
					LCD_Fill(lcddev.width-52,2,lcddev.width-50+20,18,POINT_COLOR); 
				}
				else if((tp_dev.x>(lcddev.width-60)&&tp_dev.x<(lcddev.width-50+20))&&tp_dev.y<20)
				{
				LCD_Fill(lcddev.width-52,2,lcddev.width-50+20,18,ColorTab[j%5]); 
				POINT_COLOR=ColorTab[(j++)%5];
				colorTemp=POINT_COLOR;
				delay_ms(10);
				}

				else TP_Draw_Big_Point(tp_dev.x,tp_dev.y,POINT_COLOR);		//��ͼ	  			   
			}
		}else delay_ms(10);	//û�а������µ�ʱ�� 	    
		if(key==1)	//KEY_RIGHT����,��ִ��У׼����
		{

			LCD_Clear(WHITE);//����
		    TP_Adjust();  //��ĻУ׼ 
			TP_Save_Adjdata();	 
			DrawTestPage("����9:Touch����(��KEY0��У׼)  ");
			LCD_ShowString(lcddev.width-24,0,16,"RST",1);//��ʾ��������
			POINT_COLOR=colorTemp;
			LCD_Fill(lcddev.width-52,2,lcddev.width-50+20,18,POINT_COLOR); 
		}
		i++;
		if(i==30)
		{
			i=0;
			LED0=!LED0;
			//break;
		}
	}   
}

void Button_Test(void)
{

	TP_Init();
	KEY_Init();
	RGB_Init();
	DrawTestPage("����9:Touch����(��KEY0��У׼)  ");
	LCD_Fill(100,50,150,100,RED);
	LCD_Fill(100,150,150,200,BLUE);
	LCD_Fill(100,250,150,300,GREEN);
			while(1)
	{
	 	//key=KEY_Scan();
		tp_dev.scan(0); 		 
		if(tp_dev.sta&TP_PRES_DOWN)			//������������
		{	
			//printf("x= %d\r\n",tp_dev.x);
		 	if(tp_dev.x<150&&tp_dev.x>100&&tp_dev.y>50&&tp_dev.y<100)
			{	
				printf("hello \r\n");
				RED1=1;
				GREEN1=0;
				BLUE1=0;
				}
			 if(tp_dev.x<150&&tp_dev.x>100&&tp_dev.y>150&&tp_dev.y<200)
			 {	
				printf("world \r\n");
				RED1=1;
				GREEN1=1;
				BLUE1=0;
					
				}
			 if(tp_dev.x<150&&tp_dev.x>100&&tp_dev.y>250&&tp_dev.y<300)
			{	
				printf("lalala \r\n");
		
				RED1=1;
				GREEN1=0;
				BLUE1=1;
					
				}
				
				}
		  delay_us(200000);
		   
			}

		}
	



