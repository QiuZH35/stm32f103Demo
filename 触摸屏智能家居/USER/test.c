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
//#include "FONT.h"   //�����õ�����������
#include "dht11.h"




u16 ColorTab[5]={RED,GREEN,BLUE,YELLOW,BRED};//������ɫ����


void DrawTestPage(u8 *str)
{
//���ƹ̶���up
LCD_Clear(BLACK);
LCD_Fill(0,0,lcddev.width,30,BLUE);
//���ƹ̶���down
LCD_Fill(0,lcddev.height-20,lcddev.width,lcddev.height,BLUE);
POINT_COLOR=WHITE;
Gui_StrCenter(0,1,WHITE,BLUE,"���ܼҾӼ��ϵͳ",24,1);//������ʾ
Gui_StrCenter(0,lcddev.height-18,WHITE,BLUE,"���������˽��",16,1);//������ʾ
//���Ʋ�������
//LCD_Fill(0,20,lcddev.width,lcddev.height-20,WHITE);
}



void test1(void)
{DrawTestPage("");
	Show_Str(50,100,WHITE,BLUE,"��ϵͳ�������ܼҾӼ��",24,1);
	LCD_Fill(240,380,300,450,BLUE);//8
	Show_Str(250,400,WHITE,BLUE,"����",24,1);
	
}

void test2(void)
{DrawTestPage("");
	Show_Str(80,100,WHITE,BLUE,"���ڻ������ݹ��ߣ�",24,1);
	Show_Str(50,130,WHITE,BLUE,"�Զ�����",24,1);
	LCD_Fill(240,380,300,450,BLUE);//8
	Show_Str(250,400,WHITE,BLUE,"����",24,1);
	
}
void test5(void)
{DrawTestPage("");
	Show_Str(90,70,WHITE,BLUE,"ʵʱ���",24,1);
	Show_Str(40,130,WHITE,BLUE,"�¶ȣ�",24,1);
	Show_Str(40,160,WHITE,BLUE,"ʪ�ȣ�",24,1);
	LCD_Fill(240,380,300,450,BLUE);//8
	Show_Str(250,400,WHITE,BLUE,"����",24,1);
	
}
void test6(void)
{DrawTestPage("");
	Show_Str(100,50,WHITE,BLUE,"�ƹ����",24,1);
	Show_Str(100,130,WHITE,BLUE,"������",32,0);
	Show_Str(100,200,WHITE,BLUE,"���ҵ�",32,0);
	Show_Str(100,270,WHITE,BLUE,"������",32,0);	
	LCD_Fill(240,380,300,450,BLUE);//8
	Show_Str(250,400,WHITE,BLUE,"����",24,1);
	
}
void test7(void)
{DrawTestPage("");
	Show_Str(80,100,WHITE,BLUE,"�����ַ",24,1);
	Show_Str(80,150,WHITE,BLUE,"������ӭ��",24,1);
	LCD_Fill(240,380,300,450,BLUE);//8
	Show_Str(250,400,WHITE,BLUE,"����",24,1);
	
}

void test8(void)
{DrawTestPage("");
	Show_Str(80,100,WHITE,BLUE,"��ӭ����˽��",24,1);
	LCD_Fill(240,380,300,450,BLUE);//8
	Show_Str(250,400,WHITE,BLUE,"����",24,1);
}
void Button_Test(void)
{
	u8 key;
	u8 temperature;  	    
	u8 humidity;    
	TP_Init();
	KEY_Init();
	RGB_Init();
	DHT11_Init();
	DrawTestPage("");
	LCD_Fill(0,80,120,150,BLUE);//1
	Show_Str(10,100,WHITE,BLUE,"ʵʱ���",24,1);
	LCD_Fill(180,80,300,150,BLUE);//2
	Show_Str(190,100,WHITE,BLUE,"���ܱ���",24,1);
	LCD_Fill(0,180,120,250,BLUE);//3
	Show_Str(10,200,WHITE,BLUE,"�豸����",24,1);
	LCD_Fill(180,180,300,250,BLUE);//4
	Show_Str(190,200,WHITE,BLUE,"ʹ��˵��",24,1);
	LCD_Fill(0,280,120,350,BLUE);//5
	Show_Str(10,300,WHITE,BLUE,"����״��",24,1);
	LCD_Fill(180,280,300,350,BLUE);//6
	Show_Str(190,300,WHITE,BLUE,"�ƹ����",24,1);
	LCD_Fill(0,380,120,450,BLUE);//7
	Show_Str(10,400,WHITE,BLUE,"��˾��ַ",24,1);
	LCD_Fill(180,380,300,450,BLUE);//8
	Show_Str(190,400,WHITE,BLUE,"��ϵ��ʽ",24,1);

while(1)
	{
	 	key=KEY_Scan();
		tp_dev.scan(0); 		 
		if(tp_dev.sta&TP_PRES_DOWN)			//������������
		{	
			
		 	if(tp_dev.x<120&&tp_dev.x>0&&tp_dev.y>80&&tp_dev.y<150)
			{	
			 	printf("1 \r\n");
			  test1();
				while(1){
	    	key=KEY_Scan();
		    tp_dev.scan(0); 	
			if(tp_dev.sta&TP_PRES_DOWN)			//������������
		{	
			if(tp_dev.x<300&&tp_dev.x>180&&tp_dev.y>380&&tp_dev.y<450)
			{
				
       NVIC_SystemReset();
				}

			}
				}
			}
	   if(tp_dev.x<300&&tp_dev.x>180&&tp_dev.y>80&&tp_dev.y<150)
			{	
				test2();
			while(1){
	    	key=KEY_Scan();
		    tp_dev.scan(0); 	
			if(tp_dev.sta&TP_PRES_DOWN)			//������������
		{	
			if(tp_dev.x<300&&tp_dev.x>180&&tp_dev.y>380&&tp_dev.y<450)
			{
				
       NVIC_SystemReset();
				}

			}
				}
				}
				
		if(tp_dev.x<120&&tp_dev.x>0&&tp_dev.y>180&&tp_dev.y<250)
			{	
				printf("3 \r\n");
				
			
				}
			   if(tp_dev.x<300&&tp_dev.x>180&&tp_dev.y>180&&tp_dev.y<250)
			{	
				printf("4 \r\n");
			
				}
			
			if(tp_dev.x<120&&tp_dev.x>0&&tp_dev.y>280&&tp_dev.y<350)
			{	
				printf("5 \r\n");
				
				test5();
				while(1){
	    	key=KEY_Scan();
		    tp_dev.scan(0); 	
//				POINT_COLOR=BLUE;		//��������Ϊ��ɫ 	
//				DHT11_Read_Data(&temperature,&humidity);	//��ȡ��ʪ��ֵ
//				printf("temperature: %d\r\n",temperature);
//        printf("humidity: %d\r\n",humidity);			
//				LCD_ShowNum(80,160,(int)temperature,24,1);
//	      LCD_ShowNum(40,180,(int)humidity,24,1);
        delay_ms(30);
			if(tp_dev.sta&TP_PRES_DOWN)			//������������
		{	
			if(tp_dev.x<300&&tp_dev.x>180&&tp_dev.y>380&&tp_dev.y<450)
			{
				
       NVIC_SystemReset();
				}

			}
				}
			
				}
				
			   if(tp_dev.x<300&&tp_dev.x>180&&tp_dev.y>280&&tp_dev.y<350)
			{	
				printf("6 \r\n");
				test6();
				while(1){
	    	key=KEY_Scan();
		    tp_dev.scan(0); 	
			if(tp_dev.sta&TP_PRES_DOWN)			//������������
		{	
//			printf("x= %d\r\n",tp_dev.x);
//			printf("y= %d\r\n",tp_dev.y);
//			delay_ms(3000);
			if(tp_dev.x<300&&tp_dev.x>180&&tp_dev.y>380&&tp_dev.y<450)
			{
				
       NVIC_SystemReset();
				}
			if(tp_dev.x<200&&tp_dev.x>100&&tp_dev.y>130&&tp_dev.y<150)
			{
				
       printf("����");
				}
			if(tp_dev.x<200&&tp_dev.x>100&&tp_dev.y>200&&tp_dev.y<250)
			{
				printf("����");
       
				}
			if(tp_dev.x<200&&tp_dev.x>100&&tp_dev.y>300&&tp_dev.y<350)
			{
				
      printf("����");
				}

			}
				}
			
				}
			   if(tp_dev.x<120&&tp_dev.x>0&&tp_dev.y>380&&tp_dev.y<450)
			{	
				printf("7 \r\n");
				test7();
				while(1){
	    	key=KEY_Scan();
		    tp_dev.scan(0); 	
			if(tp_dev.sta&TP_PRES_DOWN)			//������������
		{	
			if(tp_dev.x<300&&tp_dev.x>180&&tp_dev.y>380&&tp_dev.y<450)
			{
				
       NVIC_SystemReset();
				}

			}
				}
				}
			   if(tp_dev.x<300&&tp_dev.x>180&&tp_dev.y>380&&tp_dev.y<450)
			{
				printf("8 \r\n");
				test8();
				while(1){
	    	key=KEY_Scan();
		    tp_dev.scan(0); 	
			if(tp_dev.sta&TP_PRES_DOWN)			//������������
		{	
			if(tp_dev.x<300&&tp_dev.x>180&&tp_dev.y>380&&tp_dev.y<450)
			{
				
       NVIC_SystemReset();
				}

			}
				}
				}
				}
		  delay_us(200000);
		   
			}
		}


