/****************************************************************************************************
//=========================================��Դ����================================================//
//     LCDģ��                STM32��Ƭ��
//      VCC          ��        DC5V/3.3V      //��Դ
//      GND          ��          GND          //��Դ��
//=======================================Һ���������߽���==========================================//
//��ģ��Ĭ��������������ΪSPI����
//     LCDģ��                STM32��Ƭ��    
//    SDI(MOSI)      ��          PB15         //Һ����SPI��������д�ź�
//    SDO(MISO)      ��          PB14         //Һ����SPI�������ݶ��źţ��������Ҫ�������Բ�����
//=======================================Һ���������߽���==========================================//
//     LCDģ�� 					      STM32��Ƭ�� 
//       LED         ��          PB9          //Һ������������źţ��������Ҫ���ƣ���5V��3.3V
//       SCK         ��          PB13         //Һ����SPI����ʱ���ź�
//      DC/RS        ��          PB10         //Һ��������/��������ź�
//       RST         ��          PB12         //Һ������λ�����ź�
//       CS          ��          PB11         //Һ����Ƭѡ�����ź�
//=========================================������������=========================================//
//���ģ�鲻���������ܻ��ߴ��д������ܣ����ǲ���Ҫ�������ܣ�����Ҫ���д���������
//	   LCDģ��                STM32��Ƭ�� 
//      T_IRQ        ��          PC10         //�����������ж��ź�
//      T_DO         ��          PC2          //������SPI���߶��ź�
//      T_DIN        ��          PC3          //������SPI����д�ź�
//      T_CS         ��          PC13         //������Ƭѡ�����ź�
//      T_CLK        ��          PC0          //������SPI����ʱ���ź�
������Ҫ���ط��棺1����Ӣ��������ʾ������������ɫ������ɫ�������С��16λ��24λ��32λ�������ȡģ
                  2��ͼƬ��ʾ�����ȡģ��
									3���������ܵ�ʹ�ã����ĳһ��ť��������Ӧ���������ܵ���дһ������
**************************************************************************************************/	
#include "delay.h"
#include "sys.h"
#include "lcd.h"
#include "touch.h"
#include "gui.h"
#include "usart.h"
#include "touch.h"
#include "key.h" 
#include "led.h"
#include "pic.h"
#include "rgb.h"
//#include "FONT.H"
u16 ColorTab[5]={RED,GREEN,BLUE,YELLOW,BRED};//������ɫ����
void DrawTestPage(u8 *str)
{
//���ƹ̶�������
LCD_Clear(WHITE);
LCD_Fill(0,0,lcddev.width,20,BLUE);
//���ƹ̶���down
LCD_Fill(0,lcddev.height-20,lcddev.width,lcddev.height,BLUE);
POINT_COLOR=WHITE;
Gui_StrCenter(0,2,WHITE,BLUE,"��������ʹ��",16,1);//������ʾ
Gui_StrCenter(0,lcddev.height-18,WHITE,BLUE,"�ú�ѧϰ����������!",16,1);//������ʾ

}


//���������ܿ��Կ����ڴ�ӡ����������꣬������˼�����ť������һ��RGB������С���ݣ����������rgb.c
int main(void)
{	
	SystemInit();//��ʼ��RCC ����ϵͳ��ƵΪ72MHZ
  uart_init(115200);
	delay_init();
	LCD_Init();	   //Һ������ʼ��
  DrawTestPage("");
	u8 key;
	TP_Init();
	KEY_Init();
	RGB_Init();
	LCD_Fill(100,50,150,100,RED);
	Show_Str(108,65,BLUE,YELLOW,"LED1",16,1);
	LCD_Fill(100,150,150,200,BLUE);
	Show_Str(108,165,	RED,YELLOW,"LED2",16,1);
	LCD_Fill(100,250,150,300,GREEN);
	Show_Str(108,265,BLUE,YELLOW,"LED3",16,1);
	while(1)
	{
	key=KEY_Scan();
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

	
	
