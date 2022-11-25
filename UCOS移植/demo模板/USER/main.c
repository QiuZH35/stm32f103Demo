#include "led.h"
#include "delay.h"
#include "sys.h"
#include "usart.h"
#include "IO.h"
#include "includes.h"


//START ����
//�����������ȼ�
#define START_TASK_PRIO			10  ///��ʼ��������ȼ�Ϊ���
//���������ջ��С
#define START_STK_SIZE			128
//���������ջ
OS_STK START_TASK_STK[START_STK_SIZE];
//������
void start_task(void *pdata);

//LED0����
//�����������ȼ�
#define LED0_TASK_PRIO			7
//���������ջ��С
#define LED0_STK_SIZE			64
//�����ջ
OS_STK LED0_TASK_STK[LED0_STK_SIZE];
//������
void led0_task(void *pdata);

//LED1����
//�����������ȼ�
#define LED1_TASK_PRIO			6
//���������ջ��С
#define LED1_STK_SIZE			64
//�����ջ
OS_STK LED1_TASK_STK[LED1_STK_SIZE];
//������
void led1_task(void *pdata);

//�����������
#define FLOAT_TASK_PRIO			5
//���������ջ��С
#define FLOAT_STK_SIZE			128
//�����ջ
//���������ʹ��printf����ӡ�������ݵĻ�һ��Ҫ8�ֽڶ���
__align(8) OS_STK FLOAT_TASK_STK[FLOAT_STK_SIZE]; 
//������
void float_task(void *pdata);


#define SR04_TASK_PRIO 4
#define SR04_STK_SIZE 64
OS_STK SR04_TASK_STK[SR04_STK_SIZE];
void sr04_task(void * pdata);


int left_ps=0,right_ps=0;

int main(void)
{
	delay_init();       //��ʱ��ʼ��
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2); //�жϷ�������
	uart_init(9600);    //���ڲ���������
	GPIO_Drive_Init();
	SR04_GPIO_Init();
	TIM4_PWM_Init(7199,0); //72/7200
	GPIO_IFTracing_Init();  //�����ʼ��
	Encoder_Init_TIM3(0xffff,0);
	TIM1_STEngine_init(999,1439);
	TIM2_Cap_init(0XFFFF,72-1); //�� 1Mhz ��Ƶ�ʼ���

	
	
	OSInit();  		//UCOS��ʼ��
	OSTaskCreate(sr04_task,(void *)0,(OS_STK*)&SR04_TASK_STK[SR04_STK_SIZE-1],SR04_TASK_PRIO);
	OSStart(); 	//��ʼ����

}


 
void sr04_task(void *pdata){
	while(1)
	{
			delay_ms(200);
			SR04=0;
			delay_us(10);
			SR04=1;
 
	 }
		
}
 
 
//�����������
void float_task(void *pdata)
{
	OS_CPU_SR cpu_sr=0;
	static float float_num=0.01;
	while(1)
	{
		float_num+=0.01f;
		OS_ENTER_CRITICAL();	//�����ٽ���(�ر��ж�)
		printf("float_num��ֵΪ: %.4f\r\n",float_num); //���ڴ�ӡ���
		OS_EXIT_CRITICAL();		//�˳��ٽ���(���ж�)
		delay_ms(500);
	}
}

