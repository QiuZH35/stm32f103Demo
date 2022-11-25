#include "led.h"
#include "delay.h"
#include "sys.h"
#include "usart.h"
#include "IO.h"
#include "includes.h"


//START 任务
//设置任务优先级
#define START_TASK_PRIO			10  ///开始任务的优先级为最低
//设置任务堆栈大小
#define START_STK_SIZE			128
//任务任务堆栈
OS_STK START_TASK_STK[START_STK_SIZE];
//任务函数
void start_task(void *pdata);

//LED0任务
//设置任务优先级
#define LED0_TASK_PRIO			7
//设置任务堆栈大小
#define LED0_STK_SIZE			64
//任务堆栈
OS_STK LED0_TASK_STK[LED0_STK_SIZE];
//任务函数
void led0_task(void *pdata);

//LED1任务
//设置任务优先级
#define LED1_TASK_PRIO			6
//设置任务堆栈大小
#define LED1_STK_SIZE			64
//任务堆栈
OS_STK LED1_TASK_STK[LED1_STK_SIZE];
//任务函数
void led1_task(void *pdata);

//浮点测试任务
#define FLOAT_TASK_PRIO			5
//设置任务堆栈大小
#define FLOAT_STK_SIZE			128
//任务堆栈
//如果任务中使用printf来打印浮点数据的话一点要8字节对齐
__align(8) OS_STK FLOAT_TASK_STK[FLOAT_STK_SIZE]; 
//任务函数
void float_task(void *pdata);


#define SR04_TASK_PRIO 4
#define SR04_STK_SIZE 64
OS_STK SR04_TASK_STK[SR04_STK_SIZE];
void sr04_task(void * pdata);


int left_ps=0,right_ps=0;

int main(void)
{
	delay_init();       //延时初始化
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2); //中断分组配置
	uart_init(9600);    //串口波特率设置
	GPIO_Drive_Init();
	SR04_GPIO_Init();
	TIM4_PWM_Init(7199,0); //72/7200
	GPIO_IFTracing_Init();  //红外初始化
	Encoder_Init_TIM3(0xffff,0);
	TIM1_STEngine_init(999,1439);
	TIM2_Cap_init(0XFFFF,72-1); //以 1Mhz 的频率计数

	
	
	OSInit();  		//UCOS初始化
	OSTaskCreate(sr04_task,(void *)0,(OS_STK*)&SR04_TASK_STK[SR04_STK_SIZE-1],SR04_TASK_PRIO);
	OSStart(); 	//开始任务

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
 
 
//浮点测试任务
void float_task(void *pdata)
{
	OS_CPU_SR cpu_sr=0;
	static float float_num=0.01;
	while(1)
	{
		float_num+=0.01f;
		OS_ENTER_CRITICAL();	//进入临界区(关闭中断)
		printf("float_num的值为: %.4f\r\n",float_num); //串口打印结果
		OS_EXIT_CRITICAL();		//退出临界区(开中断)
		delay_ms(500);
	}
}

