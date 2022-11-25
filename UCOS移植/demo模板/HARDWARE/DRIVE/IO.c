#include "IO.h"
#include "stm32f10x_tim.h"

void GPIO_Drive_Init(void)
{
	GPIO_InitTypeDef  GPIO_InitStructure;
	//使能IO端口的时钟
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB|RCC_APB2Periph_GPIOC|RCC_APB2Periph_GPIOA,ENABLE);
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_ResetBits(GPIOB,GPIO_Pin_8);
	GPIO_Init(GPIOB,&GPIO_InitStructure);//初始化
	
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_1;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
  GPIO_ResetBits(GPIOB,GPIO_Pin_9);//PB4输出电平
	GPIO_Init(GPIOB,&GPIO_InitStructure);
	
	
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_5;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
	GPIO_ResetBits(GPIOB,GPIO_Pin_5);
	GPIO_Init(GPIOB,&GPIO_InitStructure);//初始化
	
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_12;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
  GPIO_ResetBits(GPIOA,GPIO_Pin_12);//PB4输出低电平
	GPIO_Init(GPIOA,&GPIO_InitStructure);


	
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_13;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
	GPIO_ResetBits(GPIOC,GPIO_Pin_13);
	GPIO_Init(GPIOC,&GPIO_InitStructure);//初始化

}

void TIM4_PWM_Init(u16 arr, u16 psc)
{
	GPIO_InitTypeDef 	GPIO_InitStructure;
	TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;
	TIM_OCInitTypeDef TIM_OCInitStructure;
	
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM4, ENABLE); //①使能定时器 2 时钟
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);

	
	//设置该引脚为复用输出功能,输出 TIM3 CH1 的 PWM 脉冲波形 GPIOA.1|2
		GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6; 
		GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP; //复用推挽输出
		GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
		GPIO_Init(GPIOB, &GPIO_InitStructure); 
	
		GPIO_InitStructure.GPIO_Pin = GPIO_Pin_7;
		GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP; 
		GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
		GPIO_Init(GPIOB, &GPIO_InitStructure); 
	
		//初始化 TIM2
		TIM_TimeBaseStructure.TIM_Period = arr; //设置在自动重装载周期值
		TIM_TimeBaseStructure.TIM_Prescaler =psc; //设置预分频值
		TIM_TimeBaseStructure.TIM_ClockDivision = 0; //设置时钟分割:TDTS = Tck_tim
		TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up; //TIM 向上计数模式
		TIM_TimeBaseInit(TIM4, &TIM_TimeBaseStructure); //③初始化 TIMx

		//初始化 TIM2 Channel2 PWM 模式
		TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM2; //选择 PWM 模式 2
		TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable; //比较输出使能
		TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High; //输出极性高
		
		TIM_OC1Init(TIM4, &TIM_OCInitStructure); //④初始化外设 TIM2 OC2
		TIM_OC1PreloadConfig(TIM4, TIM_OCPreload_Enable); //使能预装载寄存器
		
		
		TIM_OC2Init(TIM4, &TIM_OCInitStructure); //④初始化外设 TIM2 OC4
		TIM_OC2PreloadConfig(TIM4, TIM_OCPreload_Enable); //使能预装载寄存器
		
		
		TIM_Cmd(TIM4, ENABLE); //⑤使能 TIM2 

}
void CAR_Speed(u16 speed1,u16 speed2)
{
	TIM_SetCompare1(TIM4,speed2);
	TIM_SetCompare2(TIM4,speed1);
	
}



void Encoder_Init_TIM3(u16 arr,u16 psc)
{
	GPIO_InitTypeDef GPIO_InitStructure; //定义一个引脚初始化的结构体  
	TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;//定义一个定时器初始化的结构体
  TIM_ICInitTypeDef TIM_ICInitStructure; //定义一个定时器编码器模式初始化的结构体
	
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3, ENABLE); //使能TIM4时钟
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE); //使能CPIOB时钟
 
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6|GPIO_Pin_7;	//PB6、PB7
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING; //浮空输入
	GPIO_Init(GPIOA, &GPIO_InitStructure);	//根据GPIO_InitStructure的参数初始化GPIOB0

	TIM_TimeBaseStructure.TIM_Period = arr; //设定计数器自动重装值
	TIM_TimeBaseStructure.TIM_Prescaler = psc; // 预分频器 
	TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1; //选择时钟分频：不分频
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up; //TIM向上计数模式
	TIM_TimeBaseInit(TIM3, &TIM_TimeBaseStructure); //根据TIM_TimeBaseInitStruct的参数初始化定时器TIM4
	
	TIM_EncoderInterfaceConfig(TIM3, TIM_EncoderMode_TI12, TIM_ICPolarity_Rising, TIM_ICPolarity_Rising);//使用编码器模式3：CH1、CH2同时计数，四分频
	TIM_ICStructInit(&TIM_ICInitStructure); //把TIM_ICInitStruct 中的每一个参数按缺省值填入
	TIM_ICInitStructure.TIM_ICFilter = 10;  //设置滤波器长度
	TIM_ICInit(TIM3, &TIM_ICInitStructure); //根TIM_ICInitStructure参数初始化定时器TIM4编码器模式
	
	TIM_Cmd(TIM3, ENABLE); //使能定时器4
}

//读取编码器计数
int Read_Encoder_TIM3(void)
{
	int Encoder_TIM;
	Encoder_TIM=TIM3->CNT; //读取计数
	if(Encoder_TIM>0xefff)Encoder_TIM=Encoder_TIM-0xffff; //转化计数值为有方向的值，大于0正转，小于0反转。
	                                                      //TIM4->CNT范围为0-0xffff，初值为0。
	TIM3->CNT=0; //读取完后计数清零
	return Encoder_TIM; //返回值
}




void SR04_GPIO_Init(void){
	GPIO_InitTypeDef GPIO_InitStructure;
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA,ENABLE);
		
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_3;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOA,&GPIO_InitStructure);//初始化
	GPIO_SetBits(GPIOA,GPIO_Pin_3);
}

//定时器 2 通道 2 输入捕获配置
TIM_ICInitTypeDef TIM2_ICInitStructure;
void TIM2_Cap_init(u16 arr,u16 psc)
{
			GPIO_InitTypeDef GPIO_InitStructure;
			TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;
			NVIC_InitTypeDef NVIC_InitStructure;
	
			RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, ENABLE); //①使能 TIM2 时钟
			RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE); //①使能 GPIOA 时钟
			
	//初始化 GPIOA.0 ①
			GPIO_InitStructure.GPIO_Pin = GPIO_Pin_1; //PA0 设置 
			GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPD; //PA0 输入 
			GPIO_Init(GPIOA, &GPIO_InitStructure); //初始化 GPIOA.0
			GPIO_ResetBits(GPIOA,GPIO_Pin_1); //PA0 下拉
			
	
	//②初始化 TIM5 参数
			TIM_TimeBaseStructure.TIM_Period = arr; //设定计数器自动重装值
			TIM_TimeBaseStructure.TIM_Prescaler =psc; //预分频器 
			TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1; // TDTS = Tck_tim
			TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up; //TIM 向上计数模式
			TIM_TimeBaseInit(TIM2, &TIM_TimeBaseStructure); //初始化 TIMx
			
	//③初始化 TIM5 输入捕获通道 1
			TIM2_ICInitStructure.TIM_Channel = TIM_Channel_2; //选择输入端 IC1 映射到 TI1 上
			TIM2_ICInitStructure.TIM_ICPolarity = TIM_ICPolarity_Rising; //上升沿捕获
			TIM2_ICInitStructure.TIM_ICSelection = TIM_ICSelection_DirectTI; //映射到 TI1 上
			TIM2_ICInitStructure.TIM_ICPrescaler = TIM_ICPSC_DIV1; //配置输入分频,不分频
			TIM2_ICInitStructure.TIM_ICFilter = 0x00; //IC1F=0000 配置输入滤波器 不滤波
			TIM_ICInit(TIM2, &TIM2_ICInitStructure); //初始化 TIM5 输入捕获通道 1
			
			//⑤初始化 NVIC 中断优先级分组
			NVIC_InitStructure.NVIC_IRQChannel = TIM2_IRQn; //TIM3 中断
			NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 2; //先占优先级 2 级
			NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0; //从优先级 0 级
			NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE; //IRQ 通道被使能
			NVIC_Init(&NVIC_InitStructure); //初始化 NVIC
			
			TIM_ITConfig( TIM2,TIM_IT_Update|TIM_IT_CC2,ENABLE);//④允许更新中断捕获中断
			TIM_Cmd(TIM2,ENABLE ); //⑥使能定时器 5
}

u8 TIM5CH1_CAPTURE_STA=0; //输入捕获状态 
u16 TIM5CH1_CAPTURE_VAL;//输入捕获值
//⑤定时器 5 中断服务程序
void TIM2_IRQHandler(void)
{ 
if((TIM5CH1_CAPTURE_STA&0X80)==0)//还未成功捕获
{ 
		if (TIM_GetITStatus(TIM2, TIM_IT_Update) != RESET)
	{	 
				if(TIM5CH1_CAPTURE_STA&0X40) //已经捕获到高电平
					{
					if((TIM5CH1_CAPTURE_STA&0X3F)==0X3F)//高电平太长了
						{
								TIM5CH1_CAPTURE_STA|=0X80; //标记成功捕获了一次
								TIM5CH1_CAPTURE_VAL=0XFFFF;
						}
						else TIM5CH1_CAPTURE_STA++;

					}
	}

	if (TIM_GetITStatus(TIM2, TIM_IT_CC2) != RESET) //捕获 1 发生捕获事件
	{
			if(TIM5CH1_CAPTURE_STA&0X40) //捕获到一个下降沿
			{ 
				TIM5CH1_CAPTURE_STA|=0X80; //标记成功捕获到一次上升沿
				TIM5CH1_CAPTURE_VAL=TIM_GetCapture2(TIM2);
				TIM_OC2PolarityConfig(TIM2,TIM_ICPolarity_Rising); //设置为上升沿捕获
			}else //还未开始,第一次捕获上升沿
			{			
				TIM5CH1_CAPTURE_STA=0; //清空
				TIM5CH1_CAPTURE_VAL=0;
				TIM_SetCounter(TIM2,0);
				TIM5CH1_CAPTURE_STA|=0X40; //标记捕获到了上升沿
				TIM_OC2PolarityConfig(TIM2,TIM_ICPolarity_Falling); //设置为下降沿捕获
			} 
		} 
 
	} 
TIM_ClearITPendingBit(TIM2, TIM_IT_CC2|TIM_IT_Update); //清除中断标志位

}

int Read_distance(void){
		int time,distance;
			if(TIM5CH1_CAPTURE_STA&0X80)//成功捕获到了一次上升沿
		{
				time=TIM5CH1_CAPTURE_STA&0X3F;
				time*=65536;//溢出时间总和
				time+=TIM5CH1_CAPTURE_VAL;//得到总的高电平时间
				distance=time*0.033/2;
				//printf("HIGH:%d us cm=%.1f\r\n",temp,distance); //打印总的高点平时间
				TIM5CH1_CAPTURE_STA=0; //开启下一次捕获
		}
	return distance;


}




void TIM1_STEngine_init(u16 arr, u16 psc){ //舵机
	
		TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;
		GPIO_InitTypeDef GPIO_InitStructure;
		TIM_OCInitTypeDef TIM_OCInitStructure;

	
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM1, ENABLE);
		RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);
	

	//设置该引脚为复用输出功能,输出 TIM1 CH1 的 PWM 脉冲波形
		GPIO_InitStructure.GPIO_Pin = GPIO_Pin_8; //TIM1_CH1
		GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP; //复用功能输出
		GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
		GPIO_Init(GPIOA, &GPIO_InitStructure); //初始化 GPIO
	
		TIM_TimeBaseStructure.TIM_RepetitionCounter = 0;
	
    TIM_TimeBaseStructure.TIM_Period = arr;
    TIM_TimeBaseStructure.TIM_Prescaler = psc;
    TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1;
    TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
    TIM_TimeBaseInit(TIM1, &TIM_TimeBaseStructure);
			
		TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM2; //CH1 PWM2 模式
		TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable; //比较输出使能
		TIM_OCInitStructure.TIM_Pulse = 0; //设置待装入捕获比较寄存器的脉冲值
		TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_Low; //OC1 低电平有效
		TIM_OC1Init(TIM1, &TIM_OCInitStructure); //根据指定的参数初始化外设 TIMx
		
		TIM_OC1PreloadConfig(TIM1, TIM_OCPreload_Enable); //CH1 预装载使能
		
		TIM_ARRPreloadConfig(TIM1, ENABLE); //使能 TIMx 在 ARR 上的预装载寄存器
		TIM_CtrlPWMOutputs(TIM1,ENABLE); //MOE 主输出使能,高级定时器必须开启
		TIM_Cmd(TIM1,ENABLE);
	
}

//舵机旋转角度封装
void sg_singal(int angle)
{
		if(angle==75)
		{
		TIM_SetCompare1(TIM1,75);
		}
		if(angle==25)
		{
		TIM_SetCompare1(TIM1,25);
		}

		if(angle==125)
		{
		TIM_SetCompare1(TIM1,125);
		}


}




void GPIO_IFTracing_Init(void){   //红外
	GPIO_InitTypeDef  GPIO_InitStructure;
	//使能IO端口的时钟
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB,ENABLE);
		
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU ;  //设置默认上拉输入
	GPIO_ResetBits(GPIOB,GPIO_Pin_10);
	GPIO_Init(GPIOB,&GPIO_InitStructure);//初始化
	
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_11;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU ;   
	GPIO_ResetBits(GPIOB,GPIO_Pin_11);
	GPIO_Init(GPIOB,&GPIO_InitStructure);//初始化

}



void CAR_LRSpeed(u16 Lspeed,u16 Rspeed){
	TIM_SetCompare2(TIM2,Lspeed);
	TIM_SetCompare3(TIM2,Rspeed);
}
void CAR_Tracing(void){

		if(HW_PIN10_Left==0&&HW_PIN11_Right==1){
				car_right();
			
		}	
		if(HW_PIN10_Left==1&&HW_PIN11_Right==0){
		car_left();
		
		}
		if(HW_PIN10_Left==0&&HW_PIN11_Right==0){
		car_go();
		
		}
		if(HW_PIN10_Left==1&&HW_PIN11_Right==1){
				car_stop();	
		}

}

void car_go(void)
{
	in8=0;
	in9=1;
	in5=1;
	in6=0;	 
}
void car_back(void){
	in8=1;
	in9=0;
	in5=0;
	in6=1;
}
void car_stop(void)
{
	in8=0;
	in9=0;
	in5=0;
	in6=0;
}

void car_left(void)
{
	in8=1;
	in9=0;
	in5=1;
	in6=0;
	TIM_SetCompare1(TIM3,1500);
}
	

void car_right(void)
{
	in8=0;
	in9=1;
	in5=0;
	in6=1;
	TIM_SetCompare2(TIM3,1500);
	
}