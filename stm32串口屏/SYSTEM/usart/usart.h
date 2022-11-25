#ifndef __USART_H
#define __USART_H
#include "stdio.h"	
#include "sys.h" 


extern u8 USART_RX_BUF[64];     //���ջ���,���63���ֽ�.ĩ�ֽ�Ϊ���з� 
extern u8 USART_RX_STA;         //����״̬���	

void uart_init(u32 bound);
void UartSend(char * databuf) ;
void CheckBusy(void)  ;
#endif



