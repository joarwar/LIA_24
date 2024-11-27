#ifndef __UART_H__
#define __UART_H__

void uart_Init(void);

void uart_PrintString(char * str);
void uart_PrintFloat(float value);
void uart_PrintInt(unsigned int value, unsigned char base);


#endif 