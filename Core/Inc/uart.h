/*
 * uart.h
 *
 *  Created on: 2018. 7. 22.
 *      Author: Jeong
 */

#ifndef UART_H_
#define UART_H_

#define hWifi    huart3
#define hMonitor huart1

#define QUEUE_BUFFER_LENGTH 0XFFFF

typedef struct
{
    int head, tail, data;
    uint8_t Buffer[QUEUE_BUFFER_LENGTH];
}UARTQUEUE, *pUARTQUEUE;

extern UART_HandleTypeDef huart3;
extern UART_HandleTypeDef huart1;

extern UARTQUEUE WifiQueue;
extern UARTQUEUE MonitorQueue;

void InitUartQueue(pUARTQUEUE pQueue);
void PutDataToUartQueue(UART_HandleTypeDef *huart, uint8_t data);
void GetDataFromUartQueue(UART_HandleTypeDef *huart);

#endif /* UART_H_ */
