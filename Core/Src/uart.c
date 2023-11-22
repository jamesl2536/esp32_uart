/*
 * uart.c
 *
 *  Created on: 2018. 7. 22.
 *      Author: Jeong
 */

#include "stm32h7xx_hal.h"

#include "uart.h"

UARTQUEUE WifiQueue;
UARTQUEUE MonitorQueue;

void InitUartQueue(pUARTQUEUE pQueue)
{
  pQueue->data = pQueue->head = pQueue->tail = 0;
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    pUARTQUEUE pQueue;
    pQueue = (huart->Instance == USART3 ? &WifiQueue:&MonitorQueue);    //if usart2 WifiQueue else MonitorQueue
    pQueue->head++;                                                     //head++
    if (pQueue->head == QUEUE_BUFFER_LENGTH) pQueue->head = 0;          //if queue end head = 0
    pQueue->data++;                                                     //data++
    if (pQueue->data == QUEUE_BUFFER_LENGTH)                            //if queue is full
        GetDataFromUartQueue(huart);
    HAL_UART_Receive_DMA(huart, pQueue->Buffer + pQueue->head, 1);       //set interrupt again
}

void PutDataToUartQueue(UART_HandleTypeDef *huart, uint8_t data)
{
    pUARTQUEUE pQueue = (huart->Instance == USART3 ? &WifiQueue:&MonitorQueue);
    if (pQueue->data == QUEUE_BUFFER_LENGTH)
        GetDataFromUartQueue(huart);
    pQueue->Buffer[pQueue->head++] = data;
    if (pQueue->head == QUEUE_BUFFER_LENGTH) pQueue->head = 0;
    pQueue->data++;
}


void GetDataFromUartQueue(UART_HandleTypeDef *huart)
{
  UART_HandleTypeDef *dst = (huart->Instance == USART3 ? &hMonitor:&hWifi);
  pUARTQUEUE pQueue = (huart->Instance == USART3 ? &WifiQueue:&MonitorQueue);
//  HAL_UART_Transmit(dst, pQueue->Buffer + pQueue->tail, 10, 10000);
  if (HAL_UART_Transmit(dst, pQueue->Buffer + pQueue->tail, 1, 3000) != HAL_OK)
     {
         Error_Handler();
     }
  pQueue->tail++;
  if (pQueue->tail == QUEUE_BUFFER_LENGTH)
    pQueue->tail = 0;
  pQueue->data--;
  HAL_Delay(1);
}
