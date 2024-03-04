/*******************************************************************************
 * @File        : LL_UART_DMA.h
 * @Created_on  : 4 March 2024
 * @Author      : Ahmet Citak
 * @Email       : ahmet.cittak@gmail.com
 * @About       : Candidate Embedded Software Engineer

 ******************************************************************************/
#ifndef INC_LL_UART_DMA_H_
#define INC_LL_UART_DMA_H_


#include "main.h"
#include <stdint.h>

/************************ Defining Constants **********************************/


/********************** Defining Enums and Structs ****************************/


/********************** Defining Function Protypes ****************************/


void LL_UART_RX_Config(USART_TypeDef *USARTx ,DMA_TypeDef *DMAx, uint32_t Periphs,
		uint32_t Stream, IRQn_Type IRQn, uint8_t DstAddress[], uint8_t Rx_Buffer_Size);

void LL_UART_RX_Start(USART_TypeDef *USARTx, DMA_TypeDef *DMAx, uint32_t Stream);
void LL_UART_RX_Stop(USART_TypeDef *USARTx, DMA_TypeDef *DMAx, uint32_t Stream);



#endif /* INC_LL_UART_DMA_H_ */
