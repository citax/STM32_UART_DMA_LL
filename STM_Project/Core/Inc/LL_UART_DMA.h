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
typedef enum {

	Rx_Buffer_Size = 10

} Buffer_Size_Enum_Typedef;


/********************** Defining Function Protypes ****************************/

void LL_UART_DMA_RX_Config(USART_TypeDef *USARTx ,DMA_TypeDef *DMAx, uint32_t Periphs,
		uint32_t Stream, IRQn_Type IRQn, uint8_t DstAddress[], uint8_t Rx_Buffer_Size);

void LL_UART_DMA_TX_Config(USART_TypeDef *USARTx ,DMA_TypeDef *DMAx, uint32_t Periphs ,
		uint32_t Stream, IRQn_Type IRQn, uint8_t SourceAddress[], uint8_t Tx_Buffer_Size);

void LL_UART_DMA_RX_Start(USART_TypeDef *USARTx, DMA_TypeDef *DMAx, uint32_t Stream);
void LL_UART_DMA_TX_Start(USART_TypeDef *USARTx, DMA_TypeDef *DMAx, uint32_t Stream);
void LL_UART_DMA_RX_Stop(USART_TypeDef *USARTx, DMA_TypeDef *DMAx, uint32_t Stream);
void LL_UART_DMA_TX_Stop(USART_TypeDef *USARTx, DMA_TypeDef *DMAx, uint32_t Stream);
void LL_UART_DMA_RX_Interrupt(DMA_TypeDef *DMAx);
void LL_UART_DMA_TX_Interrupt(DMA_TypeDef *DMAx);
void LL_UART_DMA_RX_IDLE_Interrupt(USART_TypeDef *USARTx, DMA_TypeDef *DMAx, uint32_t Stream,
		 uint8_t Recieved_Data[], uint8_t Rx_Buffer[]);


#endif /* INC_LL_UART_DMA_H_ */
