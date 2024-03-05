/*******************************************************************************
 * @File        : LL_UART_DMA.c
 * @Created_on  : 4 March 2024
 * @Author      : Ahmet Citak
 * @Email       : ahmet.cittak@gmail.com
 * @About       : Candidate Embedded Software Engineer

 ******************************************************************************/

/**************************** Defining Header *********************************/
#include "LL_UART_DMA.h"

/********************** Defining Private Variables ****************************/

uint8_t	Rx_Buffer[Rx_Buffer_Size];
uint8_t Rx_Error;
volatile uint8_t Rx_Cmplt = 0;

uint8_t Recieved_Data[Rx_Buffer_Size];

/************************ Defining Private Externs ****************************/


/*******************************************************************************
 * @Function_Name   : LL_UART_DMA_RX_Config
 * @Function_Input  : USARTx, DMAx, Periphs, Stream, IRQn, DstAddress, Rx_Buffer_Size
 * @Function_Output : None
 * @Function_Brief  : Config function for LL Uart Rx. Use this function in the
 * 					  MX_USART2_UART_Init() or use after UART and DMA Init func.
 ******************************************************************************/

void LL_UART_DMA_RX_Config(USART_TypeDef *USARTx ,DMA_TypeDef *DMAx, uint32_t Periphs , uint32_t Stream, IRQn_Type IRQn, uint8_t DstAddress[], uint8_t Rx_Buffer_Size) {

	// Enable DMA Clock.
	LL_AHB1_GRP1_EnableClock(Periphs);

	/*ENABLE DMA NVIC PRIOTIRY*/
	NVIC_SetPriority(IRQn, 0);

	/*ENABLE DMA NVIC IRQN*/
	NVIC_EnableIRQ(IRQn);

	/*CONFIGURE DMA TRANSFER*/
	LL_DMA_ConfigTransfer(	 DMAx, Stream,
			  	  	  	  	 LL_DMA_DIRECTION_PERIPH_TO_MEMORY	|
	  		  	  	  	  	 LL_DMA_PRIORITY_HIGH			  	|
	  						 LL_DMA_MODE_CIRCULAR				|
	  						 LL_DMA_PERIPH_NOINCREMENT			|
	  						 LL_DMA_MEMORY_INCREMENT			|
	  						 LL_DMA_PDATAALIGN_BYTE				|
	  						 LL_DMA_MDATAALIGN_BYTE				);

	/*Configure transfer address and direction*/
	LL_DMA_ConfigAddresses(	 DMAx,
							 Stream,
							 LL_USART_DMA_GetRegAddr(USARTx),
	  		  	  	  	  	 (uint32_t)DstAddress,
							 LL_DMA_DIRECTION_PERIPH_TO_MEMORY);

	/* Set data lenght */
	LL_DMA_SetDataLength(DMAx, Stream, Rx_Buffer_Size);

	/* Enable DMA Complate, IDLE and Error Interrupt. */
	LL_DMA_EnableIT_TC(DMAx, Stream);
	LL_DMA_EnableIT_TE(DMAx, Stream);

	LL_USART_EnableIT_IDLE(USARTx);
}

/*******************************************************************************
 * @Function_Name   : LL_UART_DMA_RX_Start
 * @Function_Input  : USARTx, DMAx, Stream
 * @Function_Output : None
 * @Function_Brief  : Start func. Uart Rx LL
 ******************************************************************************/

void LL_UART_DMA_RX_Start(USART_TypeDef *USARTx, DMA_TypeDef *DMAx, uint32_t Stream){

	LL_USART_EnableDMAReq_RX(USARTx);
	LL_DMA_EnableStream(DMAx, Stream);

}

/*******************************************************************************
 * @Function_Name   : LL_UART_DMA_RX_Stop
 * @Function_Input  : USARTx, DMAx, Stream
 * @Function_Output : None
 * @Function_Brief  : Stop func. Uart Rx LL
 ******************************************************************************/

void LL_UART_DMA_RX_Stop(USART_TypeDef *USARTx, DMA_TypeDef *DMAx, uint32_t Stream) {

	LL_USART_DisableDMAReq_RX(USARTx);
	LL_DMA_DisableStream(DMAx, Stream);

}

/*******************************************************************************
 * @Function_Name   : LL_UART_DMA_RX_Interrupt
 * @Function_Input  : DMAx
 * @Function_Output : None
 * @Function_Brief  : Interrupt func.
 ******************************************************************************/

void LL_UART_DMA_RX_Interrupt(DMA_TypeDef *DMAx){

	if(LL_DMA_IsActiveFlag_TC5(DMAx)){

			LL_DMA_ClearFlag_TC5(DMAx);

			Rx_Cmplt = 1;
	}

	else if(LL_DMA_IsActiveFlag_TE5(DMAx)){

			LL_DMA_ClearFlag_TE5(DMAx);
			Rx_Error = 1;
	}

}



