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

uint8_t Pos = 0, Old_Pos = 0;

/************************ Defining Private Externs ****************************/


/*******************************************************************************
 * @Function_Name   : LL_UART_DMA_RX_Config
 * @Function_Input  : USARTx, DMAx, Periphs, Stream, IRQn, DstAddress, Rx_Buffer_Size
 * @Function_Output : None
 * @Function_Brief  : Config function for LL Uart Rx. Use this function in the
 * 					  MX_USART2_UART_Init() or use after UART and DMA Init func.
 ******************************************************************************/

void LL_UART_DMA_RX_Config(USART_TypeDef *USARTx ,DMA_TypeDef *DMAx, uint32_t Periphs ,
		uint32_t Stream, IRQn_Type IRQn, uint8_t DstAddress[], uint8_t Rx_Buffer_Size) {

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
	/* Disable Half Transfer Complate Interrupt*/
	LL_DMA_DisableIT_HT(DMAx, Stream);

	/* Enable to UART IDLE*/
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
 * @Function_Brief  : DMA Interrupt func.
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

/*******************************************************************************
 * @Function_Name   : LL_UART_DMA_RX_IDLE_Interrupt
 * @Function_Input  : USARTx, DMAx, Stream, Recieved_Data_Size, Recieved_Data[], Rx_Buffer[]
 * @Function_Output : None
 * @Function_Brief  : IDLE Interrupt func.
 ******************************************************************************/

void LL_UART_DMA_RX_IDLE_Interrupt(USART_TypeDef *USARTx, DMA_TypeDef *DMAx, uint32_t Stream,
		uint8_t Recieved_Data[], uint8_t Rx_Buffer[]){

	if(LL_USART_IsActiveFlag_IDLE(USARTx))
	{
		for(uint8_t i=0; i<Rx_Buffer_Size; i++) {
			Recieved_Data[i] = 0;	// Reset to Recieved_Data
		}

		LL_USART_ClearFlag_IDLE(USARTx); // Clear flag

		Pos = Rx_Buffer_Size - LL_DMA_GetDataLength(DMAx, Stream);

		if(LL_DMA_GetDataLength(DMAx, Stream) == Rx_Buffer_Size && Pos == 0 && Old_Pos == 0) { // Data Size = Rx_Buffer_Size

			for(uint8_t i=0; i<Rx_Buffer_Size; i++){

				Recieved_Data[i] = Rx_Buffer[i];

			}

		}

		else if (Pos != Old_Pos){

			if(Pos > Old_Pos){

				//        * [   0   ]
				//        * [   1   ] <- old_pos |---------------------------------------------------|
				//        * [   2   ]            |                                    				 |
				//        * [   3   ]            | Single block (Recieved_Data_Size = pos - old_pos) |
				//        * [   4   ]            |                                    				 |
				//        * [   5   ]            |---------------------------------------------------|
				//        * [   6   ] <- pos
				//        * [   7   ]
				//        * [ N - 1 ]
				uint8_t a = 0;
				for(uint8_t i=Old_Pos; i<Pos; i++){

					Recieved_Data[0+a] = Rx_Buffer[i];
					a++;

				}
			}
			else{
				//		             * [   0   ]            |---------------------------------|
				//		             * [   1   ]            | Second block (len = pos)        |
				//		             * [   2   ]            |---------------------------------|
				//		             * [   3   ] <- pos
				//		             * [   4   ] <- old_pos |---------------------------------|
				//		             * [   5   ]            |                                 |
				//		             * [   6   ]            | First block (len = N - old_pos) |
				//		             * [   7   ]            |                                 |
				//		             * [ N - 1 ]            |---------------------------------|
				//						DATA = FIRST BLOCK + SECOND BLOCK
//				uint8_t First_Block = Rx_Buffer_Size - Old_Pos;
//				uint8_t Second_Block = Pos;
				uint8_t Last_Position =0;

				for(uint8_t i = Old_Pos; i<Rx_Buffer_Size; i++){ // first block copy to Recieved_Data
					Recieved_Data[Last_Position] = Rx_Buffer[i];
					Last_Position++;
				}

				for(uint8_t i = 0; i < Pos; i++){ // second block copy to Recieved_Data

					Recieved_Data[Last_Position] = Rx_Buffer[i];
					Last_Position++;
				}
			}
		}

		Old_Pos = Pos;
	}

}



