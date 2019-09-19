#pragma once

// RS485 DMA send and receive version.
// Receive is a DMA circular buffer.
// Send is DMA, one packet at a time.  Set RS485 DE before send, the clears after transmission.  Handled in interrupt.

#include <stdint.h>
#include <stdlib.h>


template <bool RX, uint8_t rx_buffer_size, bool TX, uint16_t tx_buffer_size, bool TXBlocking, bool RS485, int DEPort, int DEPin >
class SerialImpl
{
	const uint8_t RX_BUFFER_MASK = (rx_buffer_size - 1);
	uint8_t RxBuf[rx_buffer_size]; 
	volatile uint8_t RxTail;
	UART_HandleTypeDef &huart;
	bool transmitting;

	void SetDERead()
	{
		HAL_GPIO_WritePin((GPIO_TypeDef *)DEPort, DEPin, GPIO_PIN_RESET);
	}

	void SetDEWrite()
	{
		HAL_GPIO_WritePin((GPIO_TypeDef *)DEPort, DEPin, GPIO_PIN_SET);
	}
	
public:
	SerialImpl(UART_HandleTypeDef &huart)
		: huart(huart)
	{
		RxTail = 0;
		transmitting = false;
		SetDERead();
	}
protected:
	void Start(uint32_t baudrate)
	{
		huart.Instance->BRR = UART_BRR_SAMPLING16(HAL_RCC_GetPCLK2Freq(), baudrate);
		HAL_UART_Receive_DMA( &huart, RxBuf, rx_buffer_size);
	}
	
	uint8_t ReadByte(void) 
	{
		while ((rx_buffer_size - huart.hdmarx->Instance->CNDTR) == RxTail)							/* Wait for incomming data */
			continue;

		uint8_t data = RxBuf[RxTail];
		RxTail = (RxTail + 1) & RX_BUFFER_MASK; /* Calculate buffer index */
		
		return data;								/* Return data */
	}

	bool ReadDataAvailable(void)
	{
		return ((rx_buffer_size - huart.hdmarx->Instance->CNDTR) != RxTail);							/* Return 0 (FALSE) if the receive buffer is empty */
	}

	void SendPacket(uint8_t *str, uint16_t len)
	{
		SetDEWrite();
		transmitting = true;
		HAL_UART_Transmit_DMA(&huart, str, len);
		
		
//		//DMA_SetConfig(huart.hdmatx, (uint32_t)str, (uint32_t)&(huart.Instance->DR), len);
//  
//		huart.hdmatx->DmaBaseAddress->IFCR = (DMA_ISR_GIF1 << huart.hdmatx->ChannelIndex);	/* Clear all flags */
//		huart.hdmatx->Instance->CNDTR = len;		/* Configure DMA Channel data length */
//		huart.hdmatx->Instance->CPAR = (uint32_t)&(huart.Instance->DR);		/* Configure DMA Channel destination address */
//		huart.hdmatx->Instance->CMAR = (uint32_t)str;		/* Configure DMA Channel source address */
//		
//		__HAL_DMA_ENABLE_IT(huart.hdmatx, DMA_IT_TC);
//		__HAL_DMA_ENABLE(huart.hdmatx);
//		__HAL_UART_CLEAR_FLAG(&huart, UART_FLAG_TC);
//		SET_BIT(huart.Instance->CR3, USART_CR3_DMAT);

	}
	
	bool Transmitting(void)
	{
		return transmitting;
	}
	
		
	uint32_t Status()
	{
		return huart.Instance->SR;
	}
	
public:
	void IRQHandler()
	{
		SetDERead();
//		__HAL_DMA_DISABLE_IT(huart.hdmatx, DMA_IT_TE | DMA_IT_TC);  
//		__HAL_DMA_CLEAR_FLAG(huart.hdmatx, __HAL_DMA_GET_TC_FLAG_INDEX(huart.hdmatx));
		transmitting = false;
	}
};

//#define SERIAL_IRQHANDLER_IMPL(c,n)			extern "C" void USART##n##_IRQHandler()		\
//											{											\
//												c.IRQHandler();							\
//											}

//#define SERIAL_IRQHANDLER_IMPL(c,n)			extern "C" void DMA1_Channel4_IRQHandler()		\
//											{											\
//												c.IRQHandler();							\
//											}


#define SERIAL_IRQHANDLER_IMPL(c,n)			extern "C" void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)	\
											{											\
												c.IRQHandler();							\
											}



class SerialBase
{
public:
	virtual void Start(uint32_t baudrate) = 0;
	virtual uint32_t Status() = 0;
	virtual uint8_t ReadByte(void)  = 0;
	virtual bool ReadDataAvailable(void) = 0;
	virtual void SendPacket(uint8_t *str, uint16_t len) = 0;
	virtual bool Transmitting(void) = 0;
};

template <class SerialImpl >
class Serial : public SerialImpl, public SerialBase
{
public:
	Serial(UART_HandleTypeDef &huart) : SerialImpl(huart)
	{
	}
	
	virtual void Start(uint32_t baudrate)
	{
		SerialImpl::Start(baudrate);
	}
	
	virtual uint32_t Status()
	{
		return SerialImpl::Status();
	}	
	
	virtual uint8_t ReadByte(void) 
	{
		return SerialImpl::ReadByte();
	}

	virtual bool ReadDataAvailable(void)
	{
		return SerialImpl::ReadDataAvailable();
	}

	virtual void SendPacket(uint8_t *str, uint16_t len)
	{
		SerialImpl::SendPacket(str, len);
	}
	
	virtual bool Transmitting(void)
	{
		return SerialImpl::Transmitting();
	}

	
};
