#include "LPC17xx.h"
#include "acpsTrace.h"
//#include "delay.h"
#include <stdint.h>
#include <string.h>



#ifdef ACPS_DEBUG_TRACE

/* used to hold debug trace msgs */
char trace_buffer[TRACE_BUFFSIZE];

void acpsTrace_init(void)
{
	/* Enable GPIOA clock */
		RCC_AHBPeriphClockCmd(RCC_AHB_PERIPHERAL_ACPS_TRACE_PORT, ENABLE);

		/* configure GPIO pins for alternate function mode */
		GPIO_InitTypeDef GPIO_InitStruct;

		GPIO_InitStruct.GPIO_Pin = ACPS_TRACE_TX_PIN | ACPS_TRACE_RX_PIN;
		GPIO_InitStruct.GPIO_Mode = GPIO_Mode_AF;
		GPIO_InitStruct.GPIO_OType = GPIO_OType_PP;
		GPIO_InitStruct.GPIO_PuPd = GPIO_PuPd_UP;
		GPIO_InitStruct.GPIO_Speed = GPIO_Speed_Level_2;
		GPIO_Init(ACPS_TRACE_GPIO_PORT, &GPIO_InitStruct);

		// disable USART1 clock first to make sure no glitches
		RCC_APB2PeriphClockCmd(RCC_APB2_PERIPHERAL_ACPS_TRACE_USART, DISABLE);

		//configure usart1 clock source as PCLK
		RCC_USARTCLKConfig(RCC_USART1CLK_PCLK);

		//enable USART1 clock
		RCC_APB2PeriphClockCmd(RCC_APB2_PERIPHERAL_ACPS_TRACE_USART, ENABLE);

		// reset peripheral
		RCC_APB2PeriphResetCmd(RCC_APB2_PERIPHERAL_ACPS_TRACE_USART, ENABLE);
		RCC_APB2PeriphResetCmd(RCC_APB2_PERIPHERAL_ACPS_TRACE_USART, DISABLE);

		//pin selection and alternate function selection
		// for TX pin
		GPIO_PinAFConfig(ACPS_TRACE_GPIO_PORT, ACPS_TRACE_GPIO_PIN_SOURCE_TX, GPIO_AF_1);
		// for RX pin
		GPIO_PinAFConfig(ACPS_TRACE_GPIO_PORT, ACPS_TRACE_GPIO_PIN_SOURCE_RX, GPIO_AF_1);

		// set up baud rate, stopbits, parity
		USART_InitTypeDef USART_InitStruct;
		USART_InitStruct.USART_BaudRate = 9600;
		USART_InitStruct.USART_WordLength = USART_WordLength_8b;
		USART_InitStruct.USART_StopBits = USART_StopBits_1;
		USART_InitStruct.USART_Parity = USART_Parity_No;
		USART_InitStruct.USART_Mode = (USART_Mode_Tx | USART_Mode_Rx);
		USART_InitStruct.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
		USART_Init(ACPS_TRACE_USART, &USART_InitStruct);

		//USART_SWAPPinCmd(ACPS_TRACE_USART, ENABLE);
		// Enab<le UART module
		USART_Cmd(ACPS_TRACE_USART, ENABLE);
}


void acps_trace(char *msg)
{
	int32_t len = 0;

	len = (int32_t) strlen((char *)msg);

	while(len)
	{
		if (USART_GetFlagStatus(ACPS_TRACE_USART, USART_FLAG_TXE) == SET)
		{
			USART_SendData(ACPS_TRACE_USART, (uint16_t)(*msg));
			len--;
			msg++;
		}
	}
}

uint32_t acps_trace_input(char *read_input, uint32_t length, uint16_t time_out)
{
	uint32_t i = 0;

	/* discard last received overrun data and clear overrun flag */
	USART_ReceiveData(ACPS_TRACE_USART);
	USART_ClearFlag(ACPS_TRACE_USART, USART_FLAG_ORE);

	/* start timer for input timeout, if user doesnt respond */
	if(time_out > 0)
	{
		tim3_start_ms_timer(time_out);
	}
	else
	{
		return i;
	}
	/* collect data bytes */
	while(i < length && !tim3_check_ms_timer())
	{
		if (USART_GetFlagStatus(ACPS_TRACE_USART, USART_FLAG_RXNE) == SET)
		{
			read_input[i] = USART_ReceiveData(ACPS_TRACE_USART);
			if(read_input[i] == '\n' || read_input[i] == '\r' )
			{
				read_input[i] = '\0';
				break;
			}
			i++;
		}
	}

	/* stop timer */
	tim3_stop_ms_timer();
	return i;
}



void diagnostic_trace(char *msg)
{
	acps_trace("\r\n[DIAG_TRACE] ");
	acps_trace(msg);
	acps_trace("\r\n");
}



void error_trace(char *msg)
{
	acps_trace("\r\n[ERROR_TRACE] ");
	acps_trace(msg);
	acps_trace("\r\n");
}

#endif

