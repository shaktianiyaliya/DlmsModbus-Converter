/******************************************************************************
* 	Description : Contains functions related to Modbus implementation.
*
*	File		:  modbus.c
*
*****************************************************************************
*/


#include "modbus.h"
#include "systick_timer.h"
#include <string.h>
#include "LPC17xx.h"



#define RS485_TX_RX_MODE_PIN	(1 << 22)

// modbus rtu request object
Modbus_Rtu_Req req;

volatile uint32_t usart1_status;
volatile uint8_t  usart1_buffer[USART_BUFSIZE];
volatile uint32_t usart1_count = 0;
volatile uint32_t timeoutCount = 1;

/*****************************************************************************/

#define PRESCALE (25000-1)

void initTimer0(void)
{
	/*Assuming that PLL0 has been setup with CCLK = 100Mhz and PCLK = 25Mhz.*/
	LPC_SC->PCONP |= (1<<1); //Power up TIM0. By default TIM0 and TIM1 are enabled.
	LPC_SC->PCLKSEL0 &= ~(0x3<<3); //Set PCLK for timer = CCLK/4 = 100/4 (default)

	LPC_TIM0->CTCR = 0x0;
	LPC_TIM0->PR = PRESCALE; //Increment TC at every 24999+1 clock cycles
	//25000 clock cycles @25Mhz = 1 mS

	LPC_TIM0->TCR = 0x02; //Reset Timer
}


void delayMS(unsigned int milliseconds) //Using Timer0
{
	LPC_TIM0->TCR = 0x02; //Reset Timer

	LPC_TIM0->TCR = 0x01; //Enable timer

	while(LPC_TIM0->TC < milliseconds); //wait until timer counter reaches the desired delay

	LPC_TIM0->TCR = 0x00; //Disable timer
}




/*
*   crc16() calculates the 16-bit crc value for Modbus protocol.
*/
uint16_t calculate_crc16(uint8_t *msg, uint8_t length)
{
    uint32_t i = 0;
    // init CRC16 with all bits 1
    uint16_t crc16 = 0xffff;

    while(length)
    {
 	   // x-or lower byte with msg byte
		crc16 ^= (0x00ff & *msg);
		i = 8;

		while(i--)
		{
            // check lsb
            if(crc16 & 0x0001)
            {
                // shift crc16 to right by 1 bit
                crc16 >>= 1;
                // ldb is 1 , so x-or crc16 with 0xA001
                crc16 ^= 0xA001;
            }
            else
            {
                // shift crc16 to right by 1 bit
                crc16 >>= 1;
            }
        }

        //point to next msg byte
        msg++;
        // update remaining length of msg left
        length--;
    }

    return crc16;
}


/*
* constructs rtu modbus request object with all fields. Expects pointer to modbus rtu req structure.
*/
void construct_rtu_req(uint8_t slave_addr, uint8_t fncode, uint16_t start_addr, uint16_t reg_count, Modbus_Rtu_Req *reqst)
{
    // set up slave addr in rtu req format
    reqst->slave_addr = slave_addr;
    // set up function code
    reqst->func_code = fncode;
    //start reg addr hi-byte
    reqst->rtu_req_data.data[0] = start_addr >> 8;
    // start reg addr lo-byte
    reqst->rtu_req_data.data[1] = start_addr;
    // no of registers hi-byte
    reqst->rtu_req_data.data[2] = reg_count >> 8;
    // no of registers lo-byte
    reqst->rtu_req_data.data[3] = reg_count;
    // calculate crc and embed into frame with lo-byte first
    reqst->crc = calculate_crc16((uint8_t *)reqst, sizeof(Modbus_Rtu_Req)-2);
    return;
}

uint32_t get_time_out_value(void)
{
	timeoutCount = 1;
	return timeoutCount;
}


uint32_t get_time_out_count(void)
{
	delayMS(1);
	timeoutCount++;
	return timeoutCount;
}



/*
* read_modbus_reg() gets req contructed. Depending on register addr it detects function code according to
* Modbus specs.
*/
uint8_t read_modbus_reg(uint8_t slave_addr, uint16_t reg_addr, uint16_t reg_count)
{
    uint8_t fncode;
    uint32_t time_stamp;
    uint32_t resp_length = 0;
    uint8_t LSRValue;
    uint8_t Dummy = Dummy;

    memset((uint8_t *)usart1_buffer, '\0', USART_BUFSIZE);
    //check if address falls in valid range
    if(reg_addr >= HOLDREG_RANGE_START && reg_addr <= HOLDREG_RANGE_END)
    {
        // check reg count validity
        if((reg_addr + reg_count - 1) > HOLDREG_RANGE_END)
        {
            return INVALID_REG_COUNT;
        }
        // function is to read hold_reg
        fncode = RD_HOLDREG;
        // parse address for modbus frame format.
        reg_addr -= HOLDREG_RANGE_START;
    }
    else if (reg_addr >= INPUTREG_RANGE_START && reg_addr <= INPUTREG_RANGE_END)
    {
        // check reg count validity
        if((reg_addr + reg_count - 1) > (INPUTREG_RANGE_END + INPUTREG_REG_COUNT))
        {
            return INVALID_REG_COUNT;
        }
        // function is to read input reg
        fncode = RD_INPUTREG;
        // parse address for modbus frame format.
        reg_addr -= INPUTREG_RANGE_START;
    }
    else
    {
        return  INVALID_ADDR;
    }

    if(slave_addr > SLAVE_ADDR_MAX)
    {
        return INVALID_SLAVE;
    }

    // construct request
    construct_rtu_req(slave_addr, fncode, reg_addr, reg_count, &req);
    // reset usart1_count
    usart1_count = 0;


    //time_stamp = get_systick_count();
    time_stamp = get_time_out_value();
            //wait for 30ms for 3.5 character silent interval
    while(get_elapsed_time(time_stamp, get_time_out_count()) < 40)
    {
            ;
    }

    // Send modbus rtu packet over serial interface(UART1)
    //rit_delay_us(PREFRAME_DLY_US);
    modbus_uart_send((uint8_t *)&req, sizeof(Modbus_Rtu_Req));
    //rit_delay_us(POSTFRAME_DLY_US);


    /***********************************polling*********************/
	time_stamp = get_time_out_value();
	//wait for 100ms max for energy meter worst case response
	while (get_elapsed_time(time_stamp, get_time_out_count()) < 300) {
		LSRValue = LPC_UART1->LSR;
		/* Receive Line Status */
		if (LSRValue & (LSR_OE | LSR_PE | LSR_FE | LSR_RXFE | LSR_BI)) {
			/* There are errors or break interrupt */
			/* Read LSR will clear the interrupt */
			//UART1Status = LSRValue;
			Dummy = LPC_UART1->RBR; /* Dummy read on RX to clear
			 interrupt, then bail out */
			return;
		}
		if (LSRValue & LSR_RDR) /* Receive Data Ready */
		{
			/* If no error on RLS, normal ready, save into the data buffer. */
			/* Note: read RBR will clear the interrupt */
			usart1_buffer[usart1_count] = LPC_UART1->RBR;
			usart1_count++;
			if (usart1_count >= USART_BUFSIZE) {
				usart1_count = 0; /* buffer overflow */
			}
		}

	}
	if (usart1_count >= 3 && resp_length == 0)
	{
		if ((usart1_buffer[0] != slave_addr) || (usart1_buffer[1] != fncode))
		{
			return INVALID_RESP;
		}
		resp_length = MODBUS_RTU_HEAD_LEN + usart1_buffer[2]
				+ MODBUS_RTU_TAIL_LEN;
	}

	if (resp_length != 0 && usart1_count == resp_length)
	{
		return READ_SUCCESS;
	}

// max wait for energy meter response is over
	return READ_TIMEOUT;


    /********************************************************************/

   /*
    	//time_stamp = get_systick_count();
    time_stamp = get_time_out_value();
    //wait for 100ms max for energy meter worst case response
    while(get_elapsed_time(time_stamp, get_time_out_count()) < 300)
    {
        if(usart1_count >= 3 && resp_length == 0)
        {
            if( (usart1_buffer[0] != slave_addr) || (usart1_buffer[1] != fncode))
            {
                return INVALID_RESP;
            }
            resp_length = MODBUS_RTU_HEAD_LEN + usart1_buffer[2] + MODBUS_RTU_TAIL_LEN;
        }

        if(resp_length != 0 && usart1_count == resp_length)
        {
            return READ_SUCCESS;
        }
    }

    // max wait for energy meter response is over
    return READ_TIMEOUT;

    */
}


/*
******************************************************************************

******************************************************************************
*/
void modbus_master_init(void)
{
	UARTInit(1, 9600);
	LPC_GPIO0->FIODIR |= RS485_TX_RX_MODE_PIN;
	LPC_GPIO0->FIOSET = RS485_TX_RX_MODE_PIN;
	initTimer0();
}


/* ------------------------------------------------------------------------------------------------------------------
 * Function: modbus_uart_send
 *
 * Writes buffer to UART byte by byte and sends over serial port.
 */
void  modbus_uart_send(uint8_t *bufferptr, uint32_t length)
{
	LPC_GPIO0->FIOSET = RS485_TX_RX_MODE_PIN;
	UARTSend(1, bufferptr, length);
	LPC_GPIO0->FIOCLR = RS485_TX_RX_MODE_PIN;
}


/*****************************************************************************
** Function name:		UART1_IRQHandler
**
** Descriptions:		UART1 interrupt handler
**
** parameters:			None
** Returned value:		None
**
*****************************************************************************/
void UART1_IRQHandler (void)
{
  uint8_t IIRValue, LSRValue;
  uint8_t Dummy = Dummy;

  IIRValue = LPC_UART1->IIR;

  IIRValue >>= 1;			/* skip pending bit in IIR */
  IIRValue &= 0x07;			/* check bit 1~3, interrupt identification */
  if ( IIRValue == IIR_RLS )		/* Receive Line Status */
  {
	LSRValue = LPC_UART1->LSR;
	/* Receive Line Status */
	if ( LSRValue & (LSR_OE|LSR_PE|LSR_FE|LSR_RXFE|LSR_BI) )
	{
	  /* There are errors or break interrupt */
	  /* Read LSR will clear the interrupt */
	  //UART1Status = LSRValue;
	  Dummy = LPC_UART1->RBR;		/* Dummy read on RX to clear
								interrupt, then bail out */
	  return;
	}
	if ( LSRValue & LSR_RDR )	/* Receive Data Ready */
	{
	  /* If no error on RLS, normal ready, save into the data buffer. */
	  /* Note: read RBR will clear the interrupt */
	  usart1_buffer[usart1_count] = LPC_UART1->RBR;
	  usart1_count++;
	  if ( usart1_count >= USART_BUFSIZE )
	  {
		  usart1_count = 0;		/* buffer overflow */
	  }
	}
  }
  else if ( IIRValue == IIR_RDA )	/* Receive Data Available */
  {
	/* Receive Data Available */
	usart1_buffer[usart1_count] = LPC_UART1->RBR;
	usart1_count++;
	if ( usart1_count >= USART_BUFSIZE )
	{
		usart1_count = 0;		/* buffer overflow */
	}
  }
  else if ( IIRValue == IIR_CTI )	/* Character timeout indicator */
  {
	/* Character Time-out indicator */
	//UART1Status |= 0x100;		/* Bit 9 as the CTI error */
  }
  else if ( IIRValue == IIR_THRE )	/* THRE, transmit holding register empty */
  {
	/* THRE interrupt */
	LSRValue = LPC_UART1->LSR;		/* Check status in the LSR to see if
								valid data in U0THR or not */

  }
}
