/**********************************************************************************
 *
 **********************************************************************************
 */

#ifndef __MODBUS_H__
#define __MODBUS_H__

#include <stdint.h>
#include "uart.h"

#define USART_BUFSIZE		128

// MODBUS_FN_CODE holds modbus function codes used it this application.
enum MODBUS_FN_CODE { RD_HOLDREG = 3, RD_INPUTREG, WR_HOLDREG = 6, WR_MULT_HOLGREG = 16};

//modbus pre-frame delay
#define PREFRAME_DLY_US        5000
// modbus post frame delay
#define POSTFRAME_DLY_US       30000

#define MODBUS_RTU_HEAD_LEN    3
#define MODBUS_RTU_TAIL_LEN    2

// NULL defined
#ifndef NULL
#define NULL ((void*) 0)
#endif

#define HOLDINGREG_REG_COUNT 	1
#define INPUTREG_REG_COUNT 		2

// holding register address range
#ifdef ENABLE_MULTICHANNEL
#define HOLDREG_RANGE_START     40001
#define HOLDREG_RANGE_END       44370
#else
#define HOLDREG_RANGE_START     40000
#define HOLDREG_RANGE_END       40036
#endif
// input register address range
#define INPUTREG_RANGE_START    30000
#define INPUTREG_RANGE_END      30058
// max addressable slave address in modbus
#define SLAVE_ADDR_MAX          247

// error messages
#define READ_SUCCESS            0
#define INVALID_ADDR            1
#define INVALID_REG_COUNT       2
#define INVALID_SLAVE           3
#define NULL_OBJECT             4
#define INVALID_RESP            5
#define READ_TIMEOUT           10

typedef struct  __attribute__((__packed__))
{
    uint16_t start_addr;
    uint16_t register_count;
}RtuReqData;

typedef union __attribute__((__packed__))
{
    uint8_t data[4];
    //RtuReqData req_data;

}Rtu_Req_DataU;

typedef struct  __attribute__((__packed__))
{
    uint8_t slave_addr;
    uint8_t func_code;
    Rtu_Req_DataU rtu_req_data;
    uint16_t crc;
}Modbus_Rtu_Req;


/*******************************************************************************/

extern volatile uint8_t usart1_buffer[USART_BUFSIZE];
volatile uint32_t usart1_count;

/******************************************************************************/

uint16_t calculate_crc16(uint8_t *msg, uint8_t length);
void construct_rtu_req(uint8_t slave_addr, uint8_t fncode, uint16_t start_addr, uint16_t reg_count, Modbus_Rtu_Req *req);
uint8_t read_modbus_reg(uint8_t slave_addr, uint16_t reg_addr, uint16_t reg_count);

/* Function Prototype Declaration */

void modbus_master_init();
void  modbus_uart_send(uint8_t *bufferptr, uint32_t length);

#endif
