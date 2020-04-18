#include"LT_energymeter.h"
#include <string.h>

/*
 * *Common API
 */
uint32_t read(enum Reg reg)
{
	uint8_t var;
	uint8_t val;
	uint32_t data = 0;
	read_modbus_reg(ENERGY_METER_SLAVE_ID, reg, INPUTREG_REG_COUNT);
	/*
	*	The order of bytes is in group of 16 bit.
	*	so when float voltage is read we have 2 pairs of 16 bits, out of which 1st pair is least sig
	*	while 2nd pair is most signiificant 16 bit.
	*/
	memcpy(&data, (uint8_t *) &usart1_buffer[5], sizeof(uint16_t));


	var = usart1_buffer[3];
	usart1_buffer[3] = usart1_buffer[4];
	usart1_buffer[4] = var;

	var = usart1_buffer[5];
	usart1_buffer[5] = usart1_buffer[6];
	usart1_buffer[6] = var;

	memcpy(&data, (uint8_t *) &usart1_buffer[5], sizeof(uint16_t));

	return data;
}





/*
*	readVoltage() reads voltage register indicated by 'reg' enum value. Reads and returns 
*	value of single voltage register(32 bits)
*/


//float readVoltage(enum Volt_Reg reg)	//rakesh
uint32_t readVoltage(enum Volt_Reg reg)
{
	uint8_t var;
	uint8_t val;
	uint32_t voltage = 0;
	read_modbus_reg(ENERGY_METER_SLAVE_ID, reg, INPUTREG_REG_COUNT);
	/*
	*	The order of bytes is in group of 16 bit.
	*	so when float voltage is read we have 2 pairs of 16 bits, out of which 1st pair is least sig
	*	while 2nd pair is most signiificant 16 bit.
	*/
	memcpy(&voltage, (uint8_t *) &usart1_buffer[5], sizeof(uint16_t));


	var = usart1_buffer[3];
	usart1_buffer[3] = usart1_buffer[4];
	usart1_buffer[4] = var;

	var = usart1_buffer[5];
	usart1_buffer[5] = usart1_buffer[6];
	usart1_buffer[6] = var;

	memcpy(&voltage, (uint8_t *) &usart1_buffer[5], sizeof(uint16_t));

	return voltage;
}

/*
*	readCurrent() reads current register indicated by 'reg' enum value. Reads and returns 
*	value of single current register(32 bits)
*/

//float readCurrent(enum Curr_Reg reg)	//rakesh
uint32_t readCurrent(enum Curr_Reg reg)
{
	uint8_t var;
	uint32_t current = 0;
	read_modbus_reg(ENERGY_METER_SLAVE_ID, reg, INPUTREG_REG_COUNT);
	/*
	*	The order of bytes is in group of 16 bit.
	*	so when float current is read we have 2 pairs of 16 bit, out of which 1st pair is least sig
	*	while 2nd pair is most significant 16 bit.
	*/
	var = usart1_buffer[3];
	usart1_buffer[3] = usart1_buffer[4];
	usart1_buffer[4] = var;

	var = usart1_buffer[5];
	usart1_buffer[5] = usart1_buffer[6];
	usart1_buffer[6] = var;

	memcpy(&current, (uint8_t *) &usart1_buffer[5], sizeof(uint16_t));

	return current;
}

/*
*	readActPower() reads Active power register indicated by 'reg' enum value. Reads and returns 
*	value of single Active Power register (32 bits)
*/

//float readActPower(enum Act_Pow_Reg reg)	//rakesh
uint32_t readActPower(enum Act_Pow_Reg reg)
{
	uint8_t var;
	uint32_t Active_Power = 0;
	read_modbus_reg(ENERGY_METER_SLAVE_ID, reg, INPUTREG_REG_COUNT);
	/*
	*	The order of bytes is in group of 16 bit.
	*	so while float active power is read, we have 2 pairs of 16 bit, out of which 1st pair is least sig
	*	while 2nd pair is most significant 16 bit.
	*/
	var = usart1_buffer[3];
	usart1_buffer[3] = usart1_buffer[4];
	usart1_buffer[4] = var;

	var = usart1_buffer[5];
	usart1_buffer[5] = usart1_buffer[6];
	usart1_buffer[6] = var;

	memcpy(&Active_Power, (uint8_t *) &usart1_buffer[5], sizeof(uint16_t));

	return Active_Power;
}

/*
*	readReActPower() reads Reactive Power register indicated by 'reg' enum value. Reads and ruturns 
*	value of single Reactive power register (32 bits).
*/

//float readReActPower(enum ReAct_Pow_Reg reg)	//rakesh
uint32_t readReActPower(enum ReAct_Pow_Reg reg)
{
	uint8_t var;
	uint32_t Reactive_Power = 0;
	read_modbus_reg(ENERGY_METER_SLAVE_ID, reg, INPUTREG_REG_COUNT);
	/*
	*	The order of bytes is in group of 16 bit.
	*	so while float Reactive Power is read, we have 2 pairs of 16 bit, out of which 1st pair is least sig,
	*	while 2nd pair is most significant 16 bit.
	*/
	var = usart1_buffer[3];
	usart1_buffer[3] = usart1_buffer[4];
	usart1_buffer[4] = var;

	var = usart1_buffer[5];
	usart1_buffer[5] = usart1_buffer[6];
	usart1_buffer[6] = var;

	memcpy(&Reactive_Power, (uint8_t *) &usart1_buffer[5], sizeof(uint16_t));

	return Reactive_Power;
}

/*
*	readAppaPower() reads Apparent power register indicated by 'reg' enum values. Reads and returns
*	value of single Apparent power register(32 bits).
*/

//float readAppaPower(enum Appa_Pow_Reg reg)	//rakesh
uint32_t readAppaPower(enum Appa_Pow_Reg reg)
{
	uint8_t var;
	uint32_t Apparent_Power = 0;
	read_modbus_reg(ENERGY_METER_SLAVE_ID, reg, INPUTREG_REG_COUNT);
	/*
	*	The order of bytes is in group of 16 bit.
	*	so while float Apparent power is read, we have 2 pairs of 16 bit, out of which 1st is least sig
	*	while 2nd pair is most significant 16 bit.
	*/
	var = usart1_buffer[3];
	usart1_buffer[3] = usart1_buffer[4];
	usart1_buffer[4] = var;

	var = usart1_buffer[5];
	usart1_buffer[5] = usart1_buffer[6];
	usart1_buffer[6] = var;

	memcpy(&Apparent_Power, (uint8_t *) &usart1_buffer[5], sizeof(float));

	return Apparent_Power;
}

/*
*	readPowerFactor() reads Power Factor regiser indicated by 'reg' enum values. Reads and returns
*	value of single Power Factor register (32 bits).
*/

//float readPowerFactor(enum Pow_Fact_Reg reg)	//rakesh
uint32_t readPowerFactor(enum Pow_Fact_Reg reg)
{
	uint8_t var;
	uint32_t Power_Factor = 0;
	read_modbus_reg(ENERGY_METER_SLAVE_ID, reg, INPUTREG_REG_COUNT);
	/*
	*	The order of bytes in in group of 16 bit.
	*	so while float power factor is read, we have 2 pairs of 16 bit, out of which 1st pair is least sig
	* 	while 2nd pair is most significant 16 bit.
	*/
	var = usart1_buffer[3];
	usart1_buffer[3] = usart1_buffer[4];
	usart1_buffer[4] = var;

	var = usart1_buffer[5];
	usart1_buffer[5] = usart1_buffer[6];
	usart1_buffer[6] = var;

	memcpy(&Power_Factor, (uint8_t *) &usart1_buffer[5], sizeof(uint16_t));

	return Power_Factor;
}
/*
*	readTotalActivePower() reads the Total Active Power from Meter's register
*/

//float readTotalActivePower(void)	//rakesh
uint32_t readTotalActivePower(void)
{
	uint8_t var;
	uint32_t Total_Active_Power = 0;
	read_modbus_reg(ENERGY_METER_SLAVE_ID, KWH_REG, INPUTREG_REG_COUNT);
	//read_modbus_reg(ENERGY_METER_SLAVE_ID, PF_REG, INPUTREG_REG_COUNT);
	//read_modbus_reg(ENERGY_METER_SLAVE_ID, FRQ_REG, INPUTREG_REG_COUNT);
	/*
	*	The order of bytes is in group of 16 bit.
	*	so while float Totla Active Power is read, we have 2 pairs of 16 bit, out of which 1st pair is least sig
	*	while 2nd pair is most significant 16 bit.
	*/
	var = usart1_buffer[3];
	usart1_buffer[3] = usart1_buffer[4];
	usart1_buffer[4] = var;

	var = usart1_buffer[5];
	usart1_buffer[5] = usart1_buffer[6];
	usart1_buffer[6] = var;

	memcpy(&Total_Active_Power, (uint8_t *) &usart1_buffer[5], sizeof(uint16_t));

	return Total_Active_Power;
}
#if 0
/*
*	readTotalReactivePower() reads the Total Reactive Power from meter's register
*/

//float readTotalReactivePower(void)	//rakesh
uint32_t readTotalReactivePower(void)
{
	uint8_t var;
	uint32_t Total_Reactive_Power = 0;
	read_modbus_reg(ENERGY_METER_SLAVE_ID, TOTAL_REACTIVE_POWER, INPUTREG_REG_COUNT);
	/*
	*	The order of bytes is in group of 16 bit.
	*	so while float Total Reactive Power is read, we have 2 pairs of 16 bit, out of which 1st pair is least sig
	*	while 2nd pair is most significant 16 bit.
	*/
	var = usart1_buffer[3];
	usart1_buffer[3] = usart1_buffer[4];
	usart1_buffer[4] = var;

	var = usart1_buffer[5];
	usart1_buffer[5] = usart1_buffer[6];
	usart1_buffer[6] = var;

	memcpy(&Total_Reactive_Power, (uint8_t *) &usart1_buffer[5], sizeof(uint16_t));

	return Total_Reactive_Power;
}
#endif
/*
*	readTotalApparentPower() reads the Total Apparent Power from meter's register
*/

//float readTotalApparentPower(void)	//rakesh
uint32_t readTotalApparentPower(void)
{
	uint8_t var;
	uint32_t Total_Apparent_Power = 0;
	read_modbus_reg(ENERGY_METER_SLAVE_ID, KV_REG, INPUTREG_REG_COUNT);
	/*
	*	The order of bytes is in group of 16 bit.
	*	so while float Total Apparent Power is read, we have 2 pairs of 16 bit, out of which 1st pair is least sig
	*	while 2nd pair is most significant 16 bit.
	*/
	var = usart1_buffer[3];
	usart1_buffer[3] = usart1_buffer[4];
	usart1_buffer[4] = var;

	var = usart1_buffer[5];
	usart1_buffer[5] = usart1_buffer[6];
	usart1_buffer[6] = var;

	memcpy(&Total_Apparent_Power, (uint8_t *) &usart1_buffer[5], sizeof(uint16_t));

	return Total_Apparent_Power;
}

/*
*	readTotalPowerFactor() reads the Total Power Factor from meter's register
*/

//float readTotalPowerFactor(void)	//rakesh
uint32_t readTotalPowerFactor(void)
{
	uint8_t var;
	uint32_t Total_Power_Factor = 0;
	read_modbus_reg(ENERGY_METER_SLAVE_ID, PF_REG, INPUTREG_REG_COUNT);
	/*
	*	The order of byter is in group of 16 bit.
	* 	so while float Total Power Factor is read, we have 2 pairs of 16 bit, out of which 1st pair is least sig
	* 	while 2nd pair is most significant 16 bit
	*/
	var = usart1_buffer[3];
	usart1_buffer[3] = usart1_buffer[4];
	usart1_buffer[4] = var;

	var = usart1_buffer[5];
	usart1_buffer[5] = usart1_buffer[6];
	usart1_buffer[6] = var;

	memcpy(&Total_Power_Factor, (uint8_t *) &usart1_buffer[5], sizeof(uint16_t));

	return Total_Power_Factor;
}
/*
*	readLineFrequency() reads the Line Frequency from meter's register
*/

//float readLineFrequency(void)	//rakesh
uint32_t readLineFrequency(void)
{
	uint8_t var;
	uint32_t Line_Frequency = 0;
	read_modbus_reg(ENERGY_METER_SLAVE_ID, FRQ_REG, INPUTREG_REG_COUNT);
	/*
	*	The order of bytes is in group of 16 bit.
	*	so while float Line frequency is read, we have 2 pairs of 16 bit, out of which 1st pair is least sig
	*	while 2nd pair is most significant 16 bit.
	*/
	var = usart1_buffer[3];
	usart1_buffer[3] = usart1_buffer[4];
	usart1_buffer[4] = var;

	var = usart1_buffer[5];
	usart1_buffer[5] = usart1_buffer[6];
	usart1_buffer[6] = var;

	memcpy(&Line_Frequency, (uint8_t *) &usart1_buffer[5], sizeof(uint16_t));

	return Line_Frequency;
}

/*
*	readPhaseSequence() reads the Phase Sequence from the meter's regiser
*/

//float readPhaseSequence(void)	//rakesh
uint32_t readPhaseSequence(void)
{
	uint8_t var;
	uint32_t Phase_Sequence = 0;
	read_modbus_reg(ENERGY_METER_SLAVE_ID, PHASE_SEQ, INPUTREG_REG_COUNT);
	/*
	*	The order of bytes is in group of 16 bit.
	*	so while float Phase Sequence is read, we have 2 pairs of 16 bit, out of which 1st pair is least sig
	*	while 2nd pair is most significant 16 bit.
	*/
	var = usart1_buffer[3];
	usart1_buffer[3] = usart1_buffer[4];
	usart1_buffer[4] = var;

	var = usart1_buffer[5];
	usart1_buffer[5] = usart1_buffer[6];
	usart1_buffer[6] = var;

	memcpy(&Phase_Sequence, (uint8_t *) &usart1_buffer[5], sizeof(uint16_t));

	return Phase_Sequence;
}

void energy_meter_init(void)
{
	modbus_master_init();
}
