#ifndef ENERGY_METER_H
#define ENERGY_METER_H

#include"modbus.h"

// Macro Definition of the energy meter slave id

#define ENERGY_METER_SLAVE_ID 3
//#define ENERGY_METER_RF_ID 0x0E

#ifdef LNT_METER
//Address of Total Active Power
#define KWH_REG 40037
//Address of Total Apparent Power 
#define KV_REG 40041
//Address of Total Power Factor
#define PF_REG 40043
//Line Frequency
#define FRQ_REG 40045
//Phase Sequence
#define PHASE_SEQ 40047
//Cumulative data

enum Reg {F_Wh = 40515,F_VArh_lag = 40517, F_VArh_lead = 40519, F_VAhs = 40513, R_Wh = 40523, R_VArh_lag = 40525, R_VArh_lead = 40527, R_Vah = 40521};
//Phase Voltage
enum Volt_Reg {VR_REG = 40001, VY_REG = 40003, VB_REG = 40005};	//rak
//enum Volt_Reg {VR_REG = 40515, VY_REG = 40517, VB_REG = 40519};	//cumulative
//Phase Current
enum Curr_Reg {CR_REG = 40007, CY_REG = 40009, CB_REG = 40011};
//Active Power
enum Act_Pow_Reg {ACT_POW_R_REG = 40013, ACT_POW_Y_REG = 40015, ACT_POW_B_REG = 40017};
//Reactive Power
enum ReAct_Pow_Reg {REACT_POW_R_REG = 40019, REACT_POW_Y_REG = 40021, REACT_POW_B_REG= 40023};
//Apparent Power
enum Appa_Pow_Reg {APPA_POW_R_REG = 40025, APPA_POW_Y_REG = 40027, APPA_POW_B_REG = 40029};
//Power Factor
enum Pow_Fact_Reg {POW_FACT_R_REG = 40031, POW_FACT_Y_REG = 40033, POW_FACT_B_REG = 40035};
#elif SELEC_METER
#define ENERGY_METER_SLAVE_ID 1
//#define ENERGY_METER_RF_ID 0x0E

// total kW register of MFM383AC
#define KW_REG  30042
// line freq register of MFM383AC
#define FRQ_REG 30056
// Average power factor register of MFM38AC
#define AVG_PF_REG 30054
//Total Kwh register of MEFM383AC
#define KWH_REG 30058

enum Volt_Reg {V1N_REG = 30000, V2N_REG = 30002, V3N_REG = 30004};
enum Curr_Reg {I1_REG = 30016, I2_REG = 30018, I3_REG = 30020};
enum Watt_Reg {KW1_REG = 30024, KW2_REG = 30026, KW3_REG = 30028};


#endif

/*
float readVoltage(enum Volt_Reg reg);
float readTotalActivePower(void);
float readTotalApparentPower(void);
float readCurrent(enum Curr_Reg reg);
float readTotalPowerFactor(void);*/

uint32_t readVoltage(enum Volt_Reg reg);
uint32_t readTotalActivePower(void);
uint32_t readTotalApparentPower(void);
uint32_t readCurrent(enum Curr_Reg reg);
uint32_t readTotalPowerFactor(void);
void energy_meter_init(void);
#endif
