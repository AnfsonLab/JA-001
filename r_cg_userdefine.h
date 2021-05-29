/***********************************************************************************************************************
* DISCLAIMER
* This software is supplied by Renesas Electronics Corporation and is only intended for use with Renesas products.
* No other uses are authorized. This software is owned by Renesas Electronics Corporation and is protected under all
* applicable laws, including copyright laws. 
* THIS SOFTWARE IS PROVIDED "AS IS" AND RENESAS MAKES NO WARRANTIES REGARDING THIS SOFTWARE, WHETHER EXPRESS, IMPLIED
* OR STATUTORY, INCLUDING BUT NOT LIMITED TO WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND
* NON-INFRINGEMENT.  ALL SUCH WARRANTIES ARE EXPRESSLY DISCLAIMED.TO THE MAXIMUM EXTENT PERMITTED NOT PROHIBITED BY
* LAW, NEITHER RENESAS ELECTRONICS CORPORATION NOR ANY OF ITS AFFILIATED COMPANIES SHALL BE LIABLE FOR ANY DIRECT,
* INDIRECT, SPECIAL, INCIDENTAL OR CONSEQUENTIAL DAMAGES FOR ANY REASON RELATED TO THIS SOFTWARE, EVEN IF RENESAS OR
* ITS AFFILIATES HAVE BEEN ADVISED OF THE POSSIBILITY OF SUCH DAMAGES.
* Renesas reserves the right, without notice, to make changes to this software and to discontinue the availability 
* of this software. By using this software, you agree to the additional terms and conditions found by accessing the 
* following link:
* http://www.renesas.com/disclaimer
*
* Copyright (C) 2011, 2019 Renesas Electronics Corporation. All rights reserved.
***********************************************************************************************************************/

/***********************************************************************************************************************
* File Name    : r_cg_userdefine.h
* Version      : CodeGenerator for RL78/G13 V2.05.04.02 [20 Nov 2019]
* Device(s)    : R5F100LE
* Tool-Chain   : CCRL
* Description  : This file includes user definition.
* Creation Date: 2021/5/2
***********************************************************************************************************************/

#ifndef _USER_DEF_H
#define _USER_DEF_H

/***********************************************************************************************************************
User definitions
***********************************************************************************************************************/

/* Start user code for function. Do not edit comment generated here */


typedef union
{
    struct
    {
        uint8_t no0:1;
        uint8_t no1:1;
        uint8_t no2:1;
        uint8_t no3:1;
        uint8_t no4:1;
        uint8_t no5:1;
        uint8_t no6:1;
        uint8_t no7:1;
    }bit;
    uint8_t flag;
}flag_t;

/*  ************************************************************* */

#define debug_e2prom_off 1
// #define debug_sw_on 1
// #define debug_rly_flag 1
// #define debug_time_short 0


#define step_10_sec	10

/*  ************************************************************* */

#define	_Control_sensor		0	//ANI0
#define	_Base_sensor		1	//ANI1
#define	_Condenser_sensor	2	//ANI2
#define	_Antifrz_sensor		3	//ANI3
#define	_Comp_out_sensor	4	//ANI4
#define	_AD_key				12

/*  ************************************************************* */

/*  ************* SW s1 & s2  ************************************************ */

#define	SWDIP2_1	P0_bit.no4
#define	SWDIP2_2	P5_bit.no1

#define	SWDIP4_1	P12_bit.no1
#define	SWDIP4_2	P6_bit.no2
#define	SWDIP4_3	P6_bit.no3
#define	SWDIP4_4	P3_bit.no1
#if 0
#define	TEMP_SELECT_SWITCH 		P3_bit.no1
#define SYSTEM_SELECT_SWITCH 	P6_bit.no3
#define POWER_RETURN_AUTO_ON  	P6_bit.no2
#define REMOTE_SW_FUNCTION		P12_bit.no1


#endif



#define	FIX_TYPE_SENSOR			0	
#define	TRACK_TYPE_SENSOR		1	

extern 	flag_t	_dip_switch_data_;
#define  _dip_switch_data		_dip_switch_data_.flag
#define  REMOTE_SW_FUNCTION		_dip_switch_data_.bit.no0
#define  SWDIP4_2_SET			_dip_switch_data_.bit.no1
#define  SYSTEM_SELECT_SWITCH	_dip_switch_data_.bit.no2
#define  TEMP_SELECT_SWITCH		_dip_switch_data_.bit.no3
#define  SWDIP2_1_SET			_dip_switch_data_.bit.no4
#define  SWDIP2_2_SET			_dip_switch_data_.bit.no5



/*  ************************************************************* */

#define   STEPMOTOR_A_1		P7_bit.no4
#define   STEPMOTOR_b_1		P7_bit.no5
#define   STEPMOTOR_a_1		P7_bit.no6
#define   STEPMOTOR_B_1		P7_bit.no7

#define   STEPMOTOR_A_2		P7_bit.no0
#define   STEPMOTOR_b_2		P7_bit.no1
#define   STEPMOTOR_a_2		P7_bit.no2
#define   STEPMOTOR_B_2		P7_bit.no3

#define   STEPMOTOR_A_3		P5_bit.no0
#define   STEPMOTOR_b_3		P3_bit.no0
#define   STEPMOTOR_a_3		P0_bit.no5
#define   STEPMOTOR_B_3		P0_bit.no6

/*  ************************************************************* */
#if 1
#define	HEATER_TEMP_PROTECT		P0_bit.no1
#define	WATERLEVER_LOW_ALARM	P0_bit.no0
#define	FLOW_SWITCH_PROTECT		P14_bit.no1
#define	LP_PROTECT				P14_bit.no0
#define	HP_PROTECT				P12_bit.no0
#define	POWER_PHASE_PROTECT		P4_bit.no3

#define	PUMP_OVERLOAD_PROTECT	P4_bit.no2
#define	FAN_OVERLOAD_PROTECT	P4_bit.no1
#define	COMP_OVERLOAD_PROTECT	P12_bit.no4
#define	RESERVE_2				P12_bit.no3
#define	RESERVE_1				P13_bit.no7

#define	RESERVE_3				P2_bit.no5		// new
#define	REMOTE_ON_OFF_SW		P12_bit.no2

#endif
/*  ************************************************************* */
#define	COMP_RLY		P5_bit.no2
#define	FAN_RLY			P5_bit.no3
#define	PUMP_RLY		P5_bit.no4
#define	HEATER_RLY		P5_bit.no5
#define	ERROR_RLY		P1_bit.no6
#define	RESERV_RLY		P1_bit.no5
/* *****************   ICEWATER LED/KEY Board define  ******************************** */
#define	MC14094_STRB	P14_bit.no6
#define	MC14094_CLK		P1_bit.no0
#define	MC14094_SER		P13_bit.no0
/*  ************************************************************* */
/* General Values */
#define LED_ON			(0)
#define LED_OFF			(1)
#define SET_BIT_HIGH	(1)
#define SET_BIT_LOW		(0)
#define SET_BYTE_HIGH	(0xFF)
#define SET_BYTE_LOW	(0x00)
#define	TRUE			(1)
#define	FALSE			(0)
#define	SET				(1)
#define	RESET			(0)
#define SetBit(VAR,Place)         ( (VAR) |= (uint8_t)((uint8_t)1<<(uint8_t)(Place)) )
#define ClrBit(VAR,Place)         ( (VAR) &= (uint8_t)((uint8_t)((uint8_t)1<<(uint8_t)(Place))^(uint8_t)255) )

#define	t_second	1000//1000*1ms
#define	t_minute	60000//60*1000*1ms
#define	t_hour		3600000//60*60*1000ms

#define step_motor_rpm 10   // old 5
#define PMV_STEP_MAX 2000
#define	PMV_STEP_Min_0 0
#define	PMV_STEP_Min_40 40

#define	PWM_CYCLE		120			// 12V about 0.12 sec
#define	PWM_CYCLE_MIN		4		//  0.67v ~ 0.68v full close
#define PWM_CYCLE_HELF	55		// 5.0v ~ 5.1v half open
#define	PWM_CYCLE_MAX	105		// 9.6v ~ 9.7v full open


/* *****************   IIC code area ******************************** */
#define 	SL_ADDR         (0xA0U)         /* Slave address */
#define 	DATA_LENGTH     (9U)           /* Tx/Rx data length. */
#define 	RESTART_COUNT   (10U)            /* Restart count value */
#define 	LOOP_COUNT_VALUE 205            /* loop_count_delay value old value 105 */


#define 	IIC_init_address	170			/* init data is 170=55, 171=aa */
#define 	IIC_key_address		172			/* init key data def 34=1,35=1  ( X* addr(34) + aadr(35) ) */

#define 	IIC_default_1 0x55
#define 	IIC_default_2 0xaa	

#define 	IIC_value_data 50
#define 	IIC_value_length 49

#define 	IIC_option_address 0
#define 	IIC_option_length 40

#define 	IIC_status_address 180
#define 	IIC_status_length 8

#define		IIC_PMV_Table 50
//#define		IIC_PMV_Time 104
#define		IIC_PMV_length 49

//#define		IIC_WaterGate_Table 122
//#define		IIC_WaterGate_Time 139
//#define		IIC_WaterGate_length 17

#define		IIC_checksum_length 178
#define		IIC_checksum_address	178

#define		IIC_error_record_address	220
#define		IIC_error_record_length	11

#define		op_error_record_index 0

extern	uint8_t	liquid_temp_buf;
extern 	uint8_t com_status;  
extern	uint8_t	IIC_operation_status[8];
extern	uint8_t	IIC_tx_buf[150];
extern	uint8_t	IIC_rx_buf[190];
extern	uint8_t	IIC_key_value_1,IIC_key_value_2;
extern 	uint8_t	IIC_error_code[15];

typedef enum{op_status_index,op_status_power,op_status_set_temp_H,op_status_set_temp_L
	,op_status_remote,op_status_t2,op_status_t3,op_status_t4}IIC_OPERATION_STATUS_T;


#if 1
	/* ******************** eeprom value  *********************************** */
#define	Liquid_high_temp_stop 80		 //0 -85
#define	Liquid_high_temp_stop_l 1		// old 0 for compile warning
#define	Liquid_high_temp_stop_h 130
	/* ****************************************************************** */
#define	Liquid_low_temp_stop 40			//20 
#define	Liquid_low_temp_stop_l 1		// old 0 for compile warning
#define	Liquid_low_temp_stop_h 130
	/* ****************************************************************** */
#define	Liquid_high_temp_alarm 10 		// 0-20
#define	Liquid_high_temp_alarm_l 0 
#define	Liquid_high_temp_alarm_h 20     
	/* ****************************************************************** */
#define	Liquid_low_temp_alarm -10		//-20 - 0
#define	Liquid_low_temp_alarm_l -20
#define	Liquid_low_temp_alarm_h 0	
	/* *************************** 4 *************************************** */
#define	Set_temp_high  40  				// 0-85 or -9.9 - +9.9   // testtt old 40 change to 30 for test
#define	Set_temp_high_l 0
#define	Set_temp_high_h 85
	/* *************************** 26 *************************************** */
#define	Set_temp_high_1  50  
#define	Set_temp_high_l_1 -99
#define	Set_temp_high_h_1 99
	/* **************************** 5 ************************************** */
#define	Set_temp_low  20				// 0-85 or -9.9 - +9.9	// testtt old 20 change to 10 for test
#define	Set_temp_low_l  0
#define	Set_temp_low_h  85
	/* *************************** 27 *************************************** */
#define	Set_temp_low_1  -50
#define	Set_temp_low_l_1  -99
#define	Set_temp_low_h_1  99
	/* ***************************** 6 ************************************* */
#define	Heater_start_offset  -50  // -9.9 - +9.9
#define	Heater_start_offset_l -99
#define	Heater_start_offset_h -1
	/* **************************** 7 ************************************** */
#define	Heater_stop_offset 50		// -9.9 - +9.9
#define	Heater_stop_offset_l 1
#define	Heater_stop_offset_h 99
	/* ***************************** 8 ************************************* */
#define	System_off_COMP_protect_timer_set  12	// 6 -18,  6*10 - 18*10
#define	System_off_COMP_protect_timer_set_l  6
#define	System_off_COMP_protect_timer_set_h  18
	/* *************************** 9 *************************************** */
#define	Control_sensor_offset  0		// -9.9 - +9.9
#define	Control_sensor_offset_l  -99
#define	Control_sensor_offset_h  99
	/* ************************** 10 **************************************** */
#if 0
#define	Reference_sensor_offset 0		// -9.9 - +9.9	
#define	Reference_sensor_offset_l -99
#define	Reference_sensor_offset_h 99
#endif
	/* ************************** 11 **************************************** */
#define	Base_sensor_offset  0			// -9.9 - +9.9
#define	Base_sensor_offset_l  -99
#define	Base_sensor_offset_h  99
	/* ****************************** 12 ************************************ */
#if 0
#define	Antifrz_sensor_offset 0			// -9.9 - +9.9
#define	Antifrz_sensor_offset_l -99	
#define	Antifrz_sensor_offset_h 99	
#endif	
	/* ******************************* 13 *********************************** */
#define	Flow_switch_start_check_time  10  // 0s - 20s
#define	Flow_switch_start_check_time_l  0
#define	Flow_switch_start_check_time_h  20
	
	/* *************************** 14 *************************************** */
#define	LP_start_check_time  30		// 1-60,  1*10 - 60*10
#define	LP_start_check_time_l  1
#define	LP_start_check_time_h  60
	/* *************************** 15 *************************************** */
#define	System_on_PUMP_delay  3		// 0-18,  1*10 - 18*10				
#define	System_on_PUMP_delay_l  0
#define	System_on_PUMP_delay_h  6
	/* ************************** 16 **************************************** */
#define	System_on_FAN_delay  3		// 1-18,  1*10 - 18*10				
#define	System_on_FAN_delay_l  0
#define	System_on_FAN_delay_h  6
	/* *************************** 17 *************************************** */
#define	System_on_COMP_start_delay  3		// 1-18,  1*10 - 18*10		
#define	System_on_COMP_start_delay_l  0
#define	System_on_COMP_start_delay_h  6
	/* ************************************ 18 ****************************** */
#define	System_off_PUMP_delay  6			// 0 - 24,  0*10 - 24*10
#define	System_off_PUMP_delay_l  0
#define	System_off_PUMP_delay_h  12
	/* ****************************** 19 ************************************ */
#define	System_off_FAN_delay  6			// 0 - 24,  0*10 - 24*10
#define	System_off_FAN_delay_l  0
#define	System_off_FAN_delay_h  12
	/* ******************************** 20 ********************************* */
#define	Temp_control_FAN_delay_off 6   // 0 - 24,  0*10 - 24*10
#define	Temp_control_FAN_delay_off_l 0
#define	Temp_control_FAN_delay_off_h 12
	/* *********************************** 21 ******************************* */
#define	Antifrz_protect_stop  0                     // 0 -20
#define	Antifrz_protect_stop_l  -50
#define	Antifrz_protect_stop_h  50
	/* ******************************** 22 ********************************** */
#define	Temp_control_COMP_protect_timer_set   12	// 6 -36,  6*10 - 36*10		
#define	Temp_control_COMP_protect_timer_set_l   6
#define	Temp_control_COMP_protect_timer_set_h   18
	/* ********************************* 23 ********************************* */
#define	COMP_start_temp_offset  20   // 1-80, 0.1*10 - 8*10          ****************** need to check code *********************
#define	COMP_start_temp_offset_l  1 
#define	COMP_start_temp_offset_h  40
	/* ********************************* 24 ********************************* */
#if 0
#define	Current_4_20mA_set  0  			 // 0 or 1     ****************** no used *********************
#endif
	/* ******************************* 25 *********************************** */
#define   System_off_PMV_delay 10		// 1s - 20s
#define   System_off_PMV_delay_l 1
#define   System_off_PMV_delay_h 20
	/* ******************************** 28 ********************************** */
#define   Condenser_temp_protect	65
#define   Condenser_temp_protect_l 	0
#define   Condenser_temp_protect_h 	70
	/* ********************************** 29 ******************************** */
#if 0
#define   Evaporator_temp_protect		12
#define   Evaporator_temp_protect_l 	-50
#define   Evaporator_temp_protect_h 	50
#endif
	/* *********************************** 30 ******************************* */
#define		comp_current_protect_0_def 100	
#define		comp_current_protect_0_l 0
#define		comp_current_protect_0_h 120
/* ************************************** 31 **************************** */
#define		comp_current_protect_1_def 100
#define		comp_current_protect_1_l 0
#define		comp_current_protect_1_h 120
/* ************************************ 32 ****************************** */
#define		comp_current_protect_2_def 100
#define		comp_current_protect_2_l 0
#define		comp_current_protect_2_h 120
/* ************************************** 33 **************************** */
#define		comp_current_protect_3_def 100	
#define		comp_current_protect_3_l 0
#define		comp_current_protect_3_h 120
	/* *********************************** 34 ******************************* */
#if 0
#define   Condenser_temp_offset		0
#define   Condenser_temp_offset_l 	99
#define   Condenser_temp_offset_h 	-99
	/* ********************************** 35 ******************************** */
#define   Evaporator_temp_offset		0
#define   Evaporator_temp_offset_l 		99
#define   Evaporator_temp_offset_h 		-99
	/* ************************************** 36 **************************** */
#define   Comp_out_temp_offset		0
#define   Comp_out_temp_offset_l 		99
#define   Comp_out_temp_offset_h 		-99
#endif
	/* ********************************** 37 ******************************** */
#define		Modbus_address_offset	1
#define		Modbus_address_max 	16
#define		Modbus_address_min	1
	/* ********************************** 37 ******************************** */
#define		PID_P_factor_def	10
#define		PID_P_factor_max 	99
#define		PID_P_factor_min	0
	/* ********************************** 37 ******************************** */
#define		PID_I_factor_def	10
#define		PID_I_factor_max 	99
#define		PID_I_factor_min	0
	/* ********************************** 37 ******************************** */
#define		PID_D_factor_def	10
#define		PID_D_factor_max 	99
#define		PID_D_factor_min	-99


#endif

#define		option_byte_count_min	_Liquid_high_temp_stop
#define		option_byte_count_max	_PID_D_factor_value



#if 0
typedef enum
{
	_Buzzer_Stop,_Buzzer_Start,_Buzzer_mute_period,_Buzzer_flash_period,_Buzzer_mute_period_2,_Buzzer_flash_period_2
}Buzzer_job_t;

typedef	enum
{
	_Buzzer_NoSound,_Buzzer_OneSound_Bi,_Buzzer_longSound_Bi,_Buzzer_TwoSound_BiBi,_Buzzer_longSound_BiBi,_Buzzer_longSound_BiBiBi
}Buzzer_bi_t;

extern	Buzzer_bi_t	Buzzer_flag;

#endif



/* *****************   IIC code area ******************************** */

typedef enum
{
		_Liquid_high_temp_stop,_Liquid_low_temp_stop,_Liquid_high_temp_alarm,_Liquid_low_temp_alarm,        						// 0-3
		_Set_temp_high,_Set_temp_low,_Set_temp_high_1,_Set_temp_low_1,																// 4-7
		_Heater_start_offset,_Heater_stop_offset,_System_off_COMP_protect_timer_set ,												// 8-10
		_Control_sensor_offset,_Base_sensor_offset,_Flow_switch_start_check_time,_LP_start_check_time,								// 11-14	
		_System_on_PUMP_delay,_System_on_FAN_delay,_System_on_COMP_start_delay,														// 15-17
		_System_off_PUMP_delay,_System_off_FAN_delay,_Condenser_temp_protect_value,													// 18-20
		_Antifrz_protect_stop,_System_off_PMV_delay,																				// 21-22
		_comp_current_protect_0_value,_comp_current_protect_1_value,_comp_current_protect_2_value,_comp_current_protect_3_value,	// 23-26
		_Modbus_address_offset,																										// 27
		_COMP_start_temp_offset,_Temp_control_COMP_protect_timer_set,_Temp_control_FAN_delay_off,									// 28-30
		_PID_P_factor_value,_PID_I_factor_value,_PID_D_factor_value																	// 31-33

}EEPROM_BUFFER_T;


typedef enum{_PRODUCTION_mode_1,_PRODUCTION_mode_2,_PRODUCTION_mode_3,_PRODUCTION_mode_4,_PRODUCTION_mode_5,_PRODUCTION_mode_6}PRODUCTION_MODE_DEFINE;
			
typedef enum{_LED_mode_1,_LED_mode_2,_LED_mode_4,_LED_mode_5,_LED_mode_6,_LED_mode_7,_LED_mode_8,_LED_mode_9,_LED_mode_option,_LED_key_check}LED_MODE_DEFINE;
	
#if 1
	typedef enum{op_state_index,op_state_power,op_state_set_temp_H,op_state_set_temp_L
		,op_state_resever1,op_state_resever2,op_state_resever3,op_stateresever4}OP_STATE_T;
					
#endif





extern	uint8_t	system_sensor_status_on;
extern 	uint8_t	error_code_backup,option_backup,LED_mode_count;

extern	int16_t 	WATERGATE_temp_delta;
extern	int16_t 	PMV_temp_delta;

extern	int16_t	step_motor_timer,motor_current_position_1,motor_current_position_2,motor_current_position_3, 
				motor_wish_position,motor_step_delta;
extern	int8_t	pwm_set,pwm_set_delta;

extern	int16_t	motor_current_position_1,motor_current_position_2,motor_current_position_3, motor_wish_position;

extern	int8_t	eeprom_option_byte[IIC_option_length+3];
extern	int8_t	eeprom_error_record[120];
extern  uint8_t	eeprom_operation[8];
extern	int8_t	relay_test_timer;
//extern	SET_MODE_T	SET_MODE,PRE_SET_MODE,REAL_MODE;
//extern	SET_FAN_T	SET_FAN,PRE_SET_FAN,REAL_FAN,PRE_REAL_FAN;
//extern	uint8_t	sys_op_power,sys_op_on_timer,sys_op_off_timer,sys_op_airclean,sys_op_swing,sys_op_rover,sys_op_sleep;
//extern	int16_t	KEY_ADV;
//extern	uint8_t	POWER_TIMER_ON,POWER_TIMER_OFF;

extern	uint8_t	adc_conv_end;
extern	int16_t	SET_TEMP;
extern	int16_t	CONTROL_TEMP,CONTROL_TEMP_TEST;
extern	int16_t	COMP_OUT_TEMP;
extern	int16_t	BASE_TEMP;
extern	int16_t	ANTIFRZ_TEMP;
extern	uint8_t	ERROR_CODE;
extern 	int16_t CONDENSER_TEMP;

/* ******************** eeprom value  *********************************** */
extern int8_t 	COMP_ON_TEMP_ADJUST;
extern int8_t	COMP_OFF_TEMP_ADJUST;
extern int8_t	TEMP_SELECT;
extern int8_t   TEMP_ADJUST;
extern int8_t   TEMP_RANGE_SELECT;
extern int8_t	POWERON_DEFAULT_SET;
extern uint16_t	COMP_PROTECT_TIME_SET;
extern uint16_t	POWERON_COMP_PROTECT_TIME_SET;
extern uint16_t LP_COMP_COUNT;
extern uint16_t	LP_COMP_DEFAULT_DELAY;

/* ******************** WATERGATE &   *********************************** */
#if 0
extern	int8_t 	WATERGATE_CONTROL_TABLE[17];
extern	int8_t	WATERGATE_TABLE_TIME[17];
extern	int8_t	WATERGATE_TABLE_VALUE[2];
extern	int8_t	PMV_TABLE_TIME[7];
#endif
extern	int8_t	PMV_TABLE_STEP[49];
/* ******************** system operation   *********************************** */
extern	uint8_t	IIC_operation_status[8],Modbus_address;
//extern	int16_t	filter_sec_count,filter_10hour_count;
/* ********************************************************************** */
#define	AD_total_chanel	20

extern	uint8_t	HC164_display_arry[5],error_code_backup_count;
extern	const uint8_t	seven_seg_code[];
extern	uint8_t 	LED_mode_count,key_repeart_timer;
extern	uint8_t		LED_1_dot,LED_2_dot,LED_3_dot,LED_RUN,LED_ALARM,LED_PV,LED_SV;
extern	uint16_t	T_DECP[AD_total_chanel];
extern 	uint16_t	key_data;
extern	int8_t		current_detect_data;
extern	uint16_t	ms_counter,mac_sec_count;
extern	uint16_t	AD_ANI7_result[10];/*Array for A/D ANI7 result*/
extern	int8_t		production_mode; 
extern	uint16_t	voltage_data_now,voltage_data_max,voltage_data_min
						,current_data_now,current_data_max,current_data_min,current_key_now,current_data_temp;

#define	default_220v 675
#define	default_0A	512
#define	key_timer_default	200
/* ********************************************************************** */

/* *****************   voltage value define ******************************** */
#define 	voltage_187_value	562
#define		voltage_253_value	783
#define		voltage_small_relay_offset	12
#define		voltage_large_relay_offset	33
/* *****************   IIC code area ******************************** */
#if 0
extern	flag_t	_sys_op_statue_;
#define	_sys_op_statue	_sys_op_statue_.flag
#define	sys_op_power	_sys_op_statue_.bit.no0
#define	sys_op_sleep	_sys_op_statue_.bit.no1
#define	sys_op_rover	_sys_op_statue_.bit.no2
#define	sys_op_swing	_sys_op_statue_.bit.no3
#define	sys_op_airclean		_sys_op_statue_.bit.no4
#define	sys_op_off_timer	_sys_op_statue_.bit.no5
#define	sys_op_on_timer		_sys_op_statue_.bit.no6


extern	flag_t	_Error_state_;
#define	_Error_state	_Error_state_.flag
#define	Error_state_E1	_Error_state_.bit.no0
#define	Error_state_E2	_Error_state_.bit.no1
#define	Error_state_E6	_Error_state_.bit.no2
#define	Error_state_EH	_Error_state_.bit.no3

extern	flag_t	_dip_switch_flag;
#define	dip_switch_flag       _dip_switch_flag.flag
#define	remote_code_AB		_dip_switch_flag.bit.no0
#define	display_roomset_humi		_dip_switch_flag.bit.no1
#define	force_comp_off		_dip_switch_flag.bit.no2
#define	fan_delay_off		_dip_switch_flag.bit.no2


extern	flag_t	_e2prom_flag;
#define	e2prom_flag       _e2prom_flag.flag
#define	e2prom_active				_e2prom_flag.bit.no0
#define	write_state_to_e2prom		_e2prom_flag.bit.no3
#define	write_option_to_e2prom		_e2prom_flag.bit.no4
#define	write_error_to_e2prom		_e2prom_flag.bit.no5
#define	read_state_from_e2prom		_e2prom_flag.bit.no6
#define	new_stm8s_mcu				_e2prom_flag.bit.no7

#endif


extern 	flag_t	_current_sys_op_status_;
#define  _current_sys_op_status	_current_sys_op_status_.flag
#define  current_sys_op_power	_current_sys_op_status_.bit.no0
#define  remote_on_off_flag		 _current_sys_op_status_.bit.no1
#define  deice_on_flag			 _current_sys_op_status_.bit.no2
#define  comp_rly_flag			 _current_sys_op_status_.bit.no3
#define  pump_rly_flag		 _current_sys_op_status_.bit.no4
#define  heater_rly_flag		_current_sys_op_status_.bit.no5
#define  fan_rly_flag				 _current_sys_op_status_.bit.no6


/* debug flag area */
#define PMV_TEST 0			/* 0 for debug off 1 for debug on */

#define ERROR_RLY_NORMAL 0   /* 1 for normal operation, 0 for error relay not on */
#define SENSOR_DEBUNSE	20   /* 1 for 0.1 sec */
#define SW_BASE	34
#define	ERROR_BACKUP_SET	2	/* 1 for 1 sec */
#define	STATE_BACKUP_SET	4	/* 1 for 1 sec */

#if ERROR_RLY_NORMAL
#define SW_version     SW_BASE
#else
#define SW_version     35
#endif





/* End user code. Do not edit comment generated here */
#endif
