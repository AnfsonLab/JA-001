#include "r_cg_macrodriver.h"
#include "r_cg_adc.h"
#include "r_cg_userdefine.h"
#include "R_system.h"

/***********************************************************************************************************************
Global variables and functions
***********************************************************************************************************************/
uint16_t	ms_counter;

flag_t	_current_sys_op_status_;

int16_t	COMP_SET_TEMP_ON,COMP_SET_TEMP_OFF,COMP_REAL_TEMP,HEATER_START_TEMP,HEATER_STOP_TEMP;

uint8_t	ERROR_CODE=0,comp_over_heat = RESET,FLOW_SWITCH_PROTECT_ON=0,ANTIFRZ_PROTECT_ON=0;

int16_t	comp_protect_timer;
int16_t	PUMP_DELAY,FAN_DELAY;
int16_t	one_sec_counter,zero_point_one_sec;
int16_t	COMP_on_count,PUMP_on_count;

int16_t	FLOW_SWITCH_PROTECT_DELAY,LP_SENSOR_DELAY,CONDENSER_PROTECT_DELAY;

int8_t 	COMP_TOP_TEMP_PROTECT_DELAY,COMP_OVERLOAD_PROTECT_DELAY,FAN_OVERLOAD_PROTECT_DELAY,
		HP_SENSOR_DELAY,ANTIFRZ_DELAY;
int8_t 	PUMP_OVERLOAD_PROTECT_DELAY,WATERLEVER_LOW_ALARM_DELAY,
		WATERLEVER_HIGH_ALARM_DELAY,WATER_HIGH_TEMP_STOP_DELAY,WATER_LOW_TEMP_STOP_DELAY,
		WATER_HIGH_TEMP_ALARM_DELAY,WATER_LOW_TEMP_ALARM_DELAY;
int8_t	POWER_PHASE_PROTECT_DELAY,HEATER_TEMP_PROTECT_DELAY,REMOTE_DEBOUNCE_DELAY,REMOTE_DEBOUNCE_DELAY_1;
int8_t	CONTROL_TEMP_DELAY,BASE_TEMP_DELAY,CONDENSER_TEMP_DELAY,ANTIFRZ_TEMP_DELAY,COMP_OUT_SENSOR_DELAY;

int8_t	water_lever_error_flag_high=0,water_lever_error_flag_low=0,water_temp_error_flag_high=0,water_temp_error_flag_low=0;
int8_t	VOLTAGE_DETECT_DELAY,VOLTAGE_DETECT_LOW_DELAY,CURRENT_DETECT_DELAY;

uint8_t	system_sensor_status_on = 1;
int8_t	system_on_fan_comp_flag,ii;
uint16_t	current_data_temp,power_on_delay,voltage_check_data;

/**    step motor area ************************************************************************** */

int16_t	step_motor_timer,motor_current_position_1,motor_current_position_2,motor_current_position_3, 
		motor_wish_position,motor_step_delta;
// uint8_t	STEP_motor_table[8]={0x06,0x0e,0x0c,0x0d,0x09,0x0b,0x03,0x07};   	// UKV-A179 SAGINOMIYA
uint8_t	STEP_motor_table[8]={0x09,0x03,0x06,0x0c,0x09,0x03,0x06,0x0c};		// K50000-817G FUJIKOKI
// uint8_t	STEP_motor_table[8]={0x08,0x0c,0x04,0x06,0x02,0x03,0x01,0x09};
uint8_t	motor_position_index,step_motor_no;
/**    step motor area ************************************************************************** */
int8_t	PMV_TABLE_STEP[49] =   { 	
								 20, 3, -8, 8,  2, 1, 5,
								 10, 3, -5, 5,  6, 1, 5,
								  5, 3, -3, 3, 20, 0, 2,
								  0, 3, -3, 3, 20, 1, -2,
								 -5, 3, -5, 5, 20, 1, 0,
								-10, 3, -5, 5, 20, 1, 0,
								-20, 3, -8, 8, 20, 1, 0
								};

								
int8_t	PMV_TIMER_COUNT,PMV_TIMER_SET = 3;
int16_t PMV_temp_delta = 23,PID_temp_old;
int8_t	PID_temp_pointer;
int16_t	PID_P,PID_I,PID_D,PID_M,PID,PID_old_1,PID_old_2,PID_P_factor,PID_I_factor,PID_D_factor;
int16_t	PID_timer,PID_temp = 230,PID_temp_buf[40],step_low,step_high,step_max;
int8_t	temp_range_old;

int16_t	test_1,test_2;	

void	sensor_operation(void);
void	system_operation(void);
void	system_operation_fix_temp(void);
void	sensor_operation_continue(void);
void	Heater_operation(void);
void	debug_operation(void);
void	UART1_send_temp(void);
void	system_sensor_status(void);

extern	uint8_t	state_code_backup_count;

/*******************************************************************************
* Function name	: PMV_close_operation
* Description 	: none
* Argument 	: none
* Return value  : none
*******************************************************************************/
void PMV_close(void)
{


	/* PMV position close */
	motor_wish_position = 0;
	motor_current_position_1 = PMV_STEP_MAX+200;
	motor_current_position_2 = PMV_STEP_MAX+200;
	motor_current_position_3 = PMV_STEP_MAX+200;


}

/*******************************************************************************
* Function name	: PMV_operation
* Description 	: none
* Argument 	: none
* Return value  : none
*******************************************************************************/
#define	step_min	0
void PMV_operation(void)
{
	PID_P_factor = eeprom_option_byte[_PID_P_factor_value];
	PID_I_factor = eeprom_option_byte[_PID_I_factor_value];
	PID_D_factor = eeprom_option_byte[_PID_D_factor_value];
	

#if 1
	if(TEMP_SELECT_SWITCH == FIX_TYPE_SENSOR)
	{
		COMP_REAL_TEMP = CONTROL_TEMP + (int16_t)eeprom_option_byte[_Control_sensor_offset];
		COMP_SET_TEMP_OFF = SET_TEMP;
		if(COMP_SET_TEMP_OFF < 0)
			COMP_SET_TEMP_OFF = 0;
		PMV_temp_delta =COMP_REAL_TEMP -  COMP_SET_TEMP_OFF;
	}
	else
	{
		COMP_REAL_TEMP = (CONTROL_TEMP + (int16_t)eeprom_option_byte[_Control_sensor_offset])
							- (BASE_TEMP + (int16_t)eeprom_option_byte[_Base_sensor_offset]);
		
		COMP_SET_TEMP_OFF = SET_TEMP;
		PMV_temp_delta =COMP_REAL_TEMP -  COMP_SET_TEMP_OFF;
	}
#endif	

#if PMV_TEST
	
	PMV_temp_delta = CONTROL_TEMP_TEST - SET_TEMP;

#endif

/* check for start PMV control ************************************ */
	if( PMV_temp_delta <= PMV_TABLE_STEP[0] )
	{
		ii=0;
		if( PMV_temp_delta <= PMV_TABLE_STEP[1*7] )
			ii=1;
		if( PMV_temp_delta <= PMV_TABLE_STEP[2*7] )
			ii=2;		
		if( PMV_temp_delta <= PMV_TABLE_STEP[3*7] )
			ii=3;
		if( PMV_temp_delta <= PMV_TABLE_STEP[4*7] )
			ii=4;
		if( PMV_temp_delta <= PMV_TABLE_STEP[5*7] )
			ii=5;
		if( PMV_temp_delta <= PMV_TABLE_STEP[6*7] )
			ii=6;
		
		PMV_TIMER_SET = PMV_TABLE_STEP[(ii*7)+1];
		if(PMV_TIMER_SET < 1 || PMV_TIMER_SET > 10)
			PMV_TIMER_SET = 3;
		step_low = PMV_TABLE_STEP[(ii*7)+2]; 
		step_high = PMV_TABLE_STEP[(ii*7)+3]; 
		step_max = PMV_TABLE_STEP[(ii*7)+4]*100;
		
		// PID_P = PID_P_factor * PMV_temp_delta;   // new spec. 2020/05/29

		/* ***************************************************** */
		PID_P = PID_P_factor * (SET_TEMP - COMP_REAL_TEMP);
#if PMV_TEST
		PID_P = PID_P_factor * (SET_TEMP - CONTROL_TEMP_TEST); 
#endif
		PID_P = PID_P / 10;
		/* ***************************************************** */
		PID_I = PID_I_factor * (PID_old_1);    // new spec. 2020/05/29
		/* ***************************************************** */
#if PMV_TEST
		PID_temp = CONTROL_TEMP_TEST;
#else
		PID_temp = COMP_REAL_TEMP;
#endif	
		PID_M = (PID_temp_old - PID_temp)/3;
		PID_temp_old = PID_temp;
		PID_D = PID_D_factor * PID_M;
		PID_D = PID_D / 10;
		/* ***************************************************** */	
		PID = PID_P + PID_I + PID_D;
		if(PID%10 == 0)
		{
			PID = PID /10;
		}
		else
		{
			if(PID >= 0)
			{
				PID = PID /10;
				PID++;
			}
			else
			{
				PID = PID /10;
				PID--;
			}	
		}
		
		if((ii==2) || (ii==3))
		{
			if((PID_M < PMV_TABLE_STEP[(2*7)+6]) && (PID_M > PMV_TABLE_STEP[(3*7)+6]))
				PID =  PID_D;
		}

#if 0		
		if((ii == 2) && (PID_M < PMV_TABLE_STEP[(ii*7)+6]))
			PID =  PID_D;
		if((ii == 3) && (PID_M > PMV_TABLE_STEP[(ii*7)+6]))
			PID =  PID_D;
#endif

		if(PID < step_low)
			PID = step_low;
		if(PID > step_high)
			PID = step_high;
		
		motor_wish_position = motor_wish_position + PID;

		if(motor_wish_position >= step_max)
			motor_wish_position = step_max;
		if(motor_wish_position <= step_min)
			motor_wish_position = step_min;	
		
		PID_old_2 = PID_old_1;
		PID_old_1 = PID;
		
		if(ii != temp_range_old)
		{
			temp_range_old = ii;
			if(PID_M > 5)
			{
				if(ii==1)
					motor_wish_position = 200;
				if(ii==2)
					motor_wish_position = 600;
			}
		}

	}
	else
	{
		motor_wish_position = 0;
	}
		
	UART1_send_temp();

	
}
/*******************************************************************************
End of function PMV_operation
*******************************************************************************/

/*******************************************************************************
* Function name	: turn_system_on
* Description 	: none
* Argument 	: none
* Return value  : none
*******************************************************************************/
/*******************************************************************************
End of function turn_system_on
*******************************************************************************/
void	turn_system_on(void)
{
	current_sys_op_power = SET;
	if(PUMP_RLY == RESET)
		PUMP_DELAY = (int16_t)eeprom_option_byte[_System_on_PUMP_delay]*step_10_sec;
	
	// PMV_close();

}
/*******************************************************************************
End of function turn_system_on
*******************************************************************************/



/*******************************************************************************
* Function name	: turn_system_off
* Description 	: none
* Argument 	: none
* Return value  : none
*******************************************************************************/
void	turn_system_off(void)
{
	if(current_sys_op_power == SET)
	{
		current_sys_op_power = RESET;
		PMV_close();
	}
}

/*******************************************************************************
End of function turn_system_off
*******************************************************************************/
/*******************************************************************************
* Function name : system_sensor_status
* Description	: none
* Argument	: none
* Return value	: none
*******************************************************************************/
void	system_sensor_status(void)
{
	switch(ERROR_CODE)
	{
		case 0x00:
		case 0xd1:	
		case 0xd3:
		case 0xd6:
		case 0xd7:
		case 0xc4:
		case 0xc6:
		case 0xc7:
			system_sensor_status_on = 1;
			break;
		default:
			system_sensor_status_on = 0;
			break;
	}

}
/*******************************************************************************
End of function turn_system_on
*******************************************************************************/


/*******************************************************************************
* Function name	: power_on_timer_reset
* Description 	: none
* Argument 	: none
* Return value  : none
*******************************************************************************/
void	power_on_timer_reset(void)
{
	
	PUMP_DELAY =0;
	comp_protect_timer = 0;
	FLOW_SWITCH_PROTECT_DELAY =0;
	PUMP_OVERLOAD_PROTECT_DELAY =0;
	WATERLEVER_LOW_ALARM_DELAY =0;
	WATERLEVER_HIGH_ALARM_DELAY = 0;
	WATER_HIGH_TEMP_STOP_DELAY = 0;
	WATER_LOW_TEMP_STOP_DELAY = 0;
	WATER_HIGH_TEMP_ALARM_DELAY = 0;
	WATER_LOW_TEMP_ALARM_DELAY = 0;
	POWER_PHASE_PROTECT_DELAY = 0;
	HEATER_TEMP_PROTECT_DELAY = 0;
	HP_SENSOR_DELAY = 0;
	HP_SENSOR_DELAY =0;
	LP_SENSOR_DELAY =0;
	ANTIFRZ_DELAY =0;
	REMOTE_DEBOUNCE_DELAY = 0;
	deice_on_flag = RESET;
//	current_sys_op_power = RESET;
	remote_on_off_flag = RESET;
	
	one_sec_counter = 1000;
	zero_point_one_sec = 1000;
	step_motor_timer = step_motor_rpm;
	
	
}
/*******************************************************************************
End of functiontimer_counter_down
*******************************************************************************/



/*******************************************************************************
* Function name : step_motor_1_operation
* Description	: none
* Argument	: none
* Return value	: none
*******************************************************************************/
void	step_motor_1_operation(void)
{
/* ******  STEP MOTOR CONTROL   ******************************************************************** */
	if(step_motor_timer-- <= 0)
	{
		step_motor_timer = step_motor_rpm;
		if(motor_current_position_1 == motor_wish_position)
		{
			STEPMOTOR_A_1 = RESET;
			STEPMOTOR_b_1= RESET;
			STEPMOTOR_a_1 = RESET;
			STEPMOTOR_B_1 = RESET;
			motor_position_index = 0;
			step_motor_no = 1;
		}
		if(motor_current_position_1 < motor_wish_position)
		{
			motor_position_index = motor_position_index +1;
			motor_position_index = motor_position_index & 0x07;
						
			if ((STEP_motor_table[motor_position_index] & 0x08))
				STEPMOTOR_A_1 = RESET;
			else
				STEPMOTOR_A_1 = SET;
			
			if ((STEP_motor_table[motor_position_index] & 0x04))
				STEPMOTOR_b_1 = RESET;
			else
				STEPMOTOR_b_1 = SET;
			
			if ((STEP_motor_table[motor_position_index] & 0x02))
				STEPMOTOR_a_1 = RESET;
			else
				STEPMOTOR_a_1 = SET;
			
			if ((STEP_motor_table[motor_position_index] & 0x01))
				STEPMOTOR_B_1 = RESET;
			else
				STEPMOTOR_B_1 = SET;

			motor_current_position_1++;
		}
		if(motor_current_position_1 > motor_wish_position)
		{
			motor_position_index = motor_position_index -1;
			motor_position_index = motor_position_index & 0x07;
						
			if (STEP_motor_table[motor_position_index] & 0x08)
				STEPMOTOR_A_1 = RESET;
			else
				STEPMOTOR_A_1 = SET;
			
			if (STEP_motor_table[motor_position_index] & 0x04)
				STEPMOTOR_b_1 = RESET;
			else
				STEPMOTOR_b_1 = SET;
			
			if (STEP_motor_table[motor_position_index] & 0x02)
				STEPMOTOR_a_1 = RESET;
			else
				STEPMOTOR_a_1 = SET;
			
			if (STEP_motor_table[motor_position_index] & 0x01)
				STEPMOTOR_B_1 = RESET;
			else
				STEPMOTOR_B_1 = SET;	

			motor_current_position_1--;
		}
	}
}
/*******************************************************************************
End of function step_motor_1_operation
*******************************************************************************/

/*******************************************************************************
* Function name : step_motor_1_operation
* Description	: none
* Argument	: none
* Return value	: none
*******************************************************************************/
void	step_motor_2_operation(void)
{
/* ******  STEP MOTOR CONTROL   ******************************************************************** */
	if(step_motor_timer-- <= 0)
	{
		step_motor_timer = step_motor_rpm;
		if(motor_current_position_2 == motor_wish_position)
		{
			STEPMOTOR_A_2 = RESET;
			STEPMOTOR_b_2= RESET;
			STEPMOTOR_a_2 = RESET;
			STEPMOTOR_B_2 = RESET;
			motor_position_index = 0;
			step_motor_no = 2;
			
		}
		if(motor_current_position_2 < motor_wish_position)
		{
			motor_position_index = motor_position_index +1;
			motor_position_index = motor_position_index & 0x07;
						
			if ((STEP_motor_table[motor_position_index] & 0x08))
				STEPMOTOR_A_2 = RESET;
			else
				STEPMOTOR_A_2 = SET;
			
			if ((STEP_motor_table[motor_position_index] & 0x04))
				STEPMOTOR_b_2 = RESET;
			else
				STEPMOTOR_b_2 = SET;
			
			if ((STEP_motor_table[motor_position_index] & 0x02))
				STEPMOTOR_a_2 = RESET;
			else
				STEPMOTOR_a_2 = SET;
			
			if ((STEP_motor_table[motor_position_index] & 0x01))
				STEPMOTOR_B_2 = RESET;
			else
				STEPMOTOR_B_2 = SET;

			motor_current_position_2++;
		
		}
		if(motor_current_position_2 > motor_wish_position)
		{
			motor_position_index = motor_position_index -1;
			motor_position_index = motor_position_index & 0x07;
						
			if (STEP_motor_table[motor_position_index] & 0x08)
				STEPMOTOR_A_2 = RESET;
			else
				STEPMOTOR_A_2 = SET;
			
			if (STEP_motor_table[motor_position_index] & 0x04)
				STEPMOTOR_b_2 = RESET;
			else
				STEPMOTOR_b_2 = SET;
			
			if (STEP_motor_table[motor_position_index] & 0x02)
				STEPMOTOR_a_2 = RESET;
			else
				STEPMOTOR_a_2 = SET;
			
			if (STEP_motor_table[motor_position_index] & 0x01)
				STEPMOTOR_B_2 = RESET;
			else
				STEPMOTOR_B_2 = SET;	

			motor_current_position_2--;
		}
		
	  	
	}

}
/*******************************************************************************
End of function step_motor_1_operation
*******************************************************************************/

/*******************************************************************************
* Function name : step_motor_1_operation
* Description	: none
* Argument	: none
* Return value	: none
*******************************************************************************/
void	step_motor_3_operation(void)
{
/* ******  STEP MOTOR CONTROL   ******************************************************************** */
	if(step_motor_timer-- <= 0)
	{
		step_motor_timer = step_motor_rpm;
		if(motor_current_position_3 == motor_wish_position)
		{
			STEPMOTOR_A_3 = RESET;
			STEPMOTOR_b_3= RESET;
			STEPMOTOR_a_3 = RESET;
			STEPMOTOR_B_3 = RESET;
			motor_position_index = 0;
			step_motor_no = 0;
		}
		if(motor_current_position_3 < motor_wish_position)
		{
			motor_position_index = motor_position_index +1;
			motor_position_index = motor_position_index & 0x07;
						
			if ((STEP_motor_table[motor_position_index] & 0x08))
				STEPMOTOR_A_3 = RESET;
			else
				STEPMOTOR_A_3 = SET;
			
			if ((STEP_motor_table[motor_position_index] & 0x04))
				STEPMOTOR_b_3 = RESET;
			else
				STEPMOTOR_b_3 = SET;
			
			if ((STEP_motor_table[motor_position_index] & 0x02))
				STEPMOTOR_a_3 = RESET;
			else
				STEPMOTOR_a_3 = SET;
			
			if ((STEP_motor_table[motor_position_index] & 0x01))
				STEPMOTOR_B_3 = RESET;
			else
				STEPMOTOR_B_3 = SET;

			motor_current_position_3++;
		}
		if(motor_current_position_3 > motor_wish_position)
		{
			motor_position_index = motor_position_index -1;
			motor_position_index = motor_position_index & 0x07;
						
			if (STEP_motor_table[motor_position_index] & 0x08)
				STEPMOTOR_A_3 = RESET;
			else
				STEPMOTOR_A_3 = SET;
			
			if (STEP_motor_table[motor_position_index] & 0x04)
				STEPMOTOR_b_3 = RESET;
			else
				STEPMOTOR_b_3 = SET;
			
			if (STEP_motor_table[motor_position_index] & 0x02)
				STEPMOTOR_a_3 = RESET;
			else
				STEPMOTOR_a_3 = SET;
			
			if (STEP_motor_table[motor_position_index] & 0x01)
				STEPMOTOR_B_3 = RESET;
			else
				STEPMOTOR_B_3 = SET;	

			motor_current_position_3--;
		}
	}
}
/*******************************************************************************
End of function step_motor_3_operation
*******************************************************************************/



/*******************************************************************************
* Function name : comp_protect_operation
* Description	: none
* Argument	: none
* Return value	: none
*******************************************************************************/
void	comp_protect_operation(void)
{
	if(COMP_OUT_TEMP > 1100)
	{
		if(comp_over_heat == RESET)
		{
			error_code_backup = 0Xff;
			COMP_RLY = RESET;
			comp_protect_timer =  (int16_t)eeprom_option_byte[_System_off_COMP_protect_timer_set]*step_10_sec;
			comp_over_heat = SET;
			ERROR_RLY = SET;
			ERROR_CODE =  0xc7;
			PMV_close();
		}
		else
		{
			if(comp_protect_timer <= 5)
				comp_protect_timer =  5;
		}
		
	}
	else
	{
		if(comp_over_heat == SET)
		{
			if(comp_protect_timer <= 5)
				comp_protect_timer =  5;
		}
		if(COMP_OUT_TEMP  <=950)
		{
			if(ERROR_CODE ==  0xc7)
			{
				ERROR_CODE =  00;
				ERROR_RLY = RESET;
			}
			comp_over_heat = RESET;
		}
		
	}
}




/*******************************************************************************
* Function name : timer_counter_down
* Description	: none
* Argument	: none
* Return value	: none
*******************************************************************************/
void	timer_counter_down(void)
{
/* *** PWM control *********************************************************************** */

	//   ms_counter++;

/* *** PID temp **************************************************** */
	if(PID_timer -- <= 0)
	{
		int8_t PID_loop;
		
		PID_timer = 100;
		
		PID_temp_buf[PID_temp_pointer++] = COMP_REAL_TEMP;
		if(PID_temp_pointer == 30)
			PID_temp_pointer = 0;
		PID_temp_buf[31] = 0;
		for(PID_loop = 0;PID_loop<=29;PID_loop++ )
		{
			PID_temp_buf[31] = PID_temp_buf[31] + PID_temp_buf[PID_loop];
		}
		PID_temp = PID_temp_buf[31]/30;       // testtt for PID
	}	


#if 0
	if(pwm_timer++ < PWM_CYCLE)
	{
		if(pwm_timer > pwm_set)
      		{
      			PWM_PORT = SET;
      		}
		else
		{
			PWM_PORT = RESET;
		}
	}
	else
		pwm_timer =0;
#endif	
/* ******  STEP MOTOR CONTROL   ******************************************************************** */
	switch(step_motor_no)
	{
		case 0:
			step_motor_1_operation();
			break;
		case 1:
			step_motor_2_operation();
			break;
		case 2:
			step_motor_3_operation();
			break;
		default:
			step_motor_no = 0;
			break;
	}
	
	if(current_sys_op_power == RESET)     
	{
		if(( motor_wish_position == 0) && (motor_current_position_1 == 0) &&  (motor_current_position_2 == 0)
			 &&  (motor_current_position_3 == 0))
		{
			motor_wish_position = PMV_STEP_Min_40;
		}
	}

/* ********  0.1 sec operation ****************************************************************** */
#if 1     
// for testtt system operation with check error
	if(zero_point_one_sec -- <= 0)
	{
		zero_point_one_sec = 100;
		if((production_mode == RESET) && (LED_mode_count != _LED_mode_option))
		{
			system_sensor_status();
			if(system_sensor_status_on)
			{
				sensor_operation_continue();
				sensor_operation(); 
			}

#if 0		
			// if(ERROR_CODE==0 || ERROR_CODE==0xd3 || ERROR_CODE==0xd4 || ERROR_CODE==0xd7 || ERROR_CODE==0xd8)
			if(ERROR_CODE==0 || ERROR_CODE==0xd3 || ERROR_CODE==0xd6 || ERROR_CODE==0xd7)	
			{
				sensor_operation_continue();
				sensor_operation(); 
			}
			else	if(ERROR_CODE==0xd1 || ERROR_CODE==0xc6 )
					{
						sensor_operation_continue();
						sensor_operation(); 
					}
#endif			
		}
	}
#endif
	
	if( one_sec_counter -- <= 0)
	{
		one_sec_counter = 1000;

		//LED_display();
		
		debug_operation();
		if((production_mode == RESET) && (LED_mode_count != _LED_mode_option))
		{
			if(SYSTEM_SELECT_SWITCH== RESET)
			{
				system_sensor_status();
				if(system_sensor_status_on)
				{
					system_operation();
					Heater_operation();
				}


				system_sensor_status();
				if(system_sensor_status_on)
				{
					comp_protect_operation();
					if(comp_over_heat == RESET)
					{
						system_operation();
						Heater_operation();
					}
				}


#if 0

			// if(ERROR_CODE==0 || ERROR_CODE==0xd3 || ERROR_CODE==0xd4 || ERROR_CODE==0xd7 || ERROR_CODE==0xd8)
				if(ERROR_CODE==0 || ERROR_CODE==0xd3 || ERROR_CODE==0xd6 || ERROR_CODE==0xd7)
				{
					comp_protect_operation();
					if(comp_over_heat == RESET)
					{
						system_operation();
						Heater_operation();
					}
				}
				else	if( ERROR_CODE==0xc7  )
						{
							comp_protect_operation();
							if(comp_over_heat == RESET)
							{
								system_operation();
								Heater_operation();
							}
						}	
#endif				
			}
			else
			{
				system_sensor_status();
				if(system_sensor_status_on)
				{
					comp_protect_operation();
					if(comp_over_heat == RESET)
					{
						system_operation_fix_temp();   // need to check add heater operation or not
						Heater_operation();
					}
				}
#if 0
				switch(ERROR_CODE)
				{
					case 0x00:
					case 0xd3:
				//	case 0xd4:
					case 0xd6:
					case 0xd7:
					case 0xc6:
					case 0xc7:
						comp_protect_operation();
						if(comp_over_heat == RESET)
						{
							system_operation_fix_temp();   // need to check add heater operation or not
							Heater_operation();
						}
						break;
					default:
						break;
				}
#endif
			}
		}

		if(COMP_RLY)
		{
			if(COMP_on_count ++ >= 360)
				COMP_on_count = 360;
		}
		else
		{
			COMP_on_count = 0;
		}
		
		if(PUMP_on_count ++ >= 10)
			PUMP_on_count = 10;
		
		if(PMV_TIMER_COUNT != 0 )
		{
			PMV_TIMER_COUNT --;
		}

#if 0
		if(WATERGATE_TIMER_COUNT != 0 )
		{
			WATERGATE_TIMER_COUNT --;
		}
#endif

		if(error_code_backup_count < ERROR_BACKUP_SET)
		{
			error_code_backup_count++;
		}

		
		if(state_code_backup_count < STATE_BACKUP_SET)
		{
			state_code_backup_count++;
		}


		
		
		if(PUMP_DELAY != 0 )
		{
			PUMP_DELAY --;
		}
		
		if(FAN_DELAY != 0 )
		{
			FAN_DELAY --;
		}
	
		if(comp_protect_timer != 0 )
		{
			comp_protect_timer--;
		}
	}
}
/*******************************************************************************
End of functiontimer_counter_down
*******************************************************************************/
/*******************************************************************************
* Function name	: Heater_operation
* Description 	: none
* Argument 	: none
* Return value  : none
*******************************************************************************/
void	Heater_operation(void)
{
	/*****************   system ON operation  ****************/
	if((PUMP_RLY == SET) && (PUMP_on_count >= 10) && (current_sys_op_power == SET))
	{
		if(TEMP_SELECT_SWITCH == FIX_TYPE_SENSOR)
		{
			if((CONTROL_TEMP + (int16_t)eeprom_option_byte[_Control_sensor_offset]) < ((int16_t)SET_TEMP + (int16_t)eeprom_option_byte[_Heater_start_offset]))
				HEATER_RLY = SET;
			if((CONTROL_TEMP + (int16_t)eeprom_option_byte[_Control_sensor_offset]) > ((int16_t)SET_TEMP + (int16_t)eeprom_option_byte[_Heater_stop_offset]))
				HEATER_RLY = RESET;	
		}
		else
		{
			if(((CONTROL_TEMP + (int16_t)eeprom_option_byte[_Control_sensor_offset])	- (BASE_TEMP + (int16_t)eeprom_option_byte[_Base_sensor_offset])) 
					< (SET_TEMP + (int16_t)eeprom_option_byte[_Heater_start_offset]))
				HEATER_RLY = SET;
			if(((CONTROL_TEMP + (int16_t)eeprom_option_byte[_Control_sensor_offset])	- (BASE_TEMP + (int16_t)eeprom_option_byte[_Base_sensor_offset])) 
					>  (SET_TEMP + (int16_t)eeprom_option_byte[_Heater_stop_offset]))
				HEATER_RLY = RESET;	
		}
		
	}
	/*****************   system OFF operation  ****************/
	else
	{
		HEATER_RLY = RESET;
	}


}

/*******************************************************************************
End of functiontimer_counter_down
*******************************************************************************/	


/*******************************************************************************
* Function name	: debug_operation
* Description 	: none
* Argument 	: none
* Return value  : none
*******************************************************************************/
void	debug_operation(void)
{
#if 0     // testtt need to check request 
		HEATER_START_TEMP = SET_TEMP + (int16_t)eeprom_option_byte[_Heater_start_offset];
		HEATER_STOP_TEMP = SET_TEMP + (int16_t)eeprom_option_byte[_Heater_stop_offset];
#endif
	

#if 0	
		/* test for switch operation */
		if(SWDIP4_1 == SET)
		{
			if(current_sys_op_power==RESET)
			{
				turn_system_on();
			}
		}
		else
		{
			if(current_sys_op_power==SET)
			{
				turn_system_off();
			}
		}
#endif	
	
#if 0
		if(SWDIP4_2 == SET)
			COMP_REAL_TEMP = 100;
		else
			COMP_REAL_TEMP = 300;
		
		if(SWDIP4_3== SET)
			COMP_SET_TEMP_ON = 260;
		else
			COMP_SET_TEMP_OFF = 240;
	
		if(SWDIP4_4 == SET)
		{
			COMP_SET_TEMP_ON = 260;
			COMP_SET_TEMP_OFF = 240;
		}
		else
		{
			COMP_SET_TEMP_ON = 260;
			COMP_SET_TEMP_OFF = 240;
		}
	
#endif
	
		
#if 0
	
		if(COMP_RLY==SET)
			comp_rly_flag = SET;
		else
			comp_rly_flag = RESET;
			
		if(FAN_RLY==RESET)
			fan_rly_flag = RESET;
		else
			fan_rly_flag = SET;
			
		if(PUMP_RLY==SET)
			pump_rly_flag = SET;
		else
			pump_rly_flag = RESET;
	
		if(HEATER_RLY==SET)
			heater_rly_flag = SET;
		else
			heater_rly_flag = RESET;
		
#endif
		  /* test for switch operation */

	
}

/*******************************************************************************
* Function name	: system_operation_fix_temp
* Description 	: none
* Argument 	: none
* Return value  : none
*******************************************************************************/
void	system_operation_fix_temp(void)
{
	if(current_sys_op_power)     
	{
#if PMV_TEST	
		if(PMV_TIMER_COUNT <= 0)      
#else			
		if((COMP_RLY == SET) && (PMV_TIMER_COUNT <= 0))			 
#endif
		{
			PMV_TIMER_COUNT = PMV_TIMER_SET;
			if(motor_current_position_1 == motor_wish_position)
			{
				if(motor_current_position_2 == motor_wish_position)
				{
					if(motor_current_position_3 == motor_wish_position)
						PMV_operation();
				}
			}		
		}
#if 0
		if((COMP_RLY == SET) && (WATERGATE_TIMER_COUNT <= 0))
		{
		 	watergate_operation();	
		}

#endif
		
		
		if(PUMP_RLY == RESET)
		{
			if(PUMP_DELAY  <= 0 )
			{
				PUMP_RLY = SET;      
				PUMP_on_count = 0;
				PUMP_DELAY = 0;
				FAN_DELAY=  (int16_t)eeprom_option_byte[_System_on_FAN_delay]*step_10_sec;
			}
		}
		else 
		{
			if(FAN_RLY == RESET)
			{
				if(FAN_DELAY  <= 0 )
				{
					FAN_RLY = SET; 
					FAN_DELAY = 0;
					if(comp_protect_timer <= 0)
						comp_protect_timer =  (int16_t)eeprom_option_byte[_System_on_COMP_start_delay]*step_10_sec;
				}
			}
			else 
			{
				if(COMP_RLY == RESET)
				{
					if(comp_protect_timer  <= 0 )
					{
					COMP_RLY = SET; 
					comp_protect_timer = 0;
					}
				}
			}
		}
	}
	else     /*****************   system off operation  ****************/
	{
		if(ERROR_RLY == SET)
			ERROR_RLY = RESET;

		if(COMP_RLY == SET)
		{
			COMP_RLY = RESET;
			comp_protect_timer =  (int16_t)eeprom_option_byte[_System_off_COMP_protect_timer_set]*step_10_sec;
			PUMP_DELAY = (int16_t)eeprom_option_byte[_System_off_PUMP_delay]*step_10_sec;
			FAN_DELAY = (int16_t)eeprom_option_byte[_System_off_FAN_delay]*step_10_sec;
		}
		else
		{
			if(PUMP_RLY == SET)
			{
				if(PUMP_DELAY == 0)
				{
					PUMP_RLY= RESET;  
				}
			}
		
			if(FAN_RLY == SET)
			{
				if(FAN_DELAY == 0)
				{
					FAN_RLY= RESET;
				}
			}
		}
	}
	
	
}
/*******************************************************************************
End of system_operation_fix_temp
*******************************************************************************/















/*******************************************************************************
* Function name	: system_operation
* Description 	: none
* Argument 	: none
* Return value  : none
*******************************************************************************/
void	system_operation(void)
{
	/*****************   system on operation  ****************/

	if(TEMP_SELECT_SWITCH == FIX_TYPE_SENSOR)
	{
		COMP_REAL_TEMP = CONTROL_TEMP + (int16_t)eeprom_option_byte[_Control_sensor_offset];
		COMP_SET_TEMP_OFF = SET_TEMP;
		if(COMP_SET_TEMP_OFF < 0)
			COMP_SET_TEMP_OFF = 0;
		COMP_SET_TEMP_ON = SET_TEMP + (int16_t)eeprom_option_byte[_COMP_start_temp_offset];
	}
	else
	{
		COMP_REAL_TEMP = (CONTROL_TEMP + (int16_t)eeprom_option_byte[_Control_sensor_offset])
							- (BASE_TEMP + (int16_t)eeprom_option_byte[_Base_sensor_offset]);
		
		COMP_SET_TEMP_OFF = SET_TEMP;
		COMP_SET_TEMP_ON = SET_TEMP + (int16_t)eeprom_option_byte[_COMP_start_temp_offset];
		
	}
	
 
	/*****************	 system on operation  ****************/

	if(current_sys_op_power)     
	{
		if(PUMP_RLY == RESET)
		{
			if(PUMP_DELAY  <= 0 )
			{
				PUMP_RLY = SET; 
				PUMP_on_count = 0;
				PUMP_DELAY = 0;
				if(comp_protect_timer <= 0)
					comp_protect_timer = eeprom_option_byte[_System_on_COMP_start_delay]*step_10_sec;
				if(COMP_REAL_TEMP >= COMP_SET_TEMP_ON)
				{
					system_on_fan_comp_flag = SET;
					FAN_DELAY= eeprom_option_byte[_System_on_FAN_delay]*step_10_sec;
				}
			}
		}
		else 
		{
			if( system_on_fan_comp_flag == SET)
			{	
				if(FAN_RLY == RESET)
				{
					if(FAN_DELAY  <= 0 )
					{
						FAN_RLY = SET; 
						PUMP_DELAY = 0;
						if(comp_protect_timer <= 0)
							comp_protect_timer = eeprom_option_byte[_System_on_COMP_start_delay]*step_10_sec;
					}
				}
				else
				{
					if(COMP_RLY == RESET)
					{
						if(comp_protect_timer  <= 0 )
						{
							COMP_RLY = SET; 
							COMP_on_count = 0; 
							comp_protect_timer = 0; 
							system_on_fan_comp_flag = RESET;
						}
					}
				}
			}
			else
			{
				comp_turn_on();
			}
				
		}
	}
	else     /*****************   system off operation  ****************/
	{
		if(ERROR_RLY == SET)
			ERROR_RLY = RESET;
		comp_turn_off();
	}
	
}
/*******************************************************************************
End of function system_operation
*******************************************************************************/


/*******************************************************************************
* Function name	: comp_turn_on
* Description 	: none
* Argument 	: none
* Return value  : none
*******************************************************************************/
void	comp_turn_on(void)
{
	if(COMP_RLY == RESET)
	{ 
		if(FAN_DELAY <= 0)
		{
			FAN_DELAY= 0;
			if(COMP_REAL_TEMP < COMP_SET_TEMP_ON)
				FAN_RLY = RESET;
		}
		
        if(COMP_REAL_TEMP >= COMP_SET_TEMP_ON)
		{
//			HEATER_RLY = RESET;
			if(comp_protect_timer  <= 0)      
       		{
       			comp_protect_timer = 0;
    			COMP_RLY = SET;
				FAN_RLY = SET;
				COMP_on_count = 0; 
			}
		}
	}
	else
	{
		if(comp_protect_timer <= 0)
		{
			comp_protect_timer = 0;
			if(COMP_REAL_TEMP <= COMP_SET_TEMP_OFF)    
           	{
        		COMP_RLY = RESET;
//				HEATER_RLY = SET;
				comp_protect_timer = (int16_t)eeprom_option_byte[_Temp_control_COMP_protect_timer_set]*step_10_sec;
				FAN_DELAY = (int16_t)eeprom_option_byte[_Temp_control_FAN_delay_off]*step_10_sec;
            }
		}
	}
}


/*******************************************************************************
* Function name	: com_turn_off
* Description 	: none
* Argument 	: none
* Return value  : none
*******************************************************************************/

void	comp_turn_off(void)
{
//	HEATER_RLY = RESET;
	if(COMP_RLY == SET)
	{
		comp_protect_timer = 0;
		COMP_RLY = RESET;
		comp_protect_timer =  (int16_t)eeprom_option_byte[_System_off_COMP_protect_timer_set]*step_10_sec;
		PUMP_DELAY = (int16_t)eeprom_option_byte[_System_off_PUMP_delay]*step_10_sec;
		FAN_DELAY = (int16_t)eeprom_option_byte[_System_off_FAN_delay]*step_10_sec;
	}
	else
	{
		if(PUMP_RLY == SET)
		{
			if(PUMP_DELAY == 0)
			{
				PUMP_RLY= RESET;
			}
		}
		
		if(FAN_RLY == SET)
		{
			if(FAN_DELAY == 0)
			{
				FAN_RLY= RESET;
			}
		}
	}
	
}
/*******************************************************************************
* Function name	: error_system_off
* Description 	: none
* Argument 	: none
* Return value  : none
*******************************************************************************/
void	error_system_off(void)
{
	COMP_RLY = RESET;
	FAN_RLY = RESET;
	HEATER_RLY = RESET;
	ERROR_RLY = SET;
	error_code_backup = 0Xff;
	turn_system_off();

}

/*******************************************************************************
* Function name	: error_system_off
* Description 	: none
* Argument 	: none
* Return value  : none
*******************************************************************************/
void	error_system_off_all(void)
{
	COMP_RLY = RESET;
	FAN_RLY = RESET;
	PUMP_RLY = RESET;
	HEATER_RLY = RESET;
	ERROR_RLY = SET;
	error_code_backup = 0Xff;
	turn_system_off();

}

/*******************************************************************************
* Function name	: sensor_operation
* Description 	: none
* Argument 	: none
* Return value  : none
*******************************************************************************/
void	sensor_operation(void)
{

	if(TEMP_SELECT_SWITCH == FIX_TYPE_SENSOR)
	{
		COMP_REAL_TEMP = CONTROL_TEMP + (int16_t)eeprom_option_byte[_Control_sensor_offset];
		COMP_SET_TEMP_OFF = SET_TEMP;
		if(COMP_SET_TEMP_OFF < 0)
			COMP_SET_TEMP_OFF = 0;
	}
	else
	{
		COMP_REAL_TEMP = (CONTROL_TEMP + (int16_t)eeprom_option_byte[_Control_sensor_offset])
							- (BASE_TEMP + (int16_t)eeprom_option_byte[_Base_sensor_offset]);
		COMP_SET_TEMP_OFF = SET_TEMP;
	}


	

#if 0
	//ERROR_CODE = 0x00;
	/***** ************   COMP_TOP_TEMP_PROTECT  operation  detect 1 sec ****************/
	//if(COMP_RLY == SET)
	{
		if(COMP_TOP_TEMP_PROTECT == 0)                       
			COMP_TOP_TEMP_PROTECT_DELAY = 0;
		else
			if(COMP_TOP_TEMP_PROTECT_DELAY ++ >= 10)
			{
				COMP_TOP_TEMP_PROTECT_DELAY  = 10;
				ERROR_CODE = 0xC1;
				error_system_off();
			}
	}
	/*****************  COMP_TOP_TEMP_PROTECT operation  ****************/
#endif	

	/*****************   COMP_OVERLOAD_PROTECT operation  detect 1 sec ****************/
	//if(COMP_RLY == SET)
	{
		if(COMP_OVERLOAD_PROTECT == 0)
			COMP_OVERLOAD_PROTECT_DELAY = 0;
		else
			if(COMP_OVERLOAD_PROTECT_DELAY ++ >= SENSOR_DEBUNSE)
			{
				COMP_OVERLOAD_PROTECT_DELAY  = SENSOR_DEBUNSE;
				ERROR_CODE = 0xC1;
				error_system_off();
			}
	}
	/*****************   COMP_OVERLOAD_PROTECT_DELAY operation  ****************/

	/*****************   FAN_OVERLOAD_PROTECT operation  detect 1 sec   ****************/
	//if(COMP_RLY == SET)
	{
		if(FAN_OVERLOAD_PROTECT == 0)
			FAN_OVERLOAD_PROTECT_DELAY = 0;
		else
			if(FAN_OVERLOAD_PROTECT_DELAY ++ >= SENSOR_DEBUNSE)
			{
				FAN_OVERLOAD_PROTECT_DELAY = SENSOR_DEBUNSE;
				ERROR_CODE = 0xC2;
				error_system_off();
			}
	}
	/*****************   FAN_OVERLOAD_PROTECT sensor operation  ****************/

	/*****************   HP sensor operation detect 1 sec   ****************/
	//if(COMP_RLY == SET)
	{
		if(HP_PROTECT == 0)
			HP_SENSOR_DELAY = 0;
		else
			if(HP_SENSOR_DELAY ++ >= SENSOR_DEBUNSE)
			{
				HP_SENSOR_DELAY  = SENSOR_DEBUNSE;
				ERROR_CODE = 0xC3;
				error_system_off();
			}	
	}
	/*****************   HP sensor operation  ****************/

	/*****************   LP sensor operation  detect after comp on  hold for 10s ~ 600s ****************/
	if((COMP_RLY == SET) && (COMP_on_count >= 180))
	{
		if(LP_PROTECT == 0)
		{
			LP_SENSOR_DELAY = 0;
		}
		else
		{
			if(LP_SENSOR_DELAY ++ >= (int16_t)eeprom_option_byte[_LP_start_check_time]*100)
			{
				LP_SENSOR_DELAY  = (int16_t)eeprom_option_byte[_LP_start_check_time]*100;
				ERROR_CODE = 0xC4;
				error_code_backup = 0Xff;
				COMP_RLY = RESET;
#if 0						// #if ERROR_RLY_NORMAL need to check if normal operation need to turn on error relay
				ERROR_RLY = SET;
#endif				
				comp_protect_timer =  (int16_t)eeprom_option_byte[_System_off_COMP_protect_timer_set]*step_10_sec;
				//error_system_off();
			}
		}	
	}
	else
	{
		if(ERROR_CODE == 0xC4)
		{
			comp_protect_timer =  (int16_t)eeprom_option_byte[_System_off_COMP_protect_timer_set]*step_10_sec;
		}
		if(LP_PROTECT == 0)
		{
			LP_SENSOR_DELAY = 0;
			if(ERROR_CODE == 0xC4)
			{
				ERROR_CODE = 00;
				ERROR_RLY = RESET;
				comp_protect_timer =  (int16_t)eeprom_option_byte[_System_off_COMP_protect_timer_set]*step_10_sec;
			}
		}
	}
	/*****************   LP sensor operation  ****************/
	
	/*****************  condenser high temp protect  check for 1 sec ****************/
	if(CONDENSER_TEMP   > (int16_t)eeprom_option_byte[_Condenser_temp_protect_value]*10 )
	{
		if(CONDENSER_PROTECT_DELAY ++ >= SENSOR_DEBUNSE)
		{
			CONDENSER_PROTECT_DELAY  = SENSOR_DEBUNSE;
			ERROR_CODE = 0xC5;
			error_system_off();
		}
	}
	else
	{
		CONDENSER_PROTECT_DELAY  = 0;
	}
	/*****************  condenser high temp protect  check for 1 sec  ****************/
		

#if 0	
	/*****************   FLOW switch  operation  after 10 sec check for hold 0 - 40 s ****************/

	if((PUMP_RLY == SET) && (PUMP_on_count >= 10))
	{
		if(FLOW_SWITCH_PROTECT == 0)
			FLOW_SWITCH_PROTECT_DELAY = 0;
		else
			if(FLOW_SWITCH_PROTECT_DELAY++ >= (int16_t)eeprom_option_byte[_Flow_switch_start_check_time]*10)
			{
				FLOW_SWITCH_PROTECT_DELAY  = (int16_t)eeprom_option_byte[_Flow_switch_start_check_time]*10;
				ERROR_CODE = 0xd1;
				error_system_off();
			}
	}
	/*****************   FLOW switch  operation  ****************/
#endif
	/*****************   PUMP overload  operation  check for hold 0.1sec ****************/
	//if(PUMP_RLY == SET) 
	{
		if(PUMP_OVERLOAD_PROTECT == 0)
			PUMP_OVERLOAD_PROTECT_DELAY = 0;
		else
			if(PUMP_OVERLOAD_PROTECT_DELAY++  >= SENSOR_DEBUNSE)     /* old 1*/
			{
				PUMP_OVERLOAD_PROTECT_DELAY  =SENSOR_DEBUNSE;
				ERROR_CODE = 0xd2;
				error_system_off_all();
			}
	}
	/*****************   PUMP overload    operation  ****************/


	/*****************   WATER_HIGH_TEMP_STOP operation check for 10 sec 0-85 degree ****************/
	if((CONTROL_TEMP + (int16_t)eeprom_option_byte[_Control_sensor_offset]) > 
								(uint16_t)eeprom_option_byte[_Liquid_high_temp_stop]*5)
	{
		if(WATER_HIGH_TEMP_STOP_DELAY ++ >= 100)
		{
			WATER_HIGH_TEMP_STOP_DELAY  = 100;
			ERROR_CODE = 0xd4;
			error_system_off();
		}
	}
	else
	{
		WATER_HIGH_TEMP_STOP_DELAY  = 0;
	}
	/*****************  WATER_HIGH_TEMP_STOP operation  ****************/

	/*****************   WATER_LOW_TEMP_STOP operation check for 1 sec 0-85 degree c ****************/
	if((CONTROL_TEMP + (int16_t)eeprom_option_byte[_Control_sensor_offset])  <
								(uint16_t)eeprom_option_byte[_Liquid_low_temp_stop]*5)
	{
		if(WATER_LOW_TEMP_STOP_DELAY ++ >= SENSOR_DEBUNSE)
		{
			WATER_LOW_TEMP_STOP_DELAY  = SENSOR_DEBUNSE;
			ERROR_CODE = 0xd5;
			error_system_off();
		}
	}
	else
	{
		WATER_LOW_TEMP_STOP_DELAY  = 0;
	}
	/*****************  WATER_LOW_TEMP_STOP operation  ****************/



	/*****************   POWER_PHASE_PROTECT operation  ****************/
	if(POWER_PHASE_PROTECT == 0)
		POWER_PHASE_PROTECT_DELAY= 0;
	else
		if(POWER_PHASE_PROTECT_DELAY++ >= SENSOR_DEBUNSE)		/* old 1*/
		{
			POWER_PHASE_PROTECT_DELAY= SENSOR_DEBUNSE;
			ERROR_CODE = 0xe1;
			error_system_off_all();
		}
	/*****************   POWER_PHASE_PROTECT operation  ****************/
	/*****************   CONTROL SENSOR PROTECT  ****************/
	if((CONTROL_TEMP  <= -150) || (CONTROL_TEMP >= 900))
	{
		if(CONTROL_TEMP_DELAY ++ >= SENSOR_DEBUNSE)
		{
			CONTROL_TEMP_DELAY = SENSOR_DEBUNSE;
			ERROR_CODE = 0xe2;
			error_system_off();
		}
	}
	else
	{
		CONTROL_TEMP_DELAY = 0;
	}
	/*****************   CONTROL SENSOR PROTECT  ****************/
	/*****************   IW BASE SENSOR ERROR  ****************/
	if(TEMP_SELECT_SWITCH != FIX_TYPE_SENSOR)
	{
		if((BASE_TEMP <= -150) || (BASE_TEMP >= 900))
		{
			if(BASE_TEMP_DELAY ++ >= SENSOR_DEBUNSE)
			{
				BASE_TEMP_DELAY = SENSOR_DEBUNSE;
				ERROR_CODE = 0xe3;
				error_system_off();
			}
		}
		else
		{
			BASE_TEMP_DELAY = 0;
		}	
	}
	
	/*****************   IW BASE SENSOR ERROR  ****************/

	/*****************   IW CONDENSER SENSOR ERROR  ****************/
	if((CONDENSER_TEMP <= -150) || (CONDENSER_TEMP >= 900))
	{
		if(CONDENSER_TEMP_DELAY ++ >= SENSOR_DEBUNSE)
		{
			CONDENSER_TEMP_DELAY = SENSOR_DEBUNSE;
			ERROR_CODE = 0xe4;
			error_system_off();
		}
	}
	else
	{
		CONDENSER_TEMP_DELAY = 0;
	}	
	/*****************   IW BASE SENSOR ERROR  ****************/	
	/*****************   IW ANTIFRZ SENSOR ERROR  ****************/
	if((ANTIFRZ_TEMP <= -150) || (ANTIFRZ_TEMP >= 900))
	{
		if(ANTIFRZ_TEMP_DELAY ++ >= SENSOR_DEBUNSE)
		{
			ANTIFRZ_TEMP_DELAY = SENSOR_DEBUNSE;
			ERROR_CODE = 0xe5;
			error_system_off();
		}
	}
	else
	{
		ANTIFRZ_TEMP_DELAY = 0;
	}	
	/*****************   IW ANTIFRZ SENSOR ERROR  ****************/

#if 1
	/*****************   IW COMP_OUT SENSOR ERROR  ****************/
	if((T_DECP[_Comp_out_sensor] < 100) || (T_DECP[_Comp_out_sensor] > 1000))
	{
		if(COMP_OUT_SENSOR_DELAY ++ >= SENSOR_DEBUNSE)
		{
			COMP_OUT_SENSOR_DELAY = SENSOR_DEBUNSE;
			ERROR_CODE = 0xe6;
			error_system_off();
		}
	}
	else
	{
		COMP_OUT_SENSOR_DELAY = 0;
	}	

	/*****************   IW COMP_OUT SENSOR ERROR  ****************/
#endif		
	/*****************   HEATER_TEMP_PROTECT operation hold for 1 sec ****************/
	if(HEATER_TEMP_PROTECT == 0)
		HEATER_TEMP_PROTECT_DELAY= 0;
	else
		if(HEATER_TEMP_PROTECT_DELAY++ >= SENSOR_DEBUNSE)
		{
			HEATER_TEMP_PROTECT_DELAY= SENSOR_DEBUNSE;
			ERROR_CODE = 0xe7;
			error_system_off();
		}
	/*****************   POWER_PHASE_PROTECT operation  ****************/

	/*****************	slim-type over current error  ****************/
	
	if((COMP_RLY == SET) && (COMP_on_count > 9) && (current_data_max != default_0A)) 
	{
		current_data_temp = current_data_max - default_0A ; 
		current_data_temp = current_data_temp * 7;
		current_data_temp = current_data_temp / 10;
#if 0		
		if(current_data_temp > 20)
			current_data_temp = current_data_temp;			//	current_data_temp = current_data_temp - 4;	   current sensor offset 4 = 0.2A, 20 = 1A, 2019/04/12 modify to 4=0.2A offset 
#endif
		current_data_temp = current_data_temp +8;
//		if(current_data_temp < 380) 							// if current over 19A no need to wait for 1 sec
		{
			if(current_data_temp > (current_detect_data *2))	   // dip switch select current value
			{
				if(CURRENT_DETECT_DELAY++ >= 20)
				{
					CURRENT_DETECT_DELAY = 20;
					ERROR_CODE = 0xe8;
					error_system_off_all();
				}
			}
			else
			{
				CURRENT_DETECT_DELAY  = 0;
			}
		}
//		else
//		{
//			ERROR_CODE = 0xe8;
//			error_system_off();
//		}
	}
	/*****************	 slim-type over current  ERROR	****************/
	/*****************	slim-type over voltage ERROR  ****************/
	voltage_check_data = voltage_253_value;
	if(PUMP_RLY == SET)
		voltage_check_data = voltage_check_data - voltage_small_relay_offset;
	if(FAN_RLY == SET)
		voltage_check_data = voltage_check_data - voltage_small_relay_offset;
	if(ERROR_RLY == SET)
		voltage_check_data = voltage_check_data - voltage_small_relay_offset;
	if(COMP_RLY == SET)
		voltage_check_data = voltage_check_data - voltage_large_relay_offset;
	
	if(voltage_data_max > voltage_check_data)		// 253V
	{
		if(VOLTAGE_DETECT_DELAY++ >= 20)
		{
			VOLTAGE_DETECT_DELAY = 20;
			ERROR_CODE = 0xe9;
			error_system_off_all();
		}
	}
	else
	{
		VOLTAGE_DETECT_DELAY  = 0;
	}

	/*****************	 slim-type over voltage ERROR  ****************/

#if 1 
	/*****************	slim-type lo voltage ERROR	****************/

	voltage_check_data = voltage_187_value;
	if(PUMP_RLY == SET)
		voltage_check_data = voltage_check_data - voltage_small_relay_offset;
	if(FAN_RLY == SET)
		voltage_check_data = voltage_check_data - voltage_small_relay_offset;
	if(COMP_RLY == SET)
		voltage_check_data = voltage_check_data - voltage_large_relay_offset;
	
	if(voltage_data_max < voltage_check_data)		// 187V
	{
		if(VOLTAGE_DETECT_LOW_DELAY++ >= 20)
		{
			VOLTAGE_DETECT_LOW_DELAY = 20;
			ERROR_CODE = 0xea;
			error_system_off_all();
		}
	}
	else
	{
		VOLTAGE_DETECT_LOW_DELAY  = 0;
	}

	/*****************	 slim-type lo voltage ERROR  ****************/
#endif




}
/*******************************************************************************
End of function system_operation
*******************************************************************************/



/*******************************************************************************
* Function name	: sensor_operation_continue
* Description 	: none
* Argument 	: none
* Return value  : none
*******************************************************************************/
void	sensor_operation_continue(void)
{

	if(TEMP_SELECT_SWITCH == FIX_TYPE_SENSOR)
	{
		COMP_REAL_TEMP = CONTROL_TEMP + (int16_t)eeprom_option_byte[_Control_sensor_offset];
		COMP_SET_TEMP_OFF = SET_TEMP;
		if(COMP_SET_TEMP_OFF < 0)
			COMP_SET_TEMP_OFF = 0;
	}
	else
	{
		COMP_REAL_TEMP = (CONTROL_TEMP + (int16_t)eeprom_option_byte[_Control_sensor_offset])
							- (BASE_TEMP + (int16_t)eeprom_option_byte[_Base_sensor_offset]);
		COMP_SET_TEMP_OFF = SET_TEMP;
	}



	/*****************   IW RESERVE_1  ****************/
//	if(REMOTE_SW_FUNCTION == RESET)
	{
		if(REMOTE_ON_OFF_SW == 0)
		{
			REMOTE_DEBOUNCE_DELAY_1 = 0;
			if(REMOTE_DEBOUNCE_DELAY ++ >= 10)
			{
				REMOTE_DEBOUNCE_DELAY = 10;
				if(remote_on_off_flag == 0)
				{
					if(current_sys_op_power==RESET)
		       			turn_system_on();
					remote_on_off_flag = 1;
				}
			}
		}
		else
		{
			REMOTE_DEBOUNCE_DELAY = 0;
			if(REMOTE_DEBOUNCE_DELAY_1 ++ >= 10)
			{
				REMOTE_DEBOUNCE_DELAY_1 = 10;
				if(remote_on_off_flag == 1)
				{
					if(current_sys_op_power==SET)
						turn_system_off();
					remote_on_off_flag = 0;
				}
			}
		}
	}
	/*****************   WATERLEVER_LOW_ALARM  operation  after 10 sec check for hold 5 sec ****************/
	if((PUMP_RLY == SET) && (PUMP_on_count >= 10) && (current_sys_op_power))
	{
		if(WATERLEVER_LOW_ALARM == 0)
		{
			WATERLEVER_LOW_ALARM_DELAY = 0;
			if ( water_lever_error_flag_low)
			{
				water_lever_error_flag_low = 0;
				ERROR_CODE = 0x00;
				ERROR_RLY = RESET;
			}
		}
		else
		{
			if(WATERLEVER_LOW_ALARM_DELAY++  >= 50)
			{
				WATERLEVER_LOW_ALARM_DELAY  =50;
				if ( water_lever_error_flag_low == 0)
					error_code_backup = 0Xff;
				ERROR_CODE = 0xd3;
#if ERROR_RLY_NORMAL
				ERROR_RLY = SET;
#endif
				// PMV_close();				// modify for V13 bug 
				water_lever_error_flag_low = 1;
			}
		}
	}
	/*****************   WATERLEVER_LOW_ALARM  operation  ****************/
#if 0
	/*****************   WATERLEVER_HIGH_ALARM  operation  after 10 sec check for hold 5sec ****************/
	if((PUMP_RLY == SET) && (PUMP_on_count >= 10) && (current_sys_op_power))
	{
		if(WATERLEVER_HIGH_ALARM == 0)
		{
			WATERLEVER_HIGH_ALARM_DELAY = 0;
			if ( water_lever_error_flag_high)
			{
				water_lever_error_flag_high = 0;
				ERROR_CODE = 0x00;
				ERROR_RLY = RESET;
			}
		}	
		else
			if(WATERLEVER_HIGH_ALARM_DELAY++  >= 50)
			{
				WATERLEVER_HIGH_ALARM_DELAY  =50;
				if ( water_lever_error_flag_high == 0)
					error_code_backup = 0Xff;
				ERROR_CODE = 0xd4;
				ERROR_RLY = SET;
				water_lever_error_flag_high = 1;
			}
	}
	/*****************    WATERLEVER_HIGH_ALARM  operation  ****************/
#endif


	
	/*****************   WATER_HIGH_TEMP_ALARM operation check for 10 sec  ****************/
	if(current_sys_op_power)     
	{
		if(COMP_REAL_TEMP  >	(COMP_SET_TEMP_OFF +	(int16_t)eeprom_option_byte[_Liquid_high_temp_alarm]*10))
		{
			if(WATER_HIGH_TEMP_ALARM_DELAY ++ >= 100)
			{
				WATER_HIGH_TEMP_ALARM_DELAY  = 100;
				if ( water_temp_error_flag_high == 0)
					error_code_backup = 0Xff;
				ERROR_CODE = 0xd6;
#if ERROR_RLY_NORMAL				
				ERROR_RLY = SET;
#endif
				// PMV_close();				// modify for V13 bug 
				water_temp_error_flag_high = 1;
			}
		}
		else
		{
			if(COMP_REAL_TEMP  < (COMP_SET_TEMP_OFF +	(int16_t)eeprom_option_byte[_Liquid_high_temp_alarm]*10 - 5))
			{
				if ( water_temp_error_flag_high)
				{
					water_temp_error_flag_high = 0;
					ERROR_RLY = RESET;
					ERROR_CODE = 0x00;
				}	
			}
			WATER_HIGH_TEMP_ALARM_DELAY  = 0;
		}
	}
	/*****************  WATER_HIGH_TEMP_ALARM operation  ****************/

	/*****************   WATER_LOW_TEMP_ALARM operation check for 5 sec  ****************/
	if(current_sys_op_power)     
	{
		if(COMP_REAL_TEMP  <  (COMP_SET_TEMP_OFF + (int16_t)eeprom_option_byte[_Liquid_low_temp_alarm]*10))
		{
			if(WATER_LOW_TEMP_ALARM_DELAY ++ >= 50)
			{
				WATER_LOW_TEMP_ALARM_DELAY  = 50;
				if ( water_temp_error_flag_low == 0)
					error_code_backup = 0Xff;
				ERROR_CODE = 0xd7;
#if ERROR_RLY_NORMAL				
				ERROR_RLY = SET;
#endif
				// PMV_close();				// modify for V13 bug 
				water_temp_error_flag_low = 1;
			}
		}
		else
		{
			if(COMP_REAL_TEMP  >  (COMP_SET_TEMP_OFF + (int16_t)eeprom_option_byte[_Liquid_low_temp_alarm]*10 + 5))
			{
				WATER_LOW_TEMP_ALARM_DELAY  = 0;
				if ( water_temp_error_flag_low)
				{
					water_temp_error_flag_low = 0;
					ERROR_CODE = 0x00;
					ERROR_RLY = RESET;
				}
			}
		}
	}
	/*****************  WATER_LOW_TEMP_ALARM operation  ****************/
	/*****************   Antifrz  operation  Antifrz temp range 0-20, for 1 sec ****************/
	if(ANTIFRZ_PROTECT_ON != 1)
	{
		if(ANTIFRZ_TEMP <= 	(int16_t)eeprom_option_byte[_Antifrz_protect_stop]*10)
		{
			if(ANTIFRZ_DELAY ++ >= SENSOR_DEBUNSE)
			{
				ANTIFRZ_DELAY  = SENSOR_DEBUNSE;
				if ( ANTIFRZ_PROTECT_ON == 0)
					error_code_backup = 0Xff;
				ERROR_CODE = 0xC6;
				COMP_RLY = RESET;
				comp_protect_timer =  (int16_t)eeprom_option_byte[_System_off_COMP_protect_timer_set]*step_10_sec;
				ERROR_RLY = SET;
				PMV_close();
				ANTIFRZ_PROTECT_ON = 1;
				
			}
		}
		else
		{
			ANTIFRZ_DELAY  = 0;
		}
	}
	else
	{
		if(comp_protect_timer <= 5)
			comp_protect_timer =  5;
		if(ANTIFRZ_TEMP  >= (int16_t)eeprom_option_byte[_Antifrz_protect_stop]*10 + 20 )       // Antifrz protect off = _Antifrz_protect_stop + 2 degree
		{
			ERROR_CODE = 0x00;
			ERROR_RLY = RESET;
			ANTIFRZ_PROTECT_ON = 0;
		}
	}
	
	/*****************   Antifrz  operation  ****************/

	/*****************	 FLOW switch  operation  after 10 sec check for hold 0 - 40 s ****************/
#if 1
	if(FLOW_SWITCH_PROTECT_ON != 1)
	{
		if((PUMP_RLY == SET) && (PUMP_on_count >= 10))
		{
			if(FLOW_SWITCH_PROTECT == 0)
				FLOW_SWITCH_PROTECT_DELAY = 0;
			else
				if(FLOW_SWITCH_PROTECT_DELAY++ >= (int16_t)eeprom_option_byte[_Flow_switch_start_check_time]*10)
				{
					FLOW_SWITCH_PROTECT_DELAY  = (int16_t)eeprom_option_byte[_Flow_switch_start_check_time]*10;
					ERROR_CODE = 0xd1;
					//error_system_off();
					error_code_backup = 0Xff;
					COMP_RLY = RESET;
					comp_protect_timer =  (int16_t)eeprom_option_byte[_System_off_COMP_protect_timer_set]*step_10_sec;
					FAN_RLY = RESET;
					PUMP_RLY = RESET;
					PUMP_DELAY = (int16_t)eeprom_option_byte[_System_off_PUMP_delay]*step_10_sec;
					ERROR_RLY = SET;
					HEATER_RLY = RESET;
					FLOW_SWITCH_PROTECT_ON = 1;
					PMV_close();
				}
		}
	}	
	else
	{
		if(FLOW_SWITCH_PROTECT == 0)
		{
			if(PUMP_RLY == RESET)
			{
				PUMP_RLY = SET;
				PUMP_on_count = 0;
			}
			else
			{
				if(PUMP_on_count >= 10)
				{
					FAN_RLY = SET;
					ERROR_RLY = RESET;
					ERROR_CODE =  00;
					FLOW_SWITCH_PROTECT_ON = 0;
				}	
			}
		}
		else
		{
			
			PUMP_DELAY = (int16_t)eeprom_option_byte[_System_off_PUMP_delay]*step_10_sec;
			comp_protect_timer =  (int16_t)eeprom_option_byte[_System_off_COMP_protect_timer_set]*step_10_sec;
		}
		
	}
	/*****************	 FLOW switch  operation  ****************/
#endif

}
/*******************************************************************************
End of function sensor_operation_continue
*******************************************************************************/


/*******************************************************************************
* Function name : sensor_operation_continue
* Description	: none
* Argument	: none
* Return value	: none
*******************************************************************************/
void	clear_flag_for_sensor_continue(void)
{
	water_lever_error_flag_low = 0;
	water_temp_error_flag_low = 0;
	water_temp_error_flag_high = 1;
	ANTIFRZ_PROTECT_ON = 0;
	FLOW_SWITCH_PROTECT_ON = 0;
	comp_over_heat = RESET;
	ERROR_CODE = 0x00;
	ERROR_RLY = RESET;
}

#if 0

	/*****************  Evaporator low temp protect  check for 1 sec ****************/
	if((EVAPORATOR_TEMP + (int16_t)eeprom_option_byte[_Evaporator_sensor_offset])  < 
					(int16_t)eeprom_option_byte[_Evaporator_temp_protect_value]*10 )
	{
		if(EVAPORATOR_PROTECT_DELAY ++ >= 10)
		{
			EVAPORATOR_PROTECT_DELAY  = 10;
			ERROR_CODE = 0xC8;
			error_system_off();
		}
	}
	else
	{
		EVAPORATOR_PROTECT_DELAY  = 0;
	}
	/*****************  Evaporator low temp protect  check for 1 sec  ****************/
	
	
		/*****************	Evaporator low temp protect  check for 1 sec ****************/
		if((EVAPORATOR_TEMP + (int16_t)eeprom_option_byte[_Evaporator_sensor_offset])  < 
						(int16_t)eeprom_option_byte[_Evaporator_temp_protect_value]*10 )
		{
			if(EVAPORATOR_PROTECT_DELAY ++ >= 10)
			{
				EVAPORATOR_PROTECT_DELAY  = 10;
				if(ERROR_CODE != 0xC8)
					error_code_backup = 0Xff;
				ERROR_CODE = 0xC8;
				if(COMP_RLY == SET)
				{
					COMP_RLY = RESET;
					comp_protect_timer =  (int16_t)eeprom_option_byte[_System_off_COMP_protect_timer_set]*step_10_sec;
				}
				
				ERROR_RLY = SET;
				// error_system_off();
			}
		}
		else
		{
			EVAPORATOR_PROTECT_DELAY  = 0;
			if((EVAPORATOR_TEMP + (int16_t)eeprom_option_byte[_Evaporator_sensor_offset])  >
						((int16_t)eeprom_option_byte[_Evaporator_temp_protect_value]*10 + 20))
			{	
				if(ERROR_CODE ==  0xc8)
				{
					ERROR_CODE =  00;
					ERROR_RLY = RESET;
				}
			}
		}
		/*****************	Evaporator low temp protect  check for 1 sec  ****************/
	
	
#endif



	

