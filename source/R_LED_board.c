/***********************************************************************************************************************
* File Name    : R_LED_board.c
* Version      : CodeGenerator for RL78/G13 V2.05.01.04 [18 May 2018]
* Device(s)    : R5F100LE
* Tool-Chain   : CCRL
* Description  : This file implements main function.
* Creation Date: 2019/2/16
***********************************************************************************************************************/

/***********************************************************************************************************************
Includes
***********************************************************************************************************************/
#include "r_cg_macrodriver.h"

/* Start user code for include. Do not edit comment generated here */

/* End user code. Do not edit comment generated here */
#include "r_cg_userdefine.h"

/***********************************************************************************************************************
Pragma directive
***********************************************************************************************************************/
/* Start user code for pragma. Do not edit comment generated here */
/* End user code. Do not edit comment generated here */

/***********************************************************************************************************************
Global variables and functions
***********************************************************************************************************************/
/* Start user code for global. Do not edit comment generated here */

#define key_value_on_off	760		// OLD 740   790-730
#define	key_value_mode		200		// OLD 630   170-230
#define	key_value_down		400		// OLD 383	 370-430
#define	key_value_up		310		// OLD 293	 280-340
#define	key_value_clear		650		// OLD 171	 620-680
#define	key_offset			35

#define DISPLAY_SPACE		16
#define DISPLAY_MINUS		17

const uint8_t	option_seg_code[]={   
	0xc0,0xf9,0xa4,0xb0,0x99,0x92,0x82,0xf8,/* 0,1,2,3,4,5,6,7  			*****   0-7 */
    0x80,0x90,0x88,0x83,0xc6,0xa1,0x86,0x8e,/* 8,9,A,b,c,d,E,F 				*****   8-15 */
    0xc7,0xa3,0x89,0xc1,0x8c,0xcf,0xff,0xbf,/* L,o,H,u,P,i,blank,-	 		*****   16-23 */
	};


const uint8_t	seven_seg_code[]={   
	0xc0,0xf9,0xa4,0xb0,0x99,0x92,0x82,0xf8,/* 0,1,2,3,4,5,6,7  			*****   0-7 */
    0x80,0x90,0x88,0x83,0xc6,0xa1,0x86,0x8e,/* 8,9,A,b,c,d,E,F 				*****   8-15 */
    0xff,0xbf,0xc7,0xa3,0x89,0xcf,0xc1,0x8c,/* blank,-,L,o,H,i,u,P,	 		*****   16-23 */
    0xcf,0xc8,0x2f,0x87,0x8b,0xf7};/* I,N,r,t,h,_							*****   24-31 */


int8_t	option_byte_high_limit[IIC_option_length+3] = { Liquid_high_temp_stop_h,Liquid_low_temp_stop_h,Liquid_high_temp_alarm_h,Liquid_low_temp_alarm_h,
														Set_temp_high_h,Set_temp_low_h,Set_temp_high_h_1,Set_temp_low_h_1,Heater_start_offset_h,Heater_stop_offset_h,
														System_off_COMP_protect_timer_set_h,Control_sensor_offset_h,Base_sensor_offset_h,
														Flow_switch_start_check_time_h,LP_start_check_time_h,System_on_PUMP_delay_h,System_on_FAN_delay_h,
														System_on_COMP_start_delay_h,System_off_PUMP_delay_h,System_off_FAN_delay_h,Condenser_temp_protect_h,
														Antifrz_protect_stop_h,System_off_PMV_delay_h,
														comp_current_protect_0_h,comp_current_protect_1_h,comp_current_protect_2_h,comp_current_protect_3_h,
														Modbus_address_max,
														COMP_start_temp_offset_h,Temp_control_COMP_protect_timer_set_h,Temp_control_FAN_delay_off_h,
														PID_P_factor_max,PID_I_factor_max,PID_D_factor_max };
														

int8_t	option_byte_low_limit[IIC_option_length+3] = { Liquid_high_temp_stop_l,Liquid_low_temp_stop_l,Liquid_high_temp_alarm_l,Liquid_low_temp_alarm_l,
														Set_temp_high_l,Set_temp_low_l,Set_temp_high_l_1,Set_temp_low_l_1,Heater_start_offset_l,Heater_stop_offset_l,
														System_off_COMP_protect_timer_set_l,Control_sensor_offset_l,Base_sensor_offset_l,
														Flow_switch_start_check_time_l,LP_start_check_time_l,System_on_PUMP_delay_l,System_on_FAN_delay_l,
														System_on_COMP_start_delay_l,System_off_PUMP_delay_l,System_off_FAN_delay_l,Condenser_temp_protect_l,
														Antifrz_protect_stop_l,System_off_PMV_delay_l,
														comp_current_protect_0_l,comp_current_protect_1_l,comp_current_protect_2_l,comp_current_protect_3_l,
														Modbus_address_min,
														COMP_start_temp_offset_l,Temp_control_COMP_protect_timer_set_l,Temp_control_FAN_delay_off_l,
														PID_P_factor_min,PID_I_factor_min,PID_D_factor_min,};

const uint8_t	option_byte_pointer[] = { _Liquid_high_temp_stop,_Liquid_low_temp_stop,_Liquid_high_temp_alarm,_Liquid_low_temp_alarm,
														_Set_temp_high,_Set_temp_low,_Set_temp_high_1,_Set_temp_low_1,_Heater_start_offset,_Heater_stop_offset,
														_System_off_COMP_protect_timer_set,_Control_sensor_offset,_Base_sensor_offset,
														_Flow_switch_start_check_time,_LP_start_check_time,_System_on_PUMP_delay,_System_on_FAN_delay,
														_System_on_COMP_start_delay,_System_off_PUMP_delay,_System_off_FAN_delay,_Condenser_temp_protect_value,
														_Antifrz_protect_stop,_System_off_PMV_delay,
														_comp_current_protect_0_value,_comp_current_protect_1_value,_comp_current_protect_2_value,_comp_current_protect_3_value,
														_Modbus_address_offset,
														_COMP_start_temp_offset,_Temp_control_COMP_protect_timer_set,_Temp_control_FAN_delay_off,
														_PID_P_factor_value,_PID_I_factor_value,_PID_D_factor_value};

int8_t	option_modify_flag,option_byte_count_value;

int16_t	voltage_value_1,voltage_value,voltage_offset;

int16_t voltage_buffer[32],voltage_buffer_1[32];
int8_t	voltage_buffer_pointer_1=0,voltage_buffer_pointer=0,voltage_loop_count,voltage_count;
	
int8_t	th_no=1;	
uint8_t	led_scan_line=0,led_data_shift,check_b7,clk_delay,LED_mode_count,key_press_check_time = 0,key_repeart_timer,key_repeart_timer_1;
uint8_t	production_key_press_check_time = 0;
uint8_t	HC164_display_arry[5],HC164_display_arry_backup[5],test_error_code;
uint8_t	LED_1_dot,LED_2_dot,LED_3_dot,LED_4_dot,LED_RUN,LED_ALARM,LED_PV,LED_SV;
uint16_t	key_data,key_press_status,flash_count_buf,flash_flag,key_debunse;
uint8_t	digi_0,digi_1,digi_2,key_debunse_count;
int16_t	th_display,disp_temp_buf;        //No_error_display_count;
//int16_t	filter_sec_count,filter_10hour_count,filter_one_sec_count;
int8_t	error_code_pointer, error_code_last, error_code_offset,loop_temp;
int8_t	production_mode,Production_mode_count,relay_test_timer;
uint8_t	adjust_addr_key,option_byte_count,check_key_1,check_key_2;
uint8_t	liquid_temp_buf;
uint16_t	LED_display_timer;
//uint8_t	Modbus_address=03;
extern	uint16_t	get_V_A_timer;
extern	int16_t 	PID_temp;
/***********************************************************************************************************************/

void 	LED_mode_display(void);
void 	get_production_key_press(void);
void 	production_process(void);
void 	convert_AD_channel_key(void);
void	turn_system_on(void);
void	turn_system_off(void);
void 	LED_display_operation_1(void);
void 	LED_display_operation_2(void);
void	clear_flag_for_sensor_continue(void);
void	system_sensor_status(void);


/* End user code. Do not edit comment generated here */

/***********************************************************************************************************************
* Function Name: LED_key_down_operation
* Description  : This function display the information on display board
* Arguments    : None
* Return Value : None
***********************************************************************************************************************/
void LED_key_down_operation(void)
{
	switch(LED_mode_count)
	{
	
		/*************************************************************************/ 
		case _LED_mode_1:
			//PID_temp--;
			break;
		/*************************************************************************/ 
		case _LED_mode_2:
			
#if 1
			if(TEMP_SELECT_SWITCH == FIX_TYPE_SENSOR)
			{
#if 0			
				SET_TEMP = SET_TEMP - 10;
				SET_TEMP = SET_TEMP/10;
				SET_TEMP = SET_TEMP*10;
#else
				SET_TEMP--;
#endif
				if(SET_TEMP > eeprom_option_byte[_Set_temp_high] * 10)
					SET_TEMP = eeprom_option_byte[_Set_temp_high] *10;
				else if(SET_TEMP < eeprom_option_byte[_Set_temp_low] *10)
					SET_TEMP = eeprom_option_byte[_Set_temp_low] *10;
				/*************************************/
			}
			else
			{
				SET_TEMP--;
				if(SET_TEMP > eeprom_option_byte[_Set_temp_high_1])
					SET_TEMP = eeprom_option_byte[_Set_temp_high_1];
				else if(SET_TEMP < eeprom_option_byte[_Set_temp_low_1])
					SET_TEMP = eeprom_option_byte[_Set_temp_low_1];
				/*************************************/ 	
			}
#endif		
			flash_flag = 1;
			flash_count_buf = 0;
			LED_mode_display();
			break;
		/*************************************************************************/ 
		case _LED_mode_4:
			
			if(TEMP_SELECT_SWITCH == FIX_TYPE_SENSOR)
			{
				if(th_no==3)
					th_no=2;
				if(--th_no < 1)
					th_no=5;
			}
			else
			{
				if(--th_no < 1)
					th_no=5;
			}
			flash_flag = 0;
			flash_count_buf = 0;
			LED_mode_display();
			break;
		
		/*************************************************************************/ 
		case _LED_mode_5:
			if(++error_code_offset >= error_code_last )
				error_code_offset = error_code_last-1 ;
			flash_flag = 1;
			flash_count_buf = 0;
			LED_mode_display();
			break;
#if 0				
		/*************************************************************************/ 
		case _LED_mode_8:
			break;
		/*************************************************************************/ 
		case _LED_mode_9:
			break;
#endif			
		/*************************************************************************/ 
		case _LED_mode_6:
			eeprom_option_byte[_Control_sensor_offset]--; 
			if(eeprom_option_byte[_Control_sensor_offset] < Control_sensor_offset_l) 
				eeprom_option_byte[_Control_sensor_offset] = Control_sensor_offset_l;
			if(eeprom_option_byte[_Control_sensor_offset] > Control_sensor_offset_h)	
				eeprom_option_byte[_Control_sensor_offset] = Control_sensor_offset_h;
			flash_flag = 1;
			flash_count_buf = 0;
			LED_mode_display();
			option_backup = 0xFF;
			break;
		/*************************************************************************/ 
		case _LED_mode_7:
			eeprom_option_byte[_Base_sensor_offset]--;
			if(eeprom_option_byte[_Base_sensor_offset] < Base_sensor_offset_l) 
				eeprom_option_byte[_Base_sensor_offset] = Base_sensor_offset_l;
			if(eeprom_option_byte[_Base_sensor_offset] > Base_sensor_offset_h)	
				eeprom_option_byte[_Base_sensor_offset] = Base_sensor_offset_h;
			flash_flag = 1;
			flash_count_buf = 0;
			LED_mode_display();
			option_backup = 0xFF;
			break;
#if 1		
		/*************************************************************************/ 
		case _LED_mode_9:
			if(Modbus_address == Modbus_address_min)
				Modbus_address=Modbus_address_max;
			else
				Modbus_address--;
			LED_mode_display();
			key_repeart_timer = 70;
			break;
#endif									
		/*************************************************************************/ 
		case _LED_mode_option:

			option_backup = 0xFF;
		/*************************************************************************/ 
			switch(option_byte_count_value)
			{
				default:
					eeprom_option_byte[option_byte_count_value]--;
					if(eeprom_option_byte[option_byte_count_value] < option_byte_low_limit[option_byte_count_value] )
						eeprom_option_byte[option_byte_count_value] = option_byte_high_limit[option_byte_count_value];
					break;
				case	_Liquid_high_temp_stop:
				case	_Liquid_low_temp_stop:
					
					liquid_temp_buf = (uint8_t)eeprom_option_byte[option_byte_count_value];
					if(liquid_temp_buf == 0)
						liquid_temp_buf = (uint8_t)option_byte_high_limit[option_byte_count_value];
					else
						liquid_temp_buf--;
					eeprom_option_byte[option_byte_count_value] = liquid_temp_buf;
					break;
			}
		/*************************************************************************/ 
			LED_mode_display();
			break;
		/*************************************************************************/ 
		/*************************************************************************/ 
		case _LED_key_check:
			if(check_key_2-- == 0)
				check_key_2 = 99;
			LED_mode_display();
			break;
		/*************************************************************************/ 


		default:
			break;




	}



}





/***********************************************************************************************************************
* Function Name: LED_key_up_operation
* Description  : This function display the information on display board
* Arguments    : None
* Return Value : None
***********************************************************************************************************************/
void LED_key_up_operation(void)
{
	switch(LED_mode_count)
	{
	
		/*************************************************************************/ 
		case _LED_mode_1:
			//PID_temp++;
			break;
		/*************************************************************************/ 
		case _LED_mode_2:
			
#if 1
			if(TEMP_SELECT_SWITCH == FIX_TYPE_SENSOR)
			{
#if 0			
				SET_TEMP = SET_TEMP + 10;
				SET_TEMP = SET_TEMP/10;
				SET_TEMP = SET_TEMP*10;
#else
				SET_TEMP++;
#endif				
				if(SET_TEMP > eeprom_option_byte[_Set_temp_high] * 10)
					SET_TEMP = eeprom_option_byte[_Set_temp_high] *10;
				else if(SET_TEMP < eeprom_option_byte[_Set_temp_low] *10)
					SET_TEMP = eeprom_option_byte[_Set_temp_low] *10;
				/*************************************/
			}
			else
			{
				SET_TEMP++;
				if(SET_TEMP > eeprom_option_byte[_Set_temp_high_1])
					SET_TEMP = eeprom_option_byte[_Set_temp_high_1];
				else if(SET_TEMP < eeprom_option_byte[_Set_temp_low_1])
					SET_TEMP = eeprom_option_byte[_Set_temp_low_1];
				/*************************************/ 	
			}
#endif		
			flash_flag = 1;
			flash_count_buf = 0;
			LED_mode_display();
			break;
		/*************************************************************************/ 
		case _LED_mode_4:
			if(TEMP_SELECT_SWITCH == FIX_TYPE_SENSOR)
			{
				if(th_no==1)
					th_no=2;
				if(++th_no > 5)
					th_no=1;
			}
			else
			{
				if(++th_no > 5)
					th_no=1;
			}

			flash_flag = 0;
			flash_count_buf = 0;
			LED_mode_display();
			break;
			
		/*************************************************************************/ 
		case _LED_mode_5:
			if(--error_code_offset <= 0 )
				error_code_offset = 0;
			flash_flag = 1;
			flash_count_buf = 0;
			LED_mode_display();
			break;
#if 0			
		/*************************************************************************/ 
		case _LED_mode_8:
			break;
		/*************************************************************************/ 
		case _LED_mode_9:
			break;
#endif		
		/*************************************************************************/ 
		case _LED_mode_6:
			eeprom_option_byte[_Control_sensor_offset]++; 
			if(eeprom_option_byte[_Control_sensor_offset] < Control_sensor_offset_l) 
				eeprom_option_byte[_Control_sensor_offset] = Control_sensor_offset_l;
			if(eeprom_option_byte[_Control_sensor_offset] > Control_sensor_offset_h)	
				eeprom_option_byte[_Control_sensor_offset] = Control_sensor_offset_h;
			flash_flag = 1;
			flash_count_buf = 0;
			LED_mode_display();
			option_backup = 0xFF;
			break;
		/*************************************************************************/ 
		case _LED_mode_7:
			eeprom_option_byte[_Base_sensor_offset]++;
			if(eeprom_option_byte[_Base_sensor_offset] < Base_sensor_offset_l) 
				eeprom_option_byte[_Base_sensor_offset] = Base_sensor_offset_l;
			if(eeprom_option_byte[_Base_sensor_offset] > Base_sensor_offset_h)	
				eeprom_option_byte[_Base_sensor_offset] = Base_sensor_offset_h;
			flash_flag = 1;
			flash_count_buf = 0;
			LED_mode_display();
			option_backup = 0xFF;
			break;
#if 1		
		/*************************************************************************/ 
		case _LED_mode_9:
			if(Modbus_address == Modbus_address_max)
				Modbus_address=Modbus_address_min;
			else
				Modbus_address++;
			LED_mode_display();
			key_repeart_timer = 70;
			break;
#endif													
		/*************************************************************************/ 
		case _LED_mode_option:
			option_backup = 0xFF;
		/*************************************************************************/ 
			switch(option_byte_count_value)
			{
				default:
					eeprom_option_byte[option_byte_count_value]++;
					if(eeprom_option_byte[option_byte_count_value] > option_byte_high_limit[option_byte_count_value] )
						eeprom_option_byte[option_byte_count_value] = option_byte_low_limit[option_byte_count_value];
					break;
				case	_Liquid_high_temp_stop:
				case	_Liquid_low_temp_stop:
					liquid_temp_buf = (uint8_t)eeprom_option_byte[option_byte_count_value];
					liquid_temp_buf++;
					if(liquid_temp_buf > (uint8_t)option_byte_high_limit[option_byte_count_value] )
						liquid_temp_buf = (uint8_t)option_byte_low_limit[option_byte_count_value];
					eeprom_option_byte[option_byte_count_value] = liquid_temp_buf;
					break;
			}
		/*************************************************************************/ 
			LED_mode_display();
			break;






		
		/*************************************************************************/ 
		case _LED_key_check:
			if(++check_key_2 > 99)
				check_key_2 = 0;
			LED_mode_display();
			break;
		/*************************************************************************/ 


		default:
			break;






	}



}

/***********************************************************************************************************************
* Function Name: get_key_press
* Description  : This function handle the key press function on display board
* Arguments    : None
* Return Value : None
***********************************************************************************************************************/
void get_key_press(void)
{
#if 0
	do
	{
		convert_AD_channel();
	}
	while(adc_conv_end == RESET);

	
	key_data = T_DECP[_AD_key] ;

#endif

	if(key_data > 900)
	{
		if(key_press_status!=0)
		{
			key_press_status = 0; 
			key_repeart_timer = 0;
		}
		else
		{
			if(key_repeart_timer > key_timer_default)
			{
				LED_mode_count = _LED_mode_1;
				key_repeart_timer = key_timer_default;
				adjust_addr_key=0;
			}
		}
	}	
	else 	if((key_data < (key_value_on_off + key_offset)) && (key_data > (key_value_on_off - key_offset)))			// key on/off   740
			{
				if((key_press_status==0) && (LED_mode_count == _LED_mode_option))
				{
					if(option_byte_count == option_byte_count_min)
						option_byte_count=option_byte_count_max;
					else
						option_byte_count--;
					flash_flag = 0;
					flash_count_buf = 0;
					LED_mode_display();
				}
				else	if(key_press_status==0)
						{
							if(current_sys_op_power == RESET)  
							{
								turn_system_on();
							}
							else
							{
								turn_system_off();
								switch(ERROR_CODE)
								{
									case 0xd3:
									case 0xd6:
									case 0xd7:
									case 0xd1:
									case 0xc6:
									case 0xc7:
									case 0xc4:	
										clear_flag_for_sensor_continue();
										break;
									default:
										break;
								}
							}
							flash_flag = 0;
							flash_count_buf = 0;
							LED_mode_count = _LED_mode_1;
							LED_mode_display();
							// Buzzer_flag = _Buzzer_OneSound_Bi;
							adjust_addr_key=0;
						}
				key_press_status = 1; 

			}
			else 	if((key_data < (key_value_mode + key_offset)) && (key_data > (key_value_mode - key_offset)))						// key mode  630
					{

						if(key_press_status==0)
						{
							if(current_sys_op_power == SET)  
							{
								if(TEMP_SELECT_SWITCH == FIX_TYPE_SENSOR)
								{
									if(LED_mode_count++ >= _LED_mode_6)
										LED_mode_count = _LED_mode_1;
								}
								else
								{
									if(LED_mode_count++ >= _LED_mode_7)
										LED_mode_count = _LED_mode_1;
								}	
							}
							else
							{
								if(LED_mode_count == _LED_mode_option)
								{
									if(option_byte_count == option_byte_count_max)
										option_byte_count=option_byte_count_min;
									else
										option_byte_count++;

#if 0								
									if(option_modify_flag == 0)
									{
										option_modify_flag = 1;
									}	
									else
									{
										option_modify_flag = 0;
									}	
#endif									
								}
								else 	if(LED_mode_count == _LED_key_check)
										{
											check_key_1 = (check_key_1 * IIC_key_value_1)+ IIC_key_value_2;
											check_key_1 = check_key_1%100;
											if(check_key_1 == check_key_2)
											{
												LED_mode_count = _LED_mode_option;
												option_byte_count = 0;
												option_modify_flag = 1;
											}
											else
												LED_mode_count = _LED_mode_1;
										}
										else
										{
											if(adjust_addr_key==0)
												adjust_addr_key=1;
											if(adjust_addr_key==3)
											{
												
												LED_mode_count = _LED_key_check;			// jump to check key
												check_key_1	= ms_counter%100;
												check_key_2	= check_key_1;
												adjust_addr_key=0;
											}
										}
							}
							flash_flag = 0;
							flash_count_buf = 0;
							LED_mode_display();
						//	Buzzer_flag = _Buzzer_OneSound_Bi;
						}
						
						error_code_offset = 0;
						key_press_status = 1; 
						flash_flag = 1;
						flash_count_buf = 0;

			

					}
					else	if((key_data < (key_value_down + key_offset)) && (key_data > (key_value_down - key_offset)))				// key down   385
							{

								if(key_press_status==0)
								{
									if(adjust_addr_key==2)
										adjust_addr_key=3;
									else
										adjust_addr_key=0;
									LED_key_down_operation();
									key_repeart_timer = 0;
								//	Buzzer_flag = _Buzzer_OneSound_Bi;
								}
								key_press_status = 1; 
								if(key_repeart_timer > 5)
								{
									LED_key_down_operation();
									key_repeart_timer = 5;
								}



							}
							else	if((key_data < (key_value_up + key_offset)) && (key_data > (key_value_up - key_offset)))		// key up   293
									{
										if(key_press_status==0)
										{
											if(adjust_addr_key==1)
												adjust_addr_key=2;
											else
												adjust_addr_key=0;
											LED_key_up_operation();
											key_repeart_timer = 0;
										//	Buzzer_flag = _Buzzer_OneSound_Bi;
										}
										key_press_status = 1; 
										if(key_repeart_timer > 5)
										{
											LED_key_up_operation();
											key_repeart_timer = 5;
										}
									}
									else	if((key_data < (key_value_clear + key_offset)) && (key_data > (key_value_clear - key_offset)))    // key clear   171
											{
												if(key_press_status==0)
												{
													adjust_addr_key=0;
													key_repeart_timer = 0;
											//		Buzzer_flag = _Buzzer_OneSound_Bi;
													if(LED_mode_count != _LED_mode_5)
														LED_mode_count = _LED_mode_1;
												}
												key_press_status = 1; 
												if(key_repeart_timer > 50)
												{
													
													if(LED_mode_count == _LED_mode_5)
													{
														for(loop_temp=0;loop_temp<13;loop_temp++)
														{
															IIC_error_code[loop_temp] = 0;
														}
														error_code_backup = 0Xff;
													}
#if 0													
													else
													{
														filter_10hour_count = 0;
													}
#endif													
												}
							
											}
									

}

/***********************************************************************************************************************
* Function Name: LED_display
* Description  : This function display the information on display board
* Arguments    : None
* Return Value : None
***********************************************************************************************************************/
void LED_temp_display(void)
{
	LED_1_dot = RESET;
	LED_2_dot = RESET;
	LED_3_dot = SET;
	LED_4_dot = RESET;
	if(disp_temp_buf >= 1000)
	{	
		digi_0 = disp_temp_buf/1000;	
		digi_1 = (disp_temp_buf%1000)/100;
		digi_2 = (disp_temp_buf%100)/10;
		LED_3_dot = RESET;
	}
	else
	{
		if(disp_temp_buf >= 100)
		{
			digi_0 = disp_temp_buf/100;	
			digi_1 = (disp_temp_buf%100)/10;
			digi_2 = (disp_temp_buf%100)%10;
		}
		else
		{
			if(disp_temp_buf >= 0)
			{
				digi_0 = DISPLAY_SPACE;   // display space
				digi_1 = (disp_temp_buf)/10;
				digi_2 = (disp_temp_buf)%10;
			}
			else
			{
				digi_0 = DISPLAY_MINUS;   // display -
				disp_temp_buf = ~disp_temp_buf;
				disp_temp_buf++;
				digi_1 = disp_temp_buf/10;
				digi_2 = disp_temp_buf%10;
			}
		}
	}
	HC164_display_arry[0] = seven_seg_code[16];
	HC164_display_arry[1] = seven_seg_code[digi_0];
	HC164_display_arry[2] = seven_seg_code[digi_1];
	HC164_display_arry[3] = seven_seg_code[digi_2];



}



/***********************************************************************************************************************
* Function Name: LED_display
* Description  : This function display the information on display board
* Arguments    : None
* Return Value : None
***********************************************************************************************************************/
void LED_mode_display(void)
{

	/* *************** LED RUN operation ******************************************************** */ 

	if(current_sys_op_power == RESET)  
	{
		LED_RUN = RESET;
	}
	else
	{
		if(COMP_RLY == SET)
		{
			LED_RUN = SET;
		}
		else
		{
			if(flash_flag==0)
			{
				LED_RUN = SET;
			}
			else
			{
				LED_RUN = RESET;
			}
		}
	}

	/* *************** LED RUN operation ******************************************************** */ 

	LED_SV = RESET;
	LED_PV = RESET;


	switch(LED_mode_count)
	{
		/*************************************************************************/ 
		case _LED_mode_1:      // display CONTROL TEMP
			if(ERROR_CODE ==0)
			{
				th_display = CONTROL_TEMP + eeprom_option_byte[_Control_sensor_offset];
				disp_temp_buf = th_display;
				LED_temp_display();	
				
				if(SYSTEM_SELECT_SWITCH== RESET)	// on/off system
				{
					HC164_display_arry[0] = seven_seg_code[0];
				}
				else
				{
					HC164_display_arry[0] = seven_seg_code[12];
				}
				LED_PV = SET;
				LED_ALARM = RESET;
			}
			else
			{
				LED_ALARM = SET;
				if(flash_flag==0)
				{
					system_sensor_status();
					if(system_sensor_status_on)
					{
						/* ************* display control temp ***********************/
						HC164_display_arry[0] = seven_seg_code[14];
						
						th_display = CONTROL_TEMP + eeprom_option_byte[_Control_sensor_offset];
						disp_temp_buf = th_display;
						LED_temp_display(); 
						LED_PV = SET;
						if(SYSTEM_SELECT_SWITCH== RESET)	// on/off system
						{
							HC164_display_arry[0] = seven_seg_code[0];
						}
						else
						{
							HC164_display_arry[0] = seven_seg_code[12];
						}
					}
#if 0					
					// if(ERROR_CODE==0xd1 || ERROR_CODE==0xc6  || ERROR_CODE==0xd3 || ERROR_CODE==0xd4 || ERROR_CODE==0xd7 || ERROR_CODE==0xd8)
					if(ERROR_CODE==0xd1 || ERROR_CODE==0xc6  || ERROR_CODE==0xd3 || ERROR_CODE==0xd6 || ERROR_CODE==0xd7)
					{
						/* ************* display control temp ***********************/
						HC164_display_arry[0] = seven_seg_code[14];
						
						th_display = CONTROL_TEMP + eeprom_option_byte[_Control_sensor_offset];
						disp_temp_buf = th_display;
						LED_temp_display(); 
						LED_PV = SET;
						if(SYSTEM_SELECT_SWITCH== RESET)	// on/off system
						{
							HC164_display_arry[0] = seven_seg_code[0];
						}
						else
						{
							HC164_display_arry[0] = seven_seg_code[12];
						}
					}
#endif					
				}
				else
				{
					LED_1_dot = RESET;
					LED_2_dot = RESET;
					LED_3_dot = RESET;
					LED_4_dot = RESET;

					/* ************* display ERROR CODE ***********************/
					test_error_code = ERROR_CODE >> 4;
					HC164_display_arry[0] = seven_seg_code[14];
					switch(test_error_code)
					{
						default:
						case 0x0c:
							HC164_display_arry[1] = seven_seg_code[12];
							break;
						case 0x0d:
							HC164_display_arry[1] = seven_seg_code[23];
							break;	
						case 0x0e:
							HC164_display_arry[1] = seven_seg_code[14];
							break;
					}
						
					test_error_code = ERROR_CODE & 0x0f;
					HC164_display_arry[2] = seven_seg_code[0];
					HC164_display_arry[3] = seven_seg_code[test_error_code];
				}				
			}
			break;
		/*************************************************************************/ 
		case _LED_mode_2:		// display set temp and adjust
		
			LED_SV = SET;
			
#if 0
			if(flash_flag==1)
			{
				/* ************* display S_0 ***********************/
				HC164_display_arry[0] = seven_seg_code[16];
				HC164_display_arry[1] = seven_seg_code[16];
				HC164_display_arry[2] = seven_seg_code[16];
				HC164_display_arry[3] = seven_seg_code[16];
				LED_1_dot = RESET;
				LED_2_dot = RESET;
				LED_3_dot = RESET;
				LED_4_dot = RESET;
			}
			else
			{
				disp_temp_buf = SET_TEMP;
				LED_temp_display();
				
			}
#else	
			disp_temp_buf = SET_TEMP;
			LED_temp_display();
			
#endif			
			break;
		/*************************************************************************/ 
		case _LED_mode_4:

			switch(th_no)
			{
				default:
					case	1:
						th_display = CONTROL_TEMP + eeprom_option_byte[_Control_sensor_offset];
						break;
					case	2:
						if(TEMP_SELECT_SWITCH == FIX_TYPE_SENSOR)
						{
							th_display = CONDENSER_TEMP;
							th_no = 3;
						}
						else
						{
							th_display = BASE_TEMP + eeprom_option_byte[_Base_sensor_offset];
						}
						break;
					case	3:
						th_display = CONDENSER_TEMP;
						break;
					case	4:
						th_display = ANTIFRZ_TEMP;
						break;
					case	5:
						th_display = COMP_OUT_TEMP;
						break;
			}

			
			if(flash_flag==0)
			{
				/* ************* display th1`5 ***********************/
				HC164_display_arry[1] = seven_seg_code[27];
				HC164_display_arry[2] = seven_seg_code[28];
				HC164_display_arry[3] = seven_seg_code[th_no];
				HC164_display_arry[0] = seven_seg_code[16];
				LED_1_dot = RESET;
				LED_2_dot = RESET;
				LED_3_dot = RESET;
				LED_4_dot = RESET;
			}
			else
			{
				disp_temp_buf = th_display;
				LED_temp_display();
			}
			
			
			break;
		/*************************************************************************/ 
		case _LED_mode_5:				// system error buffer display
			if(flash_flag==0)
			{
				/* ************* display Err ***********************/
				HC164_display_arry[1] = seven_seg_code[14];
				HC164_display_arry[2] = seven_seg_code[26];
				HC164_display_arry[3] = seven_seg_code[26];
				HC164_display_arry[0] = seven_seg_code[16];
				LED_1_dot = RESET;
				LED_2_dot = RESET;
				LED_3_dot = RESET;
				LED_4_dot = RESET;
			}
			else
			{
#if 1  
				if(IIC_error_code[op_error_record_index] == 0)
				{
					/* ************* display noE ***********************/
					HC164_display_arry[1] = seven_seg_code[25];
					HC164_display_arry[2] = seven_seg_code[19];
					HC164_display_arry[3] = seven_seg_code[14];
					HC164_display_arry[0] = seven_seg_code[16];
				}
				else
				{
					/* ************* display ERROR CODE ***********************/
					
					error_code_last = IIC_error_code[op_error_record_index];
					error_code_pointer = error_code_last - error_code_offset;
					HC164_display_arry[0] = seven_seg_code[error_code_pointer];
					test_error_code = IIC_error_code[error_code_pointer];
					test_error_code = test_error_code >> 4;
					switch(test_error_code)
					{
						default:
						case 0x0c:
							HC164_display_arry[1] = seven_seg_code[12];
							break;
						case 0x0d:
							HC164_display_arry[1] = seven_seg_code[23];
							break;	
						case 0x0e:
							HC164_display_arry[1] = seven_seg_code[14];
							break;
					}
					
					test_error_code = IIC_error_code[error_code_pointer];	
					test_error_code = test_error_code & 0x0f;
					HC164_display_arry[2] = seven_seg_code[0];
					HC164_display_arry[3] = seven_seg_code[test_error_code];
					
				}
#endif
			}
			
			
			break;
			

		/*************************************************************************/ 
		case _LED_mode_6:
			if(flash_flag==0)
			{
				/* ************* display th1 ***********************/
				HC164_display_arry[1] = seven_seg_code[27];
				HC164_display_arry[2] = seven_seg_code[28];
				HC164_display_arry[3] = seven_seg_code[1];
				HC164_display_arry[0] = seven_seg_code[16];
				LED_1_dot = RESET;
				LED_2_dot = RESET;
				LED_3_dot = RESET;
				LED_4_dot = RESET;
			}
			else
			{
				disp_temp_buf = eeprom_option_byte[_Control_sensor_offset];
				LED_temp_display();
			}
			break;
			
		/*************************************************************************/ 
		case _LED_mode_7:
			if(flash_flag==0)
			{
				/* ************* display th2 ***********************/
				HC164_display_arry[1] = seven_seg_code[27];
				HC164_display_arry[2] = seven_seg_code[28];
				HC164_display_arry[3] = seven_seg_code[2];
				HC164_display_arry[0] = seven_seg_code[16];
				LED_1_dot = RESET;
				LED_2_dot = RESET;
				LED_3_dot = RESET;
				LED_4_dot = RESET;
			}
			else
			{
				disp_temp_buf = eeprom_option_byte[_Base_sensor_offset];
				LED_temp_display();
			}
			break;
			
		/*************************************************************************/ 
		/*************************************************************************/ 
		case _LED_mode_8:        // system error status display
			if(flash_flag==0)
			{
				system_sensor_status();
				if(system_sensor_status_on)
				{
					/* ************* display control temp ***********************/
					HC164_display_arry[0] = seven_seg_code[14];
					
					th_display = CONTROL_TEMP + eeprom_option_byte[_Control_sensor_offset];
					disp_temp_buf = th_display;
					LED_temp_display(); 
					LED_PV = SET;
					if(SYSTEM_SELECT_SWITCH== RESET)	// on/off system
					{
						HC164_display_arry[0] = seven_seg_code[0];
					}
					else
					{
						HC164_display_arry[0] = seven_seg_code[12];
					}
				}

#if 0			
				// if(ERROR_CODE==0xd1 || ERROR_CODE==0xc6  || ERROR_CODE==0xd3 || ERROR_CODE==0xd4 || ERROR_CODE==0xd7 || ERROR_CODE==0xd8)
					if(ERROR_CODE==0xd1 || ERROR_CODE==0xc6  || ERROR_CODE==0xd3 || ERROR_CODE==0xd6 || ERROR_CODE==0xd7)
				{
					/* ************* display control temp ***********************/
					HC164_display_arry[0] = seven_seg_code[14];
					
					th_display = CONTROL_TEMP + eeprom_option_byte[_Control_sensor_offset];
					disp_temp_buf = th_display;
					LED_temp_display(); 
					LED_PV = SET;
					if(SYSTEM_SELECT_SWITCH== RESET)	// on/off system
					{
						HC164_display_arry[0] = seven_seg_code[0];
					}
					else
					{
						HC164_display_arry[0] = seven_seg_code[12];
					}
				}
#endif
			}
			else
			{
				if(ERROR_CODE == 0)
				{
					LED_ALARM = RESET;
					/* ************* display noE ***********************/
					HC164_display_arry[0] = seven_seg_code[14];
					HC164_display_arry[1] = seven_seg_code[25];
					HC164_display_arry[2] = seven_seg_code[19];
					HC164_display_arry[3] = seven_seg_code[14];
				}
				else
				{
					/* ************* display ERROR CODE ***********************/
					test_error_code = ERROR_CODE >> 4;
					HC164_display_arry[0] = seven_seg_code[14];
					switch(test_error_code)
					{
						default:
						case 0x0c:
							HC164_display_arry[1] = seven_seg_code[12];
							break;
						case 0x0d:
							HC164_display_arry[1] = seven_seg_code[23];
							break;	
						case 0x0e:
							HC164_display_arry[1] = seven_seg_code[14];
							break;
					}
						
					test_error_code = ERROR_CODE & 0x0f;
					HC164_display_arry[2] = seven_seg_code[0];
					HC164_display_arry[3] = seven_seg_code[test_error_code];
				}
			
			}
			
			break;

#if 1		
		/************************* display modbus no ************************************************/ 
		case _LED_mode_9:
			HC164_display_arry[0] = seven_seg_code[17];
			HC164_display_arry[1] = seven_seg_code[17];
			HC164_display_arry[2] = seven_seg_code[17];
			disp_temp_buf = Modbus_address;
			LED_1_dot = RESET;
			LED_2_dot = RESET;
			LED_3_dot = RESET;
			digi_0 = disp_temp_buf/100;	
			digi_1 = (disp_temp_buf%100)/10;
			digi_2 = (disp_temp_buf%100)%10;
			HC164_display_arry[0] = seven_seg_code[16];
			HC164_display_arry[1] = seven_seg_code[16];		// display space
			HC164_display_arry[2] = seven_seg_code[digi_1];
			HC164_display_arry[3] = seven_seg_code[digi_2];

			
			break;
#endif						
		/*************************************************************************/ 
		/*************************************************************************/ 
		case _LED_mode_option:


			LED_1_dot = RESET;
			LED_2_dot = RESET;
			LED_3_dot = RESET;
			LED_4_dot = RESET;

			option_byte_count_value = option_byte_pointer[option_byte_count];	
			if(option_byte_count_value > 19)
			{
				option_byte_count_value = option_byte_count_value - 20;
				LED_1_dot = SET;
			}
			HC164_display_arry[0] = option_seg_code[option_byte_count_value];
			
			option_byte_count_value = option_byte_pointer[option_byte_count];
			if(option_byte_count_value == 31)											// PID_P valuse
				HC164_display_arry[0] = option_seg_code[20];
			if(option_byte_count_value == 32)											// PID_I valuse	
				HC164_display_arry[0] = option_seg_code[21];

			option_byte_count_value = option_byte_pointer[option_byte_count];	
			switch(option_byte_count_value)
			{

/* ************************************** 0.0 ****************************** */
					case	_Set_temp_high_1:	
					case	_Set_temp_low_1:	
					case	_Heater_start_offset:								
					case	_Heater_stop_offset:
					case	_Control_sensor_offset:
					case	_Base_sensor_offset:	
					case	_comp_current_protect_0_value:
					case	_comp_current_protect_1_value:
					case	_comp_current_protect_2_value:
					case	_comp_current_protect_3_value:
					case	_COMP_start_temp_offset:	
					case	_PID_P_factor_value:
					case	_PID_I_factor_value:
					case	_PID_D_factor_value:	
				
						LED_3_dot = SET;
	
					case	_Liquid_high_temp_alarm:
					case	_Liquid_low_temp_alarm:
					case	_Set_temp_high:
					case	_Set_temp_low:
					case	_System_off_COMP_protect_timer_set:		
					case	_Flow_switch_start_check_time:	
					case	_LP_start_check_time:
					case	_System_on_PUMP_delay:	
					case	_System_on_FAN_delay:
					case	_System_on_COMP_start_delay:	
					case	_System_off_PUMP_delay:
					case	_System_off_FAN_delay:								
					case	_Condenser_temp_protect_value:	
					case	_Antifrz_protect_stop:
					case	_System_off_PMV_delay:	
					case	_Modbus_address_offset:
					case	_Temp_control_COMP_protect_timer_set:
					case	_Temp_control_FAN_delay_off:

						disp_temp_buf = eeprom_option_byte[option_byte_count_value];
						if(disp_temp_buf >= 100)
						{
							digi_0 = disp_temp_buf/100; 
							digi_1 = (disp_temp_buf%100)/10;
							digi_2 = (disp_temp_buf%100)%10;
						}
						else
						{
							if(disp_temp_buf >= 0)
							{
								digi_0 = DISPLAY_SPACE;   // display space
								digi_1 = (disp_temp_buf)/10;
								digi_2 = (disp_temp_buf)%10;
							}
							else
							{
								digi_0 = DISPLAY_MINUS;   // display -
								disp_temp_buf = ~disp_temp_buf;
								disp_temp_buf++;
								digi_1 = disp_temp_buf/10;
								digi_2 = disp_temp_buf%10;
							}
						}
					
						
						HC164_display_arry[1] = seven_seg_code[digi_0];
						HC164_display_arry[2] = seven_seg_code[digi_1];
						HC164_display_arry[3] = seven_seg_code[digi_2];
						
						break;

					case	_Liquid_high_temp_stop:
					case 	_Liquid_low_temp_stop:
						
						LED_3_dot = SET;
						liquid_temp_buf = (uint8_t)eeprom_option_byte[option_byte_count_value]/2;
						digi_0 = liquid_temp_buf/10; 
						digi_1 = (liquid_temp_buf)%10;
						liquid_temp_buf = (uint8_t)eeprom_option_byte[option_byte_count_value];
						if((liquid_temp_buf%2) == 0)
						{
							digi_2 = 0;
						}
						else
						{
							digi_2 = 5;
						}
						HC164_display_arry[1] = seven_seg_code[digi_0];
						HC164_display_arry[2] = seven_seg_code[digi_1];
						HC164_display_arry[3] = seven_seg_code[digi_2];

				default:
						break;

			}
			
				break;
		/*************************************************************************/ 
		/*************************************************************************/ 
		case _LED_key_check:
				
			LED_1_dot = RESET;
			LED_2_dot = RESET;
			LED_3_dot = SET;
			LED_4_dot = SET;
			
			digi_1 = (check_key_1)/10;
			digi_2 = (check_key_1)%10;
			HC164_display_arry[0] = seven_seg_code[digi_1];
			HC164_display_arry[1] = seven_seg_code[digi_2];
			
			digi_1 = (check_key_2)/10;
			digi_2 = (check_key_2)%10;
			HC164_display_arry[2] = seven_seg_code[digi_1];
			HC164_display_arry[3] = seven_seg_code[digi_2];
				break;
		/*************************************************************************/ 

		default:
			break;
			







	}



}

/***********************************************************************************************************************
* Function Name: volatage_adjust_operation
* Description  : This function display the information on display board
* Arguments    : None
* Return Value : None
***********************************************************************************************************************/
#if 0
void volatage_adjust_operation(void)
{

	voltage_buffer[voltage_buffer_pointer] = T_DECP[_Voltage_detect];
	if(++voltage_buffer_pointer >= 30)
	{
		voltage_buffer_pointer = 0;
	
		voltage_value_1 = 0;
		for(voltage_loop_count= 0;voltage_loop_count <=29;voltage_loop_count++)
		{
			voltage_value_1 = voltage_value_1 + voltage_buffer[voltage_buffer_pointer];
		}
		voltage_value_1 = voltage_value_1 / 30;
		
		voltage_offset = voltage_value_1 - default_220v;
		
		voltage_value_1 = voltage_offset * 10;
		voltage_value_1 = voltage_value_1 / 34;
		
		voltage_value_1 = voltage_value_1 + 220;
	}
	if(voltage_buffer_pointer == 0)
	{
		voltage_buffer_1[voltage_buffer_pointer_1] = voltage_value_1;
		if(++voltage_buffer_pointer_1 >= 30)
		{
			voltage_buffer_pointer_1 = 0;
		
			voltage_value = 0;
			for(voltage_loop_count= 0;voltage_loop_count <=29;voltage_loop_count++)
			{
				voltage_value = voltage_value + voltage_buffer_1[voltage_buffer_pointer];
			}
			voltage_value = voltage_value / 30;
			
		}
	}



	
	LED_mode_count = _LED_mode_9;
	LED_mode_display();
	
}

#endif
/***********************************************************************************************************************
* Function Name: LED_display
* Description  : This function display the information on display board
* Arguments    : None
* Return Value : None
***********************************************************************************************************************/
void LED_display(void)
{
uint8_t	LED_display_flag;
	LED_display_flag = 00;

#if 1

	/* ************  start LED display function *********************************** */	
		if(LED_1_dot)
			HC164_display_arry[0] &= 0x7f;
		else
			HC164_display_arry[0] |= 0x80;
	
		if(LED_2_dot)
			HC164_display_arry[1] &= 0x7f;
		else
			HC164_display_arry[1] |= 0x80;
	
		if(LED_3_dot)
			HC164_display_arry[2] &= 0x7f;
		else
			HC164_display_arry[2] |= 0x80;
		if(LED_4_dot)
			HC164_display_arry[3] &= 0x7f;
		else
			HC164_display_arry[3] |= 0x80;

		HC164_display_arry[4] = 0xff;
		if(LED_PV)
			HC164_display_arry[4] &= 0xFE;
		else
			HC164_display_arry[4] |= 0x01;
	
		if(LED_ALARM)
			HC164_display_arry[4] &= 0xFD;
		else
			HC164_display_arry[4] |= 0x02;
	
		if(LED_RUN)
			HC164_display_arry[4] &= 0xFB;
		else
			HC164_display_arry[4] |= 0x04;
		
		if(LED_SV)
			HC164_display_arry[4] &= 0xF7;
		else
			HC164_display_arry[4] |= 0x08;
	
		
	/* ************  start LED display function *********************************** */	
#endif



	
	if(HC164_display_arry[0] != HC164_display_arry_backup[0])
	{
		HC164_display_arry_backup[0] = HC164_display_arry[0];
		LED_display_flag = 01;
	}
	if(HC164_display_arry[1] != HC164_display_arry_backup[1])
	{
		HC164_display_arry_backup[1] = HC164_display_arry[1];
		LED_display_flag = 01;
	}
	if(HC164_display_arry[2] != HC164_display_arry_backup[2])
	{
		HC164_display_arry_backup[2] = HC164_display_arry[2];
		LED_display_flag = 01;
	}
	if(HC164_display_arry[3] != HC164_display_arry_backup[3])
	{
		HC164_display_arry_backup[3] = HC164_display_arry[3];
		LED_display_flag = 01;
	}
	if(HC164_display_arry[4] != HC164_display_arry_backup[4])
	{
		HC164_display_arry_backup[4] = HC164_display_arry[4];
		LED_display_flag = 01;
	}

	
	LED_display_operation_1();
	
	if(LED_display_timer++ > 500)
	{
		LED_display_flag = 01;
		LED_display_timer = 0;
	}

//	LED_display_flag = 01;
	if(LED_display_flag == 1)   
		LED_display_operation_2();
}
/***********************************************************************************************************************
* Function Name: LED_display
* Description  : This function display the information on display board
* Arguments    : None
* Return Value : None
***********************************************************************************************************************/
void LED_display_operation_1(void)
{
	get_V_A_timer++;
	
	if(flash_count_buf++ > 1000)
	{
		if(flash_flag==0)
			flash_flag = 1;
		else
			flash_flag = 0;

		flash_count_buf = 0;
		if(production_mode == RESET)
			LED_mode_display();
	}
/* ************  end of i sec operation*********************************** */	
#if 0
/* ************  filter count operation ************************************ */
	if(filter_one_sec_count++  > 1000)
	{
		filter_one_sec_count = 0;
		if(current_sys_op_power == SET)  
		{
			if(++filter_sec_count > 36000)    
			{
				filter_sec_count = 0;
				if(++filter_10hour_count > 100)
					filter_10hour_count = 100;
			}
		}
/* ************  filter led operation ************************************ */
		
		if(filter_10hour_count >= 50)
		{
			if(filter_10hour_count >= 100)
			{
				LED_FILTER = SET;
			}
			else
			{
				if(flash_flag==0)
					LED_FILTER = SET;
				else
					LED_FILTER = RESET;
			}
		}
		else
			if(production_mode == RESET)
				LED_FILTER = RESET;
	}
/* ************  filter led operation ************************************ */		
#endif



/* ************  start of 0.01 sec operation*********************************** */	

	if(key_press_check_time++ > 2 )	// old 10
	{
		key_press_check_time = 0;
		if(key_repeart_timer_1++ >50)	//old 10
		{
			key_repeart_timer_1 = 0;
			key_repeart_timer++;
		}

		system_sensor_status();
		if(system_sensor_status_on)
		{
			if(production_mode == RESET)
			{
				// if(remote_on_off_flag != SET)
				{
					key_data = T_DECP[_AD_key] ;
					if(REMOTE_SW_FUNCTION == 0)	
						key_data = 1000;

					if((key_data < (key_debunse + key_offset)) && (key_data > (key_debunse - key_offset)))
					{
						if(key_debunse_count++ > 5)					
						{
							key_debunse_count = 10;
							get_key_press();
						}
					}
					else
					{
						key_debunse = key_data; 
						key_debunse_count = 0;
					}
				}
			}
			else
			{
				if(production_key_press_check_time++ >10)
				{
					get_production_key_press();
					production_process();
					production_key_press_check_time = 0;
				}
			}
		}
		else
		{
			LED_mode_count = _LED_mode_1;
		}

#if 0		
		if( ERROR_CODE==0 || ERROR_CODE==0xd3 || ERROR_CODE==0xc7 || ERROR_CODE==0xd6 || ERROR_CODE==0xd7 || ERROR_CODE==0xc6 )
		{
			if(production_mode == RESET)
			{
				// if(remote_on_off_flag != SET)
				{
					key_data = T_DECP[_AD_key] ;
					if(REMOTE_SW_FUNCTION == 0)	
						key_data = 1000;

					if((key_data < (key_debunse + key_offset)) && (key_data > (key_debunse - key_offset)))
					{
						if(key_debunse_count++ > 5)					
						{
							key_debunse_count = 10;
							get_key_press();
						}
					}
					else
					{
						key_debunse = key_data; 
						key_debunse_count = 0;
					}
				}
			}
			else
			{
				if(production_key_press_check_time++ >10)
				{
					get_production_key_press();
					production_process();
					production_key_press_check_time = 0;
				}
			}
		}
		else
		{
			LED_mode_count = _LED_mode_1;
		}
#endif		
	}
}


/***********************************************************************************************************************
* Function Name: LED_display
* Description  : This function display the information on display board
* Arguments    : None
* Return Value : None
***********************************************************************************************************************/
void LED_display_operation_2(void)
{



/* ************  start LED display function *********************************** */	

	MC14094_CLK = RESET;
	MC14094_STRB = RESET;


	for( led_data_shift=0; led_data_shift < 8;led_data_shift++)
	{
		check_b7 = HC164_display_arry[3]<<led_data_shift;
		check_b7 &= 0x80; 
		if(check_b7 == 0)
			MC14094_SER = 0;
		else
			MC14094_SER = 1;
		
		MC14094_CLK = RESET;
		for(clk_delay = 0; clk_delay < 30; clk_delay++)
		{
			//NOP;
		}
		MC14094_CLK = SET;
		for(clk_delay = 0; clk_delay < 30; clk_delay++)
		{
			//NOP;
		}
		MC14094_CLK = RESET;
	}

#if 1
/* *************************************************************************** */
	for( led_data_shift=0; led_data_shift < 8;led_data_shift++)
	{
		check_b7 = HC164_display_arry[4]<<led_data_shift;
		check_b7 &= 0x80; 
		if(check_b7 == 0)
			MC14094_SER = 0;
		else
			MC14094_SER = 1;
		
		MC14094_CLK = RESET;
		for(clk_delay = 0; clk_delay < 30; clk_delay++)
		{
			//NOP;
		}
		MC14094_CLK = SET;
		for(clk_delay = 0; clk_delay < 10; clk_delay++)
		{
			//NOP;
		}
		MC14094_CLK = RESET;
	}

/* *************************************************************************** */
	for( led_data_shift=0; led_data_shift < 8;led_data_shift++)
	{
		check_b7 = HC164_display_arry[3]<<led_data_shift;
		check_b7 &= 0x80; 
		if(check_b7 == 0)
			MC14094_SER = 0;
		else
			MC14094_SER = 1;
		
		MC14094_CLK = RESET;
		for(clk_delay = 0; clk_delay < 30; clk_delay++)
		{
			//NOP;
		}
		MC14094_CLK = SET;
		for(clk_delay = 0; clk_delay < 10; clk_delay++)
		{
			//NOP;
		}
		MC14094_CLK = RESET;
	}
/* *************************************************************************** */
	for( led_data_shift=0; led_data_shift < 8;led_data_shift++)
	{
		check_b7 = HC164_display_arry[2]<<led_data_shift;
		check_b7 &= 0x80; 
		if(check_b7 == 0)
			MC14094_SER = 0;
		else
			MC14094_SER = 1;
		
		MC14094_CLK = RESET;
		for(clk_delay = 0; clk_delay < 30; clk_delay++)
		{
			//NOP;
		}
		MC14094_CLK = SET;
		for(clk_delay = 0; clk_delay < 10; clk_delay++)
		{
			//NOP;
		}
		MC14094_CLK = RESET;
	}

	for( led_data_shift=0; led_data_shift < 8;led_data_shift++)
	{
		check_b7 = HC164_display_arry[1]<<led_data_shift;
		check_b7 &= 0x80; 
		if(check_b7 == 0)
			MC14094_SER = 0;
		else
			MC14094_SER = 1;
		
		MC14094_CLK = RESET;
		for(clk_delay = 0; clk_delay < 30; clk_delay++)
		{
			//NOP;
		}
		MC14094_CLK = SET;
		for(clk_delay = 0; clk_delay < 10; clk_delay++)
		{
			//NOP;
		}
		MC14094_CLK = RESET;
	}

	for( led_data_shift=0; led_data_shift < 8;led_data_shift++)
	{
		check_b7 = HC164_display_arry[0]<<led_data_shift;
		check_b7 &= 0x80; 
		if(check_b7 == 0)
			MC14094_SER = 0;
		else
			MC14094_SER = 1;
	
		MC14094_CLK = RESET;
		for(clk_delay = 0; clk_delay < 30; clk_delay++)
		{
			//NOP;
		}
		MC14094_CLK = SET;
		for(clk_delay = 0; clk_delay < 10; clk_delay++)
		{
			//NOP;
		}
		MC14094_CLK = RESET;
	}

#endif
	MC14094_STRB = SET;
	MC14094_CLK = RESET;





	/*************************************************************************/	

	/*************************************************************************/	

}

/***********************************************************************************************************************
* Function Name: powerin_test_disp
* Description  : This function display the information on display board
* Arguments    : None
* Return Value : None
***********************************************************************************************************************/
void powerin_test_disp(void)
{
#if 1
	//ms_counter = 0;
	
	convert_AD_channel_key();

	key_data = current_key_now;
	production_mode = RESET;
	if((key_data < (key_value_on_off + key_offset)) && (key_data > (key_value_on_off - key_offset)))								// key on/off enter production program
	{
		production_mode = SET;
		Production_mode_count = _PRODUCTION_mode_2;
	}

		

		
#endif


}

/***********************************************************************************************************************
* Function Name: get_production_key_press
* Description  : This function handle the key press function on display board
* Arguments    : None
* Return Value : None
***********************************************************************************************************************/
void get_production_key_press(void)
{
#if 0
	do
	{
		convert_AD_channel();
	}
	while(adc_conv_end == RESET);
#endif
	
	key_data = T_DECP[_AD_key] ;

	if(key_data > 900)
	{
		if(key_press_status!=0)
		{
			key_press_status = 0; 
			key_repeart_timer = 0;
		}
		else
		{
			if(key_repeart_timer > 100)
			{
				// LED_mode_count = _LED_mode_1;
				key_repeart_timer = 100;
			}
		}
		
	}	
	else 	if((key_data < (key_value_on_off + key_offset)) && (key_data > (key_value_on_off - key_offset)))								// key on/off
			{
				if(key_press_status==0)
				{
					Production_mode_count = _PRODUCTION_mode_2;
					relay_test_timer=0;
				//	Buzzer_flag = _Buzzer_OneSound_Bi;
				}
				key_press_status = 1; 

			}
			else 	if((key_data < (key_value_mode + key_offset)) && (key_data > (key_value_mode - key_offset)))						// key mode
					{

						if(key_press_status==0)
						{
							Production_mode_count = _PRODUCTION_mode_1;
							relay_test_timer=0;
							flash_flag=0;
						//	Buzzer_flag = _Buzzer_OneSound_Bi;
						}
						key_press_status = 1; 
						
					}
					else	if((key_data < (key_value_down + key_offset)) && (key_data > (key_value_down - key_offset)))				// key down
							{
								if(key_press_status==0)
								{
									Production_mode_count = _PRODUCTION_mode_3;
									relay_test_timer=0;
									flash_flag=0;
								//	Buzzer_flag = _Buzzer_OneSound_Bi;
								}
								key_press_status = 1; 
							}
							else	if((key_data < (key_value_up + key_offset)) && (key_data > (key_value_up - key_offset)))		// key up
									{
										if(key_press_status==0)
										{
											Production_mode_count = _PRODUCTION_mode_4;
											relay_test_timer=0;
											flash_flag=0;
										//	Buzzer_flag = _Buzzer_OneSound_Bi;
										}
										key_press_status = 1; 

									}
									else	if((key_data < (key_value_clear + key_offset)) && (key_data > (key_value_clear - key_offset)))              // key clear
											{
												if(key_press_status==0)
												{
													Production_mode_count = _PRODUCTION_mode_5;
													relay_test_timer=0;
													flash_flag=0;
												//	Buzzer_flag = _Buzzer_OneSound_Bi;
												}
												key_press_status = 1; 
							
											}
									

}

/***********************************************************************************************************************
* Function Name: production_process
* Description  : This function display the information on display board
* Arguments    : None
* Return Value : None
***********************************************************************************************************************/
void production_process(void)
{
	uint8_t	switch__set_check;
	
	relay_test_timer++;
	switch(Production_mode_count)
	{
		/*************************************************************************/ 
		case _PRODUCTION_mode_1:	   // RELAY test
		
			/* ************* display Err ***********************/

				switch__set_check = 0;
#if 1

				if(SWDIP4_1)
					switch__set_check = switch__set_check | 0x01;
				if(SWDIP4_2)
					switch__set_check = switch__set_check | 0x02;
				if(SWDIP4_3)
					switch__set_check = switch__set_check | 0x04;
				if(SWDIP4_4)
					switch__set_check = switch__set_check | 0x08;
				HC164_display_arry[0] = seven_seg_code[switch__set_check];

				switch__set_check = 0;
				if(SWDIP2_1)
					switch__set_check = switch__set_check | 0x01;
				if(SWDIP2_2)
					switch__set_check = switch__set_check | 0x02;
				HC164_display_arry[1] = seven_seg_code[switch__set_check];				

#endif		
				// HC164_display_arry[0] = seven_seg_code[switch__set_check];
				// HC164_display_arry[1] = seven_seg_code[16];
				HC164_display_arry[2] = seven_seg_code[SW_version/10];
				HC164_display_arry[3] = seven_seg_code[SW_version%10];
				
				LED_1_dot = RESET;
				LED_2_dot = RESET;
				LED_3_dot = RESET;
				LED_4_dot = RESET;
			if(relay_test_timer < 10)
			{
				COMP_RLY = SET;
				PUMP_RLY = RESET;
				FAN_RLY = RESET;
				HEATER_RLY = RESET;
				ERROR_RLY = RESET;
				LED_1_dot = SET;
			}
			if((relay_test_timer > 10)  && (relay_test_timer < 20 ))
			{
				COMP_RLY = RESET;
				PUMP_RLY = SET;
				FAN_RLY = RESET;
				HEATER_RLY = RESET;
				ERROR_RLY = RESET;
				LED_2_dot = SET;
			}
			if((relay_test_timer > 20)  && (relay_test_timer < 30))
			{
				COMP_RLY = RESET;
				PUMP_RLY = RESET;
				FAN_RLY = SET;
				HEATER_RLY = RESET;
				ERROR_RLY = RESET;
				LED_3_dot = SET;
			} 
			if((relay_test_timer > 30)  && (relay_test_timer < 40))
			{
				COMP_RLY = RESET;
				PUMP_RLY = RESET;
				FAN_RLY = RESET;
				HEATER_RLY = SET;
				ERROR_RLY = RESET;
				LED_4_dot = SET;
			} 
			if((relay_test_timer > 40)  && (relay_test_timer < 50))
			{
				COMP_RLY = RESET;
				PUMP_RLY = RESET;
				FAN_RLY = RESET;
				HEATER_RLY = RESET;
				ERROR_RLY = SET;
				LED_1_dot = SET;
				LED_2_dot = SET;
			} 
			if(relay_test_timer > 50)
			{
				relay_test_timer = 0;
			} 

		
			break;
		/*************************************************************************/ 
		
		case _PRODUCTION_mode_2:	   // temp sensor test test

			COMP_RLY = RESET;
			PUMP_RLY = RESET;
			FAN_RLY = RESET;
			ERROR_RLY = RESET;



			if(relay_test_timer < 20)
			{	
				th_no = 1;
				
			}
			else	if((relay_test_timer > 20)  && (relay_test_timer < 40))
					{
						th_no = 2;
						
					}
					else	if((relay_test_timer > 40)  && (relay_test_timer < 60))
							{
								th_no = 3;
								
							} 
							else	if((relay_test_timer > 60)  && (relay_test_timer < 80))
									{
										th_no = 4;
									
									} 
									else	if((relay_test_timer > 80)  && (relay_test_timer < 100))
											{
												th_no = 5;
											} 
											else	if(relay_test_timer > 100)
													{
														relay_test_timer = 0;
														th_no = 1;
													} 	
#if 1			
			switch(th_no)
			{
				default:
					case	1:
						th_display = CONTROL_TEMP + eeprom_option_byte[_Control_sensor_offset];
						break;
					case	2:
						if(TEMP_SELECT_SWITCH == FIX_TYPE_SENSOR)
						{
							th_display = 00;
						}
						else
						{
							th_display = BASE_TEMP + eeprom_option_byte[_Base_sensor_offset];
						}
						break;
					case	3:
						th_display = CONDENSER_TEMP;
						break;	
					case	4:
						th_display = ANTIFRZ_TEMP;
						break;
					case	5:
						th_display = COMP_OUT_TEMP;
						break;
					
			}

#endif		
#if 0
			if(flash_flag==0)
			{
				/* ************* display th1`5 ***********************/
				HC164_display_arry[0] = seven_seg_code[27];
				HC164_display_arry[1] = seven_seg_code[28];
				HC164_display_arry[2] = seven_seg_code[th_no];
				HC164_display_arry[3] = seven_seg_code[16];
				LED_1_dot = RESET;
				LED_2_dot = RESET;
				LED_3_dot = RESET;
				LED_4_dot = RESET;
				LED_display();
			}
			else
#endif				
			{
				
				disp_temp_buf = th_display;
				LED_temp_display();
				HC164_display_arry[0] = seven_seg_code[th_no];
			}		
		
			break;
		/*************************************************************************/ 
		default:		
		case _PRODUCTION_mode_3:	   // Voltage  test
		
			COMP_RLY = RESET;
			PUMP_RLY = RESET;
			FAN_RLY = RESET;
			HEATER_RLY = RESET;
			ERROR_RLY = RESET;
			
			LED_1_dot = RESET;
			LED_2_dot = RESET;
			LED_3_dot = RESET;
			LED_RUN = SET;
			LED_ALARM = SET;
			
//			LED_FILTER = SET;
#if 1	
			voltage_offset = voltage_data_max;

			if(PUMP_RLY == SET)
				voltage_offset = voltage_offset + voltage_small_relay_offset;
			if(FAN_RLY == SET)
				voltage_offset = voltage_offset + voltage_small_relay_offset;
			if(ERROR_RLY == SET)
				voltage_offset = voltage_offset + voltage_small_relay_offset;
			if(HEATER_RLY == SET)
				voltage_offset = voltage_offset + voltage_small_relay_offset;
			if(COMP_RLY == SET)
				voltage_offset = voltage_offset + voltage_large_relay_offset;

			voltage_offset = voltage_offset - default_220v;
			
			voltage_value_1 = voltage_offset * 10;
			voltage_value_1 = voltage_value_1 / 34;
			
			voltage_value_1 = voltage_value_1 + 220;

#endif		

			digi_0 = voltage_value_1/100;  
			digi_1 = (voltage_value_1%100)/10;
			digi_2 = (voltage_value_1%100)%10;
			
			HC164_display_arry[0] = seven_seg_code[digi_0];
			HC164_display_arry[1] = seven_seg_code[digi_1];
			HC164_display_arry[2] = seven_seg_code[digi_2];
			HC164_display_arry[3] = seven_seg_code[16];

			break;
		/*************************************************************************/ 
		case _PRODUCTION_mode_4:	   // Current test
		
			COMP_RLY = SET;
			PUMP_RLY = RESET;
			FAN_RLY = RESET;
			HEATER_RLY = RESET;
			ERROR_RLY = RESET;
			
			LED_1_dot = RESET;
			LED_2_dot = RESET;
			LED_3_dot = SET;
			LED_4_dot = RESET;
			LED_RUN = SET;
			LED_ALARM = SET;
//			LED_FILTER = SET;

			voltage_value_1 = current_data_max - default_0A ; 
			voltage_value_1 = voltage_value_1 * 7;
			voltage_value_1 = voltage_value_1 / 10;

//			if(voltage_value_1 > 20)
//				voltage_value_1 = voltage_value_1 - 4;			//	   current sensor offset 4 = 0.2A, 20 = 1A, 2019/04/12 modify to 4=0.2A offset
	
			voltage_value_1 = voltage_value_1 + 8;				// current sensor adjust 8 = 0.4A
			digi_0 = voltage_value_1/200;  
			digi_1 = (voltage_value_1%200)/20;
			digi_2 = (voltage_value_1%20)/2;
			
			HC164_display_arry[0] = seven_seg_code[16];
			HC164_display_arry[1] = seven_seg_code[digi_0];
			HC164_display_arry[2] = seven_seg_code[digi_1];
			HC164_display_arry[3] = seven_seg_code[digi_2];
				
		
			break;
		/*************************************************************************/ 
		case _PRODUCTION_mode_5:	   // outsize sensor input test

			COMP_RLY = RESET;
			PUMP_RLY = RESET;
			FAN_RLY = RESET;
			HEATER_RLY = RESET;
			ERROR_RLY = RESET;

			HC164_display_arry[0] = seven_seg_code[16];
//			HC164_display_arry[1] = seven_seg_code[0];
//			HC164_display_arry[2] = seven_seg_code[1];

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
#define	REMOTE_ON_OFF_SW		P12_bit.no2


			
			if(HEATER_TEMP_PROTECT == 0)
				LED_1_dot = RESET;
			else
				LED_1_dot = SET;
			if(WATERLEVER_LOW_ALARM == 0)
				LED_2_dot = RESET;
			else
				LED_2_dot = SET;
			if(FLOW_SWITCH_PROTECT == 0)
				LED_3_dot = RESET;
			else
				LED_3_dot = SET;
			if(LP_PROTECT == 0)
				LED_4_dot = RESET;
			else
				LED_4_dot = SET;

			if(HP_PROTECT == 0)
				LED_PV = RESET;
			else
				LED_PV = SET;
			if(POWER_PHASE_PROTECT == 0)
				LED_SV = RESET;
			else
				LED_SV = SET;

			if(PUMP_OVERLOAD_PROTECT == 0)
				LED_RUN = RESET;
			else
				LED_RUN = SET;	
			if(FAN_OVERLOAD_PROTECT == 0)
				LED_ALARM = RESET;
			else
				LED_ALARM = SET;	


			if(COMP_OVERLOAD_PROTECT == 0)
				HC164_display_arry[0] = seven_seg_code[1];
			else
				HC164_display_arry[0] = seven_seg_code[2];
			
			if(RESERVE_2 == 0)
				HC164_display_arry[1] = seven_seg_code[1];
			else
				HC164_display_arry[1] = seven_seg_code[2];			
			if(RESERVE_1 == 0)
				HC164_display_arry[2] = seven_seg_code[1];
			else
				HC164_display_arry[2] = seven_seg_code[2];
			
			if(REMOTE_ON_OFF_SW == 0)
				HC164_display_arry[3] = seven_seg_code[1];
			else
				HC164_display_arry[3] = seven_seg_code[2];

			break;
#if 1		
			/*************************************************************************/ 
			case _PRODUCTION_mode_6:
				
				LED_1_dot = RESET;
				LED_2_dot = RESET;
				LED_3_dot = RESET;
				LED_4_dot = RESET;
				HC164_display_arry[0] = seven_seg_code[16];
				HC164_display_arry[1] = seven_seg_code[16]; 	// display space
				HC164_display_arry[2] = seven_seg_code[16];
				HC164_display_arry[3] = seven_seg_code[16];
	
				
				break;
#endif						
			/*************************************************************************/ 

		
	}

}


