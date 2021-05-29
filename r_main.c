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
* File Name    : r_main.c
* Version      : CodeGenerator for RL78/G13 V2.05.04.02 [20 Nov 2019]
* Device(s)    : R5F100LE
* Tool-Chain   : CCRL
* Description  : This file implements main function.
* Creation Date: 2021/5/2
***********************************************************************************************************************/

/***********************************************************************************************************************
Includes
***********************************************************************************************************************/
#include "r_cg_macrodriver.h"
#include "r_cg_cgc.h"
#include "r_cg_port.h"
#include "r_cg_serial.h"
#include "r_cg_adc.h"
#include "r_cg_timer.h"
#include "r_cg_wdt.h"
#include "r_cg_it.h"
/* Start user code for include. Do not edit comment generated here */

// #include "flash.h"
// #include "R_eeprom.h"
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
uint8_t	SET_HUMIDITY = 60,POWER_TIMER_ON,POWER_TIMER_OFF,error_code_backup_count;
int16_t	SET_TEMP = 250;

int8_t	switch_set = 0xff;
uint8_t	switch_set_1 = 0xfe;

int8_t	current_detect_data;
uint8_t	error_code_backup = 0,option_backup = 0x00;

extern 	int8_t	Production_mode_count;
extern	uint8_t	LED_mode_count,key_repeart_timer;



uint16_t	get_V_A_timer,V_A_count,voltage_data_now,voltage_data_max,voltage_data_m1,voltage_data_min
				,current_data_now,current_data_max,current_data_m1,current_data_min,current_key_now;


//flag_t	_sys_op_statue_,_Error_state_;
flag_t	_dip_switch_data_;


/**********************************************************************************/
void	ADC_conver(void);
void	powerin_test_disp(void);
void 	power_on_timer_reset(void);
void 	hdwinit(void);
void	init_eeprom_option_buffer(void);
void	write_value_to_eeprom(void);
void	turn_system_on(void);
void	Check_update_status_IIC(void);
void 	IIC_check_init(void);
void	check_set_temp_setting(void);
void	check_dip_switch_setting(void);
void	R_UART0_receive_start(void);
void	R_UART1_receive_start(void);
void	R_UART2_receive_start(void);
void 	convert_AD_channel_voltage(void);
void 	convert_AD_channel_current(void);
void 	R_WDT_Restart(void);
void 	check_and_update_IIC_error_record(void);
void 	check_and_update_option(void);
void 	IIC_save_option(void);
void	IIC_save_error_code(void);
void 	PMV_close(void);

/* End user code. Do not edit comment generated here */
void R_MAIN_UserInit(void);

/***********************************************************************************************************************
* Function Name: main
* Description  : This function implements main function.
* Arguments    : None
* Return Value : None
***********************************************************************************************************************/
void main(void)
{
    R_MAIN_UserInit();
    /* Start user code. Do not edit comment generated here */

	    ms_counter = 0;
//		if(ERROR_CODE == 0) 
//		LED_mode_count = _LED_mode_9;
		production_mode = SET;
		Production_mode_count = _PRODUCTION_mode_6;
		R_IT_Start();
		R_TAU0_Channel0_Start();  
		R_UART0_receive_start();
		R_UART1_receive_start();
		R_UART2_receive_start();
	
	
		
#if 1	
		while(ms_counter < 500)
		{
			ADC_conver();
		}
#endif	
		powerin_test_disp();
		power_on_timer_reset();

    /* PMV position close */
		PMV_close();

	// pwm_set= PWM_CYCLE_MIN;

	
	IIC_check_init();

//	LED_mode_count = _LED_mode_9;
//	key_repeart_timer = key_timer_default - 10;
	
	check_dip_switch_setting();
	check_set_temp_setting();
	// SET_TEMP = 250;					// testtt for test only
#if 0	
	if(POWER_RETURN_AUTO_ON == RESET)
	{
		if( IIC_operation_status[op_state_power] == 0xff )
		{
			turn_system_on();
		}
	}
#endif

	get_V_A_timer = 0;
	voltage_data_max = 0;
	voltage_data_m1 = 0;
	voltage_data_min = 800;
	current_data_max = default_0A;
	current_data_m1 = default_0A;
	current_data_min = default_0A;

	LED_mode_count = _LED_mode_1;


	
    while (1U)
    {
		ADC_conver();

/* **************************************************************** */
		if(get_V_A_timer < 40)				// check 20 ms sec for each 0.5 sec
		{
			convert_AD_channel_voltage();
			if(voltage_data_now > voltage_data_m1)
				voltage_data_m1 = voltage_data_now;
			if(voltage_data_now < voltage_data_min)
				voltage_data_min = voltage_data_now;
			V_A_count++;
		}
		else	if(get_V_A_timer < 80)
				{
					convert_AD_channel_current();
					if(current_data_now > current_data_m1)
						current_data_m1 = current_data_now;
					if(current_data_now < current_data_min)
						current_data_min = current_data_now;
					V_A_count++;
				}
				else
				{
					if(get_V_A_timer > 500)
					{
						get_V_A_timer = 0;
						V_A_count =0;
						voltage_data_max = voltage_data_m1;
						voltage_data_m1 = 0;
						voltage_data_min = 800;
						current_data_max = current_data_m1;
						current_data_m1 = default_0A;
						current_data_min = default_0A;
					}	
				}
			
/* **************************************************************** */
		R_WDT_Restart();
/* **************************************************************** */
				
		Check_update_status_IIC();
//		check_set_temp_setting();

		check_and_update_IIC_error_record();
		check_and_update_option();
		
#if debug_e2prom_off
	//	write_state_to_eeprom();
	//	write_option_to_eeprom();
	//	write_value_to_eeprom();
#endif	

    }
    /* End user code. Do not edit comment generated here */
}

/***********************************************************************************************************************
* Function Name: R_MAIN_UserInit
* Description  : This function adds user code before implementing main function.
* Arguments    : None
* Return Value : None
***********************************************************************************************************************/
void R_MAIN_UserInit(void)
{
    /* Start user code. Do not edit comment generated here */
	hdwinit();
	// _sys_op_statue = RESET;
	// _Error_state = RESET;
	//  Dataflash_Open(SYSTEM_FREQ, DF_PROG_FULL_SPEED_MODE);
//	powerin_read_dip_switch();
#if 0
	e2prom_1test_byte();
	powerin_read_e2prom();
#endif	
    EI();
    /* End user code. Do not edit comment generated here */
}

/* Start user code for adding. Do not edit comment generated here */

void	check_dip_switch_setting(void)
{
	switch_set = 00;
	if(SWDIP4_3)
	{
		REMOTE_SW_FUNCTION = SET;
		switch_set = switch_set | 0x04;
	}
	else
	{
		REMOTE_SW_FUNCTION = RESET;
	}
	
	if(SWDIP4_4)
	{
		SWDIP4_2_SET = SET;
	}
	else
	{
		SWDIP4_2_SET = RESET;
	}
	
	if(SWDIP4_1)
	{
		SYSTEM_SELECT_SWITCH = SET;
		switch_set = switch_set | 0x01;
	}
	else
	{
		SYSTEM_SELECT_SWITCH = RESET;
	}
	
	if(SWDIP4_2)
	{
		TEMP_SELECT_SWITCH = SET;
		switch_set = switch_set | 0x02;
	}
	else
	{
		TEMP_SELECT_SWITCH = RESET;
	}

	
	if(SWDIP2_1)
	{
		if(SWDIP2_2)
		{
			current_detect_data = eeprom_option_byte[_comp_current_protect_0_value];
			switch_set = switch_set | 0x30;
		}	
		else
		{
			current_detect_data = eeprom_option_byte[_comp_current_protect_1_value];
			switch_set = switch_set | 0x20;
		}	
	}	
	else
	{
		if(SWDIP2_2)
		{
			current_detect_data = eeprom_option_byte[_comp_current_protect_2_value];
			switch_set = switch_set | 0x10;
		}	
		else
		{
			current_detect_data = eeprom_option_byte[_comp_current_protect_3_value];
		}
	}	

}




void	check_set_temp_setting(void)
{
	if(TEMP_SELECT_SWITCH == FIX_TYPE_SENSOR)
	{
		if(SET_TEMP > eeprom_option_byte[_Set_temp_high] * 10)
			SET_TEMP = eeprom_option_byte[_Set_temp_high] * 10;
		else if(SET_TEMP < eeprom_option_byte[_Set_temp_low] *10)
			SET_TEMP = eeprom_option_byte[_Set_temp_low] *10;
		/*************************************/
	}
	else
	{
		if(SET_TEMP > eeprom_option_byte[_Set_temp_high_1])
			SET_TEMP = eeprom_option_byte[_Set_temp_high_1];
		else if(SET_TEMP < eeprom_option_byte[_Set_temp_low_1])
			SET_TEMP = eeprom_option_byte[_Set_temp_low_1];
		/*************************************/		
	}

}

/***********************************************************************************************************************
* Function Name: check_and_update_IIC_TH1_and_TH3
* Description  : This function adds user code before implementing main function.
* Arguments    : None
* Return Value : None
***********************************************************************************************************************/
void check_and_update_option(void)
{
	if((option_backup == 0Xff) &&(LED_mode_count == _LED_mode_1))
	{
		option_backup = 0x00;
		IIC_save_option();
	}
}



/***********************************************************************************************************************
* Function Name: check_and_update_IIC_error_record
* Description  : This function adds user code before implementing main function.
* Arguments    : None
* Return Value : None
***********************************************************************************************************************/
#define Error_code_end 9

void check_and_update_IIC_error_record(void)
{
	uint8_t temp_buf,temp_loop;
	
	if(error_code_backup == 0Xff)
	{
		if(error_code_backup_count >= ERROR_BACKUP_SET)
		{
			error_code_backup_count = 0;
			error_code_backup = 0x00;
			if(ERROR_CODE != 0)
			{
				temp_buf = IIC_error_code[op_error_record_index];
				if(temp_buf !=0)
				{
					if(temp_buf == Error_code_end)
					{
						for(temp_loop=0;temp_loop < Error_code_end;temp_loop++)
						{
							IIC_error_code[temp_loop+1] = IIC_error_code[temp_loop+2];
						}
						IIC_error_code[temp_buf] = ERROR_CODE;

					}
					else
					{
						IIC_error_code[op_error_record_index]++;
						temp_buf = IIC_error_code[op_error_record_index];
						IIC_error_code[temp_buf] = ERROR_CODE;
					}
				}
				else
				{
					if(ERROR_CODE != 0)
					{
						IIC_error_code[op_error_record_index]++;
						temp_buf = IIC_error_code[op_error_record_index];
						IIC_error_code[temp_buf] = ERROR_CODE;
					}
				}
#if 1				
				IIC_save_error_code();      // this is for test 
#endif
			}
		}
	}	
	else
	{
		error_code_backup_count = 0;
	}

}











/* End user code. Do not edit comment generated here */
