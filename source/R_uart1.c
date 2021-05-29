#include "r_cg_macrodriver.h"
#include "r_cg_userdefine.h"
#include "r_cg_serial.h"
#include "R_adc.h"	
/* ***************************************************************************************************************** */


uint8_t g_Uart1RxBuf;		/* UART1 receive buffer */
uint8_t g_Uart1RxErr;		/* UART1 receive error status */
MD_STATUS g_Uart1TxEnd;		/* UART1 transmission end */
uint8_t	UART1_RxBuff[15],UART1_TxBuff[80],UART1_RxBuff_conter,UART1_action_buff[15];
uint8_t	UART1_Data_recieve_timeout_timer,UART1_transmit_delay_timer;
uint8_t	check_UART1_Rx_data;
uint16_t	g_uart1_rx_cont_backup,UART1_timer_send;
uint16_t	Test_timer = 0;
extern volatile uint16_t  g_uart1_rx_count;            /* uart1 receive data number */

extern 	uint8_t		IPDU_frequency,real_IPDU_frequency,IPDU_SW_version;
extern	uint16_t	T_DECP[AD_total_chanel];
extern	int16_t		PID_P,PID_I,PID_D,PID_M,PID,PMV_temp_delta;






/***********************************************************************************************************************
Global variables and functions
***********************************************************************************************************************/
flag_t	_UART1_control_;
#define	UART1_control_			_UART1_control_.flag
#define	UART1_receive_done		_UART1_control_.bit.no0
#define	UART1_transmit_done		_UART1_control_.bit.no1
#define	UART1_transmit_delay	_UART1_control_.bit.no2
#define	UART1_rec_action		_UART1_control_.bit.no3

void	turn_system_off(void);
void	turn_system_on(void);
void	UART1_recieve_action(void);


/* ***************************************************************************************************************** */

uint8_t	UART1_crcchk_data(void)
{
	return	(uint8_t)TRUE;
	
}

/* ***************************************************************************************************************** */


void	convert_UART1_Data(void)
{
//	uint8_t	sum,index;
//	MD_STATUS	UART1_status;
	
	
}
/* ***************************************************************************************************************** */

void	R_UART1_receive_start(void)
{
	//MD_STATUS	UART1_status;
	uint8_t	clr;
	for(clr = 0; clr < sizeof(UART1_RxBuff)-1; clr++)
	{
		UART1_RxBuff[clr] = 0x00;
	}

	R_UART1_Receive(&UART1_RxBuff, 5);  // old 6

	R_UART1_Start();
}
/* ***************************************************************************************************************** */

void	R_UART1_receive_end(void)
{
	//MD_STATUS	UART1_status;
#if 1	
	//if(UART1_crcchk_data())
	{
		UART1_receive_done = SET;
		UART1_transmit_delay = SET;
		UART1_transmit_delay_timer = 0;
		UART1_recieve_action();
	}
	//else
#endif		
		R_UART1_receive_start();
}
/* ***************************************************************************************************************** */

void	UART1_recieve_action(void)
{
	/* ***************  check T  0x54 ************************************* */
	if(UART1_RxBuff[0] == 0x54)
	{
		if(UART1_RxBuff[1] == 0x30)
		{
			CONTROL_TEMP_TEST = (UART1_RxBuff[2] - 0x30) * 10;
			CONTROL_TEMP_TEST = (CONTROL_TEMP_TEST + (UART1_RxBuff[3] - 0x30)) *10;
			CONTROL_TEMP_TEST += (UART1_RxBuff[4] - 0x30);
		}


	}

}
/* ***************************************************************************************************************** */

void	UART1_control(void)//called in main
{

}
/* ***************************************************************************************************************** */
void	UART1_send_loop(void)
{
	
	uint8_t index;
	int16_t disp_room_temp;
		
	index = 0;	
		/* ***************************************************************************************************************** */	
	UART1_TxBuff[index++] = 0x20;    // " "
	UART1_TxBuff[index++] = 0x43;	// "C"
	UART1_TxBuff[index++] = 0x3D;	// "="
	/* ***************************************************************************************************************** */ 
	{
			disp_room_temp = current_data_temp / 2;
			UART1_TxBuff[index++] = disp_room_temp/1000 + 0x30;    
			UART1_TxBuff[index++] = (disp_room_temp%1000)/100 + 0x30;
			UART1_TxBuff[index++] = (disp_room_temp%100)/10 + 0x30;
			UART1_TxBuff[index++] = (disp_room_temp%10) + 0x30;
			
	}
	/* ***************************************************************************************************************** */	
	UART1_TxBuff[index++] = 0x20;    // " "
	UART1_TxBuff[index++] = 0x56;	// "V"
	UART1_TxBuff[index++] = 0x3D;	// "="
	/* ***************************************************************************************************************** */ 
	{
				
			disp_room_temp = voltage_data_max;

			if(PUMP_RLY == SET)
				disp_room_temp = disp_room_temp + voltage_small_relay_offset;
			if(FAN_RLY == SET)
				disp_room_temp = disp_room_temp + voltage_small_relay_offset;
			if(ERROR_RLY == SET)
				disp_room_temp = disp_room_temp + voltage_small_relay_offset;
			if(HEATER_RLY == SET)
				disp_room_temp = disp_room_temp + voltage_small_relay_offset;
			if(COMP_RLY == SET)
				disp_room_temp = disp_room_temp + voltage_large_relay_offset;

			disp_room_temp = disp_room_temp - default_220v;
			
			disp_room_temp = disp_room_temp * 10;
			disp_room_temp = disp_room_temp / 34;
			
			disp_room_temp = disp_room_temp + 220;


			UART1_TxBuff[index++] = disp_room_temp/1000 + 0x30;    
			UART1_TxBuff[index++] = (disp_room_temp%1000)/100 + 0x30;
			UART1_TxBuff[index++] = (disp_room_temp%100)/10 + 0x30;
			UART1_TxBuff[index++] = (disp_room_temp%10) + 0x30;
			
	}
	/* ***************************************************************************************************************** */ 
	UART1_TxBuff[index++] = 0x20;    // "!"
	UART1_TxBuff[index++] = 0x20;    // "!"
	UART1_TxBuff[index++] = 0x20;    // "!"

	/*****************	 system on operation  ****************/
	UART1_TxBuff[index++] = 0x53;	// "S"
	UART1_TxBuff[index++] = 0x59;	// "Y"
	UART1_TxBuff[index++] = 0x53;	// "S"
	UART1_TxBuff[index++] = 0x2D;	// "-"

	if(current_sys_op_power)     
	{
		UART1_TxBuff[index++] = 0x4F;    // "O"
		UART1_TxBuff[index++] = 0x4E;    // "N"
		UART1_TxBuff[index++] = 0x20;    // " "
	}
	else     /*****************   system off operation  ****************/
	{	
		UART1_TxBuff[index++] = 0x4F;    // "O"
		UART1_TxBuff[index++] = 0x46;    // "F"
		UART1_TxBuff[index++] = 0x46;    // "F"
	}
	/* ***************************************************************************************************************** */ 
	disp_room_temp = SW_version;
		
	UART1_TxBuff[index++] = 0x20;	// " "
	UART1_TxBuff[index++] = 0x56;	// "V"
	UART1_TxBuff[index++] = 0x2D;	// "-"
	UART1_TxBuff[index++] = disp_room_temp/10 + 0x30;
	UART1_TxBuff[index++] = (disp_room_temp%10) + 0x30;
#if PMV_TEST
	UART1_TxBuff[index++] = 0x2D;	// "-"
	UART1_TxBuff[index++] = 0x54;	// "T"
#endif
	UART1_TxBuff[index++] = 0x20;	// " "


	R_UART1_Send(UART1_TxBuff, index);
	
		
	
}

/* ***************************************************************************************************************** */
void	UART1_send_temp(void)
{
		uint8_t	index;
		int16_t	disp_room_temp;

	index = 0;	
	UART1_TxBuff[index++] = 0x20;    // "*"
	UART1_TxBuff[index++] = 0x20;    // "T"
	if(CONTROL_TEMP < 0)
	{
		UART1_TxBuff[index++] = 0x2D;    // "-"
#if PMV_TEST
		disp_room_temp = ~CONTROL_TEMP_TEST;
#else
		disp_room_temp = ~CONTROL_TEMP;
#endif
		disp_room_temp++;
		UART1_TxBuff[index++] = disp_room_temp/100 + 0x30;
		UART1_TxBuff[index++] = (disp_room_temp%100)/10 + 0x30;
		UART1_TxBuff[index++] = (disp_room_temp%10) + 0x30;
	}
	else
	{
#if PMV_TEST
		disp_room_temp = CONTROL_TEMP_TEST;
#else
		disp_room_temp = CONTROL_TEMP;
#endif	
		UART1_TxBuff[index++] = disp_room_temp/1000 + 0x30;    
		UART1_TxBuff[index++] = (disp_room_temp%1000)/100 + 0x30;
		UART1_TxBuff[index++] = (disp_room_temp%100)/10 + 0x30;
		UART1_TxBuff[index++] = (disp_room_temp%10) + 0x30;
		
	}
	UART1_TxBuff[index++] = 0x20;    // "*"
	/* ***************************************************************************************************************** */
	if(SET_TEMP < 0)
	{
		UART1_TxBuff[index++] = 0x2D;    // "-"
		disp_room_temp = ~SET_TEMP;
		disp_room_temp++;
		UART1_TxBuff[index++] = disp_room_temp/100 + 0x30;
		UART1_TxBuff[index++] = (disp_room_temp%100)/10 + 0x30;
		UART1_TxBuff[index++] = (disp_room_temp%10) + 0x30;
	}
	else
	{
		disp_room_temp = SET_TEMP;
		UART1_TxBuff[index++] = disp_room_temp/1000 + 0x30;    
		UART1_TxBuff[index++] = (disp_room_temp%1000)/100 + 0x30;
		UART1_TxBuff[index++] = (disp_room_temp%100)/10 + 0x30;
		UART1_TxBuff[index++] = (disp_room_temp%10) + 0x30;
		
	}

	

	UART1_TxBuff[index++] = 0x20;    // "*"
	/* ***************************************************************************************************************** */
	if(PMV_temp_delta < 0)
	{
		UART1_TxBuff[index++] = 0x2D;    // "-"
		disp_room_temp = ~PMV_temp_delta;
		disp_room_temp++;
//		UART1_TxBuff[index++] = disp_room_temp/100 + 0x30;
		UART1_TxBuff[index++] = (disp_room_temp%100)/10 + 0x30;
		UART1_TxBuff[index++] = (disp_room_temp%10) + 0x30;
	}
	else
	{
		disp_room_temp = PMV_temp_delta;
//		UART1_TxBuff[index++] = disp_room_temp/1000 + 0x30;    
//		UART1_TxBuff[index++] = (disp_room_temp%1000)/100 + 0x30;
		UART1_TxBuff[index++] = (disp_room_temp%100)/10 + 0x30;
		UART1_TxBuff[index++] = (disp_room_temp%10) + 0x30;
		
	}






	
	UART1_TxBuff[index++] = 0x20;    // " "
	UART1_TxBuff[index++] = 0x50;    // "P"
	/* ***************************************************************************************************************** */
#if 1
		if(PID_P < 0)
		{
			UART1_TxBuff[index++] = 0x2D;	 // "-"
			disp_room_temp = ~PID_P;
			disp_room_temp++;
			UART1_TxBuff[index++] = disp_room_temp/100 + 0x30;
			UART1_TxBuff[index++] = (disp_room_temp%100)/10 + 0x30;
			UART1_TxBuff[index++] = (disp_room_temp%10) + 0x30;
		}
		else
		{
			disp_room_temp = PID_P;
			UART1_TxBuff[index++] = disp_room_temp/1000 + 0x30;    
			UART1_TxBuff[index++] = (disp_room_temp%1000)/100 + 0x30;
			UART1_TxBuff[index++] = (disp_room_temp%100)/10 + 0x30;
			UART1_TxBuff[index++] = (disp_room_temp%10) + 0x30;
			
		}
	UART1_TxBuff[index++] = 0x20;    // " "	
	UART1_TxBuff[index++] = 0x49;    // "I"
	/* ***************************************************************************************************************** */	
	{
		if(PID_I < 0)
		{
			UART1_TxBuff[index++] = 0x2D;	 // "-"
			disp_room_temp = ~PID_I;
			disp_room_temp++;
			UART1_TxBuff[index++] = disp_room_temp/100 + 0x30;
			UART1_TxBuff[index++] = (disp_room_temp%100)/10 + 0x30;
			UART1_TxBuff[index++] = (disp_room_temp%10) + 0x30;
		}
		else
		{
			disp_room_temp = PID_I;
			UART1_TxBuff[index++] = disp_room_temp/1000 + 0x30;    
			UART1_TxBuff[index++] = (disp_room_temp%1000)/100 + 0x30;
			UART1_TxBuff[index++] = (disp_room_temp%100)/10 + 0x30;
			UART1_TxBuff[index++] = (disp_room_temp%10) + 0x30;
			
		}
	}
	/* ***************************************************************************************************************** */
	UART1_TxBuff[index++] = 0x20;    // " "
	UART1_TxBuff[index++] = 0x44;	// "D"
	/* ***************************************************************************************************************** */ 
	{
		if(PID_D < 0)
		{
			UART1_TxBuff[index++] = 0x2D;	 // "-"
			disp_room_temp = ~PID_D;
			disp_room_temp++;
			UART1_TxBuff[index++] = disp_room_temp/100 + 0x30;
			UART1_TxBuff[index++] = (disp_room_temp%100)/10 + 0x30;
			UART1_TxBuff[index++] = (disp_room_temp%10) + 0x30;
		}
		else
		{
			disp_room_temp = PID_D;
			UART1_TxBuff[index++] = disp_room_temp/1000 + 0x30;    
			UART1_TxBuff[index++] = (disp_room_temp%1000)/100 + 0x30;
			UART1_TxBuff[index++] = (disp_room_temp%100)/10 + 0x30;
			UART1_TxBuff[index++] = (disp_room_temp%10) + 0x30;
			
		}

	}

	/* ***************************************************************************************************************** */ 
	/* ***************************************************************************************************************** */	
	UART1_TxBuff[index++] = 0x20;    // " "
	UART1_TxBuff[index++] = 0x20;	// "*"
	/* ***************************************************************************************************************** */ 
	{
		if(PID < 0)
		{
			UART1_TxBuff[index++] = 0x2D;	 // "-"
			disp_room_temp = ~PID;
			disp_room_temp++;
			UART1_TxBuff[index++] = disp_room_temp/100 + 0x30;
			UART1_TxBuff[index++] = (disp_room_temp%100)/10 + 0x30;
			UART1_TxBuff[index++] = (disp_room_temp%10) + 0x30;
		}
		else
		{
			disp_room_temp = PID;
			UART1_TxBuff[index++] = disp_room_temp/1000 + 0x30;    
			UART1_TxBuff[index++] = (disp_room_temp%1000)/100 + 0x30;
			UART1_TxBuff[index++] = (disp_room_temp%100)/10 + 0x30;
			UART1_TxBuff[index++] = (disp_room_temp%10) + 0x30;
			
		}

	}
	
	/* ***************************************************************************************************************** */	
	UART1_TxBuff[index++] = 0x20;    // " "
	UART1_TxBuff[index++] = 0x4D;	// "M"
	UART1_TxBuff[index++] = 0x3D;	// "="
	/* ***************************************************************************************************************** */ 
	{
		if(PID_M < 0)
		{
			UART1_TxBuff[index++] = 0x2D;	 // "-"
			disp_room_temp = ~PID_M;
			disp_room_temp++;
			UART1_TxBuff[index++] = disp_room_temp/100 + 0x30;
			UART1_TxBuff[index++] = (disp_room_temp%100)/10 + 0x30;
			UART1_TxBuff[index++] = (disp_room_temp%10) + 0x30;
		}
		else
		{
			disp_room_temp = PID_M;
			UART1_TxBuff[index++] = disp_room_temp/1000 + 0x30;    
			UART1_TxBuff[index++] = (disp_room_temp%1000)/100 + 0x30;
			UART1_TxBuff[index++] = (disp_room_temp%100)/10 + 0x30;
			UART1_TxBuff[index++] = (disp_room_temp%10) + 0x30;
			
		}

	}
	/* ***************************************************************************************************************** */ 
	/* ***************************************************************************************************************** */	
	UART1_TxBuff[index++] = 0x20;    // " "
	UART1_TxBuff[index++] = 0x43;	// "C"
	UART1_TxBuff[index++] = 0x3D;	// "="
	/* ***************************************************************************************************************** */ 
	{
			disp_room_temp = current_data_temp;
			UART1_TxBuff[index++] = disp_room_temp/1000 + 0x30;    
			UART1_TxBuff[index++] = (disp_room_temp%1000)/100 + 0x30;
			UART1_TxBuff[index++] = (disp_room_temp%100)/10 + 0x30;
			UART1_TxBuff[index++] = (disp_room_temp%10) + 0x30;
			
	}
	/* ***************************************************************************************************************** */ 	
#endif		

	/* ***************************************************************************************************************** */ 
	UART1_TxBuff[index++] = 0x20;    // "!"
	UART1_TxBuff[index++] = 0x20;    // "!"
	UART1_TxBuff[index++] = 0x20;    // "!"

	/*****************	 system on operation  ****************/
	UART1_TxBuff[index++] = 0x53;	// "S"
	UART1_TxBuff[index++] = 0x59;	// "Y"
	UART1_TxBuff[index++] = 0x53;	// "S"
	UART1_TxBuff[index++] = 0x2D;	// "-"

	if(current_sys_op_power)     
	{
		UART1_TxBuff[index++] = 0x4F;    // "O"
		UART1_TxBuff[index++] = 0x4E;    // "N"
		UART1_TxBuff[index++] = 0x20;    // " "
	}
	else     /*****************   system off operation  ****************/
	{	
		UART1_TxBuff[index++] = 0x4F;    // "O"
		UART1_TxBuff[index++] = 0x46;    // "F"
		UART1_TxBuff[index++] = 0x46;    // "F"
	}
	/* ***************************************************************************************************************** */ 
	


	UART1_TxBuff[index++] = 0x20;    // " "
	disp_room_temp = Test_timer;
	UART1_TxBuff[index++] = disp_room_temp/1000 + 0x30;	  
	UART1_TxBuff[index++] = (disp_room_temp%1000)/100 + 0x30;
	UART1_TxBuff[index++] = (disp_room_temp%100)/10 + 0x30;
	UART1_TxBuff[index++] = (disp_room_temp%10) + 0x30;

	UART1_TxBuff[index++] = 0x20;	// " "
	disp_room_temp = motor_current_position_1;
	UART1_TxBuff[index++] = disp_room_temp/1000 + 0x30;	  
	UART1_TxBuff[index++] = (disp_room_temp%1000)/100 + 0x30;
	UART1_TxBuff[index++] = (disp_room_temp%100)/10 + 0x30;
	UART1_TxBuff[index++] = (disp_room_temp%10) + 0x30;


	
	/* ***************************************************************************************************************** */ 

	disp_room_temp = SW_version;
		
		UART1_TxBuff[index++] = 0x20;	// " "
		UART1_TxBuff[index++] = 0x56;	// "V"
		UART1_TxBuff[index++] = 0x2D;	// "-"
		UART1_TxBuff[index++] = disp_room_temp/10 + 0x30;
		UART1_TxBuff[index++] = (disp_room_temp%10) + 0x30;
#if PMV_TEST
		UART1_TxBuff[index++] = 0x2D;	// "-"
		UART1_TxBuff[index++] = 0x54;	// "T"
#endif
		UART1_TxBuff[index++] = 0x20;	// " "


	R_UART1_Send(UART1_TxBuff, index);
	
		

}

/* ***************************************************************************************************************** */

void	UART1_recieve_timeout(void)//called in RTC
{
	if(g_uart1_rx_count !=0)
	{
		if(g_uart1_rx_count == g_uart1_rx_cont_backup)
		{
			UART1_Data_recieve_timeout_timer++;
			if(UART1_Data_recieve_timeout_timer > 30) // 30ms
			{
				UART1_Data_recieve_timeout_timer = 0;
				R_UART1_receive_end();
				g_uart1_rx_count =0;
			}
		}
		else
		{
			g_uart1_rx_cont_backup = g_uart1_rx_count;
			UART1_Data_recieve_timeout_timer = 0;
		}
	}
	else
	{
		g_uart1_rx_cont_backup = g_uart1_rx_count;
		UART1_Data_recieve_timeout_timer = 0;

	}

	/* ********************************************************** */
	if(UART1_timer_send++ > 1000)
	{
		Test_timer++;
#if 1			
		UART1_send_loop();
#endif
		UART1_timer_send = 0;
	}

	
}

