#include "r_cg_macrodriver.h"
#include "r_cg_userdefine.h"
#include "r_cg_serial.h"

/* ***************************************************************************************************************** */


uint8_t g_Uart2RxBuf;		/* UART2 receive buffer */
uint8_t g_Uart2RxErr;		/* UART2 receive error status */
MD_STATUS g_Uart2TxEnd;		/* UART2 transmission end */

uint8_t	UART2_RxBuff[15],UART2_TxBuff[100],UART2_RxBuff_conter,UART2_action_buff[15];
uint8_t	UART2_Data_recieve_timeout_timer,UART2_transmit_delay_timer;
uint8_t	check_uart2_Rx_data;
uint8_t ack_command,sum,UART2_TX_buffer_pointer;

uint16_t	g_uart2_rx_cont_backup,UART2_timer_send,CYCLE_no;

extern volatile uint16_t  g_uart2_rx_count;            /* uart2 receive data number */







/***********************************************************************************************************************
Global variables and functions
***********************************************************************************************************************/
flag_t	_UART2_control_;
#define	UART2_control_			_UART2_control_.flag
#define	UART2_receive_done		_UART2_control_.bit.no0
#define	UART2_transmit_done		_UART2_control_.bit.no1
#define	UART2_transmit_delay	_UART2_control_.bit.no2
#define	UART2_rec_action		_UART2_control_.bit.no3
/* ***************************************************************************************************************** */
void	UART2_send_ack_data(void);
void	turn_system_off(void);
void	turn_system_on(void);

/* ***************************************************************************************************************** */
/* ***************************************************************************************************************** */
/* ***************************************************************************************************************** */
/* ***************************************************************************************************************** */


uint8_t	UART2_crcchk_data(void)
{
	uint8_t	crc_loop;

	/* *=0x2A, !=0x21, 0=0x30, A=0x41  */
	
	
	if((UART2_RxBuff[0]==0x2A)&&(UART2_RxBuff[g_uart2_rx_count-1]==0x21))
	{
		if((UART2_RxBuff[g_uart2_rx_count-3] >= 0x30) && (UART2_RxBuff[g_uart2_rx_count-3] <= 0x39))
		{
			sum = UART2_RxBuff[g_uart2_rx_count-3] - 0x30;
			sum = sum << 4;
		}
		else 	if((UART2_RxBuff[g_uart2_rx_count-3] >= 0x41) && (UART2_RxBuff[g_uart2_rx_count-3] <= 0x46))
				{
					sum = UART2_RxBuff[g_uart2_rx_count-3] - 0x41 + 10;
					sum = sum << 4;
				}
				else
					return	(uint8_t)FALSE;
				
		if((UART2_RxBuff[g_uart2_rx_count-2] >= 0x30) && (UART2_RxBuff[g_uart2_rx_count-2] <= 0x39))
		{
			sum = sum + UART2_RxBuff[g_uart2_rx_count-2] - 0x30;
		}
		else 	if((UART2_RxBuff[g_uart2_rx_count-2] >= 0x41) && (UART2_RxBuff[g_uart2_rx_count-2] <= 0x46))
				{
					sum = sum + UART2_RxBuff[g_uart2_rx_count-2] - 0x41 + 10;
				}
				else
					return	(uint8_t)FALSE;		
	}
	else
		return	(uint8_t)FALSE;

	for(crc_loop=1;crc_loop < g_uart2_rx_count-3;crc_loop++ )
	{
		sum += UART2_RxBuff[crc_loop];
	}
if(sum ==0xFF)
		return	(uint8_t)TRUE;
	else
		return	(uint8_t)FALSE;
}

/* ***************************************************************************************************************** */

void	UART2_recieve_action(void)
{
	ack_command = 2;
	/* ***************  check RUN 0x52 0x55  0x4E  ************************************* */
	if((UART2_RxBuff[3] == 0x52) && (UART2_RxBuff[4] == 0x55))
	{
		if(UART2_RxBuff[5] == 0x4E)
		{
			turn_system_on();
			ack_command = 0;
			UART2_send_ack_data();
		}
		
	}
	
	/* ***************  check STOP  0x53 0x54 0x4F 0x50 ************************************* */
	if((UART2_RxBuff[3] == 0x53) && (UART2_RxBuff[4] == 0x54))
	{
		if((UART2_RxBuff[5] == 0x4F) && (UART2_RxBuff[6] == 0x50))
		{
			turn_system_off();
			ack_command = 0;
			UART2_send_ack_data();
		}
		
	}
	/* ***************  check T  0x54 ************************************* */
	if(UART2_RxBuff[3] == 0x54)
	{
		if(UART2_RxBuff[4] == 0x30)
		{
			SET_TEMP = (UART2_RxBuff[5] - 0x30) * 10;
			SET_TEMP = (SET_TEMP + (UART2_RxBuff[6] - 0x30)) *10;
			SET_TEMP += (UART2_RxBuff[7] - 0x30);
		}
		else
		{
			SET_TEMP = (UART2_RxBuff[5] - 0x30) * 10;
			SET_TEMP = (SET_TEMP + (UART2_RxBuff[6] - 0x30)) *10;
			SET_TEMP += (UART2_RxBuff[7] - 0x30);
			SET_TEMP = ~SET_TEMP;
			SET_TEMP++;
		}
		//SET_TEMP = SET_TEMP * 10;
		ack_command = 0;
		UART2_send_ack_data();
	}
	/* ***************  check S  0x53 ************************************* */
	if(UART2_RxBuff[3] == 0x53)
	{
		
		if(UART2_RxBuff[4] == 0x30)
		{
			CYCLE_no = 0;
			CYCLE_no = (UART2_RxBuff[5] - 0x30) * 1000;
			CYCLE_no = CYCLE_no + ((UART2_RxBuff[6] - 0x30) *100);
			CYCLE_no = CYCLE_no + ((UART2_RxBuff[7] - 0x30) *10);
			CYCLE_no = CYCLE_no + (UART2_RxBuff[8] - 0x30);
			ack_command = 0;
			UART2_send_ack_data();
		}
		else	if(UART2_RxBuff[4] == 0x31)
				{
					CYCLE_no = 10000;
					CYCLE_no = CYCLE_no + (UART2_RxBuff[5] - 0x30) * 1000;
					CYCLE_no = CYCLE_no + ((UART2_RxBuff[6] - 0x30) *100);
					CYCLE_no = CYCLE_no + ((UART2_RxBuff[7] - 0x30) *10);
					CYCLE_no = CYCLE_no + (UART2_RxBuff[8] - 0x30);
					ack_command = 0;
					UART2_send_ack_data();
				}
	}	
	/* ***************  check FW  0x46 0x57 ************************************* */
	if((UART2_RxBuff[3] == 0x46) && (UART2_RxBuff[4] == 0x57))
	{
		ack_command = 0;
		UART2_send_ack_data();
	}

	if(ack_command == 2)
	{
		UART2_send_ack_data();        // send command error ack AE
	}

	//R_UART2_Send(UART2_TxBuff, 5);

}

/* ***************************************************************************************************************** */

void	UART2_send_ack_data(void)
{
	uint8_t	sum,index;
	//MD_STATUS	UART2_status;
	UART2_TxBuff[0] = 0x2A;    // "*"
	switch(ack_command)
	{
		default:
		case 0:   // command AY
			UART2_TxBuff[1] = 0x41;
			UART2_TxBuff[2] = 0x59;
			break;
		case 1:   // command AN
			UART2_TxBuff[1] = 0x41;
			UART2_TxBuff[2] = 0x4E;
			break;	
		case 2:   // command AE
			UART2_TxBuff[1] = 0x41;
			UART2_TxBuff[2] = 0x45;
			break;	
	}

	
	UART2_TxBuff[3] = UART2_RxBuff[1];
	UART2_TxBuff[4] = UART2_RxBuff[2];
	sum = 0;
	for(index=1;index<=4;index++)
	{
		sum += UART2_TxBuff[index];
	}
	sum = 0 - sum;
	sum = sum - 1;
	
	if((sum >> 4) > 0x09)
	{
		UART2_TxBuff[5] = (sum >> 4) + 0x41 -10;
	}
	else
	{
		UART2_TxBuff[5] = (sum >> 4) + 0x30;
	}
	if((sum & 0x0F) > 0x09)
	{
		UART2_TxBuff[6] = (sum & 0x0F) + 0x41 -10;
	}
	else
	{
		UART2_TxBuff[6] = (sum & 0x0F) + 0x30;
	}

#if 0
	UART2_TxBuff[0] = 0x2A;    // "*"
	UART2_TxBuff[1] = 0x31;
	UART2_TxBuff[2] = 0x32;
	UART2_TxBuff[3] = 0x33;
	UART2_TxBuff[4] = 0x34;    
	UART2_TxBuff[5] = 0x35;
	UART2_TxBuff[6] = 0x36;
#endif	
	UART2_TxBuff[7] = 0x21;    // "!"

	R_UART2_Send(UART2_TxBuff, 8);
	
		
	



	
	
	
}
/* ***************************************************************************************************************** */

void	R_UART2_receive_start(void)
{
	//MD_STATUS	UART2_status;
	uint8_t	clr;
	for(clr = 0; clr < sizeof(UART2_RxBuff)-1; clr++)
	{
		UART2_RxBuff[clr] = 0x00;
	}

	R_UART2_Receive(&UART2_RxBuff, 12);  // old 6

	R_UART2_Start();
}

/* ***************************************************************************************************************** */

void	R_UART2_receive_end(void)
{
	if(UART2_crcchk_data())
	{
		UART2_receive_done = SET;
		UART2_transmit_delay = SET;
		UART2_transmit_delay_timer = 0;
		UART2_recieve_action();
	}
	else
		R_UART2_receive_start();
}

/* ***************************************************************************************************************** */


void	UART2_control(void)//called in main
{

}
/* ***************************************************************************************************************** */
void	UART2_send_temp(void)
{
		// uint8_t	sum,index;
		int16_t	disp_room_temp;

	UART2_TX_buffer_pointer = 0;
		
	UART2_TxBuff[UART2_TX_buffer_pointer++] = 0x20;    // "*"
	UART2_TxBuff[UART2_TX_buffer_pointer++] = 0x20;    // "T"
	if(CONTROL_TEMP < 0)
	{
		UART2_TxBuff[UART2_TX_buffer_pointer++] = 0x2D;    // "-"
		disp_room_temp = ~CONTROL_TEMP;
		disp_room_temp++;
		UART2_TxBuff[UART2_TX_buffer_pointer++] = disp_room_temp/100 + 0x30;
		UART2_TxBuff[UART2_TX_buffer_pointer++] = (disp_room_temp%100)/10 + 0x30;
		UART2_TxBuff[UART2_TX_buffer_pointer++] = (disp_room_temp%10) + 0x30;
	}
	else
	{
		disp_room_temp = CONTROL_TEMP;
		UART2_TxBuff[UART2_TX_buffer_pointer++] = disp_room_temp/1000 + 0x30;    
		UART2_TxBuff[UART2_TX_buffer_pointer++] = (disp_room_temp%1000)/100 + 0x30;
		UART2_TxBuff[UART2_TX_buffer_pointer++] = (disp_room_temp%100)/10 + 0x30;
		UART2_TxBuff[UART2_TX_buffer_pointer++] = (disp_room_temp%10) + 0x30;
		
	}
	UART2_TxBuff[UART2_TX_buffer_pointer++] = 0x20;    // "*"
	/* ***************************************************************************************************************** */
	if(SET_TEMP < 0)
	{
		UART2_TxBuff[UART2_TX_buffer_pointer++] = 0x2D;    // "-"
		disp_room_temp = ~SET_TEMP;
		disp_room_temp++;
		UART2_TxBuff[UART2_TX_buffer_pointer++] = disp_room_temp/100 + 0x30;
		UART2_TxBuff[UART2_TX_buffer_pointer++] = (disp_room_temp%100)/10 + 0x30;
		UART2_TxBuff[UART2_TX_buffer_pointer++] = (disp_room_temp%10) + 0x30;
	}
	else
	{
		disp_room_temp = SET_TEMP;
		UART2_TxBuff[UART2_TX_buffer_pointer++] = disp_room_temp/1000 + 0x30;    
		UART2_TxBuff[UART2_TX_buffer_pointer++] = (disp_room_temp%1000)/100 + 0x30;
		UART2_TxBuff[UART2_TX_buffer_pointer++] = (disp_room_temp%100)/10 + 0x30;
		UART2_TxBuff[UART2_TX_buffer_pointer++] = (disp_room_temp%10) + 0x30;
		
	}
	UART2_TxBuff[UART2_TX_buffer_pointer++] = 0x20;    // "*"
	/* ***************************************************************************************************************** */
	if(BASE_TEMP < 0)
	{
		UART2_TxBuff[UART2_TX_buffer_pointer++] = 0x2D;    // "-"
		disp_room_temp = ~BASE_TEMP;
		disp_room_temp++;
		UART2_TxBuff[UART2_TX_buffer_pointer++] = disp_room_temp/100 + 0x30;
		UART2_TxBuff[UART2_TX_buffer_pointer++] = (disp_room_temp%100)/10 + 0x30;
		UART2_TxBuff[UART2_TX_buffer_pointer++] = (disp_room_temp%10) + 0x30;
	}
	else
	{
		disp_room_temp = BASE_TEMP;
		UART2_TxBuff[UART2_TX_buffer_pointer++] = disp_room_temp/1000 + 0x30;    
		UART2_TxBuff[UART2_TX_buffer_pointer++] = (disp_room_temp%1000)/100 + 0x30;
		UART2_TxBuff[UART2_TX_buffer_pointer++] = (disp_room_temp%100)/10 + 0x30;
		UART2_TxBuff[UART2_TX_buffer_pointer++] = (disp_room_temp%10) + 0x30;
		
	}
	UART2_TxBuff[UART2_TX_buffer_pointer++] = 0x20;    // "*"	
	/* ***************************************************************************************************************** */
	if(COMP_RLY == 1)
	{
		UART2_TxBuff[UART2_TX_buffer_pointer++] = 0x31;    // "1"
	}
	else     /*****************   system off operation  ****************/
	{	
		UART2_TxBuff[UART2_TX_buffer_pointer++] = 0x30;    // "O"
	}	
	if(FAN_RLY == 1)
	{
		UART2_TxBuff[UART2_TX_buffer_pointer++] = 0x31;    // "1"
	}
	else	 /*****************   system off operation	****************/
	{	
		UART2_TxBuff[UART2_TX_buffer_pointer++] = 0x30;    // "0"
	}	
	if(PUMP_RLY == 1)
	{
		UART2_TxBuff[UART2_TX_buffer_pointer++] = 0x31;    // "1"
	}
	else     /*****************   system off operation  ****************/
	{	
		UART2_TxBuff[UART2_TX_buffer_pointer++] = 0x30;    // "O"
	}	
	if(HEATER_RLY == 1)
	{
		UART2_TxBuff[UART2_TX_buffer_pointer++] = 0x31;    // "1"
	}
	else	 /*****************   system off operation	****************/
	{	
		UART2_TxBuff[UART2_TX_buffer_pointer++] = 0x30;    // "O"
	}
	
	if(ERROR_RLY == 1)
	{
		UART2_TxBuff[UART2_TX_buffer_pointer++] = 0x31;    // "1"
	}
	else     /*****************   system off operation  ****************/
	{	
		UART2_TxBuff[UART2_TX_buffer_pointer++] = 0x30;    // "O"
	}	

	UART2_TxBuff[UART2_TX_buffer_pointer++] = 0x20;    // "*"

	/* ***************************************************************************************************************** */	
#if 0
	if(EVAPORATOR_TEMP < 0)
	{
		UART1_TxBuff[12] = 0x2D;    // "-"
		disp_room_temp = ~EVAPORATOR_TEMP;
		disp_room_temp++;
		UART1_TxBuff[13] = disp_room_temp/100 + 0x30;
		UART1_TxBuff[14] = (disp_room_temp%100)/10 + 0x30;
		UART1_TxBuff[15] = (disp_room_temp%10) + 0x30;
	}
	else
	{
		disp_room_temp = EVAPORATOR_TEMP;
		UART1_TxBuff[12] = disp_room_temp/1000 + 0x30;    
		UART1_TxBuff[13] = (disp_room_temp%1000)/100 + 0x30;
		UART1_TxBuff[14] = (disp_room_temp%100)/10 + 0x30;
		UART1_TxBuff[15] = (disp_room_temp%10) + 0x30;
		
	}
	UART1_TxBuff[16] = 0x20;    // "*"
	/* ***************************************************************************************************************** */	
	if(CONDENSER_TEMP < 0)
	{
		UART1_TxBuff[17] = 0x2D;    // "-"
		disp_room_temp = ~CONDENSER_TEMP;
		disp_room_temp++;
		UART1_TxBuff[18] = disp_room_temp/100 + 0x30;
		UART1_TxBuff[19] = (disp_room_temp%100)/10 + 0x30;
		UART1_TxBuff[20] = (disp_room_temp%10) + 0x30;
	}
	else
	{
		disp_room_temp = CONDENSER_TEMP;
		UART1_TxBuff[17] = disp_room_temp/1000 + 0x30;    
		UART1_TxBuff[18] = (disp_room_temp%1000)/100 + 0x30;
		UART1_TxBuff[19] = (disp_room_temp%100)/10 + 0x30;
		UART1_TxBuff[20] = (disp_room_temp%10) + 0x30;
		
	}
	/* ***************************************************************************************************************** */	
	UART1_TxBuff[21] = 0x20;	// "*"
	/* ***************************************************************************************************************** */ 
	// if(0)
	if(COMP_OUT_TEMP < 0)
	{
		UART1_TxBuff[22] = 0x2D;	// "-"
		disp_room_temp = ~COMP_OUT_TEMP;
		disp_room_temp++;
		UART1_TxBuff[23] = disp_room_temp/100 + 0x30;
		UART1_TxBuff[24] = (disp_room_temp%100)/10 + 0x30;
		UART1_TxBuff[25] = (disp_room_temp%10) + 0x30;
	}
	else
	{
		disp_room_temp = COMP_OUT_TEMP;
		// disp_room_temp = T_DECP[_Room_sensor];
		UART1_TxBuff[22] = disp_room_temp/1000 + 0x30;	  
		UART1_TxBuff[23] = (disp_room_temp%1000)/100 + 0x30;
		UART1_TxBuff[24] = (disp_room_temp%100)/10 + 0x30;
		UART1_TxBuff[25] = (disp_room_temp%10) + 0x30;
		
	}

	/* ***************************************************************************************************************** */ 
	/* ***************************************************************************************************************** */	
	UART1_TxBuff[26] = 0x20;	// "*"
	/* ***************************************************************************************************************** */ 
	if(COMP_OUT_TEMP < 0)
	{
		UART1_TxBuff[27] = 0x2D;	// "-"
		disp_room_temp = ~COMP_OUT_TEMP;
		disp_room_temp++;
		UART1_TxBuff[28] = disp_room_temp/100 + 0x30;
		UART1_TxBuff[29] = (disp_room_temp%100)/10 + 0x30;
		UART1_TxBuff[30] = (disp_room_temp%10) + 0x30;
	}
	else
	{
		disp_room_temp = COMP_OUT_TEMP;
		UART1_TxBuff[27] = disp_room_temp/1000 + 0x30;	  
		UART1_TxBuff[28] = (disp_room_temp%1000)/100 + 0x30;
		UART1_TxBuff[29] = (disp_room_temp%100)/10 + 0x30;
		UART1_TxBuff[30] = (disp_room_temp%10) + 0x30;
		
	}
#endif		
	/* ***************************************************************************************************************** */ 

	

	
	UART2_TxBuff[UART2_TX_buffer_pointer++] = 0x20;    // "!"
	UART2_TxBuff[UART2_TX_buffer_pointer++] = 0x20;    // "!"
	UART2_TxBuff[UART2_TX_buffer_pointer++] = 0x20;    // "!"

	/*****************	 system on operation  ****************/
	UART2_TxBuff[UART2_TX_buffer_pointer++] = 0x53;	// "S"
	UART2_TxBuff[UART2_TX_buffer_pointer++] = 0x59;	// "Y"
	UART2_TxBuff[UART2_TX_buffer_pointer++] = 0x53;	// "S"
	UART2_TxBuff[UART2_TX_buffer_pointer++] = 0x2D;	// "-"

	if(current_sys_op_power)     
	{
		UART2_TxBuff[UART2_TX_buffer_pointer++] = 0x4F;    // "O"
		UART2_TxBuff[UART2_TX_buffer_pointer++] = 0x4E;    // "N"
		UART2_TxBuff[UART2_TX_buffer_pointer++] = 0x20;    // " "
	}
	else     /*****************   system off operation  ****************/
	{	
		UART2_TxBuff[UART2_TX_buffer_pointer++] = 0x4F;    // "O"
		UART2_TxBuff[UART2_TX_buffer_pointer++] = 0x46;    // "F"
		UART2_TxBuff[UART2_TX_buffer_pointer++] = 0x46;    // "F"
	}

	UART2_TxBuff[UART2_TX_buffer_pointer++] = 0x20;	// " "
	disp_room_temp = motor_current_position_1;
	UART2_TxBuff[UART2_TX_buffer_pointer++] = disp_room_temp/1000 + 0x30;	  
	UART2_TxBuff[UART2_TX_buffer_pointer++] = (disp_room_temp%1000)/100 + 0x30;
	UART2_TxBuff[UART2_TX_buffer_pointer++] = (disp_room_temp%100)/10 + 0x30;
	UART2_TxBuff[UART2_TX_buffer_pointer++] = (disp_room_temp%10) + 0x30;





	
	/* ***************************************************************************************************************** */ 
#if 0	
	//disp_room_temp = IPDU_frequency;
	
	UART1_TxBuff[41] = 0x20;    // " "
	UART1_TxBuff[42] = 0x57;    // "W"
	UART1_TxBuff[43] = 0x2D;    // "-"
	UART1_TxBuff[44] = disp_room_temp/10 + 0x30;
	UART1_TxBuff[45] = (disp_room_temp%10) + 0x30;

	//disp_room_temp = real_IPDU_frequency;
	
	UART1_TxBuff[46] = 0x20;    // " "
	UART1_TxBuff[47] = 0x52;    // "R"
	UART1_TxBuff[48] = 0x2D;    // "-"
	UART1_TxBuff[49] = disp_room_temp/10 + 0x30;
	UART1_TxBuff[50] = (disp_room_temp%10) + 0x30;


	UART1_TxBuff[51] = 0x20;    // " "
	//disp_room_temp = Test_timer;
	UART1_TxBuff[52] = disp_room_temp/1000 + 0x30;	  
	UART1_TxBuff[53] = (disp_room_temp%1000)/100 + 0x30;
	UART1_TxBuff[54] = (disp_room_temp%100)/10 + 0x30;
	UART1_TxBuff[55] = (disp_room_temp%10) + 0x30;

	UART1_TxBuff[56] = 0x20;	// " "
	//disp_room_temp = motor_current_position_1;
	UART1_TxBuff[57] = disp_room_temp/1000 + 0x30;	  
	UART1_TxBuff[58] = (disp_room_temp%1000)/100 + 0x30;
	UART1_TxBuff[59] = (disp_room_temp%100)/10 + 0x30;
	UART1_TxBuff[60] = (disp_room_temp%10) + 0x30;

	//disp_room_temp = IPDU_SW_version;
		
		UART1_TxBuff[61] = 0x20;	// " "
		UART1_TxBuff[62] = 0x56;	// "V"
		UART1_TxBuff[63] = 0x2D;	// "-"
		UART1_TxBuff[64] = disp_room_temp/10 + 0x30;
		UART1_TxBuff[65] = (disp_room_temp%10) + 0x30;
#endif

	R_UART2_Send(UART2_TxBuff, UART2_TX_buffer_pointer);
	
		

}

/* ***************************************************************************************************************** */


/* ***************************************************************************************************************** */
void	UARTold_send_temp(void)
{
		uint8_t	sum,index;
		int16_t	disp_room_temp;

	
	UART2_TxBuff[0] = 0x2A;    // "*"
	UART2_TxBuff[1] = 0x54;    // "T"
	if(CONTROL_TEMP < 0)
	{
		UART2_TxBuff[2] = 0x2D;    // "-"
		disp_room_temp = ~CONTROL_TEMP;
		disp_room_temp++;
		UART2_TxBuff[3] = disp_room_temp/100 + 0x30;
		UART2_TxBuff[4] = (disp_room_temp%100)/10 + 0x30;
		UART2_TxBuff[5] = (disp_room_temp%10) + 0x30;
	}
	else
	{
		disp_room_temp = CONTROL_TEMP;
		UART2_TxBuff[2] = CONTROL_TEMP/1000 + 0x30;    
		UART2_TxBuff[3] = (CONTROL_TEMP%1000)/100 + 0x30;
		UART2_TxBuff[4] = (CONTROL_TEMP%100)/10 + 0x30;
		UART2_TxBuff[5] = (CONTROL_TEMP%10) + 0x30;
	}
	
	
	sum = 0;
	for(index=1;index<=5;index++)
	{
		sum += UART2_TxBuff[index];
	}
	sum = 0 - sum;
	sum = sum - 1;
	
	if((sum >> 4) > 0x09)
	{
		UART2_TxBuff[6] = (sum >> 4) + 0x41 -10;
	}
	else
	{
		UART2_TxBuff[6] = (sum >> 4) + 0x30;
	}
	if((sum & 0x0F) > 0x09)
	{
		UART2_TxBuff[7] = (sum & 0x0F) + 0x41 -10;
	}
	else
	{
		UART2_TxBuff[7] = (sum & 0x0F) + 0x30;
	}

#if 0
	UART2_TxBuff[0] = 0x2A;    // "*"
	UART2_TxBuff[1] = 0x31;
	UART2_TxBuff[2] = 0x32;
	UART2_TxBuff[3] = 0x33;
	UART2_TxBuff[4] = 0x34;    
	UART2_TxBuff[5] = 0x35;
	UART2_TxBuff[6] = 0x36;
#endif	
	UART2_TxBuff[8] = 0x21;    // "!"

	R_UART2_Send(UART2_TxBuff, 9);
	
		

}

/* ***************************************************************************************************************** */

void	UART2_recieve_timeout(void)//called in RTC
{
	if(g_uart2_rx_count !=0)
	{
		UART2_timer_send = 0;
		if(g_uart2_rx_count == g_uart2_rx_cont_backup)
		{
			UART2_Data_recieve_timeout_timer++;
			if(UART2_Data_recieve_timeout_timer > 30) // 30ms
			{
				UART2_Data_recieve_timeout_timer = 0;
				R_UART2_receive_end();
			}
		}
		else
		{
			g_uart2_rx_cont_backup = g_uart2_rx_count;
			UART2_Data_recieve_timeout_timer = 0;
		}
	}
	else
	{
		g_uart2_rx_cont_backup = g_uart2_rx_count;
		UART2_Data_recieve_timeout_timer = 0;
		
		/* ********************************************************** */
		if(UART2_timer_send++ > 2000)
		{
			UART2_send_temp();
			UART2_timer_send = 0;
		}
	}
	
}
/* ***************************************************************************************************************** */

