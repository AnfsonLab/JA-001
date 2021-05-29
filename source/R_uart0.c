#include "r_cg_macrodriver.h"
#include "r_cg_userdefine.h"
#include "r_cg_serial.h"

/* ***************************************************************************************************************** */
#define UART0_RxBuf_Length 	15
#define UART0_TxBuf_Length 	50


uint8_t 	g_Uart0RxBuf;		/* UART0 receive buffer */
uint8_t 	g_Uart0RxErr;		/* UART0 receive error status */
MD_STATUS 	g_Uart0TxEnd;		/* UART0 transmission end */
uint8_t		UART0_RxBuff[UART0_RxBuf_Length],UART0_TxBuff[UART0_TxBuf_Length],UART0_RxBuff_conter,UART0_action_buff[UART0_RxBuf_Length];
uint8_t		UART0_Data_recieve_timeout_timer,UART0_transmit_delay_timer;
uint8_t		check_UART0_Rx_data;
uint16_t	g_uart0_rx_cont_backup;
extern volatile uint16_t  g_uart0_rx_count;            /* uart0 receive data number */

uint16_t	UART0_timer_send,Temp_adjust_buf;
uint8_t 	IPDU_SW_version,real_IPDU_frequency,clear_error_return;

uint8_t		IPDU_frequency,IPDU_command,Tx_Pointer;
uint16_t 	Modbus_CRC_check,Modbus_CRC_result;

extern 	uint8_t	UART1_TxBuff[80];
extern	int8_t	switch_set;




/***********************************************************************************************************************
Global variables and functions
***********************************************************************************************************************/
flag_t	_UART0_control_;
#define	UART0_control_			_UART0_control_.flag
#define	UART0_receive_done		_UART0_control_.bit.no0
#define	UART0_transmit_done		_UART0_control_.bit.no1
#define	UART0_transmit_delay	_UART0_control_.bit.no2
#define	UART0_rec_action		_UART0_control_.bit.no3

void	turn_system_off(void);
void	turn_system_on(void);
unsigned int Modbus_crc(unsigned char *buf, int length);


/* ***************************************************************************************************************** */

uint8_t	UART0_crcchk_data(void)
{

/* ****************** CRC check (Modbus) *************************************** */

Modbus_CRC_check =(uint16_t)UART0_RxBuff[g_uart0_rx_count-1];
Modbus_CRC_check |= (uint16_t)((uint16_t)UART0_RxBuff[g_uart0_rx_count-2] << 8);
Modbus_CRC_result = (Modbus_crc(UART0_RxBuff,g_uart0_rx_count-2));

if(Modbus_CRC_result == Modbus_CRC_result)
	return	(uint8_t)TRUE;
else
	return	(uint8_t)FALSE;


/* *********************************************************************************** */

	
}

/* ***************************************************************************************************************** */


void	convert_UART0_Data(void)
{
	//uint8_t	sum,index;
	//MD_STATUS	UART0_status;
	
	
}
/* ***************************************************************************************************************** */

void	R_UART0_receive_start(void)
{
	//MD_STATUS	UART0_status;
#if 1	
	uint8_t	clr;
	for(clr = 0; clr < sizeof(UART0_RxBuff)-1; clr++)
	{
		UART0_RxBuff[clr] = 0xAA;
	}
#endif	
	UART0_transmit_done = 0;
	R_UART0_Receive(&UART0_RxBuff, UART0_RxBuf_Length);  

	R_UART0_Start();
}
/* ***************************************************************************************************************** */

void	UART0_recieve_action(void)
{
	
	switch(UART0_RxBuff[2])
	{	
		
		
		/* ******************************** read from 0101 address************************************************************************* */
		case 01:
			if(UART0_RxBuff[3] == 01)
			{
				Tx_Pointer = 0;
				UART0_TxBuff[Tx_Pointer++]= Modbus_address;
				UART0_TxBuff[Tx_Pointer++]= UART0_RxBuff[1];
				UART0_TxBuff[Tx_Pointer++]= 02;				/* response data length */
				/* *************** 0301H *********************** */
				UART0_TxBuff[Tx_Pointer++]= 0x00;
				UART0_TxBuff[Tx_Pointer++]= 0x01;      /* set for ice water */
				
				Modbus_CRC_result = Modbus_crc(UART0_TxBuff,Tx_Pointer);
				
				UART0_TxBuff[Tx_Pointer++]= (uint8_t)Modbus_CRC_result >>8;
				UART0_TxBuff[Tx_Pointer++]= (uint8_t)(Modbus_CRC_result & 0x00FF);
				R_UART0_Send(UART0_TxBuff, Tx_Pointer);
				
			}
			break;
		/* ******************************** read from 0301 address ************************************************************************* */
		case 03:
			if(UART0_RxBuff[3] == 01)
			{
#if 1
				Tx_Pointer = 0;
				UART0_TxBuff[Tx_Pointer++]= Modbus_address;
				UART0_TxBuff[Tx_Pointer++]= UART0_RxBuff[1];
				UART0_TxBuff[Tx_Pointer++]= 30;				/* response data length */
				/* *************** 0301H *********************** */
				UART0_TxBuff[Tx_Pointer++]= 0x00;
				UART0_TxBuff[Tx_Pointer++]= 0x01;			/* set for ice water type */
				/* *************** 0302H *********************** */
				UART0_TxBuff[Tx_Pointer++]= 0x00;			/* set for system on/off */
				if(current_sys_op_power == RESET)  
						UART0_TxBuff[Tx_Pointer++]= 0x00;
					else
						UART0_TxBuff[Tx_Pointer++]= 0xFF;				
				/* *************** 0303H *********************** */
				UART0_TxBuff[Tx_Pointer++]= SET_TEMP >> 8;			/* set for set temp high */
				UART0_TxBuff[Tx_Pointer++]= (uint8_t)SET_TEMP;		/* set for set temp low */
				/* *************** 0304H *********************** */	/* set for CONTROL_TEMP */
				Temp_adjust_buf = CONTROL_TEMP + (int16_t)eeprom_option_byte[_Control_sensor_offset];
				UART0_TxBuff[Tx_Pointer++]= Temp_adjust_buf >> 8;
				UART0_TxBuff[Tx_Pointer++]= (uint8_t)Temp_adjust_buf;
				/* *************** 0305H *********************** */	/* set for set BASE_TEMP */
				if(TEMP_SELECT_SWITCH == FIX_TYPE_SENSOR)
				{
					Temp_adjust_buf = 10000;
				}
				else
				{
					Temp_adjust_buf = BASE_TEMP + (int16_t)eeprom_option_byte[_Base_sensor_offset];
				}
				
				UART0_TxBuff[Tx_Pointer++]= Temp_adjust_buf >> 8;
				UART0_TxBuff[Tx_Pointer++]= (uint8_t)Temp_adjust_buf;
				/* *************** 0306H *********************** */	/* set for CONDENSER_TEMP */
				Temp_adjust_buf = CONDENSER_TEMP;
				UART0_TxBuff[Tx_Pointer++]= Temp_adjust_buf >> 8;
				UART0_TxBuff[Tx_Pointer++]= (uint8_t)Temp_adjust_buf;
				/* *************** 0307H *********************** */	/* EVAPORATOR_TEMP */
				Temp_adjust_buf = ANTIFRZ_TEMP;
				UART0_TxBuff[Tx_Pointer++]= Temp_adjust_buf >> 8;
				UART0_TxBuff[Tx_Pointer++]= (uint8_t)Temp_adjust_buf;
				/* *************** 0308H *********************** */	/* COMP_OUT_TEMP */
				Temp_adjust_buf = COMP_OUT_TEMP;
				UART0_TxBuff[Tx_Pointer++]= Temp_adjust_buf >> 8;
				UART0_TxBuff[Tx_Pointer++]= (uint8_t)Temp_adjust_buf;
				/* *************** 0309H *********************** */	/* set for ERROR_CODE */
				UART0_TxBuff[Tx_Pointer++]= 0x00;
				UART0_TxBuff[Tx_Pointer++]= ERROR_CODE;
				/* *************** 030AH *********************** */	/* set for RELAY STATUS */
				UART0_TxBuff[Tx_Pointer++]= 0x00;
				UART0_TxBuff[Tx_Pointer]= 0x00;
				if(COMP_RLY == SET)
					UART0_TxBuff[Tx_Pointer]= UART0_TxBuff[Tx_Pointer] | 0x01;
				if(FAN_RLY == SET)
					UART0_TxBuff[Tx_Pointer]= UART0_TxBuff[Tx_Pointer] | 0x02;	
				if(PUMP_RLY == SET)
					UART0_TxBuff[Tx_Pointer]= UART0_TxBuff[Tx_Pointer] | 0x04;
				if(ERROR_RLY == SET)
					UART0_TxBuff[Tx_Pointer]= UART0_TxBuff[Tx_Pointer] | 0x08;	
				if(HEATER_RLY == SET)
					UART0_TxBuff[Tx_Pointer]= UART0_TxBuff[Tx_Pointer] | 0x10;	
				Tx_Pointer++;
				/* *************** 030BH *********************** */ /* set for switch setting */
				UART0_TxBuff[Tx_Pointer++]= 0x00;
				UART0_TxBuff[Tx_Pointer++]= switch_set;
				/* *************** 030CH *********************** */ /* set for remote on/off switch status */
				UART0_TxBuff[Tx_Pointer++]= 0x00;
				if(remote_on_off_flag == 1)
					UART0_TxBuff[Tx_Pointer++]= 0xFF;
				else
					UART0_TxBuff[Tx_Pointer++]= 0x00;
				/* *************** 030DH *********************** */ /* set for filter count */
				UART0_TxBuff[Tx_Pointer++]= 0x00;
				UART0_TxBuff[Tx_Pointer]= 0x00;
				Tx_Pointer++;
				/* *************** 030EH *********************** */
				UART0_TxBuff[Tx_Pointer++]= 0x00;
				UART0_TxBuff[Tx_Pointer++]= 0x00;
				/* *************** 030FH *********************** */
				UART0_TxBuff[Tx_Pointer++]= 0x00;
				UART0_TxBuff[Tx_Pointer++]= 0x00;

#endif
				Modbus_CRC_result = Modbus_crc(UART0_TxBuff,Tx_Pointer);
				UART0_TxBuff[Tx_Pointer++]= (uint8_t)Modbus_CRC_result >>8;
				UART0_TxBuff[Tx_Pointer++]= (uint8_t)(Modbus_CRC_result & 0x00FF);
				R_UART0_Send(UART0_TxBuff, Tx_Pointer);
				UART0_transmit_done = 1;
				
			}
			break;
		/* ********************************************************************************************************* */
		default:
			break;	
	}

}

/* ***************************************************************************************************************** */

void	R_UART0_receive_end(void)
{

	if(UART0_crcchk_data())
	{
		UART0_receive_done = SET;
		UART0_transmit_delay = SET;
		UART0_transmit_delay_timer = 0;
		if(UART0_transmit_done == 0)
		{
			if(UART0_RxBuff[0] == Modbus_address)
			{
				UART0_recieve_action();
			}
		}
		
	}
	else
	{
		R_UART0_receive_start();
	}
}

/* ***************************************************************************************************************** */

void	UART0_control(void)//called in main
{

}
/* ***************************************************************************************************************** */
void	UART0_send_loop(void)
{
	uint8_t	send_count,CRC;
	
	switch(IPDU_command)
	{
		/* ***************************************************************************************************************** */	
		default:
		case 0:						// command 00 check IPDU and get S/W version
			UART0_TxBuff[0]= 0x00;
			UART0_TxBuff[1]= 0x01;
			UART0_TxBuff[2]= 0x00;
			UART0_TxBuff[3]= 0x03;
			UART0_TxBuff[4]= 0x52;
			UART0_TxBuff[5]= 0x09;
			UART0_TxBuff[6]= 0x0b;
			UART0_TxBuff[7]= 0x00;
			CRC=0;
			for(send_count=0;send_count<8;send_count++)
			{
				CRC = CRC+UART0_TxBuff[send_count];
			}
			
			UART0_TxBuff[8]= 0-CRC;
			
			R_UART0_Send(UART0_TxBuff, 9);
#if 0			
			R_UART1_Send(UART0_TxBuff, 9);
#endif
			break;
		/* ***************************************************************************************************************** */	
		case 0x10:					// command 0x10 set IPDU_frequency

			
			UART0_TxBuff[0]= 0x00;
			UART0_TxBuff[1]= 0x01;
			UART0_TxBuff[2]= 0x10;
			UART0_TxBuff[3]= 0x02;
			UART0_TxBuff[4]= IPDU_frequency;
			UART0_TxBuff[5]= 0x00;
			UART0_TxBuff[6]= 0x00;
			CRC=0;
			for(send_count=0;send_count<7;send_count++)
			{
				CRC = CRC+UART0_TxBuff[send_count];
			}
			
			UART0_TxBuff[7]= 0-CRC;
			
			R_UART0_Send(UART0_TxBuff, 8);
#if 0			
			R_UART1_Send(UART0_TxBuff, 8);
#endif
			break;		
		/* ***************************************************************************************************************** */	
		case 0x40:						// command 0x40 Clear Error
			UART0_TxBuff[0]= 0x00;
			UART0_TxBuff[1]= 0x01;
			UART0_TxBuff[2]= 0x40;
			UART0_TxBuff[3]= 0x00;
			UART0_TxBuff[4]= 0x5A;
			CRC=0;
			for(send_count=0;send_count<5;send_count++)
			{
				CRC = CRC+UART0_TxBuff[send_count];
			}
			
			UART0_TxBuff[5]= 0-CRC;
			
			R_UART0_Send(UART0_TxBuff, 8);
			break;					
	}
	
		

}

/* ***************************************************************************************************************** */
void	UART0_recieve_timeout(void)//called in RTC
{

	if(g_uart0_rx_count !=0)
	{
		if(g_uart0_rx_count == g_uart0_rx_cont_backup)
		{
			UART0_Data_recieve_timeout_timer++;
			if(UART0_Data_recieve_timeout_timer > 15) // 30ms
			{
				UART0_Data_recieve_timeout_timer = 0;
				R_UART0_receive_end();
			}
		}
		else
		{
			g_uart0_rx_cont_backup = g_uart0_rx_count;
			UART0_Data_recieve_timeout_timer = 0;
		}
	}
	else
	{
	
		g_uart0_rx_cont_backup = g_uart0_rx_count;
		UART0_Data_recieve_timeout_timer = 0;
	
	}
}


/* ***************************************************************************************************************** */
/* ***************************************************************************************************************** */

unsigned int Modbus_crc(unsigned char *buf, int length) 
{
	unsigned int crc = 0xFFFF;
	int i,j;
	unsigned char carrayFlag; 
	unsigned int swap_crc_byte;  //swap high low and byte of crc
	unsigned int temp_word;
	
	for (i = 0; i < length; i++)
    {
	    crc ^= buf[i];
	    for (j = 0; j < 8; j++) 
    	{
			carrayFlag = crc & 1;
			crc = crc >> 1;
			if (carrayFlag) 
			{
				crc ^= 0xA001;
			};
		};
	}; 
	swap_crc_byte = crc << 8;		 //the low byte of crc is now in the high byte of swap_crc_byte
	temp_word = crc >> 8;
	swap_crc_byte = swap_crc_byte | temp_word;//the high byte of crc is now in the low bte of swap_crc_byte

	return swap_crc_byte;
}






