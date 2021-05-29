



/***********************************************************************************************************************
Includes
***********************************************************************************************************************/
#include "r_cg_macrodriver.h"
#include "r_cg_serial.h"
#include "r_cg_userdefine.h"




/* ****************************************************************** */
int8_t	eeprom_option_byte[IIC_option_length+3];
/* *******************  IIC code area ***************************************** */



uint8_t tx_data[DATA_LENGTH]=  /* Transmission data( 16 byte ) */
{
    0x00,0x01,0x02,0x03,0x04,0x05,0x06,0x07,0x08 };
//,
  //  0x08,0x09,0x0A,0x0B,0x0C,0x0D,0x0E,0x0F
//};

uint8_t com_status;                         /* Communication state */
                                            /* 0 : Idle (Tx/Rx is normal end.) */
                                            /* 1 : Communicating, */
                                            /* 2 : Tx/Rx is abnormal end. */
uint8_t rx_buffer[DATA_LENGTH];      /* Reception data preservation area */
static uint8_t com_direction;               /* Communication direction. */
                                            /* 0 : Transmission, 1 : Receiving */
static uint8_t restart_counter;             /* Restart counter */
											
uint8_t loop_counter,option_buffer_check_flag;                   /* Loop counter */
MD_STATUS iic_status;                   /* IIC status */




int8_t	error_count=0;
uint8_t	IIC_tx_buf[150];
uint8_t	IIC_rx_buf[190];
uint16_t	IIC_check_sum_16_bit=0;
uint8_t	IIC_check_sum_8_bit=0;
uint16_t	checksum_buf;
int16_t	SET_TEMP_BUF;

uint8_t	IIC_key_value_1,IIC_key_value_2;

uint8_t	i;
uint8_t	state_code_backup_count;
uint8_t	IIC_write_data_length;
uint8_t	IIC_operation_status[8]={ 0,0,0,10,0,0,1,0 },Modbus_address=01;
uint8_t	IIC_error_code[15] = { 0,0xc1,0xc2,0xc3,0xc4,0xc5,0xd1,0xd2,0xd3,0xe1,0xe2,0,0 };

void 	IIC_save_option(void);
void 	IIC_save_value(void);
void 	Get_init_value_data(void);
void	init_eeprom_option_buffer(void);
void 	Get_value_data_from_IIC(void);
void 	Set_value_data_To_IIC(void);
void 	IIC_write_checksun(void);
void 	IIC_save_status(void);
void	error_system_off(void);
void 	IIC_save_error_code(void);
void	check_eeprom_option_buffer(void);




/* *******************  IIC code area ***************************************** */

#if 0
/***********************************************************************************************************************
* Function Name: IIC_send_address_0
* Description  : This function adds user code before implementing main function.
* Arguments    : None
* Return Value : None
***********************************************************************************************************************/
void IIC_send_address_0(void)
{
        /* Initialization if variables */
        loop_counter = 0U;                      /* Loop counter */
        iic_status = MD_STATUSBASE;             /* IIC status */
        /*---------------------------------
            Communication preprocess
        ---------------------------------*/
        if (STCEN0 != 1U)                       /* Case of bus non-liberating. */
        {
            R_IICA0_StopCondition();            /* Generate stop condition. */
            /* Detection waiting of stop condition. */
            while (!SPD0)
            {
                NOP();
            }
        }
        com_direction  = 0U;                    /* Direction : transmission. */

            DI();                               /* Disable interrupt */
            TMMK00 = 0U;                        /* Mask release of timer interrupt. */
            /*---------------------------------
                Waiting communication (10ms)
            ---------------------------------*/
            while (!TMIF00)
            {
                HALT();
            }
            TMIF00 = 0U;                        /* Clear timer interrupt request. */
            TMMK00 = 1U;                        /* Mask timer interrupt. */
            EI();                               /* Enable interrupt */

            /************************************/
            /* Prepare Communication            */
            /************************************/        
	     restart_counter = RESTART_COUNT;    /* Setting of retry counter. */
            IICAIF0 = 0U;                       /* Clear IICA interrupt request. */
            com_status = 1U;                    /* Setting communication flag. */

		tx_data[0] = 0;
			/* Send data. */
                    	iic_status = R_IICA0_Master_Send(
                        (SL_ADDR),              /* Slave address. */
                        (uint8_t *)&tx_data[0], /* Transfer buffer pointer. */
				1,
                      //  DATA_LENGTH,            /* Buffer size. */
                        20U);                   /* Wait for start condition. */

			 /* IIC status check. (Nothing is done now) */
                    	switch (iic_status)
                    	{
                        case MD_OK:
                            break;
                        case MD_ERROR1:
                            break;
                        case MD_ERROR2:
                            break;
                        default:
                            break;
                    	}

                    	while (com_status == 1U)    /* Communicating. */
                    	{
                        HALT();                 /* HALT mode until Tx completed. */
                    	}
                    	if (com_status == 0U)       /* Communication is normal end. */
                    	{
                        com_direction = 0U;     /* Change receving mode. */
                    	}
}	

/***********************************************************************************************************************
* Function Name: IIC_send_address_0
* Description  : This function adds user code before implementing main function.
* Arguments    : None
* Return Value : None
***********************************************************************************************************************/
void IIC_receive_data(void)
{

 /*---------------------------------
            Communication preprocess
        ---------------------------------*/
        if (STCEN0 != 1U)                       /* Case of bus non-liberating. */
        {
            R_IICA0_StopCondition();            /* Generate stop condition. */
            /* Detection waiting of stop condition. */
            while (!SPD0)
            {
                NOP();
            }
        }
        com_direction  = 0U;                    /* Direction : transmission. */

            DI();                               /* Disable interrupt */
            TMMK00 = 0U;                        /* Mask release of timer interrupt. */
            /*---------------------------------
                Waiting communication (10ms)
            ---------------------------------*/
            while (!TMIF00)
            {
                HALT();
            }
            TMIF00 = 0U;                        /* Clear timer interrupt request. */
            TMMK00 = 1U;                        /* Mask timer interrupt. */
            EI();                               /* Enab */


 /*---------------------------------
            Communication preprocess
        ---------------------------------*/



		 com_direction = 1U;     /* Change sending mode. */
                    /* Receive data. */
                    iic_status = R_IICA0_Master_Receive(
                        (SL_ADDR),              /* Slave address. */
                        &rx_buffer[0],          /* Receive buffer pointer.  */
                        8,            /* Buffer size. */
                        20U);                   /* Wait for start condition. */

                    /* IIC status check. (Nothing is done now) */
                    switch (iic_status)
                    {
                        case MD_OK:
                            break;
                        case MD_ERROR1:
                            break;
                        case MD_ERROR2:
                            break;
                        default:
                            break;
                    }

                    while (com_status == 1U)    /* Communicating. */
                    {
                        HALT();                 /* HALT mode until Rx completed. */
                    }

                    if (com_status == 0U)       /* Communication is normal end. */
                    {
                        com_direction = 1U;     /* Change sending mode. */
                    }

		    
		    
		    
}

#endif

/***********************************************************************************************************************
* Function Name: IIC_program
* Description  : This function adds user code before implementing main function.
* Arguments    : None
* Return Value : None
***********************************************************************************************************************/
void IIC_read_data(void)
{

        /* Initialization if variables */
        loop_counter = 0U;                      /* Loop counter */
        iic_status = MD_STATUSBASE;             /* IIC status */
        

        /*---------------------------------
            Communication preprocess
        ---------------------------------*/
        if (STCEN0 != 1U)                       /* Case of bus non-liberating. */
        {
            R_IICA0_StopCondition();            /* Generate stop condition. */
            /* Detection waiting of stop condition. */
            while (!SPD0)
            {
                NOP();
            }
        }
        com_direction  = 0U;                    /* Direction : transmission. */

            DI();                               /* Disable interrupt */
            TMMK00 = 0U;                        /* Mask release of timer interrupt. */
            /*---------------------------------
                Waiting communication (10ms)
            ---------------------------------*/
            while (!TMIF00)
            {
                HALT();
            }
            TMIF00 = 0U;                        /* Clear timer interrupt request. */
            TMMK00 = 1U;                        /* Mask timer interrupt. */
            EI();                               /* Enable interrupt */

            /************************************/
            /* Prepare Communication            */
            /************************************/
            restart_counter = RESTART_COUNT;    /* Setting of retry counter. */
            IICAIF0 = 0U;                       /* Clear IICA interrupt request. */
            com_status = 1U;                    /* Setting communication flag. */

            /* It repeats until one communication is completed. */
            while (com_status)
            {
                if (com_direction == 0U)        /* Direction is transmission. */
                {
                    /* Send data. */
	
                    iic_status = R_IICA0_Master_Send(
                        (SL_ADDR),              /* Slave address. */
                        (uint8_t *)&tx_data[0], /* Transfer buffer pointer. */
			1,
                      //  DATA_LENGTH,            /* Buffer size. */
                        20U);                   /* Wait for start condition. */

                    /* IIC status check. (Nothing is done now) */
                    switch (iic_status)
                    {
                        case MD_OK:
                            break;
                        case MD_ERROR1:
                            break;
                        case MD_ERROR2:
                            break;
                        default:
                            break;
                    }

                    while (com_status == 1U)    /* Communicating. */
                    {
                        HALT();                 /* HALT mode until Tx completed. */
                    }
                    if (com_status == 0U)       /* Communication is normal end. */
                    {
                        com_direction = 1U;     /* Change receving mode. */
			com_status = 1U;
                    }
                }
                else                            /* Direction is reception. */
                {
		    R_IICA0_StopCondition();    /* Generate stop condition. */
                    /* Detection waiting of stop condition. */
                    while (!SPD0)
                    {
                        NOP();
                    }	
			
			
                    /* Receive data. */
                    iic_status = R_IICA0_Master_Receive(
                        (SL_ADDR),              /* Slave address. */
                        &rx_buffer[0],          /* Receive buffer pointer.  */
                        8,            /* Buffer size. */
                        20U);                   /* Wait for start condition. */

                    /* IIC status check. (Nothing is done now) */
                    switch (iic_status)
                    {
                        case MD_OK:
                            break;
                        case MD_ERROR1:
                            break;
                        case MD_ERROR2:
                            break;
                        default:
                            break;
                    }

                    while (com_status == 1U)    /* Communicating. */
                    {
                        HALT();                 /* HALT mode until Rx completed. */
                    }

                    if (com_status == 0U)       /* Communication is normal end. */
                    {
                        com_direction = 0U;     /* Change sending mode. */
                    }
                }

                if (com_status == 2U)           /* Incomplete communication. */
                {
                    R_IICA0_StopCondition();    /* Generate stop condition. */
                    /* Detection waiting of stop condition. */
                    while (!SPD0)
                    {
                        NOP();
                    }
                    if (restart_counter)        /* Retry is incomplete. */
                    {
                        /* Wait about 10us and restart. */
                        for (loop_counter = 0; loop_counter < LOOP_COUNT_VALUE; loop_counter++)
                        {
                            NOP();
                        }
			restart_counter -= 1U;  /* Count retry times. */
                        com_status = 1U;        /* Continue communication. */
			
                    }
                    else                        /* Retry is complete. */
                    {
                        com_status = 0U;        /* Communication is end. */
                    }
                }
            }
            /* One communication was completed. */

    }

/***********************************************************************************************************************
* Function Name: IIC_write_data
* Description  : This function adds user code before implementing main function.
* Arguments    : None
* Return Value : None
***********************************************************************************************************************/
void IIC_write_data(void)
{
	        /* Initialization if variables */
        loop_counter = 0U;                      /* Loop counter */
        iic_status = MD_STATUSBASE;             /* IIC status */
        /*---------------------------------
            Communication preprocess
        ---------------------------------*/
        if (STCEN0 != 1U)                       /* Case of bus non-liberating. */
        {
            R_IICA0_StopCondition();            /* Generate stop condition. */
            /* Detection waiting of stop condition. */
            while (!SPD0)
            {
                NOP();
            }
        }
        com_direction  = 0U;                    /* Direction : transmission. */

            DI();                               /* Disable interrupt */
            TMMK00 = 0U;                        /* Mask release of timer interrupt. */
            /*---------------------------------
                Waiting communication (10ms)
            ---------------------------------*/
            while (!TMIF00)
            {
                HALT();
            }
            TMIF00 = 0U;                        /* Clear timer interrupt request. */
            TMMK00 = 1U;                        /* Mask timer interrupt. */
            EI();                               /* Enable interrupt */

            /************************************/
            /* Prepare Communication            */
            /************************************/        
	     restart_counter = RESTART_COUNT;    /* Setting of retry counter. */
            IICAIF0 = 0U;                       /* Clear IICA interrupt request. */
            com_status = 1U;                    /* Setting communication flag. */

	    while (com_status)
            {
				/* Send data. */
                    	iic_status = R_IICA0_Master_Send(
                        (SL_ADDR),              /* Slave address. */
                        (uint8_t *)&tx_data[0], /* Transfer buffer pointer. */
				IIC_write_data_length,
                      //  DATA_LENGTH,            /* Buffer size. */
                        20U);                   /* Wait for start condition. */

			 /* IIC status check. (Nothing is done now) */
                    	switch (iic_status)
                    	{
                        case MD_OK:
                            break;
                        case MD_ERROR1:
                            break;
                        case MD_ERROR2:
                            break;
                        default:
                            break;
                    	}

						// ms_counter = 0;
                    	while (com_status == 1U)    /* Communicating. */
                    	{
                        HALT();                 /* HALT mode until Tx completed. */
                    	}
			
			
			
                    	if (com_status == 0U)       /* Communication is normal end. */
                    	{
                        com_direction = 0U;     /* Change receving mode. */
                    	}
		
		if (com_status == 2U)           /* Incomplete communication. */
                {
                	
                    R_IICA0_StopCondition();    /* Generate stop condition. */
                    /* Detection waiting of stop condition. */
                    while (!SPD0)
                    {
                        NOP();
                    }
		    if (restart_counter)        /* Retry is incomplete. */
                    {
                        /* Wait about 10us and restart. */
                        for (loop_counter = 0; loop_counter < LOOP_COUNT_VALUE; loop_counter++)
                        {
                            NOP();
                        }
			restart_counter -= 1U;  /* Count retry times. */
                        com_status = 1U;        /* Continue communication. */
			
                    }
                    else                        /* Retry is complete. */
                    {
                        com_status = 0U;        /* Communication is end. */
                    }
		}
	    }


}
/***********************************************************************************************************************
* Function Name: IIC_init_data
* Description  : This function adds user code before implementing main function.
* Arguments    : None
* Return Value : None
***********************************************************************************************************************/
void IIC_init_data(void)
{
	init_eeprom_option_buffer();
	IIC_save_option();
	/*************************************************************************/	
	tx_data[0] = IIC_init_address;
	tx_data[1] = IIC_default_1;
	tx_data[2] = IIC_default_2;
	tx_data[3] = 1;
	tx_data[4] = 1;
	IIC_write_data_length = 5;
	IIC_write_data();
	/*************************************************************************/	
	Set_value_data_To_IIC();
	IIC_save_value();
	IIC_save_status();
#if 0						// testtt for error code test
	for(i=0;i<13;i++)
	{
		IIC_error_code[i] = 0;
	}
#endif	
	IIC_save_error_code();
}
/***********************************************************************************************************************
* Function Name: IIC_check_init
* Description  : This function adds user code before implementing main function.
* Arguments    : None
* Return Value : None
***********************************************************************************************************************/
void IIC_check_init(void)
{
	
	
	for (i=0;i<190;i=i+8)
	{
		tx_data[0] = i;
		IIC_read_data();
		IIC_rx_buf[i] = rx_buffer[0];
		IIC_rx_buf[i+1] = rx_buffer[1];
		IIC_rx_buf[i+2] = rx_buffer[2];
		IIC_rx_buf[i+3] = rx_buffer[3];
		IIC_rx_buf[i+4] = rx_buffer[4];
		IIC_rx_buf[i+5] = rx_buffer[5];
		IIC_rx_buf[i+6] = rx_buffer[6];
		IIC_rx_buf[i+7] = rx_buffer[7];
	}
	IIC_check_sum_16_bit = 0;
	for (i=0;i<IIC_checksum_length;i=i+1)
	{
		IIC_check_sum_16_bit = IIC_check_sum_16_bit + IIC_rx_buf[i];
	}

	checksum_buf = IIC_rx_buf[IIC_checksum_address];
	checksum_buf = checksum_buf << 8;
	checksum_buf = checksum_buf + IIC_rx_buf[IIC_checksum_address+1];

	IIC_key_value_1 = IIC_rx_buf[IIC_key_address];
	IIC_key_value_2 = IIC_rx_buf[IIC_key_address+1];
	
	if( (IIC_rx_buf[IIC_init_address] != IIC_default_1) || (IIC_rx_buf[IIC_init_address + 1] != IIC_default_2) 
		|| (checksum_buf != IIC_check_sum_16_bit)) 
	{
		if(production_mode == SET)
		{
			IIC_init_data();
		}
		else
		{
			ERROR_CODE = 0xeb;       // need to modify errror code
			error_system_off();
		}
	}
	else
	{
		if(production_mode == SET)
		{
			IIC_init_data();
		}	
		for (i=0;i<IIC_option_length;i++)
		{
			eeprom_option_byte[i] = IIC_rx_buf[i]; 
		}

		
		check_eeprom_option_buffer();
		if(option_buffer_check_flag == 0)
		{
			ERROR_CODE = 0xec;       // test need to modify back to 0xeb 
			error_system_off();
		}
		
		Modbus_address = eeprom_option_byte[_Modbus_address_offset];

		
		Get_value_data_from_IIC();

		for (i=0;i<IIC_status_length;i++)
		{
			IIC_operation_status[i] = IIC_rx_buf[IIC_status_address+i]; 
		}
		
		SET_TEMP = IIC_operation_status[op_state_set_temp_H];
		SET_TEMP = SET_TEMP << 8;
		SET_TEMP = SET_TEMP | IIC_operation_status[op_state_set_temp_L];
#if 1		
		IIC_check_sum_8_bit =0;
		for (i=0;i<7;i++)
		{
			IIC_check_sum_8_bit = IIC_check_sum_8_bit + IIC_operation_status[i];
		}
	 
		if(IIC_operation_status[7] != IIC_check_sum_8_bit)
		{
			ERROR_CODE = 0xed;       //  test need to modify back to 0xeb 
			error_system_off();
		}
#endif		
		
#if 1		
		tx_data[0] =IIC_error_record_address;
		IIC_read_data();
		IIC_error_code[0] = rx_buffer[0];
		IIC_error_code[1] = rx_buffer[1];
		IIC_error_code[2] = rx_buffer[2];
		IIC_error_code[3] = rx_buffer[3];
		IIC_error_code[4] = rx_buffer[4];
		IIC_error_code[5] = rx_buffer[5];
		IIC_error_code[6] = rx_buffer[6];
		IIC_error_code[7] = rx_buffer[7];
		
		tx_data[0] =IIC_error_record_address+8;
		IIC_read_data();
		IIC_error_code[8] = rx_buffer[0];
		IIC_error_code[9] = rx_buffer[1];
		IIC_error_code[10] = rx_buffer[2];
		IIC_error_code[11] = rx_buffer[3];
		if(IIC_error_code[0] > 10)
			IIC_error_code[0] = 0;
#endif				
	}

}

/***********************************************************************************************************************
* Function Name: IIC_save_option
* Description  : This function adds user code before implementing main function.
* Arguments    : None
* Return Value : None
***********************************************************************************************************************/

void Get_value_data_from_IIC(void)
{
	for(i=0;i<IIC_PMV_length;i++)
	{
		PMV_TABLE_STEP[i] = IIC_rx_buf[IIC_PMV_Table+i];
	}
	
#if 0	
	for(i=0;i<IIC_PMV_length;i++)
	{
		PMV_TABLE_TIME[i] = IIC_rx_buf[IIC_PMV_Time+i];
	}

	
	for(i=0;i<IIC_WaterGate_length;i++)
	{
		WATERGATE_CONTROL_TABLE[i] = IIC_rx_buf[IIC_WaterGate_Table+i];
	}
	for(i=0;i<IIC_WaterGate_length;i++)
	{
		WATERGATE_TABLE_TIME[i] = IIC_rx_buf[IIC_WaterGate_Time+i];
	}
#endif	
}

/***********************************************************************************************************************
* Function Name: IIC_save_option
* Description  : This function adds user code before implementing main function.
* Arguments    : None
* Return Value : None
***********************************************************************************************************************/

void Set_value_data_To_IIC(void)
{
	//uint8_t	pointer_buf;
	
	for(i=0;i<IIC_PMV_length;i++)
	{
		IIC_tx_buf[i] = PMV_TABLE_STEP[i];
	}
#if 0	
	pointer_buf = 3*IIC_PMV_length;
	
	for(i=0;i<IIC_PMV_length;i++)
	{
		IIC_tx_buf[pointer_buf+i] = PMV_TABLE_TIME[i];
	}
	pointer_buf = pointer_buf + IIC_PMV_length;
	
	for(i=0;i<IIC_WaterGate_length;i++)
	{
		IIC_tx_buf[pointer_buf+i] = WATERGATE_CONTROL_TABLE[i];
	}
	pointer_buf = pointer_buf + IIC_WaterGate_length;
	
	for(i=0;i<IIC_WaterGate_length;i++)
	{
		IIC_tx_buf[pointer_buf+i] = WATERGATE_TABLE_TIME[i];
	}
#endif	
}


/***********************************************************************************************************************
* Function Name: IIC_save_option
* Description  : This function adds user code before implementing main function.
* Arguments    : None
* Return Value : None
***********************************************************************************************************************/
void IIC_save_option(void)
{
	for ( i = 0; i < IIC_option_length; i++)
	{
		IIC_tx_buf[i] = eeprom_option_byte[i];
	}
	// ms_counter = 0;
	for (i=0;i<IIC_option_length;i=i+1)
	{
		tx_data[0] = IIC_option_address + i;
		tx_data[1] = IIC_tx_buf[i];
		IIC_write_data_length = 2;
		IIC_write_data();
	}
	IIC_write_checksun();
}
/***********************************************************************************************************************
* Function Name: IIC_save_option
* Description  : This function adds user code before implementing main function.
* Arguments    : None
* Return Value : None
***********************************************************************************************************************/
void IIC_save_value(void)
{
	
	for (i=0;i<IIC_value_length;i=i+1)
	{
		tx_data[0] = IIC_value_data+i;
		tx_data[1] = IIC_tx_buf[i];
		IIC_write_data_length = 2;
		IIC_write_data();
	}
	IIC_write_checksun();
}
/***********************************************************************************************************************
* Function Name: IIC_save_status
* Description  : This function adds user code before implementing main function.
* Arguments    : None
* Return Value : None
***********************************************************************************************************************/
void IIC_save_status(void)
{

	IIC_check_sum_8_bit =0;
	IIC_operation_status[0]++;
	
	for (i=0;i<7;i++)
	{
		IIC_check_sum_8_bit = IIC_check_sum_8_bit + IIC_operation_status[i];
	}
	 
	IIC_operation_status[7] = IIC_check_sum_8_bit; 
	for ( i = 0; i < IIC_status_length; i++)
	{
		IIC_tx_buf[i] = IIC_operation_status[i];
	}
	for (i=0;i<IIC_status_length;i=i+1)
	{
		tx_data[0] = IIC_status_address+i;
		tx_data[1] = IIC_tx_buf[i];
		IIC_write_data_length = 2;
		IIC_write_data();
	}
}
/***********************************************************************************************************************
* Function Name: IIC_save_error_code
* Description  : This function adds user code before implementing main function.
* Arguments    : None
* Return Value : None
***********************************************************************************************************************/
void IIC_save_error_code(void)
{

	for ( i = 0; i < IIC_error_record_length; i++)
	{
		IIC_tx_buf[i] = IIC_error_code[i];
	}
	for (i=0;i<IIC_error_record_length;i=i+1)
	{
		tx_data[0] = IIC_error_record_address+i;
		tx_data[1] = IIC_tx_buf[i];
		IIC_write_data_length = 2;
		IIC_write_data();
	}
}

/***********************************************************************************************************************
* Function Name: IIC_read_error_code
* Description  : This function adds user code before implementing main function.
* Arguments    : None
* Return Value : None
***********************************************************************************************************************/
void IIC_read_error_code(void)
{

	for ( i = 0; i < IIC_error_record_length; i++)
	{
		IIC_tx_buf[i] = IIC_error_code[i];
	}
	for (i=0;i<IIC_error_record_length;i=i+1)
	{
		tx_data[0] = IIC_error_record_address+i;
		tx_data[1] = IIC_tx_buf[i];
		IIC_write_data_length = 2;
		IIC_write_data();
	}
}



/* *******************  IIC code area ***************************************** */

#if 0
/***********************************************************************************************************************
* Function Name: write_status_to_IIC
* Description  : This function adds user code before implementing main function.
* Arguments    : None
* Return Value : None
***********************************************************************************************************************/
void write_status_to_IIC(void)
{
		
	IIC_check_sum_8_bit =0;
	IIC_operation_status[0]++;
	
	for (i=0;i<7;i++)
	{
		IIC_check_sum_8_bit = IIC_check_sum_8_bit + IIC_operation_status[i];
	}
	 
	IIC_operation_status[7] = IIC_check_sum_8_bit; 

	IIC_save_status();




}

/* *******************  IIC code area ***************************************** */
#endif
/***********************************************************************************************************************
* Function Name: IIC_write_checksun
* Description  : This function adds user code before implementing main function.
* Arguments    : None
* Return Value : None
***********************************************************************************************************************/
void IIC_write_checksun(void)
{
	
	for (i=0;i<IIC_checksum_length;i=i+8)
	{
		tx_data[0] = i;
		IIC_read_data();
		IIC_rx_buf[i] = rx_buffer[0];
		IIC_rx_buf[i+1] = rx_buffer[1];
		IIC_rx_buf[i+2] = rx_buffer[2];
		IIC_rx_buf[i+3] = rx_buffer[3];
		IIC_rx_buf[i+4] = rx_buffer[4];
		IIC_rx_buf[i+5] = rx_buffer[5];
		IIC_rx_buf[i+6] = rx_buffer[6];
		IIC_rx_buf[i+7] = rx_buffer[7];
	}
	IIC_check_sum_16_bit = 0;
	for (i=0;i<IIC_checksum_length;i=i+1)
	{
		IIC_check_sum_16_bit = IIC_check_sum_16_bit + IIC_rx_buf[i];
	}
	/*************************************************************************/	
	tx_data[0] = IIC_checksum_address;
	tx_data[1] = IIC_check_sum_16_bit >> 8;
	tx_data[2] = (uint8_t)IIC_check_sum_16_bit;
	IIC_write_data_length = 3;
	IIC_write_data();
	/*************************************************************************/	


}


/***********************************************************************************************************************
* Function Name: IIC_write_checksun
* Description  : This function adds user code before implementing main function.
* Arguments    : None
* Return Value : None
***********************************************************************************************************************/

void	Check_update_status_IIC(void)
{
	uint8_t	write_state_to_IIC;
	
	
	write_state_to_IIC= RESET;
#if 0	
	if(current_sys_op_power == SET)
	{
		if(IIC_operation_status[op_state_power] != 0xff )
		{
			write_state_to_IIC = SET;
			IIC_operation_status[op_state_power] = 0xff;
		}
	}
	else
	{
		if(IIC_operation_status[op_state_power] != 0x00 )
		{
			write_state_to_IIC = SET;
			IIC_operation_status[op_state_power] = 0x00;
		}
	}
#endif	
				
	/* ************************************************************************** */	
	SET_TEMP_BUF= IIC_operation_status[op_state_set_temp_H];
	SET_TEMP_BUF = SET_TEMP_BUF << 8;
	SET_TEMP_BUF = SET_TEMP_BUF | IIC_operation_status[op_state_set_temp_L];

	if( SET_TEMP != SET_TEMP_BUF)
	{
		if(state_code_backup_count >= STATE_BACKUP_SET)
		{
			write_state_to_IIC = SET;
			IIC_operation_status[op_state_set_temp_H] = SET_TEMP >> 8;
			IIC_operation_status[op_state_set_temp_L] =(uint8_t)SET_TEMP ;
		}
	}
	else
	{
		state_code_backup_count = 0;
	}
	/* ************************************************************************** */
#if 0	
	if(IIC_operation_status[op_state_filter] != filter_10hour_count)
	{
		IIC_operation_status[op_state_filter] = filter_10hour_count;
		write_state_to_IIC = SET;
	}
	/* ************************************************************************** */
	if(IIC_operation_status[op_state_Modbus_addr] != Modbus_address)
	{
		IIC_operation_status[op_state_Modbus_addr] = Modbus_address;
		write_state_to_IIC = SET;
	}
	/* ************************************************************************** */
#endif	
	if(write_state_to_IIC == SET)
	{
		IIC_save_status();
		write_state_to_IIC = RESET;
	}

}


void	check_eeprom_option_buffer(void)
{
		option_buffer_check_flag = 1;
		liquid_temp_buf = (uint8_t)eeprom_option_byte[_Liquid_high_temp_stop];
		if((liquid_temp_buf < Liquid_high_temp_stop_l) || 
			(liquid_temp_buf > Liquid_high_temp_stop_h))
		{
			option_buffer_check_flag = 0;
			eeprom_option_byte[_Liquid_high_temp_stop] = Liquid_high_temp_stop;
		}
		/******************************************************************************************/
		liquid_temp_buf = (uint8_t)eeprom_option_byte[_Liquid_low_temp_stop];	
		if((liquid_temp_buf < Liquid_low_temp_stop_l) || 
			(liquid_temp_buf > Liquid_low_temp_stop_h))			
		{
			option_buffer_check_flag = 0;
			eeprom_option_byte[_Liquid_low_temp_stop] = Liquid_low_temp_stop;
		}
		/******************************************************************************************/
		if((eeprom_option_byte[_Liquid_high_temp_alarm] < Liquid_high_temp_alarm_l) || 
			(eeprom_option_byte[_Liquid_high_temp_alarm] > Liquid_high_temp_alarm_h))	
		{
			option_buffer_check_flag = 0;
			eeprom_option_byte[_Liquid_high_temp_alarm] = Liquid_high_temp_alarm;
		}
		/******************************************************************************************/
		if((eeprom_option_byte[_Liquid_low_temp_alarm] < Liquid_low_temp_alarm_l) || 
			(eeprom_option_byte[_Liquid_low_temp_alarm] > Liquid_low_temp_alarm_h))	
		{
			option_buffer_check_flag = 0;
			eeprom_option_byte[_Liquid_low_temp_alarm] = Liquid_low_temp_alarm;
		}
		/******************************************************************************************/
		if((eeprom_option_byte[_Set_temp_high] < Set_temp_high_l) || (eeprom_option_byte[_Set_temp_high] > Set_temp_high_h))	
		{
			option_buffer_check_flag = 0;
			eeprom_option_byte[_Set_temp_high] = Set_temp_high;
		}
		/******************************************************************************************/
		if((eeprom_option_byte[_Set_temp_low] < Set_temp_low_l) || 
			(eeprom_option_byte[_Set_temp_low] > Set_temp_low_h))	
		{
			option_buffer_check_flag = 0;
			eeprom_option_byte[_Set_temp_low] = Set_temp_low;
		}
		/******************************************************************************************/
		if((eeprom_option_byte[_Set_temp_high_1] < Set_temp_high_l_1) || 
			(eeprom_option_byte[_Set_temp_high_1] > Set_temp_high_h_1))	
		{
			option_buffer_check_flag = 0;
			eeprom_option_byte[_Set_temp_high_1] = Set_temp_high_1;
		}
		/******************************************************************************************/
		if((eeprom_option_byte[_Set_temp_low_1] < Set_temp_low_l_1) || 
			(eeprom_option_byte[_Set_temp_low_1] > Set_temp_low_h_1))	
		{
			option_buffer_check_flag = 0;
			eeprom_option_byte[_Set_temp_low_1] = Set_temp_low_1;	
		}
		/******************************************************************************************/
		if((eeprom_option_byte[_Heater_start_offset] < Heater_start_offset_l) || 
			(eeprom_option_byte[_Heater_start_offset] > Heater_start_offset_h))				
		{
			option_buffer_check_flag = 0;
			eeprom_option_byte[_Heater_start_offset] = Heater_start_offset;
		}
		/******************************************************************************************/
		if((eeprom_option_byte[_Heater_stop_offset] < Heater_stop_offset_l) || 
			(eeprom_option_byte[_Heater_stop_offset] > Heater_stop_offset_h))							
		{
			option_buffer_check_flag = 0;
			eeprom_option_byte[_Heater_stop_offset] = Heater_stop_offset;
		}
		/******************************************************************************************/
		if((eeprom_option_byte[_System_off_COMP_protect_timer_set] < System_off_COMP_protect_timer_set_l) || 
			(eeprom_option_byte[_System_off_COMP_protect_timer_set] > System_off_COMP_protect_timer_set_h))						
		{
			option_buffer_check_flag = 0;
			eeprom_option_byte[_System_off_COMP_protect_timer_set] = System_off_COMP_protect_timer_set;
		}
		/******************************************************************************************/
		if((eeprom_option_byte[_Control_sensor_offset] < Control_sensor_offset_l) || 
			(eeprom_option_byte[_Control_sensor_offset] > Control_sensor_offset_h))				
		{
			option_buffer_check_flag = 0;
			eeprom_option_byte[_Control_sensor_offset] = Control_sensor_offset;
		}
		/******************************************************************************************/
		if((eeprom_option_byte[_Base_sensor_offset] < Base_sensor_offset_l) || 
			(eeprom_option_byte[_Base_sensor_offset] > Base_sensor_offset_h))	
		{
			option_buffer_check_flag = 0;
			eeprom_option_byte[_Base_sensor_offset] = Base_sensor_offset;
		}
		/******************************************************************************************/
		if((eeprom_option_byte[_Flow_switch_start_check_time] < Flow_switch_start_check_time_l) || 
			(eeprom_option_byte[_Flow_switch_start_check_time] > Flow_switch_start_check_time_h))					
					eeprom_option_byte[_Flow_switch_start_check_time] = Flow_switch_start_check_time ;
		/******************************************************************************************/
		if((eeprom_option_byte[_LP_start_check_time] < LP_start_check_time_l) || 
			(eeprom_option_byte[_LP_start_check_time] > LP_start_check_time_h))			
		{
			option_buffer_check_flag = 0;
			eeprom_option_byte[_LP_start_check_time] = LP_start_check_time;
		}
		/******************************************************************************************/
		if((eeprom_option_byte[_System_on_PUMP_delay] < System_on_PUMP_delay_l) || 
			(eeprom_option_byte[_System_on_PUMP_delay] > System_on_PUMP_delay_h))				
		{
			option_buffer_check_flag = 0;
			eeprom_option_byte[_System_on_PUMP_delay] = System_on_PUMP_delay;
		}
		/******************************************************************************************/
		if((eeprom_option_byte[_System_on_FAN_delay] < System_on_FAN_delay_l) || 
			(eeprom_option_byte[_System_on_FAN_delay] > System_on_FAN_delay_h))				
		{
			option_buffer_check_flag = 0;
			eeprom_option_byte[_System_on_FAN_delay] = System_on_FAN_delay;
		}
		/******************************************************************************************/
		if((eeprom_option_byte[_System_on_COMP_start_delay] < System_on_COMP_start_delay_l) || 
			(eeprom_option_byte[_System_on_COMP_start_delay] > System_on_COMP_start_delay_h))		
		{
			option_buffer_check_flag = 0;
			eeprom_option_byte[_System_on_COMP_start_delay] = System_on_COMP_start_delay;
		}
		/******************************************************************************************/
		if((eeprom_option_byte[_System_off_PUMP_delay] < System_off_PUMP_delay_l) || 
			(eeprom_option_byte[_System_off_PUMP_delay] > System_off_PUMP_delay_h))					
		{
			option_buffer_check_flag = 0;
			eeprom_option_byte[_System_off_PUMP_delay] = System_off_PUMP_delay;
		}
		/******************************************************************************************/
		if((eeprom_option_byte[_System_off_FAN_delay] < System_off_FAN_delay_l) || 
			(eeprom_option_byte[_System_off_FAN_delay] > System_off_FAN_delay_h))						
		{
			option_buffer_check_flag = 0;
			eeprom_option_byte[_System_off_FAN_delay] = System_off_FAN_delay;
		}
		/******************************************************************************************/
		if((eeprom_option_byte[_Temp_control_FAN_delay_off] < Temp_control_FAN_delay_off_l) || 
			(eeprom_option_byte[_Temp_control_FAN_delay_off] > Temp_control_FAN_delay_off_h))			
		{
			option_buffer_check_flag = 0;
			eeprom_option_byte[_Temp_control_FAN_delay_off] = Temp_control_FAN_delay_off;
		}
		/******************************************************************************************/
		if((eeprom_option_byte[_Antifrz_protect_stop] < Antifrz_protect_stop_l) || 
			(eeprom_option_byte[_Antifrz_protect_stop] > Antifrz_protect_stop_h))						
		{
			option_buffer_check_flag = 0;
			eeprom_option_byte[_Antifrz_protect_stop] = Antifrz_protect_stop;
		}
		/******************************************************************************************/
		if((eeprom_option_byte[_Temp_control_COMP_protect_timer_set] < Temp_control_COMP_protect_timer_set_l) || 
			(eeprom_option_byte[_Temp_control_COMP_protect_timer_set] > Temp_control_COMP_protect_timer_set_h))						
		{
			option_buffer_check_flag = 0;
			eeprom_option_byte[_Temp_control_COMP_protect_timer_set ] = Temp_control_COMP_protect_timer_set;
		}
		/******************************************************************************************/
		if((eeprom_option_byte[_COMP_start_temp_offset] < COMP_start_temp_offset_l) || 
			(eeprom_option_byte[_COMP_start_temp_offset] > COMP_start_temp_offset_h))				
		{
			option_buffer_check_flag = 0;
			eeprom_option_byte[_COMP_start_temp_offset] = COMP_start_temp_offset;
		}
		/******************************************************************************************/
		if((eeprom_option_byte[_System_off_PMV_delay] < System_off_PMV_delay_l) || 
			(eeprom_option_byte[_System_off_PMV_delay] > System_off_PMV_delay_h))			
		{
			option_buffer_check_flag = 0;
			eeprom_option_byte[_System_off_PMV_delay] = System_off_PMV_delay;
		}
		/******************************************************************************************/
		if((eeprom_option_byte[_Modbus_address_offset] < Modbus_address_min) || 
			(eeprom_option_byte[_Modbus_address_offset] > Modbus_address_max))			
		{
			option_buffer_check_flag = 0;
			eeprom_option_byte[_Modbus_address_offset] = Modbus_address_offset;
		}
		/******************************************************************************************/			
		if((eeprom_option_byte[_PID_P_factor_value] < PID_P_factor_min) || 
			(eeprom_option_byte[_PID_P_factor_value] > PID_P_factor_max))			
		{
			option_buffer_check_flag = 0;
			eeprom_option_byte[_PID_P_factor_value] = PID_P_factor_def;
		}
		/******************************************************************************************/	
		if((eeprom_option_byte[_PID_I_factor_value] < PID_I_factor_min) || 
			(eeprom_option_byte[_PID_I_factor_value] > PID_I_factor_max))			
		{
			option_buffer_check_flag = 0;
			eeprom_option_byte[_PID_I_factor_value] = PID_I_factor_def;
		}
		/******************************************************************************************/	
		if((eeprom_option_byte[_PID_D_factor_value] < PID_D_factor_min) || 
			(eeprom_option_byte[_PID_D_factor_value] > PID_D_factor_max))			
		{
			option_buffer_check_flag = 0;
			eeprom_option_byte[_PID_D_factor_value] = PID_D_factor_def;
		}
		/******************************************************************************************/				
}


/* *******************  IIC code area ***************************************** */
void	init_eeprom_option_buffer(void)
{
		eeprom_option_byte[_Liquid_high_temp_stop] = Liquid_high_temp_stop;
		eeprom_option_byte[_Liquid_low_temp_stop] = Liquid_low_temp_stop;
		eeprom_option_byte[_Liquid_high_temp_alarm] = Liquid_high_temp_alarm;	
		eeprom_option_byte[_Liquid_low_temp_alarm] = Liquid_low_temp_alarm;
		eeprom_option_byte[_Set_temp_high] = Set_temp_high;
		eeprom_option_byte[_Set_temp_low] = Set_temp_low;
		eeprom_option_byte[_Set_temp_high_1] = Set_temp_high_1;
		eeprom_option_byte[_Set_temp_low_1] = Set_temp_low_1;	
		eeprom_option_byte[_Heater_start_offset] = Heater_start_offset;
		eeprom_option_byte[_Heater_stop_offset] = Heater_stop_offset;
		eeprom_option_byte[_System_off_COMP_protect_timer_set ] = System_off_COMP_protect_timer_set ;
		eeprom_option_byte[_Control_sensor_offset] = Control_sensor_offset;
		eeprom_option_byte[_Base_sensor_offset] = Base_sensor_offset;
		eeprom_option_byte[_Flow_switch_start_check_time ] = Flow_switch_start_check_time ;
		eeprom_option_byte[_LP_start_check_time] = LP_start_check_time;
		eeprom_option_byte[_System_on_PUMP_delay] = System_on_PUMP_delay;
		eeprom_option_byte[_System_on_FAN_delay] = System_on_FAN_delay;
		eeprom_option_byte[_System_on_COMP_start_delay] = System_on_COMP_start_delay;
		eeprom_option_byte[_System_off_PUMP_delay] = System_off_PUMP_delay;
		eeprom_option_byte[_System_off_FAN_delay] = System_off_FAN_delay;
		eeprom_option_byte[_Condenser_temp_protect_value] = Condenser_temp_protect;
		eeprom_option_byte[_Antifrz_protect_stop] = Antifrz_protect_stop;
		eeprom_option_byte[_System_off_PMV_delay] = System_off_PMV_delay;
		eeprom_option_byte[_comp_current_protect_0_value] = comp_current_protect_0_def;
		eeprom_option_byte[_comp_current_protect_1_value] = comp_current_protect_1_def;
		eeprom_option_byte[_comp_current_protect_2_value] = comp_current_protect_2_def;
		eeprom_option_byte[_comp_current_protect_3_value] = comp_current_protect_3_def;
		eeprom_option_byte[_Modbus_address_offset] = Modbus_address_offset;
		eeprom_option_byte[_COMP_start_temp_offset] = COMP_start_temp_offset;
		eeprom_option_byte[_Temp_control_COMP_protect_timer_set ] = Temp_control_COMP_protect_timer_set ;
		eeprom_option_byte[_Temp_control_FAN_delay_off] = Temp_control_FAN_delay_off;
		eeprom_option_byte[_PID_P_factor_value] = PID_P_factor_def;
		eeprom_option_byte[_PID_I_factor_value] = PID_I_factor_def;
		eeprom_option_byte[_PID_D_factor_value] = PID_D_factor_def;
}


/* *******************  IIC code area ***************************************** */


