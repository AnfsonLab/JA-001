#include "flash.h"
#include "pfdl.h"
#include "pfdl_types.h"

/***********************************************************************************************************************
* Function Name: Dataflash_Open
* Description  : None 
* Arguments    : freq : System clock frequency
                 mode : flash mode
* Return Value : None
***********************************************************************************************************************/
void Dataflash_Open(unsigned char freq,unsigned char mode){
    pfdl_descriptor_t descriptor;
    
    descriptor.fx_MHz_u08 = freq;
    descriptor.wide_voltage_mode_u08 = mode;
    PFDL_Open(&descriptor);
}

/***********************************************************************************************************************
* Function Name: Dataflash_Close
* Description  : None 
* Arguments    : None
* Return Value : None
***********************************************************************************************************************/
void Dataflash_Close(void){
    PFDL_Close();
}

/***********************************************************************************************************************
* Function Name: Dataflash_Erase
* Description  : None 
* Arguments    : block : erase block
* Return Value : None
***********************************************************************************************************************/
unsigned char Dataflash_Erase(unsigned char block){
    pfdl_request_t request;
    unsigned char ret;
    
    request.command_enu = PFDL_CMD_ERASE_BLOCK;
    request.index_u16 = block;
    ret = PFDL_Execute(&request);
    
    while (ret == PFDL_BUSY)
    {
        ret = PFDL_Handler();             /* Status check process */
    }
    
    return (ret);
}

/***********************************************************************************************************************
* Function Name: Dataflash_Write
* Description  : None 
* Arguments    : addr : Write address
                 size : write size (byte)
		 *data : data to be write
* Return Value : None
***********************************************************************************************************************/
unsigned char Dataflash_Write(unsigned short addr,
   				 unsigned char size,
				     unsigned char *data){
    pfdl_request_t request;
    unsigned char ret;
    
    request.command_enu   = PFDL_CMD_WRITE_BYTES;
    request.index_u16     = (addr - 0xF1000);
    request.bytecount_u16 = size;
    request.data_pu08     = data; 
    
    ret = PFDL_Execute(&request);
    
    while (ret == PFDL_BUSY)
    {
        ret = PFDL_Handler();                                                          /* Status check process */
    }

    return (ret);
}

/***********************************************************************************************************************
* Function Name: Dataflash_Read
* Description  : None 
* Arguments    : addr : Read address
                 size : Read size (byte)
		 *data : Read data buffer
* Return Value : None
***********************************************************************************************************************/
unsigned char Dataflash_Read(unsigned short addr,
                                 unsigned char size,
				     unsigned char *data){
    pfdl_request_t request;
    unsigned char ret;
    
    request.command_enu   = PFDL_CMD_READ_BYTES;
    request.index_u16     = (addr - 0xF1000);
    request.bytecount_u16 = size;
    request.data_pu08     = data;
    
    ret = PFDL_Execute(&request);
    
    return (ret);				     
}


