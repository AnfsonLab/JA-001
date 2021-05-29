#ifndef FLASH_H
#define	FLASH_H

#define	SYSTEM_FREQ			(16)
#define	DF_PROG_FULL_SPEED_MODE		(00)
#define	DF_PROG_WIDE_VOLTATE_MODE	(01)

void Dataflash_Open(unsigned char freq,unsigned char mode);
void Dataflash_Close(void);
unsigned char Dataflash_Erase(unsigned char block);
unsigned char Dataflash_Write(unsigned short addr,
   			          unsigned char size,
			              unsigned char *data);
unsigned char Dataflash_Read(unsigned short addr,
                                 unsigned char size,
				     unsigned char *data);

#endif