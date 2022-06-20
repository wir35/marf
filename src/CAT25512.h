#ifndef __CAT25512_H
#define __CAT25512_H

#include <stdint.h>

/* Instruction list for CAT25512 */
#define INTSRUCTION_WREN	0x06
#define INTSRUCTION_WRDI	0x04
#define INTSRUCTION_RDSR	0x05	//Read status register
#define INTSRUCTION_WRSR	0x01	//Write status register
#define INTSRUCTION_READ	0x03
#define INTSRUCTION_WRITE	0x02

/* Status Register bits define for CAT25512 */
#define SR_WPEN		0x80	//Write Protect Enable
#define SR_IPL		0x40	//Identification Page Latch
#define SR_LIP		0x10	//Lock Identification page
#define SR_BP1		0x08	//Block protect 1 bit
#define SR_BP0		0x04  //Block protect 0 bit
#define SR_WEL		0x02	//Write Enable Latch
#define SR_RDY		0x01	//Ready - 1 during write cycle, 0- when ready to write

#define CAT25512_CS_SET		GPIO_ResetBits(GPIOD, GPIO_Pin_2)
#define CAT25512_CS_CLEAR	GPIO_SetBits(GPIOD, GPIO_Pin_2)

void CAT25512_init(void);

void CAT25512_write_block(uint16_t address, uint8_t* data, uint16_t size);

void CAT25512_read_block(uint16_t address, uint8_t* data, uint16_t size);

void CAT25512_erase();


#endif //__CAT25512_H
