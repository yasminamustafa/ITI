/*
 * DIO_PROG.c

 *
 *  Created on: Jan 27, 2016
 *      Author: MANDO
 */

#include "types.h"
#include "util.h"
#include "DIO_interface.h"
#include "DIO_config.h"
#include "DIO_private.h"


#define PORTA  (*(volatile u8*)(0x3B))
#define PORTB  (*(volatile u8*)(0x38))
#define PORTC  (*(volatile u8*)(0x35))
#define PORTD  (*(volatile u8*)(0x32))



#define DDRA  (*(volatile u8*)(0x3A))
#define DDRB  (*(volatile u8*)(0x37))
#define DDRC  (*(volatile u8*)(0x34))
#define DDRD  (*(volatile u8*)(0x31))



#define DIO_u8PORTADIR 		CONC(DIO_u8PIN0DIR,DIO_u8PIN1DIR,DIO_u8PIN2DIR,DIO_u8PIN3DIR,DIO_u8PIN4DIR,DIO_u8PIN5DIR,DIO_u8PIN6DIR,DIO_u8PIN7DIR)
#define DIO_u8PORTADIR 		CONC(DIO_u8PIN0DIR,DIO_u8PIN1DIR,DIO_u8PIN10DIR,DIO_u8PIN11DIR,DIO_u8PIN12DIR,DIO_u8PIN13DIR,DIO_u8PIN14DIR,DIO_u8PIN15DIR)
#define DIO_u8PORTADIR 		CONC(DIO_u8PIN0DIR,DIO_u8PIN1DIR,DIO_u8PIN2DIR,DIO_u8PIN3DIR,DIO_u8PIN20DIR,DIO_u8PIN21DIR,DIO_u8PIN22DIR,DIO_u8PIN23DIR)
#define DIO_u8PORTADIR 		CONC(DIO_u8PIN0DIR,DIO_u8PIN1DIR,DIO_u8PIN2DIR,DIO_u8PIN3DIR,DIO_u8PIN28DIR,DIO_u8PIN29DIR,DIO_u8PIN30DIR,DIO_u8PIN31DIR)

#define DIO_u8PORTAINIT 	CONC(DIO_u8PIN0_INITVAL,DIO_u8PIN1_INITVAL,DIO_u8PIN2_INITVAL,DIO_u8PIN3_INITVAL,DIO_u8PIN4_INITVAL,DIO_u8PIN5_INITVAL,DIO_u8PIN6_INITVAL,DIO_u8PIN7_INITVAL)
#define DIO_u8PORTAINIT 	CONC(DIO_u8PIN8_INITVAL,DIO_u8PIN9_INITVAL,DIO_u8PIN10_INITVAL,DIO_u8PIN11_INITVAL,DIO_u8PIN12_INITVAL,DIO_u8PIN13_INITVAL,DIO_u8PIN14_INITVAL,DIO_u8PIN15_INITVAL)
#define DIO_u8PORTAINIT 	CONC(DIO_u8PIN16_INITVAL,DIO_u8PIN17_INITVAL,DIO_u8PIN18_INITVAL,DIO_u8PIN19_INITVAL,DIO_u8PIN20_INITVAL,DIO_u8PIN21_INITVAL,DIO_u8PIN22_INITVAL,DIO_u8PIN23_INITVAL)
#define DIO_u8PORTAINIT 	CONC(DIO_u8PIN24_INITVAL,DIO_u8PIN25_INITVAL,DIO_u8PIN26_INITVAL,DIO_u8PIN27_INITVAL,DIO_u8PIN28_INITVAL,DIO_u8PIN29_INITVAL,DIO_u8PIN30_INITVAL,DIO_u8PIN31_INITVAL)


extern u8 DIO_u8ReadPinVal(u8 Copy_u8PinIdx,u8* Copy_u8PtrToVal);
extern u8 DIO_u8WritePinVal(u8 Copy_u8PinIdx,u8 Copy_u8PinVal);
extern u8 DIO_u8ReadPortVal(u8 Copy_u8PortIdx,u8* Copy_u8PtrToVal);
extern u8 DIO_u8WritePortVal(u8 Copy_u8PortIdx,u8 Copy_u8PortVal);
extern u8 DIO_u8WritePinDir(u8 Copy_u8PinIdx,u8 Copy_u8PinDir);
extern u8 DIO_u8WritePortDir(u8 Copy_u8PortIdx,u8 Copy_u8PortDir);
extern u8 DIO_u8ReadPinDir(u8 Copy_u8PinIdx,u8* Copy_u8PtrToDir);
extern u8 DIO_u8ReadPortDir(u8 Copy_u8PortIdx,u8* Copy_u8PtrToDir);
extern void DIO_voidInit(void);


extern void DIO_voidInit(void)
{
DDRA = DIO_u8PORTADIR;
DDRB = DIO_u8PORTBDIR;
DDRC = DIO_u8PORTCDIR;
DDRD = DIO_u8PORTADIR;



PORTA| = DIO_u8PORTADIR & DIO_u8PORTAVAL;
PORTB| = DIO_u8PORTADIR & DIO_u8PORTAVAL;
PORTC| = DIO_u8PORTADIR & DIO_u8PORTAVAL;
PORTD| = DIO_u8PORTADIR & DIO_u8PORTAVAL;
}




extern u8 DIO_u8ReadPinVal(u8 Copy_u8PinIdx,u8* Copy_u8PtrToVal)
{


	u8 local_u8port_value = Copy_u8PinIdx /NO_OF_PINS;
	u8 local_u8pin_value = Copy_u8PinIdx % NO_OF_PINS;

switch(local_u8port_value) // switch case for ports

	{

	case DIO_u8PORT0:
		 Copy_u8PtrToVal = GetBit(PORTA,local_u8pin_value);
		 * Copy_u8PtrToVal = OK;
		 break;
	case DIO_u8PORT1:
		 Copy_u8PtrToVal = GetBit(PORTB,local_u8pin_value);
		 * Copy_u8PtrToVal = OK;
		 break;
	case DIO_u8PORT2:
		 Copy_u8PtrToVal = GetBit(PORTC,local_u8pin_value);
		 * Copy_u8PtrToVal = OK;
		 break;
	case DIO_u8PORT3:
		 Copy_u8PtrToVal = GetBit(PORTD,local_u8pin_value);
	     * Copy_u8PtrToVal = OK;
		break;

	default:
		* Copy_u8PtrToVal = ERROR;
	}
return  Copy_u8PtrToVal;
}
/////////////////////////////////////




extern u8 DIO_u8WritePinVal(u8 Copy_u8PinIdx,u8 Copy_u8PinVal)
{

	u8 local_u8port_value = Copy_u8PinIdx /NO_OF_PINS;
	u8 local_u8pin_value = Copy_u8PinIdx % NO_OF_PINS;

if(Copy_u8PinIdx >= 0 && Copy_u8PinIdx < 32)
{
	switch(local_u8port_value) // switch case for ports

		{

		case DIO_u8PORT0:
			  SetBit(PORTA,local_u8pin_value);

			 break;
		case DIO_u8PORT1:
			  SetBit(PORTB,local_u8pin_value);

			 break;
		case DIO_u8PORT2:
			 SetBit(PORTC,local_u8pin_value);

			 break;
		case DIO_u8PORT3:
			 SetBit(PORTD,local_u8pin_value);

			break;

		default:
		}
	}
else
{
	return Error;
}
}


extern u8 DIO_u8ReadPortVal(u8 Copy_u8PortIdx,u8* Copy_u8PtrToVal)
{

	u8 local_u8port_value = Copy_u8PinIdx /NO_OF_PINS;
	u8 local_u8pin_value = Copy_u8PinIdx % NO_OF_PINS;

	switch(port_value) // switch case for ports

		{

		case DIO_u8PORT0:
			 Copy_u8PtrToVal = GetBit(PORTA,local_u8port_value);
			 * Copy_u8PtrToVal = OK;
			 break;
		case DIO_u8PORT1:
			 Copy_u8PtrToVal = GetBit(PORTB,local_u8port_value);
			 * Copy_u8PtrToVal = OK;
			 break;
		case DIO_u8PORT2:
			 Copy_u8PtrToVal = GetBit(PORTC,local_u8port_value);
			 * Copy_u8PtrToVal = OK;
			 break;
		case DIO_u8PORT3:
			 Copy_u8PtrToVal = GetBit(PORTD,local_u8port_value);
		     * Copy_u8PtrToVal = OK;
			break;

		default:
			* Copy_u8PtrToVal = ERROR;
		}
	return  Copy_u8PtrToVal;
	}




extern u8 DIO_u8WritePortVal(u8 Copy_u8PortIdx,u8 Copy_u8PortVal)
{


	u8 local_u8port_value = Copy_u8PinIdx /NO_OF_PINS;
	u8 local_u8pin_value = Copy_u8PinIdx % NO_OF_PINS;
	switch(local_u8port_value) // switch case for ports

			{

			case DIO_u8PORT0:
				 Copy_u8PortVal = SetBit(PORTA,local_u8port_value);
			     break;

			case DIO_u8PORT1:
				 Copy_u8PortVal = SetBit(PORTB,local_u8port_value);
				 break;

			case DIO_u8PORT2:
				 Copy_u8PortVal = SetBit(PORTC,local_u8port_value);
			     break;

			case DIO_u8PORT3:
				 Copy_u8PortVal = SetBit(PORTD,local_u8port_value);
				 break;

			default:
			}
}

extern u8 DIO_u8WritePinDir(u8 Copy_u8PinIdx,u8 Copy_u8PinDir)

{
	u8 local_u8port_value = Copy_u8PinIdx /NO_OF_PINS;
	u8 local_u8pin_value = Copy_u8PinIdx % NO_OF_PINS;

	switch(local_u8port_value) // switch case for ports

				{

				case DIO_u8PORT0:
					 Copy_u8PinDir = SetBit(DDRA,local_u8pin_value);
				     break;

				case DIO_u8PORT1:
					 Copy_u8PinDir = SetBit(DDRB,local_u8pin_value);
					 break;

				case DIO_u8PORT2:
					 Copy_u8PinDir = SetBit(DDRCC,local_u8pin_value);
				     break;

				case DIO_u8PORT3:
					 Copy_u8PinDir = SetBit(DDRD,local_u8pin_value);
					 break;

				default:
				}
}


extern u8 DIO_u8WritePortDir(u8 Copy_u8PortIdx,u8 Copy_u8PortDir)
{


	u8 local_u8port_value = Copy_u8PinIdx /NO_OF_PINS;
	u8 local_u8pin_value = Copy_u8PinIdx % NO_OF_PINS;

	switch(local_u8port_value) // switch case for ports

				{

				case DIO_u8PORT0:
					 Copy_u8PortDir = SetBit(DDRA,local_u8port_value);
				     break;

				case DIO_u8PORT1:
					 Copy_u8PortDir = SetBit(DDRB,local_u8port_value);
					 break;

				case DIO_u8PORT2:
					 Copy_u8PortDir = SetBit(DDRCC,local_u8port_value);
				     break;

				case DIO_u8PORT3:
					 Copy_u8PortDir = SetBit(DDRD,local_u8port_value);
					 break;

				default:
				}
}


extern u8 DIO_u8ReadPinDir(u8 Copy_u8PinIdx,u8* Copy_u8PtrToDir)

{
	u8 local_u8port_value = Copy_u8PinIdx /NO_OF_PINS;
	u8 local_u8pin_value = Copy_u8PinIdx % NO_OF_PINS;

		switch(local_u8port_value) // switch case for ports

			{

			case DIO_u8PORT0:
				 Copy_u8PtrToDir = GetBit(DDRA,local_u8pin_value);
				 * Copy_u8PtrToDir = OK;
				 break;
			case DIO_u8PORT1:
				 Copy_u8PtrToDir = GetBit(DDRB,local_u8pin_value);
				 * Copy_u8PtrToDir = OK;
				 break;
			case DIO_u8PORT2:
				 Copy_u8PtrToDir = GetBit(DDRC,local_u8pin_value);
				 * Copy_u8PtrToDir = OK;
				 break;
			case DIO_u8PORT3:
				 Copy_u8PtrToDir = GetBit(DDRD,local_u8pin_value);
			     * Copy_u8PtrToDir = OK;
				break;

			default:
				* Copy_u8PtrToDir = ERROR;
			}
		return  Copy_u8PtrToDir;

}


extern u8 DIO_u8ReadPortDir(u8 Copy_u8PortIdx,u8* Copy_u8PtrToDir)

{

	u8 local_u8port_value = Copy_u8PinIdx /NO_OF_PINS;
	u8 local_u8pin_value = Copy_u8PinIdx % NO_OF_PINS;

			switch(local_u8port_value) // switch case for ports

				{

				case DIO_u8PORT0:
					 Copy_u8PtrToDir = GetBit(DDRA,local_u8port_value);
					 * Copy_u8PtrToVal = OK;
					 break;
				case DIO_u8PORT1:
					 Copy_u8PtrToDir = GetBit(DDRB,local_u8port_value);
					 * Copy_u8PtrToVal = OK;
					 break;
				case DIO_u8PORT2:
					 Copy_u8PtrToDir = GetBit(DDRC,local_u8port_value);
					 * Copy_u8PtrToVal = OK;
					 break;
				case DIO_u8PORT3:
					 Copy_u8PtrToDir = GetBit(DDRD,local_u8port_value);
				     * Copy_u8PtrToVal = OK;
					break;

				default:
					* Copy_u8PtrToDir = ERROR;
				}
			return  Copy_u8PtrToDir;


}

