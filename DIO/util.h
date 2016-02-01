/*
 * util.h
 *
 *  Created on: Jan 30, 2016
 *      Author: MANDO
 */

#ifndef UTIL_H_
#define UTIL_H_

#define GetBit(Reg,BitNo) (Reg&(1<<BitNo))>>BitNo
#define ClrBit(Reg,BitNo)  (Reg&~(1<<BitNo)
#define SetBit(Reg,BitNo)  (Reg | (1<<BitNo)
#define ToggelBit(Reg,BitNo) (Reg^(1<<BitNo))
#define SetReg(Reg) (Reg = 0xff)
#define ClrReg(Reg) (Reg = 0x00;)
#define ToggleReg(leg) =~ Reg
#define AssignReg(Reg,value) Reg = value
#define GetLowNibble(Reg) Reg & 0x0F
#define ClrLowNibbles(Reg)Reg & 0xF0
#define SetLowNibble(Reg) Reg|0x0F
#define ToggleLowNibble(Reg) Reg^0x0F
#define AssignLowNibble(Reg,value) Reg&(0xF0)|value)
#define GetHighNibble(Reg) (Reg & 0xF0)>>4
#define ClrHighNibbles(Reg)Reg & 0xF0
#define SetHighNibble(Reg) Reg|0xF0
#define ToggleHighNibble(Reg) Reg^0xF0
#define AssignHighNibble(Reg,value) Reg&0xF0>>4|value

#endif /* UTIL_H_ */
