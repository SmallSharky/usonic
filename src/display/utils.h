#pragma once


#define		SetBit(reg, x)		reg |= (1<<x)
#define		ClearBit(reg, x)	reg &= (~(1<<x))
#define		InvBit(reg, x)		reg ^= (1<<x)
#define		BitIsSet(reg, x)	((reg & (1<<x)) != 0)
#define		BitIsClear(reg, x)	((reg & (1<<x)) == 0)
#define		bit(x)				(1 << (x))


#define		hibyte(x)			(uint8_t)((x>>8) & 0xFF)
#define		lobyte(x)			(uint8_t)((x) & 0xFF)


