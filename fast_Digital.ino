#include "fast_Digital.h"

MMIO<(uint16_t)&PORTD,Bit_Mask>
#undef PORTD
PORTD;
MMIO<(uint16_t)&DDRD,Bit_Mask>
#undef DDRD
DDRD;
MMIO<(uint16_t)&PIND,Bit_Mask>
#undef PIND
PIND;

MMIO<(uint16_t)&TWCR,Bit_Mask>
#undef TWCR
TWCR;
MMIO<(uint16_t)&TWSR,Bit_Mask>
#undef TWSR
TWSR;
MMIO<(uint16_t)&TWBR,Bit_Mask>
#undef TWBR
TWBR;
MMIO<(uint16_t)&TWDR,Bit_Mask>
#undef TWDR
TWDR;




void setup()
{
	DDRD	=	_B{0,3,5,7};
	PORTD	=	PORTD ^ 0xf0;
	PORTD	=	~PORTD;
	PORTD->b0 = true;
	PORTD	^=	0xff;
	PORTD	=	PORTD + 23;
	if( PORTD && false ){
		PORTD << _B{2,3};
	}

}

void loop()
{
}
