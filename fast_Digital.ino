#include "fast_Digital.h"

#define LHS PORTD
#define RHS 42

#define DATATYPE Bit_Mask

REG(PORTD,Bit_Mask)
#undef PORTD
PORTD;



void setup()
{
	PORTD	=	_B{0,3,5,7};

	PORTD	=	~PORTD;

	PORTD->b0 = true;
	PORTD->upper_nibble = 0xF;

	PORTD	=	LHS +  RHS;
	PORTD	=	LHS -  RHS;
	PORTD	=	LHS *  RHS;
	PORTD	=	LHS /  RHS;
	PORTD	=	LHS %  RHS;
	PORTD	=	LHS ^  RHS;
	PORTD	=	LHS &  RHS;
	PORTD	=	LHS |  RHS;
	PORTD	=	LHS << RHS;
	PORTD	=	LHS >> RHS;

	PORTD	+=	RHS;
	PORTD	-=	RHS;
	PORTD	*=	RHS;
	PORTD	/=	RHS;
	PORTD	%=	RHS;
	PORTD	&=	RHS;
	PORTD	|=	RHS;
	PORTD	^=	RHS;
	PORTD	<<=	RHS;
	PORTD	>>=	RHS;


	if( PORTD && false );
	if( !PORTD && false );

}

void loop()
{
}
