#include "fast_Digital.h"

#include <Arduino.h>

#define LHS PORTD
#define RHS 42

#define DATATYPE Bit_Mask

REG(PORTD,Bit_Mask)
#undef PORTD
PORTD;

REG(PIND,Bit_Mask)
#undef PIND
PIND;
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

void inline print_pin_registers(){
	for(unsigned i=0; i < NUM_DIGITAL_PINS;++i){
		Serial.print( String(F("\n Digital PIN ")) + String(i) + F(" is ") );
		switch(digitalPinToPort(i)){
		case NOT_A_PIN: Serial.print(F("NOT a pin.")); break;
		default: Serial.print( String(F("PORT")) + char(digitalPinToPort(i)-1+'A') ); break;
		}
	}
}
