#include <fast_Digital.h>

// Pin values are chosen using a pin map (google it).
// This example is for the Ardiono Mini: http://i.stack.imgur.com/oOtkU.png

// Feel free to structure and group your pins, with no space cost!
struct pins{
  
// Pins for constrolling LEDs
struct LED{
  // A green LED attached to "digital" 8 (PCINT8)
  using green = PIN<'C',0>;//labeled "PC0"
  // A blue LED attached to "digital" 10 (PCINT10)
  using blue = PIN<'C',2>;//labeled "PC2"
};

// Pins for constrolling two stepper motors
struct MOTOR{
  // pins for controlling the left motor
  using left = PORT<'D'>;//digital {PCINT16,PCINT17,PCINT18,PCINT19,...}
  // pins for controlling the right motor
  using right = PORT<'D'>;//digital {...,PCINT20,PCINT21,PCINT22,PCINT23}
  // pins for controlling the both motors
  using both = PORT<'D'>;
};
};


void setup() {
  // set LED pins as OUTPUT
  pins::LED::green::mode( OUTPUT );
  pins::LED::blue::mode( OUTPUT );

  // set green LED pins as HIGH
  pins::LED::green::write(true);
  // set blue LED pins as HIGH
  pins::LED::blue::write(false);

  // set all of MOTORs pins at OUTPUT
  pins::MOTOR::both::mode( OUTPUT );

}

void loop() {
  // put your main code here, to run repeatedly:
	pins::LED::green::toggle();
	pins::LED::blue::toggle();

	move(1000);

	turn(480);

	delay(1000);

	pins::LED::green::toggle();
	pins::LED::blue::toggle();

	move(-1000);

	turn(-480);

	delay(1000);

}

struct motor{
	const Bit_Mask step_masks[4] = { B10011001, B10101010, B01100110, B01010101 };
	unsigned step_index : 2;
} LEFT,RIGHT;

void inline move(int steps){
	if(steps>0){
		for(;steps!=0;--steps){
			pins::MOTORS::both::write((LEFT.step_mask[LEFT.step_index++]&0x0f) | (RIGHT.step_mask[RIGHT.step_index++]&0xf0));
		}
	}else
	if(steps<0){
		for(;steps!=0;++steps){
			pins::MOTORS::both::write((LEFT.step_mask[LEFT.step_index--]&0x0f) | (RIGHT.step_mask[RIGHT.step_index--]&0xf0));
		}
	}else
}

void inline turn(int clockwise){
	if(clockwise>0){
		for(;clockwise!=0;--clockwise){
			pins::MOTORS::both::write((LEFT.step_mask[LEFT.step_index++]&0x0f) | (RIGHT.step_mask[RIGHT.step_index--]&0xf0));
		}
	}else
	if(clockwise<0){
		for(;clockwise!=0;++clockwise){
			pins::MOTORS::both::write((LEFT.step_mask[LEFT.step_index--]&0x0f) | (RIGHT.step_mask[RIGHT.step_index++]&0xf0));
		}
	}else
}

