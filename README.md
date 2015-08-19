# fast_Digital
>A fast, compile-time, port-manipulation (MMIO) library -- intended to replace the Arduino library's `wiring_digital.c` family of functions.


Motivation
----
This class was created as a part of my Sr. Design project. I was working with the Atmel 328p and finding that the provided (via Arduino) pin manipulation functions were not fast enough to meet the timing constraints of the project. Due to budget and PCB size limitations, we could not use a faster processor. 

Upon inspection of the `wiring_digital.c` function library -- which includes `digitalRead()` , `digitalWrite()` , and `pinMode()` (among others) -- I found that most of the performance overhead was consumed with run-time configuration and checking operations. I sought to remove this limitation by moving those operations to compile-time through the use of meta-template programming techniques.

Design
---
The `wiring_digital.c` function library is essentially backed by the AVR port-manipulation function macros `DDRX` and `PORTX`, where `X` is the letter designation of the port. These functions are as maximally fast as possible, but cumbersome to use because they require a more intensive knowledge of the underlying hardware. This also makes code written with these function non-portable, as different microprocessors have different port maps. To solve this problem, the nice folks at Arduino abstracted this problem with functions and look-up tables that allow code to be portable by figuring out what port to manipulate at run-time. The problem here is that if the provided pin number is incorrect (e.g. `digitalWrite(-1,HIGH)`), an invalid index of the look-up table will be used. To prevent this, `digitalWrite()` performs a number of input checks (at run-time) and returns an error value if they fail. All of this takes extra time.

My library works on all the same principles as the Arduino library, it even uses the same reference tables. The significant difference here is that all of the input checking happens at compile-time. As such, if there is an invalid input, the result is a **compiler error** instead of mysterious run-time behavior. On top of this, a large number of convenience features were added to aid in common port/pin manipulation tasks -- all of which perform their calculations at compile-time. 


Development
---
After writing this library, it went on to be used in a number of other projects; each of these influenced its continual development and revision.

For example, a small [stepper motor](https://gist.github.com/lylemoffitt/c8b70259ea1f622f1edd) control library used this project to optimize phase transitions sequences. It inspired me to add the ability to use bit-masks to manipulate ports and to be able to specify those masks in binary notation.

Later on, to ensure cross-platform stability, a series of compile-time checks was added to verify that the library was always compiling as expected. If not, compile-time error messages are generated. 

Documentation
---
All methods and classes are documented doxy-style. For further information check out [doxygen](www.doxygen.org).
