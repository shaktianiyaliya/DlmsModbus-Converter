The SystemInit function in the <system_LPC17xx.c> file sets the 
Vector Table Offset Register, "VTOR", to point to flash address 0 if not the 
define __RAM_MODE__ has been set. 

If code has been located to address 0 in Flash this works fine and nothing 
needs to be done with project settings. 

If code has been located to RAM and interrupts are used the 
define  __RAM_MODE__ needs to be defined when building the project. Otherwise
the interrupt table located in flash will be used. 
 
The define can be set by adding the __RAM_MODE__ symbol in the C/C++ build 
settings for the project. 
