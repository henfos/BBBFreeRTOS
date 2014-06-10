BBBFreeRTOS
===========

FreeRTOS for BeagleBone Black


----------------------
Makefile located in in Demo/AM3359_BeagleBone_GCC/

Also includes working MLO and u-boot, stage 1 and 2 bootloaders.

Working:
- System tick
- Interrupts
- GPIO
- Output serial

Not finished:
- An ISR routine for serial input //Should however be no problem as core FreeRTOS is fully functioning 

-----------------------
Test main.c file
Sets up GPIO interrupts for three input pins. Waits for interrupts, then responds on three other output pins.

Use the AM335x technical reference manual to look up the registers used.

