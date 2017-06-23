# UM17 ETC

## Introduction
This is the ETC controller program for the UVigo Motorsport UM17 car. It is written for an Arduino Nano or compatible, but should be relatively easy to move to other platforms.

All settings that are changable save to EEPROM and are persistant throughout resets.

## Arduino specific libraries
The ETC leans heavily on the Servo, Serial and EEPROM libraries of Arduino. These should be portable to most ATMegas but care should be taken.

It may be ported to ATTiny84, watch this space.

## Wiring
**D2**: Emerg-High  
**D3**: Emerg-Low  
**A0**: APPS1  
**A1**: APPS2  
**A2**: TPS1  
**A3**: TPS2  
**A4**: BPS  
**D11**: Cut input (provisional)  
**D12**: Blip input (provisonal)  

## Normal use
The ETC will set the throttle to idle on start up. Everything else is rather intutive - press pedal to open, let go to close. Freaks out appropriately according to the 2017 Formula Student Germany rules for electric throttles.

## Commands
The ETC controller has a set of commands for adjusting various elements of the ETC in-situ. They are described below. Two modes exist - Normal and Set. Normal mode is the normal mode of functioning - throttle follows pedal. Set mode is to change settings - the controller is stuck in a loop and only responds to serial input.

### How to send commands
Use the arduino serial monitor or any other qualified RS-232 terminal emulator and send 1 character at a time. The software does not accept any numbers. It will accept a chain of characers (say, 20 +s in a row) quite well should you need to make big changes.

### Normal mode
**s**: Enter set mode.  

### Set mode
There are two types of setting - point set and range set.  

#### Point set
Point set is for setting a specific limit or position, and always has the same set of commands:  
**+**: Increase the control variable, usually microseconds of the servo.  
**-**: Decrease the control variable, usually microseconds of the servo.  
**q**: Quit.  

#### Range set
Range set requires you to set, at minimum, the minimum and maximum of the range. Generally, this type of setting only has 1 command:  
**d**: Save the current control variable value and move to the next.

#### Set mode commands
**i**: Set idle position of throttle as a percentage of the throttle's total range. **Point set**.  
**a**: Set the range of the APPS sensors. **Range set**.  
**t**: Set the range of the TPS sensors. **Range set**.  
**b**: Set the range of the BPS sensor. **Range set**.  
**p**: Set the throttle's range.  
##### Throttle range setting commands
**+**: Set maximum (open) throttle position. **Range set**.  
**-**: Set minimum (close) throttle position. **Range set**. *NOTE: THIS IS NOT THE IDLE POSITION*.  
**q**: Quit.  
