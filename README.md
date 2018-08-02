# AOA audio tone indicator from EFIS serial data
Audio tone indicator of AOA from a aircraft EFIS serial port. Using a Ardunino Due connected to a Dynon Efis D10, D100, D180 , or SkyView via the serial port.

Normally AOA is presented in a visual form of green,yellow,red to indicate how close to a stall the aircraft is.  This project turns the visual indicator into a audioable tone so the pilot does not have to look at the instrument panel.

USE AT YOUR OWN RISK.  THIS IS FOR EXPERIMENTAL AIRCRAFT ONLY. 

# Background

Adopted from USAF F-4 System.  AOA tone system developed to assist pilot with aircraft handling during maneuvering flight and approach/landing operations.  

# What does it do?

There are 2 different tone frequencies and multiple pulses per second (PPS) heard.   The diagram below shows what tone and pulses are heard at different AOA levels.

![AOA chart](https://github.com/dinglewanker/aoa-tone-efis-serial/blob/master/docs/chart.png?raw=true)


# Requirements
 - Dynon EFIS D10, D100, or D180
 - Arduino Due.  Can be purchased at many different online stores like [amazon](https://www.amazon.com/OSOYOO-Compatible-Shield-Module-Arduino/dp/B010SCWGE2/) or  [ebay](http://www.ebay.com/sch/items/?_nkw=arduino+due) 
   * The due board has 2 usb ports.  Use the "programming" usb port plugged into your computer to program the board.  When using in a aircraft you can power the board with either usb port.
 - [Ardino IDE](https://www.arduino.cc/en/Main/Software) for mac, linux , or windows. This is used to program your arduino board. And a [beginers guide to Arduino Due](https://www.arduino.cc/en/Guide/ArduinoDue) is a good idea.  You will need to [install Arduino SAM Board Core](https://www.arduino.cc/en/Guide/Cores) in order to build for a Due board. "Arduino SAM Boards (32-bits ARM Cortex-M3)" ver 1.6.8 is what was used during development.
 - You will need to know [how to install arduino libraries](https://www.arduino.cc/en/Guide/Libraries). The following libraries are required.
   * *DueTimer* (version 1.4 used)
   * *Gaussian* (version 1.0.7 used)
   * *LinkedList* (version 1.0.7 used)
 - And few other things to finish this project.
   * 10kΩ Variable Potentiometer - used for adjusted audio output level 
   * 1kΩ Resistor (2 needed)
   * Audio Jack (TRS) 3.5mm
   * [Serial to TTL DB9 Adapter](http://www.ebay.com/sch/i.html_max232+serial+ttl+DB9) used to convert the serial data to TTL serial datat that the arduino can understand.  There are several different types.  Ones that use the MAX3232 chipset seem to work the best.
   * Red LED (flashes when valid serial data is received)
   * Green LED (flashes when valid AOA data is received)
   * USB micro cable to power Arduino Due board
   * 12v to usb charger adapter. (useful for powering the arduino in a aircraft)

# Schematic
![schematic](https://github.com/dinglewanker/aoa-tone-efis-serial/blob/master/docs/AOA_Due_schem_5Jun18.png?raw=true)

# Notes
 * The audio out is designed to plug into the audio panel of your aircraft.
 
 * Using a 6k resistor directly from pin 2 (audio out) of the due board would be a quick hack for hooking into a headphone or headphone jack.  You could also use a variable resistor here to adjust the volume by hand.

 * It may be a good idea to hook a switch inline with the serial RX or audio out of the board.  This could be useful to turn the device on/off when you don't want to hear a annoying beep in your ear.

 * The red led (Serial RX) switches on/off every time it recieves a serial line of data that it understands.  Since the dynon sends data pretty fast this led may look like a flicking candle.
 
 * For setting up on skyview system need to setup skyview to trasmit(TX) AHRS data to available serial port.  Receiveing does not need to be enabled because this box does not send data back to dynon.  Default is using 9600 baud for serial output.  But this can be changed in source code.
 
 
# Todo
- Test on Dynon Skyview EFIS (Completed and works!)
- Support other EFIS units.  Like MGL, Advanced, GRT
 
