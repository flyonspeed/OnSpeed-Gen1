# AOA audio tone indicator from EFIS serial data
Audio tone indicator of AOA from a aircraft EFIS serial port. Using a Ardunino Due connected to a Dynon Efis D10, D100 or D180 via the serial port.

Normally AOA is presented in a visual form or green,yellow,red to indicate how close to a stall the aircraft is.  This project turns the visual indicator into a audioable tone so the pilot does not have to look down at the panel to know how close to a stall they are.

# Requirements
 - Dynon EFIS D10, D100, or D180
 - Arduino Due.  Can be purchased at many different online stores. 
 - [Ardino IDE](https://www.arduino.cc/en/Main/Software) for mac, linux , or windows. This is used to program your arduino board. And a [beginers guide to Arduino Due](https://www.arduino.cc/en/Guide/ArduinoDue) is a good idea.  You will need to [install Arduino SAM Board Core](https://www.arduino.cc/en/Guide/Cores) in order to build for a Due board.
 - You will need to know [how to install arduino libraries](https://www.arduino.cc/en/Guide/Libraries). The following libraries are required.
   * *DueTimer*
   * *Gaussian*
   * *LinkedList*
 - And few other things to finish this project.
   * 0.22µF Ceramic Capacitor
   * 0.02µF Ceramic Capacitor
   * 100Ω Resistor
   * 1kΩ Resistor
   * Audio Jack (TRS) 3.5mm
   * [Serial to TTL DB9 Adapter](http://www.ebay.com/sch/i.html_max232+serial+ttl+DB9) used to convert the serial data to TTL serial datat that the arduino can understand.
   * Red LED
   * USB micro cable to power Arduino Due board
   * 12v to usb charger adapter. (useful for powering the arduino in a aircraft)

# Schematic
![schematic](https://github.com/dinglewanker/aoa-tone-efis-serial/blob/master/docs/AOA_Due_schem.png?raw=true)

# Notes
The audio out is designed to go into a audio panel of your aircraft.

It may be a good idea to hook a switch inline with the serial RX or audio out of the board.  This could be useful to turn the device on/off when you don't want to hear a annoying beep in your ear.
# Todo
- Test on Dynon Skyview EFIS 
- Support other EFIS units.  Like MGL, Advanced, GRT
 
