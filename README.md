# AOA aural indicator for flying "On Speed" from EFIS serial data
Goal of this project is to find the best aural logic and give the pilot a "Head up, eyes out solution".

Audio tone indicator of aircraft's level of AOA. Using an Ardunino Due connected to a Dynon Efis D10, D100, D180 , or SkyView via the serial port.

Normally AOA is presented in a visual form of green, yellow, red to indicate how close to a stall the aircraft is.  This project turns the visual indicator into a audioable tone to let the pilot know where they are on the power curve (not just the stall).  

This is different from other AOA audio indicators because this system is helping you get to best perforance on the AOA curve (also known as "On Speed").  Most AOA tones are telling you how close the aircraft is to a stall, which is helpful but doesn't tell the whole story.  Our tests and final results will be shared with the avaition community and hopefully adopted or built in to exsiting avaition products.  Currently MGL avionics has added this to their iEFIS products.

This project won the [2018 Founder’s Innovation Prize](https://www.eaa.org/en/airventure/eaa-airventure-news-and-multimedia/eaa-airventure-news/eaa-airventure-oshkosh/07-25-2018-aural-angle-of-attack-project-wins-founders-innovation-prize).  Thanks so much to the EAA and the other fianalists.  

USE AT YOUR OWN RISK.  THIS IS FOR EXPERIMENTAL AIRCRAFT ONLY. 

# Background

Adopted from USAF F-4 System.  AOA tone system developed to assist pilot with aircraft handling during maneuvering flight and approach/landing operations.  

# What does it do?

There are 2 different tone frequencies and multiple pulses per second (PPS) heard.   The diagram below shows what tone and pulses are heard at different AOA levels.

![AOA chart](https://github.com/dinglewanker/aoa-tone-efis-serial/blob/master/docs/chart.png?raw=true)


# Requirements
 - Dynon EFIS D10, D100, D180, Skyview
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
 * The audio out is designed to plug into the audio panel of your aircraft. Or into the aux audio input on some headsets.
 
 * It may be a good idea to hook a switch inline with the audio out of the board.  This could be useful to turn the device on/off when you don't want to hear a annoying beep in your ear.  Or just using the variable potentiometer to turn down the audio when you don't want to hear it.

 * The red led (Serial RX) switches on/off every time it recieves a serial line of data that it understands.  Since the dynon sends data pretty fast this led may look like a flicking candle. At slower baud rates on the skyview the red led will flash slower.
 
 * The green led indicates that valid AOA data is being recieved and tones are being played.  If aircraft is not moving or no air is flowing through the pitot this light will not flash.
 
 * For setting up on skyview system need to setup skyview to trasmit(TX) AHRS data to available serial port.  Receiveing does not need to be enabled because this box does not send data back to dynon.  Default is using 9600 baud for serial output.  But this can be changed in source code.
 
 
# Todo
- Test on Dynon Skyview EFIS (Completed and works!)
- Support other EFIS units.  Like Advanced, GRT, Garmin, etc.
- Support other external devices like Levil BOM.
- Work with airball.  (airball.aero)
- Work with whomever wants to work with us.

 
