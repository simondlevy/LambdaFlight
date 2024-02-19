<img src="media/lambdaflight2.png" align="center">

# Miniflie
Miniflie is an experimental minimalist version of the 
[Crazyflie firmware](https://github.com/bitcraze/crazyflie-firmware),
with support for SITL simulation in Webots.  This minimalist version doesn't 
support Loco positioning or swarms, and requires you to have a Crazyflie Flowdeck v2,
so for running your Crazyflie you'll probably want to use the official
Crazyflie firmware.

If you do want to try out Miniflie, here's how:

## Webots

1. Install [Webots](https://cyberbotics.com/) on your computer.  

2. Launch Webots and use <b>File / Open World</b> to open <b>Minifile/webots/worlds/cplusplus.wbt</b>.

3. In the upper-right of the Webots app you will see a label <b>Text Editor</b> with a little file-folder
icon below it.  Use that icon to open <b>Controllers/cplusplus/cplusplus.cpp</b>.

4.  Build and run the code.  If you have a game controller or R/C transmitter with
adapter dongle, you can use that to fly;  otherwise, the simulator will tell you 
how to fly using the keyboard.

## Crazyflie (2.1 and Bolt 1.1)

1. Attach the
[Crazyflie Flowdeck v2](https://www.bitcraze.io/products/flow-deck-v2/)
to your Crazyflie board.

2. Install [STM32Duino](https://github.com/stm32duino)

3. Install the following Arduino libraries:

* [VL53L1](https://github.com/simondlevy/VL53L1)
* [PWM3901](https://github.com/simondlevy/PMW3901)
* [BoschSensors](https://github.com/simondlevy/BoschSensors)

4. Edit <tt>Miniflie/crazyflie/Makefile</tt> to reflect where you 
installed STM32Duino

5. <tt>cd Miniflie/crazyflie</tt>

6. For CF2, <tt>make clean && make cf2_defconfig</tt> 

7. For Bolt, <tt>make clean && make bolt_defconfig</tt> 

8. <tt>make -j 32 && make cload</tt>


