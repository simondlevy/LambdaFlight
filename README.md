<a href="https://koerbitz.me/posts/Why-I-love-Haskell.html"><img src="media/lambdaflight2.png" align="center"></a>

# LambdaFlight
LambdaFlight is a Haskell-based flight-control package for miniature aerial
vehicles (MAVs), a.k.a.drones.  It currently works with the 
[Crazyflie](https://www.bitcraze.io/products/crazyflie-2-1/)
quadcopter and
has support for Software-In-The-Loop (SITL) simulation using
[Webots](https://cyberbotics.com/).  

To try out LambdaFlight, you'll first need to install
[Haskell](https://www.haskell.org/)
and
[NASA Copilot](https://copilot-language.github.io) (the &ldquo;secret sauce&rdquo; that
allows you to compile Haskell code to a form suitable for running on a flight 
controller.)  I was able to do this via:

```
cabal install copilot
cabal install copilot-c99
```

Then you can run a simulated
quadcopter in Webots or an actual Crazyflie, following the instructions below:

## Webots

1. Install [Webots](https://cyberbotics.com/) on your computer.  

2. Launch Webots and use <b>File / Open World</b> to open <b>Minifile/webots/worlds/simple.wbt</b>.

3. In the upper-right of the Webots app you will see a label <b>Text Editor</b> with a little file-folder
icon below it.  Use that icon to open <b>Controllers/simple/simple.cpp</b>.

4.  Build and run the code.  If you have a game controller or R/C transmitter with
adapter dongle, you can use that to fly;  otherwise, the simulator will advise you
that no such device was found and will tell you how to fly using the keyboard.

## Crazyflie

LambdaFlight takes a minimalist approach, not
supporting Crayzyflie's awesome Loco positioning or swarms, and requiriing you
to have a Crazyflie Flowdeck v2, so for running your Crazyflie you'll probably
want to use the official Crazyflie firmware. If you want to try LambdaFlight
on your Crazyflie, here's how:

1. Attach the
[Crazyflie Flowdeck v2](https://www.bitcraze.io/products/flow-deck-v2/)
to your Crazyflie board.

2. Install [STM32Duino](https://github.com/stm32duino)

3. Install the following Arduino libraries:

* [VL53L1](https://github.com/simondlevy/VL53L1)
* [PWM3901](https://github.com/simondlevy/PMW3901), <b>cross-platform</b> branch
* [BoschSensors](https://github.com/simondlevy/BoschSensors)
* [TinyEKF](https://github.com/simondlevy/TinyEKF)

4. Edit <tt>LambdaFlight/crazyflie/Makefile</tt> to reflect where you 
installed STM32Duino

5. <tt>cd LambdaFlight/crazyflie</tt>

6. <tt> make cf2_defconfig && make links && make copilot && make -j 32</tt>

7. <tt>make cload</tt>

