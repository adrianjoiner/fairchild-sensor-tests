# Development setup

## Libraries - Generic
Pinned and referenced in the platformio.ini file
- Adafruit GFX Library@1.4.7
- Adafruit BMP280@1.0.2
- Adafruit Unified Sensor@1.0.3 

## libraries - 3rd Party
Download the zip and copy to the libs folder. Decided to do this and not use git submodlues as gives more flexability if any of them need changing
[MPU9250](https://github.com/bolderflight/MPU9250)
[Ported Adafruit PCD8544 - Nokia 5110](https://github.com/KenjutsuGH/Adafruit-PCD8544-Nokia-5110-LCD-library)
[Roger clarks Adafruit_STM32](https://github.com/rogerclarkmelbourne/Arduino_STM32.git)


# Information used
## Travis CI
[Build PR's and pushes to master only](https://stackoverflow.com/questions/31882306/how-to-configure-travis-ci-to-build-pull-requests-merges-to-master-w-o-redunda)


## STM32
[Using SPI with the STM32](https://circuitdigest.com/microcontroller-projects/stm32-spi-communication-tutorial)

## Pin connections
ic2 bus
SCL/SCK ->      B6
SDA/SDI ->      B7
5v      ->      5v from battery

Note connecting to a 5v supply; it will supply 3.3v from its 3.3v pins for use by other components. __doesn't work the other way round__ - connecting to a 3.3v supply the 5v pin will not give 5v but 3.3v

However see individual sensors for more details


# Sensors

## MPU9255
This sensor...
- Acts as an accelerometer; measures accelleration in a particular direction (X, Y Z) - this gives us the ability to see in which direction we are actually moving.
- Acts as a gyroscope; gives rotational information around an access - used to achieve stable flight by balancing out tilt

### MPU9255 Pin connections
You need only connect it to the ic2 bus which can be shared with other components.
VCC     ->      3.3v STM32
GND     ->      GND
SCL/SCK ->      B6
SDA/SDI ->      B7


## Barometer - BMP-280
- Gives us hight (calculated pressure / temp.)
Value needs to be zeroed at takeoff to give us relative hight above ground. Also an adjustment needs to be made to account for the different pressure readings at you global location. This information can be found on [this website](http://???)

### BMP-280 Pin connections
Connects to the STM32's ic2 bus
VCC     ->      3.3v STM32
GND     ->      GND
SCL     ->      B6
SDA     ->      B7




## Nokia 5110 LCD
Using forked [Adafruit-PCD8544-Nokia-5110-LCD-library](https://github.com/KenjutsuGH/Adafruit-PCD8544-Nokia-5110-LCD-library.git) which needs to be cloned into 
[also here ](https://randomnerdtutorials.com/complete-guide-for-nokia-5110-lcd-with-arduino/) for example code
[Nokia5110 lcd](https://lastminuteengineers.com/nokia-5110-lcd-arduino-tutorial/)

Note the screen is graphic, ie not char based, but can contain 6 rows of 14 chars. Once you have printed to the display you then have to 'display' the text before any output appears

### Nokia 5110 LCD pin connections
VCC     ->      3.3v on STM32
GND     ->      GND
RST     ->      B13
CE      ->      B14
DC      ->      B15
DIN     ->      A8
CLK     ->      B12


## Debug screen layout on 5110
    0123456789ABCDE
0   Tilt-X
1   Tilt-Y
2   FWD
3   BWD
4   Up
5   Down
6

# FSiA6 RX
## Pin connections
PPM (top pin, far left)     ->  A0  
VCC (middle far left pin)   ->  5v  
GND (bottom left as facing) ->  GND 




__SCRATCH__
https://www.stm32duino.com/viewtopic.php?t=4186

Watch this https://www.youtube.com/watch?v=utTi0awlUzg