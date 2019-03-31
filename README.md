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

## Connecting a Nokia 5110 LCD to the STM32
Using forked [Adafruit-PCD8544-Nokia-5110-LCD-library](https://github.com/KenjutsuGH/Adafruit-PCD8544-Nokia-5110-LCD-library.git) which needs to be cloned into 
[also here ](https://randomnerdtutorials.com/complete-guide-for-nokia-5110-lcd-with-arduino/) for example code
[Nokia5110 lcd](https://lastminuteengineers.com/nokia-5110-lcd-arduino-tutorial/)

Note the screen is graphic, ie not char based, but can contain 6 rows of 14 chars. Once you have printed to the display you then have to 'display' the text before any output appears


## Debug screen layout
    0123456789ABCDE
0   Tilt-X
1   Tilt-Y
2   FWD
3   BWD
4   Up
5   Down
6





## Using SPI with the STM32
https://circuitdigest.com/microcontroller-projects/stm32-spi-communication-tutorial


# Sensors and their purpose

## Accelerometer - MPU9255
These measure accelleration in a particular direction (X, Y Z) - this gives us the ability to see in which direction we are actually moving

## Gyroscope - MPU9255
Gives rotational information around and access - used to achieve stable flight by alancing out tilt


## Barometer - BMP-280
Gives us hight - zero at takeoff to give us relative hight above ground