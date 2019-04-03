#include <Arduino.h>
#include <Wire.h>
#include "MPU9250.h"
#include "Adafruit_BMP280.h" //include the Adafruit BMP280 library
#include "Adafruit_PCD8544.h"
#include "Adafruit_GFX.h"

// global constants
const byte BMP280_address=0x76;
const byte MPU9250_address= 0x68;
bool barometerOnline = false;
bool mpuOnline = false;

// Software SPI (slower updates, more flexible pin options):
// pin 7 - Serial clock out (SCLK)
// pin 6 - Serial data out (DIN)
// pin 5 - Data/Command select (D/C)
// pin 4 - LCD chip select (CS)
// pin 3 - LCD reset (RST)

// Adafruit_PCD8544::Adafruit_PCD8544(int8_t SCLK, int8_t DIN, int8_t DC,
//    int8_t CS, int8_t RST)
//
// LCD Pin outs:
// LCD:  GND : LIGHT : VCC : CLK(PB12) : DIN(PA8) : DC(PB15) : CE(PB14) : RST(PB13)
 #if defined (__STM32F1__) || defined (ARDUINO_ARCH_STM32)
   Adafruit_PCD8544 display = Adafruit_PCD8544(PB12, PA8, PB15, PB14, PB13);
 #else
   Adafruit_PCD8544 display = Adafruit_PCD8544(7, 6, 5, 4, 3);
 #endif

Adafruit_BMP280 barometer;
MPU9250 mpu(Wire, MPU9250_address); 
 

void ic2_scan();
void displayBarometerReadings(Adafruit_BMP280&);
void displayMpuReadings(MPU9250&);

void setup() {  
  
  // Open a serial port for debug and printing
  Serial.begin(9600);
  while(!Serial) {};

  // Initialise the wire library
  Wire.begin();
  Wire.setClock(400000); //Increase to fast I2C speed!

  // scan for connected sensors, epecting to see ...
  //    MPU9255 at 0x68
  //    BMP-280 at 0x76
  // Check for any connected sensors
  display.begin();
  display.clearDisplay();
  display.display();
  display.setCursor(0,0);
  display.print("ic2 bus scan");
  display.display();
  ic2_scan();


 
  // Initialise the BMP280 barometer (altitude sensor)
  //Adafruit_BMP280 barometer;
  display.setCursor(2,0);
  display.print("BMP280 ");
  display.display();
  display.setCursor(2,9);
  if (!barometer.begin(BMP280_address)) //check if you are able to read the sensor
  {  
    Serial.println("SENSOR: unable to initialise BMP-280 barometer, sensor will be ignored.");
    barometerOnline = false;
    display.print("Offline");
  } else
  {
    barometerOnline = true;
    display.print("Online");
  }

  
  

  // Initialise the MPU9250_address
  //MPU9250 mpu(Wire, MPU9250_address);
  int status = mpu.begin();
  if (status < 0) {
    Serial.println("SENSOR: unable to initialise MPU9255, sensor will be ignored.");
    Serial.print("Returned staus: ");
    Serial.println(status);
    mpuOnline = false;
  } else 
  {
    mpuOnline = true;
  }
  
  // setting the accelerometer full scale range to +/-8G 
  mpu.setAccelRange(MPU9250::ACCEL_RANGE_8G);
  
  // setting the gyroscope full scale range to +/-500 deg/s
  mpu.setGyroRange(MPU9250::GYRO_RANGE_500DPS);
  
  // setting DLPF bandwidth to 20 Hz
  mpu.setDlpfBandwidth(MPU9250::DLPF_BANDWIDTH_20HZ);
  
  // setting SRD to 19 for a 50 Hz update rate
  mpu.setSrd(19);

  Serial.println("Waiting 3 seconds for MPU to settle down");
  delay(3000);
  Serial.println("Assuming sensor is level and stable, reading bias measurements");
  delay(1000);

  Serial.println("Setting up the LCD");
  display.begin();
  display.setContrast(50);
  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(BLACK);
  display.setCursor(0,0);
  //display.println("0123456789ABCDEFGHIJKLMNOPQRST");
}

void loop() {
  
  if (barometerOnline)
  {
    displayBarometerReadings(barometer);
    Serial.println("\n============================\n");
  }

  if (mpuOnline)
  {
    displayMpuReadings(mpu);
    Serial.println("\n============================\n");
  }
  // display.begin();
  // display.clearDisplay();
  // display.println("0123456789ABCD");
  // display.println("1");
  // display.println("2");
  // display.println("3");
  // display.println("4");
  // display.println("5");
  // display.println("6");
  // display.display();
	delay(3000);
	// display.clearDisplay();
}

// Display current output of the barometer
void displayBarometerReadings(Adafruit_BMP280& _barometer)
{
  // Temperature
  Serial.print("Temperature: ");
  Serial.println(_barometer.readTemperature()); // in celsius
  
  // Pressure
  Serial.print(_barometer.readPressure()/3389.39); 
  Serial.println(" inHg.");

  // Altitude
  Serial.print("Alt = ");
  Serial.print(_barometer.readAltitude(997)+10); // this should be adjusted to your local forcase
  Serial.println(" m  ");
}

void displayMpuReadings(MPU9250& _mpu)
{
  // read the sensor
  _mpu.readSensor();

  // display the data
  Serial.print("X-acc:");
  Serial.print(_mpu.getAccelX_mss(),6);
  Serial.print("\t\t");
  Serial.print("Y-acc:");
  Serial.print(_mpu.getAccelY_mss(),6);
  Serial.print("\t\t");
  Serial.print("Z-acc:");
  Serial.print(_mpu.getAccelZ_mss(),6);
  Serial.println("");

  Serial.print("X-gyro:");
  Serial.print(_mpu.getGyroX_rads(),6);
  Serial.print("\t\t");
  Serial.print("Y-gyro:");
  Serial.print(_mpu.getGyroY_rads(),6);
  Serial.print("\t\t");
  Serial.print("Z-gyro:");
  Serial.print(_mpu.getGyroZ_rads(),6);
  Serial.println("");

  Serial.print("X-mag:");
  Serial.print(_mpu.getMagX_uT(),6);
  Serial.print("\t\t");
  Serial.print("Y-mag:");
  Serial.print(_mpu.getMagY_uT(),6);
  Serial.print("\t\t");
  Serial.print("Z-mag:");
  Serial.print(_mpu.getMagZ_uT(),6);

  Serial.println(" ");
  Serial.println(_mpu.getTemperature_C(),6);
}


//
// Scans the ic2 bus and prints the address of any attached sensors
//
void ic2_scan() {
  byte error, address;
  int nDevices;

  Serial.println("Scanning for ic2 devices...");

  nDevices = 0;
  for(address = 1; address < 127; address++) {
    // The i2c_scanner uses the return value of
    // the Write.endTransmisstion to see if
    // a device did acknowledge to the address.

    Wire.beginTransmission(address);
    error = Wire.endTransmission();
    
    if (error == 0) {
      Serial.print("I2C device found at address 0x");
      if (address < 16) 
        Serial.print("0");
      Serial.println(address, HEX);

      nDevices++;
    }
    else if (error == 4) {
      Serial.print("Unknown error at address 0x");
      if (address < 16) 
        Serial.print("0");
      Serial.println(address, HEX);
    }    
  }
  if (nDevices == 0)
    Serial.println("No I2C devices found");
  else
    Serial.println("Scan complete.");
}

//
//In this file the timers for reading the receiver pulses and for creating the output ESC pulses are set.
//More information can be found in these two videos:
//STM32 for Arduino - Connecting an RC receiver via input capture mode: https://youtu.be/JFSFbSg0l2M
//STM32 for Arduino - Electronic Speed Controller (ESC) - STM32F103C8T6: https://youtu.be/Nju9rvZOjVQ
//
// We are attaching this code to an interrupt handler so it is called whenerver the interrupt fires
// It resets the input registers and sets the mode used for input
int16_t loop_counter;
uint8_t warning;
int32_t channel_3_start, channel_3;
void handler_channel_1(void);

void timer_setup(void) {
  Timer2.attachCompare1Interrupt(handler_channel_1); // handler_channel_1 is called whenever interrupt is fired
  TIMER2_BASE->CR1 = TIMER_CR1_CEN;
  TIMER2_BASE->CR2 = 0;
  TIMER2_BASE->SMCR = 0;
  TIMER2_BASE->DIER = TIMER_DIER_CC1IE;
  TIMER2_BASE->EGR = 0;
  TIMER2_BASE->CCMR1 = TIMER_CCMR1_CC1S_INPUT_TI1;
  TIMER2_BASE->CCMR2 = 0;
  TIMER2_BASE->CCER = TIMER_CCER_CC1E;
  //TIMER2_BASE->CCER |= TIMER_CCER_CC1P;    //Detect falling edge.
  TIMER2_BASE->CCER &= ~TIMER_CCER_CC1P; //Detect rising edge.
  TIMER2_BASE->PSC = 71;
  TIMER2_BASE->ARR = 0xFFFF;
  TIMER2_BASE->DCR = 0;

  //A test is needed to check if the throttle input is active and valid. Otherwise the ESC's might start without any warning.
  loop_counter = 0;
  while ((channel_3 > 2100 || channel_3 < 900) && warning == 0) {
    delay(100);
    loop_counter++;
    if (loop_counter == 40) {
      Serial.println(F("Waiting for a valid receiver channel-3 input signal"));
      Serial.println(F(""));
      Serial.println(F("The input pulse should be between 1000 till 2000us"));
      Serial.print(F("Current channel-3 receiver input value = "));
      Serial.println(channel_3);
      Serial.println(F(""));
      Serial.println(F("Is the receiver connected and the transmitter on?"));
      Serial.println(F("For more support and questions: www.brokking.net"));
      Serial.println(F(""));
      Serial.print(F("Waiting for another 5 seconds."));
    }
    if (loop_counter > 40 && loop_counter % 10 == 0)Serial.print(F("."));

    if (loop_counter == 90) {
      Serial.println(F(""));
      Serial.println(F(""));
      Serial.println(F("The ESC outputs are disabled for safety!!!"));
      warning = 1;
    }
  }
  if (warning == 0) {
    TIMER4_BASE->CR1 = TIMER_CR1_CEN | TIMER_CR1_ARPE;
    TIMER4_BASE->CR2 = 0;
    TIMER4_BASE->SMCR = 0;
    TIMER4_BASE->DIER = 0;
    TIMER4_BASE->EGR = 0;
    TIMER4_BASE->CCMR1 = (0b110 << 4) | TIMER_CCMR1_OC1PE | (0b110 << 12) | TIMER_CCMR1_OC2PE;
    TIMER4_BASE->CCMR2 = (0b110 << 4) | TIMER_CCMR2_OC3PE | (0b110 << 12) | TIMER_CCMR2_OC4PE;
    TIMER4_BASE->CCER = TIMER_CCER_CC1E | TIMER_CCER_CC2E | TIMER_CCER_CC3E | TIMER_CCER_CC4E;
    TIMER4_BASE->PSC = 71;
    TIMER4_BASE->ARR = 4000;
    TIMER4_BASE->DCR = 0;
    TIMER4_BASE->CCR1 = 1000;

    TIMER4_BASE->CCR1 = channel_3;
    TIMER4_BASE->CCR2 = channel_3;
    TIMER4_BASE->CCR3 = channel_3;
    TIMER4_BASE->CCR4 = channel_3;
    pinMode(PB6, PWM);
    pinMode(PB7, PWM);
    pinMode(PB8, PWM);
    pinMode(PB9, PWM);
  }
}

int32_t channel_1_start, channel_1;
int32_t channel_2_start, channel_2; 
int32_t channel_4_start, channel_4;
int32_t channel_5_start, channel_5;
int32_t channel_6_start, channel_6;
int32_t measured_time, measured_time_start;
uint8_t channel_select_counter;

void handler_channel_1(void) {
    measured_time = TIMER2_BASE->CCR1 - measured_time_start;
    if (measured_time < 0)measured_time += 0xFFFF;
    measured_time_start = TIMER2_BASE->CCR1;
    if (measured_time > 3000)channel_select_counter = 0;
    else channel_select_counter++;

    if (channel_select_counter == 1)channel_1 = measured_time;
    if (channel_select_counter == 2)channel_2 = measured_time;
    if (channel_select_counter == 3)channel_3 = measured_time;
    if (channel_select_counter == 4)channel_4 = measured_time;
    if (channel_select_counter == 5)channel_5 = measured_time;
    if (channel_select_counter == 6)channel_6 = measured_time;
  }

 