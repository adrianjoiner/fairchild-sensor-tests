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
void handler_channel_1(void);
void handler_channel_2(void);
void handler_channel_3(void);
void handler_channel_3(void);
void handler_channel_4(void);
void handler_channel_5(void);
void handler_channel_6(void);
int16_t loop_counter;
uint8_t warning;
int32_t channel_3_start, channel_3;



int32_t channel_1_start, channel_1;
int32_t channel_2_start, channel_2; 
int32_t channel_4_start, channel_4;
int32_t channel_5_start, channel_5;
int32_t channel_6_start, channel_6;
int32_t measured_time, measured_time_start;
uint8_t channel_select_counter;

 

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
  display.println("ic2 bus scan");
  display.display();

  // check for connected sensors
  ic2_scan();

  // Initialise the BMP280 barometer (altitude sensor)
  //Adafruit_BMP280 barometer;
  //isplay.setCursor(2,0);
  display.print("BMP280 ");
  display.display();
  //display.setCursor(2,9);
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

  //
  // setup RX connection
  pinMode(PA0, INPUT);
  delay(250);
  Serial.println("Attaching handlers to interrupts"); 
  Timer2.attachCompare1Interrupt(handler_channel_1);
  Timer2.attachCompare2Interrupt(handler_channel_2);
  Timer2.attachCompare3Interrupt(handler_channel_3);
  Timer2.attachCompare4Interrupt(handler_channel_4);
  TIMER2_BASE->CR1 = TIMER_CR1_CEN;
  TIMER2_BASE->CR2 = 0;
  TIMER2_BASE->SMCR = 0;
  TIMER2_BASE->DIER = TIMER_DIER_CC1IE | TIMER_DIER_CC2IE | TIMER_DIER_CC3IE | TIMER_DIER_CC4IE;
  TIMER2_BASE->EGR = 0;
  TIMER2_BASE->CCMR1 = 0b100000001; //Register is set like this due to a bug in the define table (30-09-2017)
  TIMER2_BASE->CCMR2 = 0b100000001; //Register is set like this due to a bug in the define table (30-09-2017)
  TIMER2_BASE->CCER = TIMER_CCER_CC1E | TIMER_CCER_CC2E | TIMER_CCER_CC3E | TIMER_CCER_CC4E;
  TIMER2_BASE->PSC = 71;
  TIMER2_BASE->ARR = 0xFFFF;
  TIMER2_BASE->DCR = 0;

  Timer3.attachCompare1Interrupt(handler_channel_5);
  Timer3.attachCompare2Interrupt(handler_channel_6);
  TIMER3_BASE->CR1 = TIMER_CR1_CEN;
  TIMER3_BASE->CR2 = 0;
  TIMER3_BASE->SMCR = 0;
  TIMER3_BASE->DIER = TIMER_DIER_CC1IE | TIMER_DIER_CC2IE;
  TIMER3_BASE->EGR = 0;
  TIMER3_BASE->CCMR1 = 0b100000001; //Register is set like this due to a bug in the define table (30-09-2017)
  TIMER3_BASE->CCMR2 = 0;
  TIMER3_BASE->CCER = TIMER_CCER_CC1E | TIMER_CCER_CC2E;
  TIMER3_BASE->PSC = 71;
  TIMER3_BASE->ARR = 0xFFFF;
  TIMER3_BASE->DCR = 0;
}

void loop() {

  delay(500);
  Serial.print("1:");
  Serial.print(channel_1);
  Serial.print(" 2:");
  Serial.print(channel_2);
  Serial.print(" 3:");
  Serial.print(channel_3);
  Serial.print(" 4:");
  Serial.print(channel_4);
  Serial.print(" 5:");
  Serial.print(channel_5);
  Serial.print(" 6:");
  Serial.println(channel_6);
  
  // if (barometerOnline)
  // {
  //   displayBarometerReadings(barometer);
  //   Serial.println("\n============================\n");
  // }

  // if (mpuOnline)
  // {
  //   displayMpuReadings(mpu);
  //   Serial.println("\n============================\n");
  // }
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
      Serial.print("ic2 device found at address 0x");
      display.print("device at 0x");
      display.println(address, HEX);
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
    Serial.println("No ic2 devices found");
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



void handler_channel_1(void) {
  if (0b1 & GPIOA_BASE->IDR  >> 0) {
    channel_1_start = TIMER2_BASE->CCR1;
    TIMER2_BASE->CCER |= TIMER_CCER_CC1P;
  }
  else {
    channel_1 = TIMER2_BASE->CCR1 - channel_1_start;
    if (channel_1 < 0)channel_1 += 0xFFFF;
    TIMER2_BASE->CCER &= ~TIMER_CCER_CC1P;
  }
}

void handler_channel_2(void) {
  if (0b1 & GPIOA_BASE->IDR >> 1) {
    channel_2_start = TIMER2_BASE->CCR2;
    TIMER2_BASE->CCER |= TIMER_CCER_CC2P;
  }
  else {
    channel_2 = TIMER2_BASE->CCR2 - channel_2_start;
    if (channel_2 < 0)channel_2 += 0xFFFF;
    TIMER2_BASE->CCER &= ~TIMER_CCER_CC2P;
  }
}

void handler_channel_3(void) {
  if (0b1 & GPIOA_BASE->IDR >> 2) {
    channel_3_start = TIMER2_BASE->CCR3;
    TIMER2_BASE->CCER |= TIMER_CCER_CC3P;
  }
  else {
    channel_3 = TIMER2_BASE->CCR3 - channel_3_start;
    if (channel_3 < 0)channel_3 += 0xFFFF;
    TIMER2_BASE->CCER &= ~TIMER_CCER_CC3P;
  }
}

void handler_channel_4(void) {
  if (0b1 & GPIOA_BASE->IDR >> 3) {
    channel_4_start = TIMER2_BASE->CCR4;
    TIMER2_BASE->CCER |= TIMER_CCER_CC4P;
  }
  else {
    channel_4 = TIMER2_BASE->CCR4 - channel_4_start;
    if (channel_4 < 0)channel_4 += 0xFFFF;
    TIMER2_BASE->CCER &= ~TIMER_CCER_CC4P;
  }
}

void handler_channel_5(void) {
  if (0b1 & GPIOA_BASE->IDR >> 6) {
    channel_5_start = TIMER3_BASE->CCR1;
    TIMER3_BASE->CCER |= TIMER_CCER_CC1P;
  }
  else {
    channel_5 = TIMER3_BASE->CCR1 - channel_5_start;
    if (channel_5 < 0)channel_5 += 0xFFFF;
    TIMER3_BASE->CCER &= ~TIMER_CCER_CC1P;
  }
}

void handler_channel_6(void) {
  if (0b1 & GPIOA_BASE->IDR >> 7) {
    channel_6_start = TIMER3_BASE->CCR2;
    TIMER3_BASE->CCER |= TIMER_CCER_CC2P;
  }
  else {
    channel_6 = TIMER3_BASE->CCR2 - channel_6_start;
    if (channel_6 < 0)channel_6 += 0xFFFF;
    TIMER3_BASE->CCER &= ~TIMER_CCER_CC2P;
  }
}