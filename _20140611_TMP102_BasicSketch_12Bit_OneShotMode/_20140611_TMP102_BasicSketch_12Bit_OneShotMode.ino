// TMP102 I2C temperature sensor script for Cave Pearl Project by Edward Mallon
// 12 bit, Negative numbers are binary twos complement format.
// Up to four devices on one bus, ACCURACY: 0.5°C (–25°C to +85°C), One lsb is 0.0625°C
// Datasheet: http://www.ti.com/lit/ds/sbos397b/sbos397b.pdf

//char foo;  //this is bogus code to fix a software bug in the compiler see: http://forum.arduino.cc/index.php?topic=84412.0

#include <Wire.h> 

// comments from  https://github.com/systronix/Arduino-TMP102/blob/master/TMP102_proof_13Jul20.ino
/** --------  Addressing --------
if ADDR is GND, address is 0x48 (default for the sparkfun board)
if ADDR is VDD, address is 0x49
if ADDR is SDA, address is 0x4A
if ADDR is SCL, address is 0x4B

The two lsb of the pointer register hold the register bits, which 
are used to address one of the four directly-accessible registers.

There are four pointer addresses:
00  Temperature Register (Read Only) 12-13 bits in ms position
    bit 0 in LS byte is 1 if in 13-bit 'extended mode' (EM)
    If temp is positive (msb=0) the value is just the binary value.
    If temp is negative (msb=0) the value is in 2's complement form,
    so take the whole binary value, complement it, and add 1.
01  Configuration Register (Read/Write) 16 bits, MSB first
10  TLOW Limit Register (Read/Write)
11  THIGH Limit Register (Read/Write)

*/

// 3 color indicator LED pin connections
#define RED_PIN 3
#define BLUE_PIN 5
#define GREEN_PIN 4

byte errorflag=0;

#define TMP102_CFG_default_byte1 B01100001  // 12 bit WITH ShutDown bit turned ON as well
#define TMP102_CFG_default_byte2 B10100000  // just the defaults from pg 7
#define TMP102_OneShotBit B10000000  // one-shot by ORing D7 of CFG byte 1 to 1

int TMP102_I2C_ADDR = 0x48;  // 72=base address (four possible values)

// 1-byte pointer to write to tmp102 BEFORE reading back 2 bytes of data from that register
#define TMP102_TEMP_REG_pointer 0x00  // temperature register, read only, 16bits
#define TMP102_CONF_REG_pointer 0x01  // config register, read/write, 16 bits

void setup() 
{ 
 delay (5000);  // give some time to open serial monitor window
 Wire.begin();  // join I2C bus as master
 Serial.begin(9600); 
 pinMode(RED_PIN, OUTPUT);digitalWrite(RED_PIN, LOW);
 pinMode(GREEN_PIN, OUTPUT);digitalWrite(GREEN_PIN, LOW);
 pinMode(BLUE_PIN, OUTPUT);digitalWrite(BLUE_PIN, LOW);
 
  Serial.println("Initializing TMP102 Temperature sensor..."); 
  Wire.beginTransmission(TMP102_I2C_ADDR);
  Wire.write(TMP102_CONF_REG_pointer);
  Wire.write(TMP102_CFG_default_byte1);  //Sets to 12bit, sd mode on
  Wire.write(TMP102_CFG_default_byte2);  //none of these settings matter in one shot mode, at my temperature range, but set them to defaults anyway
  errorflag = Wire.endTransmission();
  if ( errorflag != 0) {Serial.print ("TMP102 initial reg writinr failed, Error =");Serial.println((int)errorflag);errorflag=0;}
  
  // set one-shot bit to "1" - starts a single conversion then sleeps the sensor
  Wire.beginTransmission(TMP102_I2C_ADDR);
  Wire.write(TMP102_CONF_REG_pointer); // Select control register.
  Wire.write(TMP102_CFG_default_byte1 | TMP102_OneShotBit); // Start one-shot conversion 40μA during conv
  //Wire.write(TMP102_CFG_default_byte2);  //dont need to bother writing the second byte
  errorflag = Wire.endTransmission();
  if ( errorflag != 0) {Serial.print ("OOPS! problem setting OneSHOT bit =");Serial.println((int)errorflag);errorflag=0;} 
  
  getTMP102();  
  Serial.println("...TMP102 has been initialized");
 
} 


//
void loop() 
{ 
 digitalWrite(GREEN_PIN, HIGH);;delay(1);digitalWrite(GREEN_PIN, LOW);
 getTMP102();
 delay(1500); // wait for a while  
}

//---------------- Functions--------------------

// a oneshot reading later has two steps first clear the OS bit to 0, and then reset it back to 1
void getTMP102() {
  float deg_c; errorflag=0; 
  // start by resetting the one shot bit back to zero
  Wire.beginTransmission(TMP102_I2C_ADDR);
  Wire.write(TMP102_CONF_REG_pointer); 
  Wire.write(TMP102_CFG_default_byte1);  //Sets to 12bit, sd mode on 
  errorflag = Wire.endTransmission();
  if ( errorflag != 0) {Serial.print ("TMP102 clearing OS bit in CFG reg failed...");errorflag=0;} 
  // now seting the one-shot bit to "1" will start a single conversion
  Wire.beginTransmission(TMP102_I2C_ADDR); 
  Wire.write(TMP102_CONF_REG_pointer); // point at the control register.
  Wire.write(TMP102_CFG_default_byte1 | TMP102_OneShotBit); // ORing the bits together
  //Wire.write(TMP102_CFG_default_byte2);  //I don't need to bother writing the second byte?
  errorflag = Wire.endTransmission();
  if ( errorflag != 0) {Serial.print ("OOPS! problem setting OneSHOT bit...");errorflag=0;}
  
  delay(28); // use wdt to sleep here?
  // wait for the conversion to happen: 26ms according to the sheet, will get zeros if I try to read the register too soon!
  // during the conversion the OS bit will temporarily read "0", then revert to "1" after the conversion
  // so you could check for that event
  
  Wire.beginTransmission(TMP102_I2C_ADDR); //now read the temp
  Wire.write(TMP102_TEMP_REG_pointer); // Select temperature register.
  errorflag = Wire.endTransmission();
  if ( errorflag != 0) {Serial.print ("Can't set temp reg pointer...");errorflag=0;}
  Wire.requestFrom(TMP102_I2C_ADDR, 2);
  const byte TempByte1 = Wire.read(); // MSByte, should be signed whole degrees C.
  const byte TempByte2 = Wire.read(); // unsigned because I am not reading any negative temps
  const int Temp16 = (TempByte1 << 4) | (TempByte2 >> 4);    // builds 12-bit value
  Serial.print("Integer data before conversion: ");Serial.println(Temp16);
  deg_c = Temp16*0.0625;
  Serial.print("Temperature in deg C is = ");Serial.println(deg_c,4);
  
}

  
