// Demo for getting individual unified sensor data from the LSM6DS series

// Can change this to be LSM6DSOX or whatever ya like
#include <Adafruit_LSM6DSOX.h>
#include <Wire.h>
#include <vl53l4cd_class.h>
#include "Adafruit_VEML7700.h"
#include "Adafruit_MPR121.h"
#include "Adafruit_NeoKey_1x4.h"
#include "seesaw_neopixel.h"
#include "Adafruit_seesaw.h"


#define DEV_I2C Wire
#define SerialPort Serial


#ifndef _BV
#define _BV(bit) (1 << (bit)) 
#endif

#define SS_SWITCH 24

bool IMU_valid = false;
Adafruit_LSM6DSOX IMU;
Adafruit_Sensor *imu_temp, *imu_accel, *imu_gyro;

bool TOF_valid = false;
VL53L4CD sensor_vl53l4cd_sat(&DEV_I2C, A1);
int distance_mm = 0;

bool LUX_valid = false;
Adafruit_VEML7700 veml = Adafruit_VEML7700();
float lux_val = 0;

bool CAP_valid= false;
Adafruit_MPR121 cap = Adafruit_MPR121();
uint16_t lasttouched = 0;
uint16_t currtouched = 0;
int touch_val = 0;

bool KEY_valid = false;
Adafruit_NeoKey_1x4 neokey;
uint8_t buttons;
int button1 = 0, button2 = 0, button3 = 0, button4 = 0;

bool ENC_valid = false;
seesaw_NeoPixel sspixel = seesaw_NeoPixel(1, 6, NEO_GRB + NEO_KHZ800);
Adafruit_seesaw ss;
int32_t encoder_position;


String message = "";


void setup(void) {
  Serial.begin(115200);
  while (!Serial)
    delay(10); // will pause Zero, Leonardo, etc until serial console opens

  Wire.begin();

  checkI2CBus();

  Serial.print("TOF status: ");
  Serial.println(TOF_valid);
  Serial.print("IMU status: ");
  Serial.println(IMU_valid);
  Serial.print("LUX status: ");
  Serial.println(LUX_valid);
  Serial.print("CAP status: ");
  Serial.println(CAP_valid);
  Serial.print("KEY status: ");
  Serial.println(KEY_valid);
  Serial.print("ENC status: ");
  Serial.println(ENC_valid);


  if(IMU_valid){
    Serial.println("Adafruit LSM6DS test!");
    if (!IMU.begin_I2C()) {
      Serial.println("Failed to find LSM6DS chip");
      while (1) {
        delay(10);
      }
    }

    Serial.println("LSM6DS Found!");
    imu_temp = IMU.getTemperatureSensor();
    imu_temp->printSensorDetails();
  
    imu_accel = IMU.getAccelerometerSensor();
    imu_accel->printSensorDetails();
  
    imu_gyro = IMU.getGyroSensor();
    imu_gyro->printSensorDetails();
  }

  if(TOF_valid)
  {
    // Configure VL53L4CD satellite component.
    sensor_vl53l4cd_sat.begin();
  
    // Switch off VL53L4CD satellite component.
    sensor_vl53l4cd_sat.VL53L4CD_Off();
  
    //Initialize VL53L4CD satellite component.
    sensor_vl53l4cd_sat.InitSensor();
  
    // Program the highest possible TimingBudget, without enabling the
    // low power mode. This should give the best accuracy
    sensor_vl53l4cd_sat.VL53L4CD_SetRangeTiming(50, 0);
  
    // Start Measurements
    sensor_vl53l4cd_sat.VL53L4CD_StartRanging();
  }

  if(LUX_valid)
  {
    veml.begin();
    veml.setLowThreshold(10000);
    veml.setHighThreshold(20000);
    veml.interruptEnable(true);
    //veml.setGain(VEML7700_GAIN_1_8);
    veml.setIntegrationTime(VEML7700_IT_25MS);
  }

  if(CAP_valid)
  {
    cap.begin(0x5A);
  }

  if(KEY_valid)
  {
    neokey.begin(0x30);
    for (uint16_t i=0; i<neokey.pixels.numPixels(); i++) {
      neokey.pixels.setPixelColor(i, 0x808080); // make each LED white
      neokey.pixels.show();
      delay(50);
    }
    for (uint16_t i=0; i<neokey.pixels.numPixels(); i++) {
      neokey.pixels.setPixelColor(i, 0x000000);
      neokey.pixels.show();
      delay(50);
    }
  }

  if(ENC_valid)
  {
    sspixel.begin(0x36);
    ss.begin(0x36);
    sspixel.setBrightness(20);
    sspixel.show();
    ss.pinMode(SS_SWITCH, INPUT_PULLUP);
    encoder_position = ss.getEncoderPosition();
    delay(10);
    ss.setGPIOInterrupts((uint32_t)1 << SS_SWITCH, 1);
    ss.enableEncoderInterrupt();
  }
}

void loop() {
  message = "data;";
  uint8_t NewDataReady = 0;
  VL53L4CD_Result_t results;
  uint8_t status;
  distance_mm = 0;

  if(TOF_valid){
    do {
      status = sensor_vl53l4cd_sat.VL53L4CD_CheckForDataReady(&NewDataReady);
    } while (!NewDataReady);
  
    if ((!status) && (NewDataReady != 0)) {
    // (Mandatory) Clear HW interrupt to restart measurements
    sensor_vl53l4cd_sat.VL53L4CD_ClearInterrupt();
  
    // Read measured distance. RangeStatus = 0 means valid data
    sensor_vl53l4cd_sat.VL53L4CD_GetResult(&results);
    }
    distance_mm = results.distance_mm;
  
    message += "dis_mm,";
    message.concat(distance_mm);
    message+= ";";
  }
  if(IMU_valid)
  {
    //  /* Get a new normalized sensor event */
    sensors_event_t temp;
    imu_temp->getEvent(&temp);
  
    sensors_event_t accel;
    imu_accel->getEvent(&accel);
    
    sensors_event_t gyro;
    imu_gyro->getEvent(&gyro);
  
    //   serial plotter friendly format
    
    message += "temp,";
    message.concat(temp.temperature);
    message+= ";";
    message += "accelX,";
    message.concat(accel.acceleration.x);
    message+= ";";
    message += "accelY,";
    message.concat(accel.acceleration.y);
    message+= ";";
    message += "accelZ,";
    message.concat(accel.acceleration.z);
    message+= ";";
    message += "gyroX,";
    message.concat(gyro.gyro.x);
    message+= ";";
    message += "gyroY,";
    message.concat(gyro.gyro.y);
    message+= ";";
    message += "gyroZ,";
    message.concat(gyro.gyro.z);
    message+= ";";  
  }

  if(LUX_valid)
  {
    lux_val = veml.readLux();
    message += "lux,";
    message.concat(lux_val);
    message+= ";"; 
  }

  if(CAP_valid)
  {
    currtouched = cap.touched();
        
    for (int i=0; i<12; i++) {
      // it if *is* touched and *wasnt* touched before, alert!
      touch_val = 0;

      if ((currtouched & _BV(i)) && !(lasttouched & _BV(i)) ) {
        //Serial.print(i); Serial.println(" 1");
        touch_val = 1;
      }
      // if it *was* touched and now *isnt*, alert!
      //if (!(currtouched & _BV(i)) && (lasttouched & _BV(i)) ) {
        //Serial.print(i); Serial.println(" 0");
        
      //}
      message += "cap";
      message.concat(i);
      message += ",";
      message.concat(touch_val);
      message+= ";"; 
    }
    //lasttouched = currtouched; 
  }

  if(KEY_valid)
  {
    uint8_t buttons = neokey.read();
    if (buttons & (1<<0)) {
      button1 = 1;
      neokey.pixels.setPixelColor(0, 0xFF0000); // red
    } else {
      button1 = 0;
      neokey.pixels.setPixelColor(0, 0);
    }
  
    if (buttons & (1<<1)) {
      button2 = 1;
      neokey.pixels.setPixelColor(1, 0xFFFF00); // yellow
    } else {
      button2 = 0;
      neokey.pixels.setPixelColor(1, 0);
    }
    
    if (buttons & (1<<2)) {
      button3 = 1;
      neokey.pixels.setPixelColor(2, 0x00FF00); // green
    } else {
      button3 = 0;
      neokey.pixels.setPixelColor(2, 0);
    }
  
    if (buttons & (1<<3)) {
      button4 = 1;
      neokey.pixels.setPixelColor(3, 0x00FFFF); // blue
    } else {
      button4 = 0;
      neokey.pixels.setPixelColor(3, 0);
    }  
    neokey.pixels.show();

    
    message += "key1,";
    message.concat(button1);
    message+= ";"; 
    message += "key2,";
    message.concat(button2);
    message+= ";"; 
    message += "key3,";
    message.concat(button3);
    message+= ";"; 
    message += "key4,";
    message.concat(button4);
    message+= ";"; 
  }

  if(ENC_valid)
  {
    message += "EncButton,";
    message.concat(!ss.digitalRead(SS_SWITCH));
    message+= ";"; 

    message += "EncPos,";
    message.concat(ss.getEncoderPosition());
    message+= ";"; 
    
    /*int32_t new_position = ss.getEncoderPosition();
     // did we move arounde?
     
     if (encoder_position != new_position) {
        Serial.println(new_position);         // display new position

        // change the neopixel color
//        sspixel.setPixelColor(0, Wheel(new_position & 0xFF));
//        sspixel.show();
        encoder_position = new_position;      // and save for next round
    }
    */
  }

  if(message != "")
    message = message.substring(0, message.length()-1);

  Serial.println(message);
  delay(10);
}

void checkI2CBus(){
  byte error, address;
  int nDevices;

  Serial.println("Scanning...");

  nDevices = 0;
  for(address = 1; address < 127; address++ ) 
  {
    // The i2c_scanner uses the return value of
    // the Write.endTransmisstion to see if
    // a device did acknowledge to the address.
    DEV_I2C.beginTransmission(address);
    error = DEV_I2C.endTransmission();
    if (error == 0)
    {
//      Serial.print("I2C device found at address 0x");
//      if (address<16) 
//        Serial.print("0");
//      Serial.print(address,HEX);
//      Serial.println("  !");


      
      if(address == 0x30)
        KEY_valid = true;
      if(address == 0x5A)
        CAP_valid = true;
      if(address == 0x29)
        TOF_valid = true;
      if(address == 0x1C || address == 0x6A)
        IMU_valid = true;
      if(address == 0x10)
        LUX_valid = true;
      if(address == 0x36)
        ENC_valid = true;
      

      nDevices++;
    }
    else if (error==4) 
    {
      Serial.print("Unknown error at address 0x");
      if (address<16) 
        Serial.print("0");
      Serial.println(address,HEX);
    }    
  }
  if (nDevices == 0)
    Serial.println("No I2C devices found\n");
  else
    Serial.println("done\n");

  return;
}
