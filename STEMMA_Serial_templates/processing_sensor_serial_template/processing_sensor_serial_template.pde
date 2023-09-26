import processing.serial.*;

/*
*
* This code is based around the floatdict object in processing
* Definitely read up on the documentation of the object 
* https://processing.org/reference/FloatDict.html
* If things get weird, reset the processing sketch AND the arduino
*
*
*/

//Valid keys are:
//Use any of these constants (if the appropriate sensor is plugged in)

//----Distance Sensor----
final String dis_mm = "dis_mm";

//----IMU----
final String temp = "temp";
final String accelX = "accelX";
final String accelY = "accelY";
final String accelZ = "accelZ";
final String gyroX = "gyroX";
final String gyroY = "gyroY";
final String gyroZ = "gyroZ";

//----Lux Sensor----
final String lux = "lux";

//----Capacitive Touch Sensors----
final String cap1 = "cap1";
final String cap2 = "cap2";
final String cap3 = "cap3";
final String cap4 = "cap4";
final String cap5 = "cap5";
final String cap6 = "cap6";
final String cap7 = "cap7";
final String cap8 = "cap8";
final String cap9 = "cap9";
final String cap10 = "cap10";
final String cap11 = "cap11";

//----Key Button----
final String key1 = "key1";
final String key2 = "key2";
final String key3 = "key3";
final String key4 = "key4";

//-----Encoder Knob Button-----
final String EncButton = "EncButton";
final String EncPos = "EncPos";

//variables that the template is using for data parsing and storing
//sensor_data is the important one to note that you will reference to get sensor data from
Serial serial;
String[] data_parsed, sensor_parsed;
FloatDict sensor_data;
boolean parsed = false;
String message = "";

void setup(){
  /*
  *
  *  IMPORTANT FOR MAC USERS
  *  Because of how mac (and also linux) assigns hardware devices
  *  you'll need to change the value of the portnumber variable below whenever you replug in usb devices
  *  unfortunately the best method is run the trial and error by running the code and changing the number until it works
  *
  */
  int portNumber = 0; //change the 0 to a 1 or 2 etc. to match your port
  String portName = Serial.list()[portNumber]; 
  serial = new Serial(this, portName, 115200);
  sensor_data = new FloatDict();
  
  
}

void draw(){
  //this method needs to run in order to translate incoming serial data
  parseSerial();

  if(parsed){
    /*
    * write any code that uses sensor data inside of this if statement
    * reference the FloatDict object in documentation to see what methods can be used with the sensor_data object
    * If you have the sensor plugged in, you can reference that sensor data by using any of the keys that are listed on the top
    */
    
    //for example, calling the .get method and looking for the constant EndPos will give you the value if the sensor is plugged in 
    println(sensor_data.get(EncPos));
    
  }
  else{
     println("waiting for first data message"); 
  }
  delay(10);
}


/*
parseSerial is expecting Serial information in the format that the Stemma template code is sending. The data format is data;SENSOR1,VAL1;SENSOR2,VAL2;etc
In theory, you shouldn't need to mess with this function
*/
void parseSerial(){
  try{
    if (serial.available() > 0) // If data is available,
    {  
      message = serial.readStringUntil('\n');    // read it and store it in message
      if(message != null){
        data_parsed = split(message, ';');
        if(data_parsed[0].equals("data")){
          for(int i = 1; i < data_parsed.length; i+=1)
          {
            sensor_parsed = split(data_parsed[i], ',');
            sensor_data.set(sensor_parsed[0].toString(), float(sensor_parsed[1]));         
          }
        }
      }
    }
    if(sensor_data.size() > 0)
      parsed = true;
  }
  catch(Exception e){
    System.out.println("Error trying to read from array");
  }    
  
}
