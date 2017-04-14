//SD libraries
#include <SPI.h>
#include <SD.h>
#include <string.h>
#include <SoftwareSerial.h>

//GPS library
#include <TinyGPS.h>
TinyGPS gps;

//10DOF libraries
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BMP085_U.h>
#include <Adafruit_L3GD20_U.h>
#include <Adafruit_LSM303_U.h>

//GPS/SD card constants
#define GPSBAUD 4800
#define SERIALBAUD 115200
SoftwareSerial uart_gps(2, 3);
File logFile;

//10DOF constants

Adafruit_BMP085_Unified bmp = Adafruit_BMP085_Unified(10085);
Adafruit_L3GD20_Unified gyro = Adafruit_L3GD20_Unified(12121);
Adafruit_LSM303_Mag_Unified mag = Adafruit_LSM303_Mag_Unified(12345);
Adafruit_LSM303_Accel_Unified accel = Adafruit_LSM303_Accel_Unified(54321);
sensor_t sensor;
sensors_event_t event;

void getgps(TinyGPS &gps);

void displaySensorDetails(){}

void setup() {
  pinMode(6, OUTPUT);
  pinMode(5, OUTPUT);
  //Serial setup
  Serial.begin(SERIALBAUD);

  bmp.begin(); 
  //gyro.enableAutoRange(true);
  gyro.begin();
  mag.enableAutoRange(true);
  mag.begin(); accel.begin();
  
  //GPSBAUD setup
  uart_gps.begin(GPSBAUD);
  
  while (!Serial) {
    ; // wait for serial port to connect. Needed for native USB port only
  }
  //SD setup
  pinMode(8, OUTPUT);
  Serial.print("Init...");

  if (!SD.begin(4)) {
    Serial.println("failed");
    digitalWrite(5, HIGH);
    delay(100);
    digitalWrite(5, LOW);
    return;
  }else{
    digitalWrite(6, HIGH);
    delay(100);
    digitalWrite(6, LOW);
  }
  Serial.println("done.");   

  logFile = SD.open("log.txt", FILE_WRITE);

  if(logFile){
    Serial.println("SD init");
  }else{
    Serial.println("SD err");
  }
  logFile.println("Initialized.");
  logFile.println("");
  logFile.close();

  bmp.getSensor(&sensor);
  Serial.print("IDed: "); Serial.println(sensor.name);
  gyro.getSensor(&sensor);
  Serial.print("IDed: "); Serial.println(sensor.name);
  mag.getSensor(&sensor);
  Serial.print("IDed: "); Serial.println(sensor.name);
  accel.getSensor(&sensor);
  Serial.print("IDed: "); Serial.println(sensor.name);

  
}

void loop() {
  // put your main code here, to run repeatedly:
  while(uart_gps.available())     // While there is data on the RX pin...
  {
      /*
      int c = uart_gps.read();    // load the data into a variable...
      if(gps.encode(c))      // if there is a new valid sentence...
      {
        
        getgps(gps);         // then grab the data.
      }
      */
      getgps(gps);
      delay(100);
  }
}

void getgps(TinyGPS &gps)
{
  logFile = SD.open("log.txt", FILE_WRITE);
  // To get all of the data into varialbes that you can use in your code, 
  // all you need to do is define variables and query the object for the 
  // data. To see the complete list of functions see keywords.txt file in 
  // the TinyGPS and NewSoftSerial libs.

  // Define the variables that will be used
  float latitude, longitude;
  int year;
  byte month, day, hour, minute, second, hundredths;
  // Then call this function
  gps.f_get_position(&latitude, &longitude);
  gps.crack_datetime(&year,&month,&day,&hour,&minute,&second,&hundredths);
  Serial.print("Lat/Long: "); 
  Serial.print(latitude,5); 
  Serial.print(", "); 
  Serial.println(longitude,5);
  

  //logFile.println("y, m, d, h, m, s, hdr, lat, lon, gps_alt_m, gps_crs_deg, gps_spd_kmph, prs_hPA, temp_C, calc_alt, gy_x_rad/s, gy_y, gy_z, mg_x_uT, mg_y, mg_z, ac_x_m/s^2, ac_y, ac_z");
  
  logFile.print(year, DEC); logFile.print(", ");
  logFile.print(month, DEC); logFile.print(", ");
  logFile.print(day, DEC); logFile.print(", ");
  logFile.print(hour, DEC); logFile.print(", ");
  logFile.print(minute, DEC); logFile.print(", ");
  logFile.print(second, DEC); logFile.print(", ");
  logFile.print(hundredths, DEC); logFile.print(", ");
  logFile.print(latitude,5); logFile.print(", ");
  logFile.print(longitude,5); logFile.print(", ");
  logFile.print(gps.f_altitude()); logFile.print(", ");
  logFile.print(gps.f_course()); logFile.print(", ");
  logFile.print(gps.f_speed_kmph()); logFile.print(", ");
  
  bmp.getSensor(&sensor);
  bmp.getEvent(&event);
  logFile.print(event.pressure); logFile.print(", "); 
  
  float temperature;
  bmp.getTemperature(&temperature);
  logFile.print(temperature); logFile.print(", ");
  logFile.print(bmp.pressureToAltitude(SENSORS_PRESSURE_SEALEVELHPA,
                                        event.pressure));
  logFile.print(", ");
  
  gyro.getSensor(&sensor);
  gyro.getEvent(&event);

  logFile.print(event.gyro.x); logFile.print(", ");
  logFile.print(event.gyro.y); logFile.print(", ");
  logFile.print(event.gyro.z); logFile.print(", ");

  mag.getSensor(&sensor);
  mag.getEvent(&event);

  logFile.print(event.magnetic.x); logFile.print(", ");
  logFile.print(event.magnetic.y); logFile.print(", ");
  logFile.print(event.magnetic.z); logFile.print(", ");
  
  accel.getSensor(&sensor);
  accel.getEvent(&event);

  logFile.print(event.acceleration.x); logFile.print(", ");
  logFile.print(event.acceleration.y); logFile.print(", ");
  logFile.println(event.acceleration.z);
  
  logFile.close();
  Serial.println("done");
}
