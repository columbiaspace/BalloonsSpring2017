#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BMP085_U.h>
#include <Adafruit_LiquidCrystal.h>  // LiquidCrystal using the Wire library
   
Adafruit_BMP085_Unified bmp = Adafruit_BMP085_Unified(10085);
Adafruit_LiquidCrystal lcd(0);
 
void setup(void) 
{
  Serial.begin(9600);
  Serial.println("Pressure Sensor Test"); Serial.println("");
  
  lcd.begin(16, 2);  // set up the LCD's number of rows and columns: 
  lcd.setBacklight(HIGH); // Set backlight (HIGH - on)
  
  /* Initialise the sensor */
  if(!bmp.begin())
  {
    /* There was a problem detecting the BMP085 ... check your connections */
    Serial.print("Ooops, no BMP085 detected ... Check your wiring or I2C ADDR!");
    while(1);
  }
}
 
void loop(void) 
{
  sensors_event_t event;
  bmp.getEvent(&event);
 
  /* Display the results (barometric pressure is measure in hPa) */
  if (event.pressure)
  {
    /* Display atmospheric pressue in hPa */
    Serial.print("Pressure:    ");
    Serial.print(event.pressure);
    Serial.println(" hPa");
    
    /* Calculating altitude with reasonable accuracy requires pressure    *
     * sea level pressure for your position at the moment the data is     *
     * converted, as well as the ambient temperature in degress           *
     * celcius.  If you don't have these values, a 'generic' value of     *
     * 1013.25 hPa can be used (defined as SENSORS_PRESSURE_SEALEVELHPA   *
     * in sensors.h), but this isn't ideal and will give variable         *
     * results from one day to the next.                                  *
     *                                                                    *
     * You can usually find the current SLP value by looking at weather   *
     * websites or from environmental information centers near any major  *
     * airport.                                                           *
     *                                                                    *
     * For example, for Paris, France you can check the current mean      *
     * pressure and sea level at: http://bit.ly/16Au8ol                   */
     
    /* First we get the current temperature from the BMP085 */
    float temperature;
    bmp.getTemperature(&temperature);
    Serial.print("Temperature: ");
    Serial.print(temperature);
    Serial.println(" C");
 
    /* Then convert the atmospheric pressure, SLP and temp to altitude    */
    /* Update this next line with the current SLP for better results      */
    float seaLevelPressure = SENSORS_PRESSURE_SEALEVELHPA;
    Serial.print("Altitude:    "); 
    Serial.print(bmp.pressureToAltitude(seaLevelPressure,
                                        event.pressure,
                                        temperature)); 
    Serial.println(" m");
    Serial.println("");
    
    lcd.setCursor(0,1);
    lcd.print("Alt: ");
    lcd.setCursor(9,1);
    lcd.print(bmp.pressureToAltitude(seaLevelPressure,event.pressure,temperature));
    lcd.setCursor(15,1);
    lcd.print("m");

  }
  else
  {
    Serial.println("Sensor error");
    lcd.setCursor(0,1);
    lcd.print("BMP ERROR");
  }
  delay(1000);
}
