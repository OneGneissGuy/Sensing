/********************************************************************/
//Script to read climate data and print ot an LCD screen
//Read temperature, pressure and humidity from a BME280 using I2C
//Read temperature, from a DS18B20 temperature sensor using One-wire
//Print all readings to a 20x4 LCD screen once per second
// Hardware
// LCD: Sunfounder 20 x 4 LCD
// https://www.sunfounder.com/iic-i2c-twi-serial-2004-20x4-lcd-module-shield-for-arduino-uno-mega2560.html
// Arduino : Adafruit ESP8266 Feather Huzzah
// https://www.adafruit.com/product/2821
// Temperature Sensor(s) : WATERPROOF DS18B20 DIGITAL TEMPERATURE SENSOR
// https://www.adafruit.com/product/381
// ADAFRUIT BME280 I2C OR SPI TEMPERATURE HUMIDITY PRESSURE SENSOR
// https://www.adafruit.com/product/2652
// Writen by JF Saraceno
// jfsaraceno@gmail.com
// Wiring:

//LCD SCREEN
// LCD ~~~~~~~~~ ARDUINO
// VCC ~~~~~~~~~ USB
// GND ~~~~~~~~~ GND
// SDA ~~~~~~~~~ SDA ()
// SCL ~~~~~~~~~ SCL ()

// BME280
// BME280 ~~~~~~~~~ ARDUINO
// VCC ~~~~~~~~~ USB
// GND ~~~~~~~~~ GND
// SDI ~~~~~~~~~ SDA ()
// SCK ~~~~~~~~~ SCL ()

// DS18B20 Temperature Sensor
// DS18B20 ~~~~~~~~~ ARDUINO
// Vin ~~~~~~~~~~~~~~ 3V
// GND ~~~~~~~~~~~~~ GND
// DATA ~~~~~~~~~~~~ 12
// 4.7KOhm resistor pull-up from 3V to DATA

/********************************************************************/
// First we include the libraries
#include <OneWire.h>
#include <DallasTemperature.h>
#include <Wire.h>
#include <LiquidCrystal_I2C.h>
#include <SPI.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BME280.h>

#define BME_SCK 13
#define BME_MISO 12
#define BME_MOSI 11
#define BME_CS 10
#define SEALEVELPRESSURE_HPA (1013.25) //update this to match local conditions
/********************************************************************/
Adafruit_BME280 bme; // I2C
/********************************************************************/
// Set the LCD address to 0x27 for a 20 chars and 4 line display
LiquidCrystal_I2C lcd(0x27, 20, 4);
/********************************************************************/
// Data wire is plugged into pin 2 on the Arduino
#define ONE_WIRE_BUS 12
/********************************************************************/
// Setup a oneWire instance to communicate with any OneWire devices
// (not just Maxim/Dallas temperature ICs)
OneWire oneWire(ONE_WIRE_BUS);
/********************************************************************/
// Pass our oneWire reference to Dallas Temperature.
DallasTemperature sensors(&oneWire);
/********************************************************************/
void setup(void)
{
  // start serial port
  Serial.begin(9600);
  // Start up the libraries
  bool status;
  status = bme.begin();
  if (!status) {
    Serial.println("Could not find a valid BME280 sensor, check wiring!");
    while (1);
  }
  Wire.begin();
  sensors.begin();
  lcd.begin();

  setup_lcd();
}
void loop(void)
{
  read_sensors();
  //debug();
  update_lcd();
}

void read_sensors(void) {
  float temp;
  float humidity;
  float pressure;
  sensors.requestTemperatures(); // Send the command to get temperature readings
  temp  = bme.readTemperature();
  pressure = (bme.readPressure() / 100.0F);
  humidity = bme.readHumidity();
}

void setup_lcd(void) {
  lcd.clear();
  lcd.backlight();
  //Setup the screen headers and units
  lcd.setCursor(0, 0);
  lcd.print("DS18B20 Temp:");
  lcd.setCursor(19, 0);
  lcd.print("C");

  lcd.setCursor(0, 1);
  lcd.print("BME280 Temp:");
  lcd.setCursor(19, 1);
  lcd.print("C");

  lcd.setCursor(0, 2);
  lcd.print("Humidity:");
  lcd.setCursor(19, 2);
  lcd.print("%");

  lcd.setCursor(0, 3);
  lcd.print("Pressure:");
  lcd.setCursor(17, 3);
  lcd.print("hPa");
}
void update_lcd(void)
{
  lcd.setCursor(13, 0);
  lcd.print(sensors.getTempCByIndex(0)); // Why "byIndex"?
  // You can have more than one DS18B20 on the same bus.
  // 0 refers to the first IC on the wire
  lcd.setCursor(13, 1);
  lcd.print(bme.readTemperature()); //  temperature
  lcd.setCursor(13, 2);
  lcd.print(bme.readHumidity()); //  humidity
  lcd.setCursor(9, 3);
  lcd.print(bme.readPressure() / 100.0F); //  humidity
  delay(1000);
}

void debug(void) {
  Serial.print("ds18B20 Temperature = ");
  Serial.print(sensors.getTempCByIndex(0));
  Serial.println(" *C");

  Serial.print("BME280 Temperature = ");
  Serial.print(bme.readTemperature());
  Serial.println(" *C");

  Serial.print("BME280 Pressure = ");
  Serial.print(bme.readPressure() / 100.0F);
  Serial.println(" hPa");

  Serial.print("Approx. Altitude = ");
  Serial.print(bme.readAltitude(SEALEVELPRESSURE_HPA));
  Serial.println(" m");

  Serial.print("BME280 Humidity = ");
  Serial.print(bme.readHumidity());
  Serial.println(" %");
  Serial.println();
}

float Celcius2Fahrenheit(float celcius) {
  return 1.8 * celcius + 32;
}

