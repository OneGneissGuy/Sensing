/********************************************************************/
//Script to read climate data and print to an i2c LCD screen and report
//data to a thingspeak channel
//Read temperature, pressure and humidity from a BME280 using I2C
//Read temperature, from a DS18 temperature sensor using One-wire
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
#include <ESP8266WiFi.h> //ESP8266 library
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
//wifi settings
const char* ssid     = "";
const char* password = "";
WiFiClient client;

//thingspeak settings
const char* server = "api.thingspeak.com";
String apiKey = "";
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
unsigned long previousMillis = 0;        // will store last time LED was updated
unsigned long previousLongMillis = 0;
const long interval = 1000;           // interval at which to sample (milliseconds)
const long send_interval = 180000;           // interval at which to sample (milliseconds)

void setup(void)
{
  // start serial port
  Serial.begin(9600);
  //connect to the wifi
  WiFi.begin(ssid, password);

  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println("");
  Serial.println("WiFi connected");

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
  unsigned long currentMillis = millis();
  //sample loop
  if (currentMillis - previousMillis >= interval) {
    // save the last time you blinked the LED
    previousMillis = currentMillis;
    double t1, p, h, t2;
    read_sensors(t1, p, h, t2);
    Serial.println("Sampling...");

    //debug();
    update_lcd();
    //send loop
    if (currentMillis - previousLongMillis >= send_interval) {
      // save the last time you blinked the LED
      previousLongMillis = currentMillis;
      send_data(t1, h, p, t2);
      Serial.println("Sending...");
      // thingspeak needs minimum 15 sec delay between updates
    }
  }
}

void send_data(float temp1, float humid, float pressu, float temp2)
{
  if (client.connect(server, 80)) { // "184.106.153.149" or api.thingspeak.com
    String postStr = apiKey;
    postStr += "&field1=";
    postStr += String(temp1);
    postStr += "&field2=";
    postStr += String(humid);
    postStr += "&field3=";
    postStr += String(pressu);
    postStr += "&field4=";
    postStr += String(temp2);
    postStr += "\r\n\r\n";

    client.print("POST /update HTTP/1.1\n");
    client.print("Host: api.thingspeak.com\n");
    client.print("Connection: close\n");
    client.print("X-THINGSPEAKAPIKEY: " + apiKey + "\n");
    client.print("Content-Type: application/x-www-form-urlencoded\n");
    client.print("Content-Length: ");
    client.print(postStr.length());
    client.print("\n\n");
    client.print(postStr);

    Serial.print("BME280 Temperature: ");
    Serial.print(temp1);
    Serial.print(" degrees Celcius, Humidity: ");
    Serial.print(humid);
    Serial.print(" %, & Pressure: ");
    Serial.print(pressu);
    Serial.println(" hPa sent to Thingspeak");
  }
  else
  {
    Serial.println("Connection error");
  }
  client.stop();
}

void read_sensors(double & temp1, double & pressure, double & humidity, double & temp2) {
  // sample and read BME280
  temp1  = bme.readTemperature();
  pressure = (bme.readPressure() / 100.0F);
  humidity = bme.readHumidity();
  //sample DS18B20
  sensors.requestTemperatures(); // Send the command to get temperature readings
  //read DS18B20
  temp2 = sensors.getTempCByIndex(0);
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
  //  Serial.print("ds18B20 Temperature = ");
  //  Serial.print(sensors.getTempCByIndex(0));
  //  Serial.println(" *C");

  Serial.print("BME280 Temperature = ");
  Serial.print(bme.readTemperature());
  Serial.println(" *C");

  Serial.print("BME280 Humidity = ");
  Serial.print(bme.readHumidity());
  Serial.println(" %");

  Serial.print("BME280 Pressure = ");
  Serial.print(bme.readPressure() / 100.0F);
  Serial.println(" hPa");

  Serial.println();
}

float Celcius2Fahrenheit(float celcius) {
  return 1.8 * celcius + 32;
}
