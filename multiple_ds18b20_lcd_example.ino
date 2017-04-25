/********************************************************************/
// Script to read several (3) DS18B20 sensors and print to an LCD
// Hardware
// LCD: Sunfounder 20 x 4 LCD
// https://www.sunfounder.com/iic-i2c-twi-serial-2004-20x4-lcd-module-shield-for-arduino-uno-mega2560.html
// Arduino : Adafruit ESP8266 Feather Huzzah
// https://www.adafruit.com/product/2821
// Temperature Sensor(s) : WATERPROOF DS18B20 DIGITAL TEMPERATURE SENSOR
// https://www.adafruit.com/product/381
// Writen by JF Saraceno
// jfsaraceno@gmail.com
// Wiring:
// LCD ~~~~~~~~~ ARDUINO
// VCC ~~~~~~~~~ USB
// GND ~~~~~~~~~ GND
// SDA ~~~~~~~~~ SDA ()
// SCL ~~~~~~~~~ SCL ()

// DS18B20 ~~~~~~~~~ ARDUINO
// +V ~~~~~~~~~~~~~~ 3V
// GND ~~~~~~~~~~~~~ GND
// DATA ~~~~~~~~~~~~ 12
// 4.7KOhm resistor pull-up from 3V to DATA
/********************************************************************/
// First we include the libraries
#include <OneWire.h>
#include <DallasTemperature.h>
#include <LiquidCrystal_I2C.h>
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
  Serial.println("DS18B20 Temperature Sensor to LCD Demo");
  // Start up the library
  sensors.begin();
  lcd.begin();
  setup_lcd;
}

void loop(void)
{
  // request to all devices on the bus
  sensors.requestTemperatures(); // Send the command to get temperature readings
  update_lcd();
  delay(1000);
}

void update_lcd(void)
{
  lcd.setCursor(13, 0);
  lcd.print(sensors.getTempCByIndex(0)); // Why "byIndex"?
  lcd.setCursor(13, 1);
  lcd.print(sensors.getTempCByIndex(1)); // Why "byIndex"?
  lcd.setCursor(13, 2);
  lcd.print(sensors.getTempCByIndex(2)); // Why "byIndex"?

}
void setup_lcd(void) {
  lcd.backlight();
  lcd.setCursor(0, 0);
  lcd.print("TempSensor1:");
  lcd.setCursor(0, 1);
  lcd.print("TempSensor2:");
  lcd.setCursor(0, 2);
  lcd.print("TempSensor3:");

  lcd.setCursor(19, 0);
  lcd.print("C");
  lcd.setCursor(19, 1);
  lcd.print("C");
  lcd.setCursor(19, 2);
  lcd.print("C");
}
void debug(void) {
  Serial.print("Temperature sensor 1 reading is: ");
  Serial.println(sensors.getTempCByIndex(0)); // Why "byIndex"?
  Serial.print("Temperature sensor 2 reading is: ");
  Serial.println(sensors.getTempCByIndex(1)); // Why "byIndex"?
  Serial.print("Temperature sensor 3 reading is: ");
  Serial.println(sensors.getTempCByIndex(2)); // Why "byIndex"?
}
