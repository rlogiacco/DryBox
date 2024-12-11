#include <Arduino.h>
#include <FormattingSerialDebug.h>
#include "EEPROM.h"
#define EEPROM_ADDR 0;
#define EEPROM_SIZE 64

#include <Encoder.h>
#define ENCODER_A 2
#define ENCODER_B 3
Encoder enc(ENCODER_A, ENCODER_B);
long encPos  = -999;

#define HEATER 3
#define HEATER_FAN 4
#define VENTING_FAN 5

#include <Wire.h> 
#include <LiquidCrystal_I2C.h>
//#include <LiquidMenu.h>
#define LCD_BACKLIGHT 6
LiquidCrystal_I2C lcd(0x27, 16, 2);

//temp & humidity
#include <Adafruit_Sensor.h>
#include <DHT.h>
#define SENSOR 2
DHT dht(SENSOR, DHT11);
float humidity,temperature;

#include <QuickPID.h>
#define PIN_INPUT 0
#define PIN_OUTPUT 3
float setpoint, input, output;
float Kp = 2, Ki = 5, Kd = 1;
QuickPID pid(&input, &output, &setpoint);

#include <sTune.h>
#define TUNE_TEMP_LIMIT 100
#define TUNE_TIMER 30
#define TUNE_SAMPLES 10
#define TUNE_PWM_DURATION 50
sTune tuner = sTune();


void update() {
  humidity = dht.readHumidity();
  temperature = dht.readTemperature();

  input = analogRead(PIN_INPUT);
  pid.Compute();
  analogWrite(PIN_OUTPUT, output);

}

void tune(int x, int y) {
  float optimumOutput = tuner.softPwm(HEATER, input, output, setpoint, TUNE_PWM_DURATION, 0); // ssr mode
  if (pid.Compute()) {
    tuner.plotter(input, optimumOutput, setpoint, 0.5f, 3); // output scale 0.5, plot every 3rd sample
  }
}

void save() {
  int addr = EEPROM_ADDR;
  EEPROM.put(addr, Kp);
  DEBUG("addr %i %f", addr, Kp);
  EEPROM.put(addr += sizeof(float), Ki);
  DEBUG("addr %i %f", addr, Ki);
  EEPROM.put(addr += sizeof(float), Kd);
  DEBUG("addr %i %f", addr, Kd);
  EEPROM.commit();
}

void load() {
  int addr = EEPROM_ADDR;
  EEPROM.get(addr, Kp);
  DEBUG("addr %i %f", addr, Kp);
  EEPROM.get(addr += sizeof(float), Ki);
  DEBUG("addr %i %f", addr, Ki);
  EEPROM.get(addr += sizeof(float), Kd);
  DEBUG("addr %i %f", addr, Kd);
}

void setup() {
  Serial.begin(9600);
  //eeprom
  EEPROM.begin(EEPROM_SIZE);
  load();

  //lcd
  lcd.init();
  lcd.backlight();
  Serial.begin(9600);

  //dht
  dht.begin();

  //pid
  input = analogRead(PIN_INPUT);
  setpoint = 100;
  pid.SetTunings(Kp, Ki, Kd);
  pid.SetMode(pid.Control::automatic);

  //tuner
  tuner.Configure(0, 0, 0, 0, TUNE_TIMER, 0, TUNE_SAMPLES);
  tuner.SetEmergencyStop(TUNE_TEMP_LIMIT);
}

void loop() {
  // encoder
  long position = enc.read();
  if (position != encPos) {
    encPos = position;
  }

  //lcd
  // when characters arrive over the serial port...
  if (Serial.available()) {
    // wait a bit for the entire message to arrive
    delay(100);
    // clear the screen
    lcd.clear();
    // read all the available characters
    while (Serial.available() > 0) {
      // display each character to the LCD
      lcd.write(Serial.read());
    }
  }
}