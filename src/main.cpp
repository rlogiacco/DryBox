#include <Arduino.h>
#include "EEPROM.h"
int addr = 0;
#define EEPROM_SIZE 64

#include <Encoder.h>
Encoder enc(5, 6);
long encPos  = -999;

#include <Wire.h> 
#include <LiquidCrystal_I2C.h>
#include <LiquidMenu.h>
LiquidCrystal_I2C lcd(0x27,20,4);  // set the LCD address to 0x27 for a 16 chars and 2 line display

//temp & humidity
#include <Adafruit_Sensor.h>
#include <DHT.h>
#define DHTPIN 2
#define DHTTYPE    DHT11     // DHT 11
DHT dht(DHTPIN, DHTTYPE);

#include <QuickPID.h>
#define PIN_INPUT 0
#define PIN_OUTPUT 3
float setpoint, input, output;
float Kp = 2, Ki = 5, Kd = 1;
QuickPID pid(&input, &output, &setpoint);

#include <sTune.h>
sTune tuner = sTune(); // for softPWM and tempLimit

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

  //dht
  dht.begin();

  //pid
  input = analogRead(PIN_INPUT);
  pid.Compute();
  analogWrite(PIN_OUTPUT, output);
}

void setup() {
  //eeprom
  EEPROM.begin(EEPROM_SIZE);
  load();

  Serial.println(" bytes read from Flash . Values are:");
  for (int i = 0; i < EEPROM_SIZE; i++) {
    Serial.print(byte(EEPROM.read(i)));
    Serial.print(" ");
  }

  //lcd
  lcd.init();
  lcd.backlight();
  Serial.begin(9600);

  //dht
  float h = dht.readHumidity();
  float t = dht.readTemperature();

  //pid
  input = analogRead(PIN_INPUT);
  setpoint = 100;
  pid.SetTunings(Kp, Ki, Kd);
  pid.SetMode(pid.Control::automatic);

  //tuner
  tuner.Configure(0, 0, 0, 0, testTimeSec, 0, samples);
  tuner.SetEmergencyStop(tempLimit);
}

void loop() {
  // put your main code here, to run repeatedly:
}

// put function definitions here:
int myFunction(int x, int y) {
  float optimumOutput = tuner.softPwm(relayPin, input, output, setpoint, outputSpan, 0); // ssr mode
  if (pid.Compute()) {
    if (!digitalRead(drdyPin)) Input = maxthermo.readThermocoupleTemperature();
    tuner.plotter(input, optimumOutput, setpoint, 0.5f, 3); // output scale 0.5, plot every 3rd sample
  }
}

int save() {
  EEPROM.put(addr, Kp);
  EEPROM.put(addr + sizeof(float), Ki);
  EEPROM.put(addr + (sizeof(float) * 2), Kp);
  EEPROM.commit();
}

int load() {
  EEPROM.get(addr, Kp);
  EEPROM.get(addr + sizeof(float), Ki);
  EEPROM.get(addr + (sizeof(float) * 2), Kd);
}