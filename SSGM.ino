#include <DHT.h>
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include "Adafruit_PM25AQI.h"

// Define sensor pins
#define DHTPIN 2
#define DHTTYPE DHT11
#define MQ4PIN A0
#define MQ7PIN A1
#define ALARMPIN 8
#define TRIGPIN 9
#define ECHOPIN 10
#define LEDPIN 11

// Initialize DHT sensor
DHT dht(DHTPIN, DHTTYPE);

// Initialize PMSA003I sensor
Adafruit_PM25AQI aqi = Adafruit_PM25AQI();

// Threshold values for sensors
const int methaneThreshold = 300;  // Example threshold value
const int coThreshold = 300;       // Example threshold value
const int pmThreshold = 50;        // Example threshold value (for PM2.5)
const int distanceThreshold = 50;  // Distance threshold in cm

void setup() {
  // Initialize serial communication
  Serial.begin(9600);
  
  // Initialize DHT sensor
  dht.begin();
  
  // Set alarm and LED pins as output
  pinMode(ALARMPIN, OUTPUT);
  pinMode(LEDPIN, OUTPUT);
  
  // Set HC-SR04 pins
  pinMode(TRIGPIN, OUTPUT);
  pinMode(ECHOPIN, INPUT);
  
  // Initialize PMSA003I sensor
  if (!aqi.begin_I2C()) {
    Serial.println("Could not find PMSA003I sensor!");
    while (1);
  }
}

void loop() {
  // Read temperature and humidity from DHT11
  float temperature = dht.readTemperature();
  float humidity = dht.readHumidity();
  
  // Read gas sensor values
  int methaneValue = analogRead(MQ4PIN);
  int coValue = analogRead(MQ7PIN);
  
  // Read particulate matter data from PMSA003I
  PM25_AQI_Data data;
  if (!aqi.read(&data)) {
    Serial.println("Could not read from PMSA003I sensor!");
  }
  
  // Print sensor values to serial monitor
  Serial.print("Temperature: ");
  Serial.print(temperature);
  Serial.print(" °C, Humidity: ");
  Serial.print(humidity);
  Serial.print(" %, Methane: ");
  Serial.print(methaneValue);
  Serial.print(", CO: ");
  Serial.print(coValue);
  Serial.print(", PM2.5: ");
  Serial.println(data.pm25_standard);
  
  // Send sensor values over Bluetooth
  Serial.print("Temperature: ");
  Serial.print(temperature);
  Serial.print(" °C, Humidity: ");
  Serial.print(humidity);
  Serial.print(" %, Methane: ");
  Serial.print(methaneValue);
  Serial.print(", CO: ");
  Serial.print(coValue);
  Serial.print(", PM2.5: ");
  Serial.println(data.pm25_standard);

  // Check if any sensor values exceed thresholds
  if (methaneValue > methaneThreshold || coValue > coThreshold || data.pm25_standard > pmThreshold) {
    // Sound the alarm
    digitalWrite(ALARMPIN, HIGH);
  } else {
    // Turn off the alarm
    digitalWrite(ALARMPIN, LOW);
  }
  
  // Ultrasonic sensor to detect objects
  long duration, distance;
  digitalWrite(TRIGPIN, LOW);
  delayMicroseconds(2);
  digitalWrite(TRIGPIN, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIGPIN, LOW);
  duration = pulseIn(ECHOPIN, HIGH);
  distance = (duration / 2) / 29.1;

  if (distance < distanceThreshold) {
    // Blink the LED
    digitalWrite(LEDPIN, HIGH);
    delay(500);
    digitalWrite(LEDPIN, LOW);
    delay(500);
  } else {
    digitalWrite(LEDPIN, LOW);
  }
  
  // Wait 2 seconds before next reading
  delay(2000);
}
void setup() {
  // put your setup code here, to run once:

}

void loop() {
  // put your main code here, to run repeatedly:

}
