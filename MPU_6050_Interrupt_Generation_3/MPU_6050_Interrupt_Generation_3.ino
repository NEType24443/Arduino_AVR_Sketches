#include <Wire.h>
#include <MPU6050.h>
#include <avr/sleep.h>
#include<Streaming.h>

#define LED_PIN 13
#define INTERRUPT_PIN 2

#define MOTION_THRESHOLD 13
#define MOTION_EVENT_DURATION 100

MPU6050 accelgyro;
volatile bool ledState = false;

void setup() {
  Wire.begin();
  Serial.begin(115200);
  pinMode(LED_PIN, OUTPUT);
  accelgyro.initialize();
  // verify connection to MPU6050
  Serial<<"Testing device connections..."<<endl;
  Serial<<(accelgyro.testConnection() ? "MPU6050 connection successful" : "MPU6050 connection failed")<<endl;
  delay(1);
  accelgyro.setAccelerometerPowerOnDelay(3);
  
  accelgyro.setInterruptMode(true); // Interrupts enabled
  
  accelgyro.setInterruptLatch(0); // Interrupt pulses when triggered instead of remaining on until cleared
  
  accelgyro.setIntMotionEnabled(true); // Interrupts sent when motion detected
  // Set sensor filter mode.
  // 0 -> Reset (disable high pass filter)
  // 1 -> On (5Hz)
  // 2 -> On (2.5Hz)
  // 3 -> On (1.25Hz)
  // 4 -> On (0.63Hz)
  // 5 -> Hold (Future outputs are relative to last output when this mode was set)
  accelgyro.setDHPFMode(1);

  // Motion detection acceleration threshold. 1LSB = 2mg.
  accelgyro.setMotionDetectionThreshold(MOTION_THRESHOLD);
  // Motion detection event duration in ms
  accelgyro.setMotionDetectionDuration(MOTION_EVENT_DURATION);
  digitalWrite(LED_PIN,LOW);
  delay(1);
}

void sleepNow() {
  Serial<<"Sleeping Now"<<endl;
  delay(5);
  attachInterrupt(digitalPinToInterrupt(INTERRUPT_PIN), wakeUp, RISING);
  sleep_enable();
  set_sleep_mode(SLEEP_MODE_PWR_DOWN);
  sleep_cpu();
}

void wakeUp() {
  sleep_disable();
  detachInterrupt(digitalPinToInterrupt(INTERRUPT_PIN));
  ledState ^= 1;
  Serial<<"Woke Up!, "<<ledState<<endl;
  delay(30);
}

void loop() {
  digitalWrite(LED_PIN, ledState);
  sleepNow();
}
