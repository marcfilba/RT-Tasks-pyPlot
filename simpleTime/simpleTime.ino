#include <SimpleTimer.h>

SimpleTimer timer;

#include "I2Cdev.h"
#include "MPU6050.h"

#include <Wire.h>

#include <SPI.h>
#include <Ethernet.h>
#include <EthernetUdp.h>

#define FACTOR_ESCALAT_TEMPS        3

#define MPU6050_ACCEL_FS_2_EQUIV    16384
#define MPU6050_ACCEL_FS_4_EQUIV    8192
#define MPU6050_ACCEL_FS_8_EQUIV    4096
#define MPU6050_ACCEL_FS_16_EQUIV   2048

#define MPU6050_GYRO_FS_250_EQUIV   131
#define MPU6050_GYRO_FS_500_EQUIV   65,5
#define MPU6050_GYRO_FS_1000_EQUIV  32,8
#define MPU6050_GYRO_FS_2000_EQUIV  16,4

#define MOTOR_ENABLE   5
#define MOTOR_LEFT     6
#define MOTOR_RIGHT    7

#define MOTOR_ENC_A    3
#define MOTOR_ENC_B    2

#define MOTOR_MAX_RPM  220

int data [3];

MPU6050 accelgyro;

String inString = "";

int equivAccel = MPU6050_ACCEL_FS_2_EQUIV;
int equivGyro = MPU6050_GYRO_FS_250_EQUIV;

int realRPM, desiredRPM, correctedRPM = 0;
int error, integralError = 0;
int Kp, Ki = 1;

int sum = 0;
int numSum = 0;

byte myMAC [] = { 0x90, 0xA2, 0xDA, 0x0F, 0x49, 0x21 };
IPAddress myIP (172, 16, 10, 239);
IPAddress RIP (172, 16, 10, 255);
//IPAddress RIP (172, 16, 10, 209);

unsigned int localPort = 8889; // local port to listen on
int packetBuffer;
EthernetUDP Udp; // EthernetUDP instance to let us receive packets over UDP

void thread1() {
  data[0] = 1;
  data[1] = 0;
  data[2] = 0;
  //printData ();
  
  // velocity=readEencoder();
  realRPM = 1 / ((pulseIn (MOTOR_ENC_B, HIGH) * 3 / 1000000.0 ));

  // error=velocitySetpoint-velocity;
  error = desiredRPM - realRPM;

  // integralError=integralError+error;
  integralError += (desiredRPM - realRPM);    

  // controlSignal=Kp*error+Ki*IntegralError;
  correctedRPM = desiredRPM + ((Kp * error) + (Ki * integralError));

  // applyInActuators(controlSignal);
  analogWrite (MOTOR_ENABLE, max (min (correctedRPM, 255), 0));

  // en cas d'overflow, corregim els càlculs fets
  if (correctedRPM > 255 or correctedRPM < 0) integralError -= (desiredRPM - realRPM);
  
  data[0] = 0;
  data[1] = 0;
  data[2] = 0;
  //printData ();
}

void thread2() {
  data[0] = 0;
  data[1] = 1;
  data[2] = 0;
  //printData ();
  
  float AcX = (float (accelgyro.getAccelerationX()) / equivAccel);
  Udp.beginPacket (RIP, 8889);
  //Udp.write (String(abs(AcX*MOTOR_MAX_RPM) + "\n").c_str());
  Udp.write ((int) min((AcX*220), 220));
  Udp.endPacket();
  
  data[0] = 0;
  data[1] = 0;
  data[2] = 0;
 //printData ();
}

void thread3() {
  data[0] = 0;
  data[1] = 0;
  data[2] = 1;
  //printData ();
  if (Udp.parsePacket()){
    Udp.read ( (char*) &packetBuffer, UDP_TX_PACKET_MAX_SIZE);
    numSum++;
    sum += packetBuffer;
    
    Serial.print("From ");
    IPAddress remote = Udp.remoteIP();
    for (int i = 0; i < 4; i++) {
      Serial.print(remote[i], DEC);
      if (i < 3) {
        Serial.print(".");
      }
    }
    Serial.print (" ");
    Serial.println (packetBuffer);
    
    if (numSum == 5){
      desiredRPM = min(MOTOR_MAX_RPM, sum/5);
      Serial.println ("Mitja: " + String(sum/5));
      numSum = 0;
      sum = 0;
    }
  }
  for (int i = 0; i < 1000; ++i);
  data[0] = 0;
  data[1] = 0;
  data[2] = 0;
  //printData ();
}

void setRPM (int rpm){
  desiredRPM = max( min (rpm, MOTOR_MAX_RPM), 0);
}

void startMotor (){
  setRPM (desiredRPM);
  digitalWrite (MOTOR_LEFT,   HIGH);
  digitalWrite (MOTOR_RIGHT,  LOW);
}

void printData (){
  for (int i = 0; i < 3; ++i){
    Serial.print (data[i]);
    if (i != 2) Serial.print(',');
  }
  Serial.print('\n');
}

void setup() {
    Serial.begin(115200);
    timer.setInterval (20 * FACTOR_ESCALAT_TEMPS,  thread1);
    timer.setInterval (100 * FACTOR_ESCALAT_TEMPS, thread2);
    timer.setInterval (100 * FACTOR_ESCALAT_TEMPS, thread3);
    
    pinMode(MOTOR_ENABLE, OUTPUT);
    pinMode(MOTOR_LEFT,   OUTPUT);
    pinMode(MOTOR_RIGHT,  OUTPUT);
    
    pinMode (MOTOR_ENC_A, INPUT);
    pinMode (MOTOR_ENC_B, INPUT);
    
    digitalWrite (MOTOR_ENC_B, HIGH);
    
    for (int i = 0; i < 3; ++i) data[i] = 0;
    
    Ethernet.begin (myMAC, myIP);
    Udp.begin (localPort);
    
    
    Serial.begin(115200);
    Serial.println ("dale");
    
    accelgyro.initialize();
    
    setRPM (220);
    startMotor();

}

void loop() {
    timer.run();
}
