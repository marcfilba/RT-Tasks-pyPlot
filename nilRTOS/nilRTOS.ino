#include <NilRTOS.h>

#include "I2Cdev.h"
#include "MPU6050.h"

#include <Wire.h>

#include <SPI.h>
#include <Ethernet.h>
#include <EthernetUdp.h>

#define FACTOR_ESCALAT_TEMPS        100

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

byte myMAC [] = { 0x90, 0xA2, 0xDA, 0x0F, 0x49, 0x41 };
IPAddress myIP (172, 16, 10, 50);
IPAddress RIP (172, 16, 10, 255);

unsigned int localPort = 8889; // local port to listen on
char packetBuffer [UDP_TX_PACKET_MAX_SIZE]; //buffer to hold incoming packet,
EthernetUDP Udp; // EthernetUDP instance to let us receive packets over UDP

// Declare a stack with 128 bytes beyond context switch and interrupt needs.
NIL_WORKING_AREA(waThread1, 128);

// Declare the thread function for thread 1.
NIL_THREAD(Thread1, arg) {
  while (1){
    data[0] = 1;
    data[1] = 0;
    data[2] = 0;
    printData ();
    
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
    printData ();
    nilThdSleepMilliseconds (20 * FACTOR_ESCALAT_TEMPS);
  }
}

NIL_WORKING_AREA(waThread2, 128);

NIL_THREAD(Thread2, arg) {
  while (1){
    data[0] = 0;
    data[1] = 1;
    data[2] = 0;
    printData ();
    
    float AcX = (float (accelgyro.getAccelerationX()) / equivAccel);
    desiredRPM = min(MOTOR_MAX_RPM, AcX*MOTOR_MAX_RPM);
    
    /*Udp.beginPacket (RIP, 8889);
    Udp.write (String(desiredRPM + "\n").c_str());
    Udp.endPacket();*/
    
    data[0] = 0;
    data[1] = 0;
    data[2] = 0;
    printData ();
    nilThdSleepMilliseconds (100 * FACTOR_ESCALAT_TEMPS); 
  }
}

NIL_WORKING_AREA(waThread3, 128);

NIL_THREAD(Thread3, arg) {
  while (1){
    data[0] = 0;
    data[1] = 0;
    data[2] = 1;
    printData ();
    /*if (Udp.parsePacket()){
      Udp.read (packetBuffer, UDP_TX_PACKET_MAX_SIZE);
      int udpRPM = String(packetBuffer).toInt();
      // CAL FER LA MITJA???
    }*/
    for (int i = 0; i < 1000; ++i);
    data[0] = 0;
    data[1] = 0;
    data[2] = 0;
    printData ();
    nilThdSleepMilliseconds (100 * FACTOR_ESCALAT_TEMPS);
  }
}

NIL_WORKING_AREA(waThread4, 128);

NIL_THREAD(Thread4, arg) {
  while (1){
    printData ();
    nilThdSleepMilliseconds (5 * FACTOR_ESCALAT_TEMPS);
  }
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

NIL_THREADS_TABLE_BEGIN()
NIL_THREADS_TABLE_ENTRY("motorControl",              Thread1, NULL, waThread1, sizeof(waThread1))
NIL_THREADS_TABLE_ENTRY("generateAndSendLocalSpeed", Thread2, NULL, waThread2, sizeof(waThread2))
NIL_THREADS_TABLE_ENTRY("receiveAndAverageSpeed",    Thread3, NULL, waThread3, sizeof(waThread3))
//NIL_THREADS_TABLE_ENTRY("monitor",                   Thread4, NULL, waThread4, sizeof(waThread4))
NIL_THREADS_TABLE_END()
//------------------------------------------------------------------------------
void setup() {
  // Start Nil RTOS.

  pinMode(MOTOR_ENABLE, OUTPUT);
  pinMode(MOTOR_LEFT,   OUTPUT);
  pinMode(MOTOR_RIGHT,  OUTPUT);
  
  pinMode (MOTOR_ENC_A, INPUT);
  pinMode (MOTOR_ENC_B, INPUT);

  digitalWrite (MOTOR_ENC_B, HIGH);
  
  for (int i = 0; i < 3; ++i) data[i] = 0;
  
  //Ethernet.begin (myMAC, myIP);

  Serial.begin(115200);

  accelgyro.initialize();

  setRPM (120);
  startMotor();
  
  nilSysBegin();
}
//------------------------------------------------------------------------------
void loop() {
  // Not used.
}
