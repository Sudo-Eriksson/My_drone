#include <Arduino.h>
#include <Servo.h>
#include <Wire.h>
#include "SoftwareSerial.h"


#define STATUS_PIN                      13

#define MPU_POWER_REG                   0x6B
#define MPU_POWER_CYCLE                 0b00000000
#define MPU_READ_TIMEOUT                2000
#define MPU_SAMP_FREQ                   250

#define MPU_GYRO_CFG_REG                0x1B
#define MPU_GYRO_READ_REG               0x43
#define MPU_GYRO_READ_REG_SIZE          6
#define MPU_GYRO_CFG_500DEG             0b00001000
#define MPU_GYRO_READINGSCALE_500DEG    65.5
#define MPU_CALIBRATE_READING_NUM       2000

#define MPU_ACCEL_CFG_REG               0x1C
#define MPU_ACCEL_READ_REG              0x3B
#define MPU_ACCEL_READ_REG_SIZE         6
#define MPU_ACCEL_CFG_8G                0b00010000
#define MPU_ACCEL_READINGSCALE_8G       4096.0

#define MPU1_I2C_ADDRESS                0b1101000

//Set up servo motors
Servo m1, m2, m3, m4;             // Motor 1 - 4

// Speed variables for the motors
float speed_m1 = 0;               // Initial speed for motor 1
float speed_m2 = 0;               // Initial speed for motor 2
float speed_m3 = 0;               // Initial speed for motor 3
float speed_m4 = 0;               // Initial speed for motor 4

long loopTimer = 0;               // Initial value for the loop timer

// Setup sensor variables
float gForceX, gForceY, gForceZ;  // Accelerometer sensor values
float rotX, rotY, rotZ;           // Gyroscope sensor values
float calibX, calibY, calibZ;     // Gyroscope calibration values

float tot_roll, tot_pitch;        // Angle calculating from the accelerometer
float roll_acc, pitch_acc;        // Total estimation from complementery filter

int looptime = 6700;//4000;              // Looptime in microseconds
float dt = 0.000001*looptime;     // loop time in seconds

float ref_roll = 0;               // Reference value for the roll
float ref_pitch = 0;              // Reference value for the pitch

float e_roll, e_pitch;                     // Error for the roll and pitch

// PID-controller values
float kp_roll = 10;               // Kp for the roll
float kp_pitch = 10;              // Kp for the pitch

int max_milli_speed = 1600;       // Max value the motor should be allowed to run
int min_milli_speed = 1100;       // Min value the motor should be allowed to run

float speed_base_thrust = 1100;   // Base value the motors should have

// Variables for the bluetooth communication
SoftwareSerial Serial_connection(10,11);
#define BUFFER_SIZE 4
char inData[BUFFER_SIZE];
char inChar=-1;
int i=0;
int bt;

int first_bytes;
int remaining_bytes;

String indata_str;

// ************* GENERAL ************* //

void shutDownSetup(){
  // Here, something in the setup failed. We do not want to start the drone.
  Serial.println("Drone setup failed.");
  while(1);
}

// ************* MPU6050 ************* //
bool MPUReadAccel(){
  Wire.beginTransmission(MPU1_I2C_ADDRESS);
  Wire.write(MPU_ACCEL_READ_REG);
  Wire.endTransmission();
  Wire.requestFrom(MPU1_I2C_ADDRESS, MPU_ACCEL_READ_REG_SIZE);
  long timeout = millis() + MPU_READ_TIMEOUT;
  while(Wire.available() < MPU_ACCEL_READ_REG_SIZE && timeout < millis());
  if (timeout <= millis()) return false;
  gForceX = (long)(Wire.read() << 8 | Wire.read()) / MPU_ACCEL_READINGSCALE_8G;
  gForceY = (long)(Wire.read() << 8 | Wire.read()) / MPU_ACCEL_READINGSCALE_8G;
  gForceZ = (long)(Wire.read() << 8 | Wire.read()) / MPU_ACCEL_READINGSCALE_8G;
  return true;
}

bool MPUReadGyro(){
  Wire.beginTransmission(MPU1_I2C_ADDRESS);
  Wire.write(MPU_GYRO_READ_REG);
  Wire.endTransmission();
  Wire.requestFrom(MPU1_I2C_ADDRESS, MPU_GYRO_READ_REG_SIZE);
  long timeout = millis() + MPU_READ_TIMEOUT;
  while(Wire.available() < MPU_ACCEL_READ_REG_SIZE && timeout < millis());
  if (timeout <= millis()) return false;
  rotX = (long)(Wire.read() << 8 | Wire.read()) / MPU_GYRO_READINGSCALE_500DEG;
  rotY = (long)(Wire.read() << 8 | Wire.read()) / MPU_GYRO_READINGSCALE_500DEG;
  rotZ = (long)(Wire.read() << 8 | Wire.read()) / MPU_GYRO_READINGSCALE_500DEG;

  return true;
}

void calibrateGyro(){
  loopTimer = 0;

  digitalWrite(STATUS_PIN, HIGH);
  Serial.println("Calibrating Gyro");

  calibX = 0;
  calibY = 0;
  calibZ = 0;


  for(int i=0; i<MPU_CALIBRATE_READING_NUM;i++)
  {
    if(MPUReadGyro())
    {
      calibX += rotX;
      calibY += rotY;
      calibZ += rotZ;

      //wait for the next sample cycle
      while(micros() - loopTimer < 4000);
      loopTimer = micros();
    }
    else
    {
      i--;
    }
  }
  calibX = calibX / MPU_CALIBRATE_READING_NUM;
  calibY = calibY / MPU_CALIBRATE_READING_NUM;
  calibZ = calibZ / MPU_CALIBRATE_READING_NUM;

  Serial.print("x: ");
  Serial.print(calibX);
  Serial.print("y: ");
  Serial.print(calibY);
  Serial.print("z: ");
  Serial.println(calibZ);

  Serial.println("Calibration of Gyro Done.");

  digitalWrite(STATUS_PIN, LOW);
}

boolean SetupMPU(){
  Serial.println("Seting Up MPU6050");

  int res = 0;

  Wire.beginTransmission(MPU1_I2C_ADDRESS);
  Wire.write(MPU_POWER_REG);
  Wire.write(MPU_POWER_CYCLE);
  res += Wire.endTransmission();

  Wire.beginTransmission(MPU1_I2C_ADDRESS);
  Wire.write(MPU_GYRO_CFG_REG);
  Wire.write(MPU_GYRO_CFG_500DEG);
  res += Wire.endTransmission();

  Wire.beginTransmission(MPU1_I2C_ADDRESS);
  Wire.write(MPU_ACCEL_CFG_REG);
  Wire.write(MPU_ACCEL_CFG_8G);
  res += Wire.endTransmission();

  // Check the result of the setup
  if(res != 0){
    Serial.print("MPU setup failed. Return:"); Serial.println(res);
    return false;
  }
  else{
    return true;
  }

}

void complementaryFilter(){
  // Calculate the angles according to the accelerometer
  // 180/pi ≈ 57.29
  roll_acc = atan(gForceY/gForceZ)*(57.29);
  pitch_acc = -atan( (gForceX) / sqrt((gForceY*gForceY) + (gForceZ*gForceZ))) *(57.29);

  // Complementary filter
  tot_roll = 0.99*(tot_roll + ((rotX - calibX)*dt)) + 0.01*(roll_acc);
  tot_pitch = 0.99*(tot_pitch + ((rotY - calibY)*dt)) + 0.01*(pitch_acc);

  //Serial.print("Pitch: "); Serial.println(tot_pitch);

}

bool ReadMPU(){
  if(MPUReadAccel() && MPUReadGyro())
  {
    complementaryFilter();
    return true;
  }
  return false;
}

// ************* Motorer ************* //
void setMilliSpeed(int speed_in_micro, Servo motor){
  if(speed_in_micro < min_milli_speed){
    // Lower limit to never stop the motor
    motor.writeMicroseconds(min_milli_speed);
  }
  else if(speed_in_micro > max_milli_speed){
    // Upper limit to avoid motor go cray-cray
    motor.writeMicroseconds(max_milli_speed);
  }
  else {
    motor.writeMicroseconds((int)speed_in_micro);
  }
}

// ************* PID ************* //
void runPIDcontroller(){

  // Calculate the error

  e_roll = tot_roll - ref_roll;
  e_pitch = tot_pitch - ref_pitch;

  //Serial.print("Pitch e: "); Serial.println(e_pitch);
  //Serial.print("Roll e: "); Serial.println(e_roll);

  //P-controll of roll and pitch
  speed_m1 = speed_base_thrust - (e_roll * kp_roll) + (e_pitch * kp_pitch);
  speed_m2 = speed_base_thrust + (e_roll * kp_roll) + (e_pitch * kp_pitch);
  speed_m3 = speed_base_thrust - (e_roll * kp_roll) - (e_pitch * kp_pitch);
  speed_m4 = speed_base_thrust + (e_roll * kp_roll) - (e_pitch * kp_pitch);


  //Serial.println(speed_m1);
  // Set speeds according to the PID output
  setMilliSpeed(speed_m1, m1);
  setMilliSpeed(speed_m2, m2);
  setMilliSpeed(speed_m3, m3);
  setMilliSpeed(speed_m4, m4);
}

// ************* Bluetooth ************* //
bool getCtrlSignal(){

  int byte_count = Serial_connection.available();

  if(byte_count>=BUFFER_SIZE){                          // Om tillräckligt många bytes har kommit
    remaining_bytes=byte_count - BUFFER_SIZE;       // Hur många bytes till övers som kommit
    first_bytes = BUFFER_SIZE;                        // Hur många bytes vi vill läsa

    // Läs datan som blev skickad
    for(i=0;i<first_bytes;i++){
      inChar=Serial_connection.read();
      inData[i]=inChar;
      }
    inData[i]="\0";
    indata_str = String(inData);

    return true;                                      // Här har vi alltså fått tillräckligt många bytes
  }
  else{
    return false;
  }
}

void cleanBluetooth(){
  // Dags att ta bort de bytes som var överflödiga.
    for(i=0;i<remaining_bytes;i++){
      inChar=Serial_connection.read();    // Här läser vi bara de bytes som blev över. Tar bort dem så vi inte får buffer overrun.
     }
  }

bool validateRecivedData(String data){

  if (data.startsWith("FFF")){
    // PANIC MODE
    Serial.println("PANIC MODE ACTIVATED. Shutting down drone.");
    speed_base_thrust = 0;
    setMilliSpeed(1000, m1);
    setMilliSpeed(1000, m2);
    setMilliSpeed(1000, m3);
    setMilliSpeed(1000, m4);
    shutDownSetup();
  }
  else{
    // Normal data got
    if(data.startsWith("1") && data.length() == 5){
        return true;
      }
    else{
        return false;
      }
  }
}

void updateBaseThrust(){
  if(getCtrlSignal()){
      if(validateRecivedData(indata_str)){
        bt = indata_str.toInt();
        if(bt >= min_milli_speed && bt <= max_milli_speed){
          speed_base_thrust = bt;
        }
      }
      cleanBluetooth();
    }
}


void setup(){
  //digitalWrite(12, HIGH);
  Serial.begin(57600);
  Serial.println("-------- START SETUP --------");
  Serial.println("Attach battery now");
  delay(8000);

  Serial.println("-------- SETUP MPU6050 --------");
  boolean res_MPU = SetupMPU();
  if(!res_MPU){
    shutDownSetup();
  }

  Serial.println("-------- CALIBRATION --------");
  calibrateGyro();
  //calibrateAcc();

  // TODO: Fixa handskakning så vi vet att allt är korrekt uppsatt.
  // Typ handskakning mellan dator och python
  // Healthcheck på motorer
  // osv...
  Serial.println("-------- MOTORS --------");

  Serial.println("Arming m1");
  m1.attach(3, 1000, 2000);
  m1.writeMicroseconds(1000);

  delay(3000);
  Serial.println("Arming m2");
  m2.attach(5, 1000, 2000);
  m2.writeMicroseconds(1000);

  delay(3000);
  Serial.println("Arming m3");
  m3.attach(9, 1000, 2000);
  m3.writeMicroseconds(1000);

  delay(3000);
  Serial.println("Arming m4");
  m4.attach(6, 1000, 2000);
  m4.writeMicroseconds(1000);

  delay(3000);

  Wire.begin();

  delay(2000);
  Serial.println("-------- BLUETOOTH ---------");

  //Serial.begin(9600);
  //delay(7000);
  Serial_connection.begin(9600);
  /*
  // We must wait for the python program to start before setup is done.
  boolean python_ready = false;
  Serial.println("Start python now");
  Serial.println("Waiting on python...");
  while(!python_ready){
    if(getCtrlSignal()){
        if(indata_str == "0001\n"){
          python_ready = true;
          cleanBluetooth();
        }
      }
      Serial.println(indata_str);
      delay(1000);
  }
  */

  // Send to the python program that the drone is ready.
  Serial_connection.println("start");

  tot_roll = 0;
  tot_pitch = 0;

  Serial.println("-------- SETUP DONE ---------");
  digitalWrite(12, LOW);
}

void loop(){
  loopTimer = micros();

  updateBaseThrust();
  ReadMPU();
  runPIDcontroller();

  if(micros() - loopTimer > looptime){
      Serial.print("WARNING!! TOO LONG MAIN LOOP!: "); Serial.println(micros() - looptime);
    }

  while(micros() - loopTimer < looptime);
 }
