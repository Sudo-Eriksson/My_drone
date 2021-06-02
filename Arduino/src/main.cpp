#include <Arduino.h>
#include <ServoTimer2.h>
#include <Wire.h>
#include <AltSoftSerial.h>

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
ServoTimer2 m1, m2, m3, m4;             // Motor 1 - 4

// Speed variables for the motors
float speed_m1 = 0;               // Initial speed for motor 1
float speed_m2 = 0;               // Initial speed for motor 2
float speed_m3 = 0;               // Initial speed for motor 3
float speed_m4 = 0;               // Initial speed for motor 4

unsigned long loopTimer = 0;               // Initial value for the loop timer

// Setup sensor variables
float gForceX, gForceY, gForceZ;  // Accelerometer sensor values
float rotX, rotY, rotZ;           // Gyroscope sensor values
float calibX, calibY, calibZ;     // Gyroscope calibration values

float tot_roll, tot_pitch;        // Angle calculating from the accelerometer
float roll_acc, pitch_acc;        // Total estimation from complementery filter

unsigned long looptime = 6700; //4000              // Looptime in microseconds
float dt = 0.000001*looptime;                     // loop time in seconds

float ref_roll = 0;                           // Reference value for the roll
float ref_pitch = 0;              // Reference value for the pitch

float e_roll, e_pitch, de_roll, de_pitch;        // Error for the roll and pitch
float old_e_roll = 0;                  // Differentiated error for D-part in PID
float old_e_pitch = 0;

// PID-controller values
float kp_roll = 0.008;//6;               // Kp for the roll
float kp_pitch = 0.008;//6;              // Kp for the pitch
float kd_roll = 0.075;//0.15 DENNA TAR JAG 25 % av ;//1;               // Kd for the roll 2 är nog ok
float kd_pitch = 0.075;//0.15 DENNA TAR JAG 25 % av ;//1;              // Kd for the pitch 2 är nog ok

unsigned int max_milli_speed = 1900;       // Max value the motor should be allowed to run
unsigned int min_milli_speed = 1100;       // Min value the motor should be allowed to run

float speed_base_thrust = 1100;   // Base value the motors should have

// Variables for the bluetooth communication
AltSoftSerial altSerial;

#define BUFFER_SIZE 10//4
char inData[BUFFER_SIZE];
char inChar=-1;
int i=0;

unsigned long last_data;

//int first_bytes;
int remaining_bytes;

String indata_str;

// ************* GENERAL ************* //
void shutDown(){
  // Here, something in the setup failed. We do not want to start the drone.
  Serial.println("Drone failed.");
  while(1);
}

// ************* MPU6050 ************* //
bool MPUReadAccel(){
  Wire.beginTransmission(MPU1_I2C_ADDRESS);
  Wire.write(MPU_ACCEL_READ_REG);
  Wire.endTransmission();
  Wire.requestFrom(MPU1_I2C_ADDRESS, MPU_ACCEL_READ_REG_SIZE);
  unsigned long timeout = millis() + MPU_READ_TIMEOUT;
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
  unsigned long timeout = millis() + MPU_READ_TIMEOUT;
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

void setMilliSpeed(unsigned int speed_in_micro, ServoTimer2 motor){

  if(speed_in_micro < min_milli_speed){
    // Lower limit to never stop the motor
    motor.write(min_milli_speed);
  }
  else if(speed_in_micro > max_milli_speed){
    // Upper limit to avoid motor go cray-cray
    motor.write(max_milli_speed);
  }
  else {
    motor.write((int)speed_in_micro);
  }
}

// ************* PID ************* //

void runPIDcontroller(){

  // Calculate the error
  e_roll = tot_roll - ref_roll;
  e_pitch = tot_pitch - ref_pitch;

  // Calculating the differentiated error for D
  de_roll = (e_roll - old_e_roll)/dt;
  de_pitch = (e_pitch - old_e_pitch)/dt;

  // Set the new e to old ones after using them in calculation
  old_e_roll = e_roll;
  old_e_pitch = e_pitch;

  //P-controll of roll and pitch
  // OLD MPU below
  //speed_m1 = speed_base_thrust - (e_roll * kp_roll) + (e_pitch * kp_pitch);
  //speed_m2 = speed_base_thrust + (e_roll * kp_roll) + (e_pitch * kp_pitch);
  //speed_m3 = speed_base_thrust - (e_roll * kp_roll) - (e_pitch * kp_pitch);
  //speed_m4 = speed_base_thrust + (e_roll * kp_roll) - (e_pitch * kp_pitch);

  //P-controll of roll and pitch
  // New MPU where x and y is flipped. Y forward, X sideways and Z up.
  speed_m1 = speed_base_thrust - (e_roll * kp_roll) - (e_pitch * kp_pitch);
  speed_m2 = speed_base_thrust - (e_roll * kp_roll) + (e_pitch * kp_pitch);
  speed_m3 = speed_base_thrust + (e_roll * kp_roll) - (e_pitch * kp_pitch);
  speed_m4 = speed_base_thrust + (e_roll * kp_roll) + (e_pitch * kp_pitch);

  //D-controll of roll and pitch
  speed_m1 = speed_m1 - (de_roll * kd_roll) - (de_pitch * kd_pitch);
  speed_m2 = speed_m2 - (de_roll * kd_roll) + (de_pitch * kd_pitch);
  speed_m3 = speed_m3 + (de_roll * kd_roll) - (de_pitch * kd_pitch);
  speed_m4 = speed_m4 + (de_roll * kd_roll) + (de_pitch * kd_pitch);

  // Set speeds according to the PID output
  setMilliSpeed(speed_m1, m1);
  setMilliSpeed(speed_m2, m2);
  setMilliSpeed(speed_m3, m3);
  setMilliSpeed(speed_m4, m4);
}

// ************* Bluetooth ************* //

bool waitForSetupSignal(int continueByte){

  int byte_count = altSerial.available();

  if(byte_count > 0){ 
    int byte_read = altSerial.read();
    if (byte_read == continueByte){
      Serial.print("Got setup msg "); Serial.println(String(byte_read));
      return true;
    }
    else{
      return false;
    }
  }
  return false;
}

bool getCtrlSignal(){

  int byte_count = altSerial.available();

  if(byte_count=BUFFER_SIZE){                          // Om tillräckligt många bytes har kommit
    remaining_bytes=byte_count - BUFFER_SIZE;           // Hur många bytes till övers som kommit
    //first_bytes = BUFFER_SIZE;                        // Hur många bytes vi vill läsa

    // Läs datan som blev skickad
    for(i=0;i<BUFFER_SIZE;i++){
      inChar=altSerial.read();
      inData[i]=inChar;
      }
    inData[i]="\0";
    indata_str = String(inData);

    return true;    // Här har vi alltså fått tillräckligt många bytes
  }
  else{
    return false;   // Här har inte tillräckligt många kommit
  }
}

void cleanBluetooth(){
  // Dags att ta bort de bytes som var överflödiga.
    for(i=0;i<remaining_bytes;i++){
      inChar=altSerial.read();    // Här läser vi bara de bytes som blev över. Tar bort dem så vi inte får buffer overrun.
     }
  }

bool validateRecivedData(String data){

  if (data.startsWith("FFF")){
    // PANIC MODE
    Serial.println("PANIC MODE ACTIVATED. Shutting down drone.");
    speed_base_thrust = 0;

    m1.write(1000);
    m2.write(1000);
    m3.write(1000);
    m4.write(1000);

    shutDown();
    return true; // Denna behövs egentligen inte men man får bort en varning.
  }
  else{
    return true;
    /*
    // Normal data got TODO: ÅTERIMPLEMENTERA DETTA!
    if(data.startsWith("1") && data.length() == 5){
        return true;
      }
    else{
        return false;
      }
    */
  }
}

void updateBaseThrust(){
  // If we have a new controll signal
  if(getCtrlSignal()){
      // If the data is not corrupted
      if(validateRecivedData(indata_str)){
        // Update the global base thrust variable
        speed_base_thrust = indata_str.substring(0,4).toInt();
        ref_pitch = indata_str.substring(6,8).toFloat() - 30.0;
        ref_roll = indata_str.substring(8,10).toFloat() - 30.0;
        last_data = millis();
      }
      cleanBluetooth();
    }
}

// ************* Main Program ************* //

void setup(){
  Serial.begin(9600);
  altSerial.begin(9600);
  
  // Wait for the computer to send permission to start setup.
  Serial.println("Waiting for connect msg to start setup.");
  while(!waitForSetupSignal(1)){ delay(100); }
  Serial.println(". Sending acc back."); 
  altSerial.print(String(1));

  Serial.println("------- SETUP MPU6050 -------");
  Serial.println("Waiting for setup and calibration msg of MPU6050.");
  while(!waitForSetupSignal(2)){ delay(100); }
  boolean res_MPU = SetupMPU();
  if(!res_MPU){
    shutDown();
  }
  Serial.println("-------- CALIBRATION --------");
  calibrateGyro();
  //calibrateAcc();
  Serial.println(". Sending acc back."); 
  altSerial.print(String(2));


  Serial.println("-------- MOTORS --------");
  while(!waitForSetupSignal(3)){ delay(100); }
  Serial.println("Arming m1");
  m1.attach(3);
  m1.write(1000);

  delay(3000);
  Serial.println("Arming m2");
  m2.attach(5);
  m2.write(1000);

  delay(3000);
  Serial.println("Arming m3");
  m3.attach(11);
  m3.write(1000);

  delay(3000);
  Serial.println("Arming m4");
  m4.attach(6);
  m4.write(1000);

  delay(2000);

  Serial.println(". Sending acc back."); 
  altSerial.print(String(3));

  Wire.begin();

  delay(2000);

  tot_roll = 0;
  tot_pitch = 0;

  Serial.println("-------- SETUP DONE ---------");
  digitalWrite(12, LOW);

  while(!waitForSetupSignal(4)){ delay(100); }
  Serial.println(". Sending acc back."); 
  altSerial.print(String(4));
}

void loop(){
  // How To start:
  // 1. Start bluetooth on the computer
  // 2. Start python program
  // 3. Start Drone by connecting the power
  // 4. Wait for calibration to complete
  // 5. Wait for the arming of the motors to complete
  // 6. Fly away!

  loopTimer = micros();
  ReadMPU();
  updateBaseThrust();
  runPIDcontroller();


  if(millis() - last_data > 1000){
    Serial.println("Connection to computer lost.");
    speed_base_thrust = 0;
    m1.write(1000);
    m2.write(1000);
    m3.write(1000);
    m4.write(1000);

    shutDown();
  }

  if(micros() - loopTimer > looptime){
      Serial.print("WARNING!! TOO LONG MAIN LOOP!: "); Serial.println(micros() - looptime);
    }

  while(micros() - loopTimer < looptime);
 }
