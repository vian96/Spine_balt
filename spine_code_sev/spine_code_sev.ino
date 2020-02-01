#include <Kalman.h>
#include<Wire.h>

#define N_PINS 10
int pins[N_PINS];

const int MPU_addr=0x68;  // I2C address of the MPU-6050
Kalman kalmanX[N_PINS];
Kalman kalmanY[N_PINS];
int16_t AcX[N_PINS],AcY[N_PINS],AcZ[N_PINS],Tmp[N_PINS],GyX[N_PINS],GyY[N_PINS],GyZ[N_PINS];
double accXangle[N_PINS]; // Angle calculate using the accelerometer
double accYangle[N_PINS];
double temp[N_PINS];
double gyroXangle[N_PINS];// = {180, 180}; // Angle calculate using the gyro
double gyroYangle[N_PINS];// = {180, 180};
double gyroXrate[N_PINS], gyroYrate[N_PINS];
double kalAngleX[N_PINS]; // Calculate the angle using a Kalman filter
double kalAngleY[N_PINS];
uint32_t timer[N_PINS];

void setup(){ 
  for (int i=0; i<N_PINS; ++i)
    pins[i]=i+2;
  Wire.begin();
  for(int num_dat=0;num_dat<N_PINS;++num_dat){
    pinMode(pins[num_dat], OUTPUT);
    digitalWrite(pins[num_dat], LOW);
    Wire.beginTransmission(MPU_addr);
    Wire.write(0x6B);  // PWR_MGMT_1 register
    Wire.write(0);     // set to zero (wakes up the MPU-6050)
    Wire.endTransmission(true);
    digitalWrite(pins[num_dat], HIGH);
  }
  Serial.begin(9600);
}

void loop(){
  for(int num_dat=0;num_dat<N_PINS;++num_dat){
    digitalWrite(pins[num_dat], LOW);
       
    Wire.beginTransmission(MPU_addr);
    Wire.write(0x3B);  // starting with register 0x3B (ACCEL_XOUT_H)
    Wire.endTransmission(false);
    Wire.requestFrom(MPU_addr,14,true);  // request a total of 14 registers
    AcX[num_dat]=Wire.read()<<8|Wire.read();  // 0x3B (ACCEL_XOUT_H) & 0x3C (ACCEL_XOUT_L)    
    AcY[num_dat]=Wire.read()<<8|Wire.read();  // 0x3D (ACCEL_YOUT_H) & 0x3E (ACCEL_YOUT_L)
    AcZ[num_dat]=Wire.read()<<8|Wire.read();  // 0x3F (ACCEL_ZOUT_H) & 0x40 (ACCEL_ZOUT_L)
    Tmp[num_dat]=Wire.read()<<8|Wire.read();  // 0x41 (TEMP_OUT_H) & 0x42 (TEMP_OUT_L)
    GyX[num_dat]=Wire.read()<<8|Wire.read();  // 0x43 (GYRO_XOUT_H) & 0x44 (GYRO_XOUT_L)
    GyY[num_dat]=Wire.read()<<8|Wire.read();  // 0x45 (GYRO_YOUT_H) & 0x46 (GYRO_YOUT_L)
    GyZ[num_dat]=Wire.read()<<8|Wire.read();  // 0x47 (GYRO_ZOUT_H) & 0x48 (GYRO_ZOUT_L)
  //  Serial.print("AcX = "); Serial.print(AcX);
  //  Serial.print(" | AcY = "); Serial.print(AcY);
  //  Serial.print(" | AcZ = "); Serial.print(AcZ);
  //  Serial.print(" | Tmp = "); Serial.print(Tmp/340.00+36.53);  //equation for temperature in degrees C from datasheet
  //  Serial.print(" | GyX = "); Serial.print(GyX);
  //  Serial.print(" | GyY = "); Serial.print(GyY);
  //  Serial.print(" | GyZ = "); Serial.println(GyZ);
  
    /* Calculate the angls based on the different sensors and algorithm */
    accYangle[num_dat] = (atan2(AcX[num_dat],AcZ[num_dat])+PI)*RAD_TO_DEG;
    accXangle[num_dat] = (atan2(AcY[num_dat],AcZ[num_dat])+PI)*RAD_TO_DEG;  
    gyroXrate[num_dat] = (double)GyX[num_dat]/131.0;
    gyroYrate[num_dat] = -((double)GyY[num_dat]/131.0);
    gyroXangle[num_dat] += kalmanX[num_dat].getRate()*((double)(micros()-timer[num_dat])/1000000); // Calculate gyro angle using the unbiased rate
    gyroYangle[num_dat] += kalmanY[num_dat].getRate()*((double)(micros()-timer[num_dat])/1000000);
    kalAngleX[num_dat] = kalmanX[num_dat].getAngle(accXangle[num_dat], gyroXrate[num_dat], (double)(micros()-timer[num_dat])/1000000) - 180; // Calculate the angle using a Kalman filter
    kalAngleY[num_dat] = kalmanY[num_dat].getAngle(accYangle[num_dat], gyroYrate[num_dat], (double)(micros()-timer[num_dat])/1000000) - 180 ;
    timer[num_dat] = micros();
   
      //Serial.print(num_dat);
      //Serial.print(" ");
      //Serial.print("X:");
      Serial.print(kalAngleX[num_dat],0);
      Serial.print(" ");
      //Serial.print("Y:");
      Serial.print(kalAngleY[num_dat]-90,0);
      Serial.print(" ");
    
    digitalWrite(pins[num_dat], HIGH);
    delay(20);
  }
Serial.println(" ");
}
