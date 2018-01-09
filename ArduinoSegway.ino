/* Arduino Segway Code

 Self balancing 2 wheeled robot using Arduino UNO & MPU6050 gyroscope/accelerometer module
 Additional nRF24L01 wireless module used to remotely drive the segway
 (Rough) Combination of other open source codes for similar projects + variations that worked best

Thanks! 
Shannon Charls
email: charlsshannon@gmail.com 
if you have questions or want schematics or MATLAB code used to calculate PID lmk 
.************************************************************************************************************/


//Libraries used
#include <Wire.h>
#include <SPI.h>  
#include <RF24.h>   //I think I decided on the nRF24L01 library instead of this one
#include <nRF24L01.h> //its up to you
#include <I2Cdev.h>
#include <MPU6050.h>

MPU6050 accelgyro;
MPU6050 initialize;
int16_t ax, ay, az;
int16_t gx, gy, gz;

/*These depend on specific model. Try guessing or use MATLAB/Simulink for an iverted pendulum*/
#define Gry_offset 1  //The offset of the gyro
#define Gyr_Gain 16.348
#define Angle_offset -10  // The offset of the accelerator
#define RMotor_offset -0.5 // The offset of right motor
#define LMotor_offset 0.5  // The offset of left motor
#define pi 3.14159 

float kp, ki, kd;
float Angle_Raw, Angle_Filtered, omega;
float Turn_Speed = 0, Turn_Speed_K = 0;
float Run_Speed = 0, Run_Speed_K = 0;
float LOutput,ROutput;

unsigned long preTime = 0;
unsigned long lastTime;
float Input, Output;
float errSum, dErr, error, lastErr;
int timeChange; 

//Motor driver pin assignments
int TN1=6;
int TN2=4;
int ENA=5;
int TN3=3;
int TN4=9;
int ENB=10;

byte addresses[][6] = {"0"}; 

//nRF24L01 wireless reciever & transiever for remote 
RF24 myRadio (7, 8); 
struct package
{
  int X;
  int Y;
  float angle;
  float omega;
  int speed; 
  uint16_t null_1; 
  uint16_t null_2;
};

typedef struct package Package;
Package data;

/*Extended data from the joysticks
 Not currently used but could change axis' from 8bit to 16bit depending on how much control you want
 
struct Axis  // Datas from remote control 
{
  uint16_t axis_1;
  uint16_t axis_2;
  uint16_t axis_3;
  uint16_t axis_4;
  uint16_t axis_5;
  uint16_t axis_6;
  uint16_t axis_7;
  uint16_t axis_8;
};
Axis axis_x;
*/

void setup()
{
  Wire.begin();
  Serial.begin(115200);
  
  /* If turned on with angle > |45| degrees, dont start the robot. Saves batteries. */
  accelgyro.initialize();
  initialize.setFullScaleGyroRange(MPU6050_GYRO_FS_2000);;
  for(int i = 0; i < 200; i++)  // Looping 200 times to get the real gesture when starting
  {
    Filter();   //Kalman filter
  }
  if (abs(Angle_Filtered) < 45)  // Start after filtering data & checking starting angle
  {
    omega = Angle_Raw = Angle_Filtered = 0;
    Filter();
    Output = error = errSum = dErr = Run_Speed = Turn_Speed = 0;
    myPID();
  }

  pinMode(TN1,OUTPUT);
  pinMode(TN2,OUTPUT);
  pinMode(TN3,OUTPUT);
  pinMode(TN4,OUTPUT);
  pinMode(ENA,OUTPUT);
  pinMode(ENB,OUTPUT);
  
  // 24L01 initialization
   myRadio.begin(); 
  myRadio.setChannel(115); 
  myRadio.setPALevel(RF24_PA_MAX);
  myRadio.setDataRate( RF24_250KBPS ) ; 
  myRadio.openReadingPipe(1, addresses[0]);
  myRadio.startListening();
}

void loop() 
{
  Filter();
  
  //Serial.print("  Angle = ");  //Testing angle values recieved
  //Serial.print(Angle_Filtered); //Slows down loop but good for testing gyroscope 
  
  // If angle > 45 or < -45 then stop the segway
  if (abs(Angle_Filtered) < 45)
  {
    Recive();
    myPID();
    PWMControl();
  }
  else
  {
    analogWrite(ENA, 0);  // Stop the wheels
    analogWrite(ENB, 0);  // Stop the wheels
    
    for(int i = 0; i < 100; i++)  // Keep reading the gesture after falling down
    {
      Filter();
    }
    
    if(abs(Angle_Filtered) < 45)  // Empty datas and restart the robot automaticly
    {
      for(int i = 0; i <= 500; i++)  // Reset the robot and delay 2 seconds
      {
        omega = Angle_Raw = Angle_Filtered = 0;
        Filter();
        Output = error = errSum = dErr = Run_Speed = Turn_Speed = 0;
        myPID();
      }
    }
  }
 
  Serial.println(); //Loop delay. The loop is going as fast as the Ardunio can process which is +- (dis)advantageous depending
  
}

//For radio tranciever / reciever
void Recive()
{
    if (myRadio.available() ){
       while (myRadio.available())
    {
      myRadio.read( &data, sizeof(data) );
    }
    
    Serial.print("X:");
    Serial.print(data.X);
    Serial.print("      Y");
    Serial.println(data.Y);


    int X = data.X;
    int Y = data.Y;
  //  int X2 = data.X2;
   // int Y2 = data.Y2;*/
    
   int foward = map(X,524,1024,0,255);
    int backward = map(X,500,0,0,255);
    int right = map(Y,524,1024,0,255);
    int left = map(Y,500,0,0,255);
   
    /*Testing the alternative axis' didn't end up using the option
    Serial.print("axis_1=");
    Serial.print(axis_x.axis_1);
    Serial.print("  axis_2=");
    Serial.println(axis_x.axis_2);
    Serial.print("  axis_3=");
    Serial.print(axis_x.axis_3);
    Serial.print("  axis_4=");
    Serial.print(axis_x.axis_4);
    Serial.print("  axis_5=");
    Serial.print(axis_x.axis_5);
    Serial.print("  axis_6=");
    Serial.print(axis_x.axis_6);
    Serial.print("  axis_7=");
    Serial.print(axis_x.axis_7);
    Serial.print("  axis_8=");
    Serial.println(axis_x.axis_8);*/

    //Data recieved from joysticks
  if(X > 524 && Y < 524 && Y > 500){
      analogWrite(TN3, foward);
      analogWrite(TN4, 0);
      analogWrite(TN1, foward);
      analogWrite(TN2, 0);
    }else if(X < 500 && Y < 524 && Y > 500){
      analogWrite(TN4, backward);
      analogWrite(TN3, 0);
      analogWrite(TN2, backward);
      analogWrite(TN1, 0);
    }else if(X < 524 && X > 500 && Y < 524 && Y > 500){
      analogWrite(TN4, 0);
      analogWrite(TN3, 0);
      analogWrite(TN2, 0);
      analogWrite(TN1, 0);
    }else if(X < 524 && X > 500 && Y > 524){
      analogWrite(TN4, 0);
      analogWrite(TN3, left);
      analogWrite(TN2, left);
      analogWrite(TN1, 0);
    }else if(X < 524 && X > 500 && Y < 500){
      analogWrite(TN4, right);
      analogWrite(TN3, 0);
      analogWrite(TN2, 0);
      analogWrite(TN1, right);
    }else if(X > 524 && Y > 524){
      analogWrite(TN3, foward);
      analogWrite(TN4, 0);
      analogWrite(TN1, foward-right);
      analogWrite(TN2, 0);
    }else if(X > 524 && Y < 500){
      analogWrite(TN3, foward-left);
      analogWrite(TN4, 0);
      analogWrite(TN1, foward);
      analogWrite(TN2, 0);
    }else if(X < 500 && Y > 524){
      analogWrite(TN4, backward);
      analogWrite(TN3, 0);
      analogWrite(TN2, backward-right);
      analogWrite(TN1, 0);
    }else if(X < 500 && Y < 500){
      analogWrite(TN4, backward-left);
      analogWrite(TN3, 0);
      analogWrite(TN2, backward);
      analogWrite(TN1, 0);
    }
    
  } 
}

//Kalman filter to confirm correct data from gyroscope, optional but helpful 
void Filter()
{
  // Raw datas from MPU6050
  accelgyro.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
  Angle_Raw = (atan2(ay, az) * 180 / pi + Angle_offset);
  omega =  gx / Gyr_Gain + Gry_offset;
  // Filters datas to get the real gesture
  unsigned long now = millis();
  float dt = (now - preTime) / 1000.00;
  preTime = now;
  float K = 0.95;
  float A = K / (K + dt);
  Angle_Filtered = A * (Angle_Filtered + omega * dt) + (1 - A) * Angle_Raw;
 
  // Preparing datas which can be shown on the LCD. Still investigating adding LCD screen
  //Cool but takes up power + loop time
  data.omega = omega;
  data.angle = Angle_Filtered;
}

void myPID()
{ 
  //I calculated PID using MATLAB at best phase margin & gain
  //Sometimes its easier to guess & test (or add potentometers) if you dont have MATLAB
  kp=170;
  ki=0.2;
  kd=1;
  
  // Calculating the output values using the gesture values and the PID values.
  unsigned long now = millis();
  timeChange = (now - lastTime);


  lastTime = now;
  
  error = Angle_Filtered;  // Proportion
  errSum += error * timeChange;  // Integration
  dErr = (error - lastErr) / timeChange;  // Differentiation
  Output = kp * error + ki * errSum + kd * dErr;
  lastErr = error;
  
  LOutput = Output - Turn_Speed + Run_Speed;
  ROutput = Output + Turn_Speed + Run_Speed;
  data.speed = (LOutput + ROutput) / 2;
}

//Pulse width mod control for motors
void PWMControl()
{
  if(LOutput > 0)
  {
    digitalWrite(TN1, HIGH);
    digitalWrite(TN2, LOW);
  }
  else if(LOutput < 0)
  {
    digitalWrite(TN1, LOW);
    digitalWrite(TN2, HIGH);
  }
  else
  {
    digitalWrite(ENA, 0);
  }
  if(ROutput > 0)
  {
    digitalWrite(TN3, HIGH);
    digitalWrite(TN4, LOW);
  }
  else if(ROutput < 0)
  {   
    digitalWrite(TN3, LOW);
    digitalWrite(TN4, HIGH);
  }
  else
  {
    digitalWrite(ENB, 0);
  }
    analogWrite(ENA, min(255, (abs(LOutput) + LMotor_offset)));
    analogWrite(ENB, min(255, (abs(ROutput) + RMotor_offset)));
}
