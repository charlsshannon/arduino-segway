// Remote to control the Arduino Segway

#include <SPI.h>
#include <nRF24L01.h>
#include <RF24.h>

#include <Wire.h> 
#include <LiquidCrystal_I2C.h>
LiquidCrystal_I2C lcd(0x27,16,2); //For the LCD, not used yet.

unsigned int Display_Counter, Button_Delay = 0; //For LCD

/* Alternative choice for joystick data
  Not used yet.
 
struct Axis{

  uint16_t axis_1;
  uint16_t axis_2;
  uint16_t axis_3;
  uint16_t axis_4;
  uint16_t axis_5;
  uint16_t axis_6;
  uint16_t axis_7;
  uint16_t axis_8;
};
Axis axis_x; */

byte addresses[][6] = {"0"}; 

RF24 myRadio (7, 8); 
struct package
{
  int X1=1;
  int Y1=1;
  int X2=1;
  int Y2=1;
  float angle;
  float omega;
  int speed; 
  int P;
  int I;
  int D;
  uint16_t null_1; 
  uint16_t null_2;
};

typedef struct package Package;
Package data;

void setup(){
  pinMode(2, INPUT);

 Serial.begin(115200);
  delay(100);
  myRadio.begin();  
  myRadio.setChannel(115); 
  myRadio.setPALevel(RF24_PA_MIN);
  myRadio.setDataRate( RF24_250KBPS ) ; 
  myRadio.openWritingPipe( addresses[0]);
  delay(100);
}

void loop(){
 
//**********************************************************************************************************/  
  myRadio.write(&data, sizeof(data)); 

  Serial.print("X:");
  Serial.print(data.X1);
  Serial.print("    Y");
  Serial.println(data.Y1);
  data.X1 = analogRead(A0);
  data.Y1 = analogRead(A1);
  data.X2 = analogRead(A2);
  data.Y2 = analogRead(A3);
  delay(100);
 // ***************************************************************************************************/

}


