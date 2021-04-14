#include <LiquidCrystal.h>

LiquidCrystal lcd(12,11,10,9,8,7);


int kp=50 ; int ki=3 ; int kd=12;  // nilai Kp, Ki, Kd ini bisa di-tune sendiri
int PID_p=0 ; int PID_i=0 ; int PID_d=0;
float PID_error = 0;
float previous_error = 0;
float elapsedTime, Time, timePrev;
float PID_value = 0;
float set_temperature;


void setup() {
  // put your setup code here, to run once:
pinMode(2,OUTPUT);
pinMode(3,OUTPUT);
pinMode(4,OUTPUT);
pinMode(A0,INPUT);
pinMode(A1,INPUT);

digitalWrite(2,HIGH);
digitalWrite(4,HIGH);
digitalWrite(3,LOW);

lcd.begin(16,2);
lcd.display();
Serial.begin(9600);
}

void loop() {
  // put your main code here, to run repeatedly:
set_temperature=analogRead(A1);
set_temperature=map(set_temperature,0,1024,0,151);

float T=analogRead(A0);
T=map(T,0,1024,0,151);

//PID Loop
PID_error=set_temperature-T;
timePrev=Time;
Time=millis();
elapsedTime=(Time-timePrev)/1000;
PID_p=kp*PID_error;
PID_i=PID_i+(ki*PID_error*elapsedTime);
PID_d=kd*((PID_error-previous_error)/elapsedTime);
PID_value=PID_p+PID_i+PID_d;
if(PID_value<0)
{PID_value=0;}
if(PID_value>255)
{PID_value=250;}

//Display message on LCD 
lcd.setCursor(0,0);
lcd.print("Temp    Set_T");

lcd.setCursor(0,1);
lcd.print(T);
lcd.setCursor(10,1);
lcd.print(set_temperature);
digitalWrite(2,HIGH);
digitalWrite(4,HIGH);
analogWrite(3,PID_value);
previous_error=PID_error;
}
