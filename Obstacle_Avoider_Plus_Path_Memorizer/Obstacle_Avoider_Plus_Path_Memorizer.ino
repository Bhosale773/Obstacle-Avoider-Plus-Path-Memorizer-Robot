/*
  Obstacle Avoider Plus Path Memorizer robot with moving Ultrasonic Sensor, acting as RADAR 

  Robot senses obstacle and chooses its direction to move further
  Servo Motor is used to move Ultrasonic Sensor as RADAR
  Moving Ultrasonic Sensor is used to sense obstacle as RADAR and to increase efficiency of obstacle avoiding robot
  Robot moves are contolled by changing polarities of BO motors
  Distance between Obstacle and Robot is measured by moving Ultrasonic Sensor
  Further feature of Path Memorizing is added to robot which is influenced by intensity of light
  That is if robot goes from position A to position B with avoiding obstacle then if at position B Intensity of light increases beyond a limit,
  which is measured by Light Dependent Resistance then Robot moves back from position B to position A by following memorized path
  Array Data Strucure is used to memorize path followed and numbers are used to indicate movements taken by robot 

  Note : All delay times used in code can be varied according to choice
  
  Circuit involves:
  UNIKO EKAM, 1 HC SR04 ULTRASONIC SENSOR, 1 SERVO MOTOR, 2 BO MOTORS, MOTOR DRIVER, LIGHT DEPENDENT RESISTANCE, BATTERY, WIRES, CHASIS

  Connections:
  All the modules are arranged on Chasis and fixed by screws
  On Uniko Ekam board motor driver is fixed to drive the 2 BO motors
  Pin 8,9,10,11 are connected to motor driver to control polarities of motors
  Ultrasonic Sensor is fixed on Servo motor to move as RADAR at front
  pin 3 and pin 4 is set as trigger pin and echo pin of ultrasonic sensor
  PWM enabled pin 6 is used to attach Servo Motor
  Pin A5 is used to read value of LDR and hence determining intensity of light
  Battery is used to power the board 

  Created by
  Pratik Vijay Bhosale
  Last modified on
  10 February,2019
*/

#include<Servo.h>   //include servo library in program

void forward();   //function prototype
void left(int deg);    //function prototype
void right(int deg);   //function prototype
void reverse();   //function prototype
void stp();   //function prototype
void findpath();    //function prototype
int distance();   //function prototype
int LDR();    //function prototype

Servo S1;  //create servo object to control servo

int movement[200],del[100];   //declare array movement[] to store movements of robot and del[] to store time for which robot is in forward motion
int m=1,d=0;    //declare index counters of array
int enterTime=0,exitTime=0;   //Set variables enterTime and exitTime at Zero
int triggerPin = 3;   //define pin 3 for triggerPin
int echoPin = 4;      //define pin 4 for echoPin
double duration;    //declare variable duration of type double to store duration taken by sound wave
int dist,leftdist,rightdist;   //declare variables to store distances measured by Ultrasonic Sensor

void setup() 
{
  pinMode(8,OUTPUT);    //Set pin 8 as output
  pinMode(9,OUTPUT);    //Set pin 9 as output
  pinMode(10,OUTPUT);   //Set pin 10 as output
  pinMode(11,OUTPUT);   //Set pin 11 as output
  pinMode(A5,INPUT);    //Set pin A5 as input
  pinMode(triggerPin,OUTPUT);    //Sets the triggerPin as an Output 
  pinMode(echoPin,INPUT);    //Sets the echoPin as an Input
  S1.attach(6);  //attaches the servo on pin 6 to the servo object S1
  S1.write(90);   //Set Servo motor at 90 degree position
  delay(100);   //Wait for 100ms
  movement[0]=5;    //Store value 5 (any value that is not used for movement indication) in first position of array
}

void loop() 
{
  if(LDR()==0)    //Do this if light intensity is normal
  {
    if(distance()>20)    //Do this if distance between robot and obstacle is greater than 20cm (Distance to avoid can be varied)
    {
      enterTime=millis();   //Store number of milliseconds passed since program is start in variable enterTime before loop
      delay(100);   //Wait for 100ms
      while(distance()>20 && LDR()==0)    //Loop till distance between robot and obstacle is greater than 20cm and intensity of light is normal
      {
        forward();   //function to move robot in forward direction
      }
      exitTime=millis();    //Store number of milliseconds passed since program is start in variable exitTime after loop
      delay(100);   //Wait for 100ms
      del[d]=exitTime-enterTime;    //Store time for which robot is in forward motion in array del[]
      d++;    //Increment array index counter d by 1
      exitTime=0;   //Set exitTime at zero
      enterTime=0;    //Set enterTime at zero
    }
    else  //Do this if distance between robot and obstacle is less than or equal to 20cm
    {
      findpath();   //Call function findpath() to decide path to move ahead by robot
    }
  }
  else    //Do this if light intensity is more than normal
  {
    stp();    //Function to stop robot
    delay(5000);    //Wait for 5000ms
    for(int i=m-1;i>0;i--)    //Loop till all the movements taken by robot gets over in reverse manner
    {
      if(movement[i]==0)    //Do this if movement taken by robot is stop
      {
        stp();    //Function to stop robot
        delay(500);   //Wait for 500ms
      }
      else if(movement[i]==1)   //Do this if movement taken by robot is forward
      {
        reverse();    //Function to move robot in reverse direction
        delay(del[d-1]);    //Wait for time for which robot moves in forward direction
        d--;    //Decrement array counter d by 1
      }
      else if(movement[i]==2)   //Do this if movement taken by robot is left turn
      {
        right(90);    //Function to take right turn by 90 degree
      }
      else if(movement[i]==3)   //Do this if movement taken by robot is right turn
      {
        left(90);   //Function to take left turn by 90 degree
      }
      else if(movement[i]==4)   //Do this if movement taken by robot is reverse
      {
        forward();    //Function to move robot in forward direction
        delay(1000);    //Wait for 1000ms
      }
    }
    m=1;    //Set array index counter m to 1
    d=0;    //Set array index counter d to 0
    stp();    //Function to stop robot
    delay(5000);    //Wait for 5000ms
  }
}

void forward()    //Function to move robot in forward direction
{
  if(LDR()==0 && movement[m-1]!=1)    //Do this if light intensity is normal and previous block in array is not contained forward movement
  {
    movement[m]=1;    //Store forward movement in array movement[]
    m++;    //Increment array counter m by 1
  }
  digitalWrite(8,LOW);    //Set pin 8 at low
  digitalWrite(9,HIGH);   //Set pin 9 at high
  digitalWrite(10,HIGH);   //Set pin 10 at high
  digitalWrite(11,LOW);   //Set pin 11 at low
}

void left(int deg)   //function to take left turn by input degree
{
  if(LDR()==0)    //Do this if light intensity is normal
  {
    movement[m]=2;    //Store left turn movement in array movement[]
    m++;    //Increment array counter m by 1
  }
  for(int i=0;i<deg;i++)    //Loop till robot moves from 0 degree to input degree
  {
    digitalWrite(8,HIGH);   //Set pin 8 at high
    digitalWrite(9,LOW);    //Set pin 9 at low
    digitalWrite(10,HIGH);    //Set pin 10 at high
    digitalWrite(11,LOW);   //Set pin 11 at low
    delay(7.5);   //Wait for 7.5ms (This delay time can be varied with power input)
  }
}

void right(int deg)    //function to take right turn by input degree
{
  if(LDR()==0)    //Do this if light intensity is normal
  {
    movement[m]=3;    //Store right turn movement in array movement[]
    m++;    //Increment array counter m by 1
  }
  for(int j=0;j<deg;j++)    //Loop till robot moves from 0 degree to input degree
  {
    digitalWrite(8,LOW);    //Set pin 8 at low
    digitalWrite(9,HIGH);   //Set pin 9 at high
    digitalWrite(10,LOW);   //Set pin 10 at low
    digitalWrite(11,HIGH);    //Set pin 11 at high
    delay(7.5);   //Wait for 7.5ms (This delay time can be varied with power input)
  }
}

void reverse()    //function to move robot in backward direction
{
  if(LDR()==0)     //Do this if light intensity is normal
  {
    movement[m]=4;    //Store reverse movement in array movement[]
    m++;    //Increment array counter m by 1
  }
  digitalWrite(8,HIGH);   //Set pin 8 at high
  digitalWrite(9,LOW);    //Set pin 9 at low
  digitalWrite(10,LOW);   //Set pin 10 at low
  digitalWrite(11,HIGH);    //Set pin 11 at high
}

void stp()    //function to stop the robot 
{
  if(LDR()==0)    //Do this if light intensity is normal
  {
    movement[m]=0;    //Store stop movement in array movement[]
    m++;    //Increment array counter m by 1
  }
  digitalWrite(8,LOW);    //Set pin 8 at low
  digitalWrite(9,LOW);    //Set pin 9 at low
  digitalWrite(10,LOW);   //Set pin 10 at low
  digitalWrite(11,LOW);   //Set pin 11 at low
}

void findpath()
{
  stp();    //function to stop the robot
  delay(200);   //Wait for 200ms
  S1.write(170);    //Set Servo motor at 170 degree
  delay(500);   //Wait for 500ms
  leftdist=distance();    //Store distance measured on left side into variable leftdist  
  delay(100);   //Wait for 100ms
  S1.write(90);   //Set Servo motor at 90 degree
  delay(200);   //Wait for 200ms
  S1.write(10);   //Set Servo motor at 10 degree
  delay(500);   //Wait for 500 ms
  rightdist=distance();   //Store distance measured on right side into variable rightdist
  delay(100);   //Wait for 100ms
  S1.write(90);   //Set Servo Motor at 90 degree
  delay(200);   //Wait for 200 ms
  if(leftdist>20 && rightdist>20)    //Do this if no obstacle present on both sides
  {
    if(leftdist>rightdist)    //Do this if there is more free space on left side
    {
      left(90);   //Function to take left turn by 90 degree
    }
    else    //Do this if there is more free space on right side
    {
      right(90);    //Function to take right turn by 90 degree
    }
    stp();    //Function to stop robot
    delay(500);   //Wait for 500ms
  }
  else if(leftdist>20)    //Do this if no obstacle present on left side
  {
    left(90);   //Function to take left turn by 90 degree
    stp();    //Function to stop robot
    delay(500);   //Wait for 500ms
  }
  else if(rightdist>20)    //Do this if no obstacle present on right side
  {
    right(90);    //Function to take right turn by 90 degree
    stp();    //Function to stop robot
    delay(500);   //Wait for 500ms
  }
  else   //Do this if obstacle is present on both sides
  {
    reverse();    //Function to move robot in backward direction
    delay(1000);   //Wait for 1000ms
    stp();    //Function to stop robot
    delay(500);   //Wait for 500ms
    findpath();   //Recursively called function to decide path to move robot ahead
  }
}

int distance()    //function to calculate and return distance between robot and obstacle
{
  digitalWrite(triggerPin,HIGH);    //Sets the triggerPin on HIGH state for 10 micro seconds
  delayMicroseconds(10);            //wait for 10 micro seconds
  digitalWrite(triggerPin,LOW);     //Sets the triggerPin on LOW state
  duration = pulseIn(echoPin,HIGH);   //Reads the echoPin, returns the sound wave travel time in microseconds to variable duration
  dist = duration*0.034/2;   // Calculating the distance in centimeter
  return dist;    //return calculated distance between robot and obstacle
}

int LDR()   //Function to determine light intensity is normal or high
{
  int ldr,l=analogRead(A5);   //Declare variables ldr and variable l to store read value of Light Dependent Resistance according to light intensity
  if(l<300)   //Do this if light intensity is normal i.e. value of LDR is less than 300 (Value of LDR for normal itensity may varies with place)
  {
    ldr=0;    //Set variable ldr to 0
  }
  else    //Do this if light intensity is high
  {
    ldr=1;    //Set variable ldr to 1
  }
  return ldr;   //Return value store in ldr
}
    
