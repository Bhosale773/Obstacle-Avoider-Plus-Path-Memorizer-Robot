/*
  Obstacle avoider robot with moving Ultrasonic Sensor and acting as RADAR

  Robot senses obstacle and chooses its direction to move further
  Servo Motor is used to move Ultrasonic Sensor as RADAR
  Moving Ultrasonic Sensor is used to sense obstacle as RADAR and to increase efficiency of obstacle avoiding robot
  Robot moves are contolled by changing polarities of BO motors
  Distance between Obstacle and Robot is measured by moving Ultrasonic Sensor

  Note : All delay times used in code can be varied according to choice 
  
  Circuit involves:
  UNIKO EKAM, 1 HC SR04 ULTRASONIC SENSOR, 1 SERVO MOTOR, 2 BO MOTORS, MOTOR DRIVER, BATTERY, WIRES, CHASIS

  Connections:
  All the modules are arranged on Chasis and fixed by screws
  On Uniko Ekam board motor driver is fixed to drive the 2 BO motors
  Pin 8,9,10,11 are connected to motor driver to control polarities of motors
  Ultrasonic Sensor is fixed on Servo motor to move as RADAR at front
  pin 3 and pin 4 is set as trigger pin and echo pin of ultrasonic sensor
  PWM enabled pin 6 is used to attach Servo Motor
  Battery is used to power the board 

  Created by
  Pratik Vijay Bhosale
  Last modified on
  04 February,2019
*/

#include<Servo.h>   //include servo library in program

void forward();   //function prototype
void left(int deg);    //function prototype
void right(int deg);   //function prototype
void reverse();   //function prototype
void stp();   //function prototype
void findpath();    //function prototype
int distance();   //function prototype

Servo S1;  //create servo object to control servo

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
  pinMode(triggerPin,OUTPUT);    //Sets the triggerPin as Output 
  pinMode(echoPin,INPUT);    //Sets the echoPin as Input
  S1.attach(6);  //attaches the servo on pin 6 to the servo object S1
  S1.write(90);   //Set Servo motor at 90 degree position
  delay(100);   //Wait for 100ms
}

void loop() 
{
  if(distance()>20)    //Do this if distance between robot and obstacle is greater than 20cm (Distance to avoid can be varied)
  {
    forward();   //function to move robot in forward direction
  }
  else  //Do this if distance between robot and obstacle is less than or equal to 20cm 
  {
    findpath();   //function to decide path to move ahead by robot
  }
}

void forward()    //Function to move robot in forward direction
{
  digitalWrite(8,LOW);    //Set pin 8 at low
  digitalWrite(9,HIGH);   //Set pin 9 at high
  digitalWrite(10,HIGH);   //Set pin 10 at high
  digitalWrite(11,LOW);   //Set pin 11 at low
}

void left(int deg)   //function to take left turn by input degree
{
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
  digitalWrite(8,HIGH);   //Set pin 8 at high
  digitalWrite(9,LOW);    //Set pin 9 at low
  digitalWrite(10,LOW);   //Set pin 10 at low
  digitalWrite(11,HIGH);    //Set pin 11 at high
}

void stp()    //function to stop the robot 
{
  digitalWrite(8,LOW);    //Set pin 8 at low
  digitalWrite(9,LOW);    //Set pin 9 at low
  digitalWrite(10,LOW);   //Set pin 10 at low
  digitalWrite(11,LOW);   //Set pin 11 at low
}

void findpath()   //function to decide path to move ahead by robot
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
    
    
    
