/*
  Obstacle avoider robot with non moving Ultrasonic Sensor

  Robot senses obstacle and chooses its direction to move further
  Stationary Ultrasonic Sensor is used to sense obstacle
  Robot moves are contolled by changing polarities of BO motors
  Distance between Obstacle and Robot is measured by fixed Ultrasonic Sensor

  Note : All delay times used in code can varied according to choice
  
  Circuit involves:
  UNIKO EKAM, 1 HC SR04 ULTRASONIC SENSOR, 2 BO MOTORS, MOTOR DRIVER, BATTERY, WIRES, CHASIS

  Connections:
  All the modules are arranged on Chasis and fixed by screws
  On Uniko Ekam board motor driver is fixed to drive the 2 BO motors
  Pin 8,9,10,11 are connected to motor driver to control polarities of motors
  Ultrasonic Sensor is fixed at front, pin 3 and pin 4 is set as trigger pin and echo pin of ultrasonic sensor
  Battery is used to power the board 

  Created by
  Pratik Vijay Bhosale
  Last modified on
  04 February,2019
*/

void forward();   //function prototype
void left(int deg);    //function prototype
void stp();   //function prototype
int distance();   //function prototype

int triggerPin = 3;   //define pin 3 for triggerPin
int echoPin = 4;      //define pin 4 for echoPin
double duration;    //declare variable duration of type double to store duration taken by sound wave
int dist;   //declare variable dist to store calculated distance between robot and obstacle

void setup() 
{
  pinMode(8,OUTPUT);    //Sets pin 8 as Output
  pinMode(9,OUTPUT);    //Sets pin 9 as Output
  pinMode(10,OUTPUT);   //Sets pin 10 as Output
  pinMode(11,OUTPUT);   //Sets pin 11 as Output
  pinMode(triggerPin,OUTPUT);    //Sets the triggerPin as Output 
  pinMode(echoPin,INPUT);    //Sets the echoPin as Input
}

void loop() 
{
  if(distance()>20)   //Do this if distance of robot from obstacle is greater than 20cm (Distance to avoid can be varied)
  {
    forward();    //Function to move robot in forward direction
  }
  else    //Do this if distance of robot from obstacle is less than or equal to 20cm
  {
    stp();    //Function to stop robot
    delay(500);   //Wait for 500ms
    left(90);   //Function to take left turn by 90 degree
  }    
}

void forward()    //Function to move robot in forward direction
{
  digitalWrite(8,LOW);    //Set pin 8 at low
  digitalWrite(9,HIGH);   //Set pin 9 at high
  digitalWrite(10,HIGH);   //Set pin 10 at high
  digitalWrite(11,LOW);   //Set pin 11 at low
}

void stp()    //Function to stop robot
{
  digitalWrite(8,LOW);    //Set pin 8 at low
  digitalWrite(9,LOW);   //Set pin 9 at low
  digitalWrite(10,LOW);   //Set pin 10 at low
  digitalWrite(11,LOW);   //Set pin 11 at low
}

void left(int deg)   //Function to take left turn by input degree
{
  for(int i=0;i<deg;i++)    //Loop till Robot turn by input degree
  {
    digitalWrite(8,HIGH);   //Set pin 8 at high
    digitalWrite(9,LOW);    //Set pin 9 at low
    digitalWrite(10,HIGH);    //Set pin 10 at high
    digitalWrite(11,LOW);   //Set pin 11 at low
    delay(7.5);   //Wait for 7.5ms (This delay time can be varied wrt power input)
  }
}

int distance()    //function to calculate and return distance between robot from obstacle
{
  digitalWrite(triggerPin,HIGH);    //Sets the triggerPin on HIGH state for 10 micro seconds
  delayMicroseconds(10);            //Wait for 10 micro seconds
  digitalWrite(triggerPin,LOW);     //Sets the triggerPin on LOW state
  duration = pulseIn(echoPin,HIGH);   //Reads the echoPin, returns the sound wave travel time in microseconds to variable duration
  dist = duration*0.034/2;   // Calculating the distance in centimeter
  return dist;    //return calculated distance between robot and obstacle
}
    

    
