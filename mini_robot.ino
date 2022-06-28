//This code is for the small robot

//The main components for the code are:

/*
 * Motors controlling using Dual motor driver LM249 
 * Encoder feed back. Opto coupler so only single direction
 * Servo control with ultrasonic sensor
 * ESP32 for controlling and bluetooth communication
 * 
 * The robot will move in a direction untill the ultrasonic sensor sees an obstacle
 * when obstacle is seen servo will move 90 degrees right and if right i also obstructed it will move 90 degrees left.
 * If one of the two are not obstructed it will go in that direction. 
 * If both are obstructed it will go back 2 turns and check again untill it gets free area and move in that direction
 */
#include <ESP32Servo.h>
#include <Arduino.h>
#include <analogWrite.h>

Servo myservo;  // create servo object to control a servo


// ISR routines for the encoder

int Encoder_L = 18; //ardituray pins for now
int Encoder_R = 19; 

int enA = 20;
int enB = 21;

int in1 = 22;
int in2 = 23;
int in3 = 24;
int in4 = 25;

int servoPin = 13;
int mSpeed = 255;
////
#define echoPin 2 // attach pin D2 Arduino to pin Echo of HC-SR04
#define trigPin 3 //attach pin D3 Arduino to pin Trig of HC-SR04

// defines variables
long duration; // variable for the duration of sound wave travel
int distance; // variable for the distance measurement


/////
volatile int pulsesL = 0; // pass to the compiler in arduino ide as if compiler sees the code and as these are not used in the main the compiler will remove space whcih is not good. Volatile
// means to tell the compiler to keep some space for our code
volatile int pulsesR = 0;

void ISR_Pulse_L()
{
  
  pulsesL++;
  
}

void ISR_Pulse_R()
{
  
  pulsesR++;
  
}

void setup_wheelencoder()
{
 
   pinMode(Encoder_L, INPUT);
   attachInterrupt(digitalPinToInterrupt (Encoder_L), ISR_Pulse_L, RISING);
   pulsesL = 0;
   pinMode(Encoder_R, INPUT);
   attachInterrupt(digitalPinToInterrupt (Encoder_R), ISR_Pulse_R, RISING);
   pulsesR = 0;
   
}


void Routine()
{
  if(!obstacle)
  {
      move_straight("forward", 0);
  }else
      {
          brake();
          sensor_sweep();  
      }
}

void sensor_sweep()
{
  move_sensor("right");
  if(!obstacle())
  {
    turn_robot("right");
    move_sensor("straight");
  }else{
    move_sensor("left"); //this has to be a 180 as the sesor is already to right
    if(!obstacle())
      {
        turn_robot("left");
        move_sensor("straight");
      }else
      {
        move_straight("backwards", 10);
        brake();
        sensor_sweep();  
      }
  }
}

void move_sensor(String direction){
  if(direction == "right"){
    myservo.write(180);              
  }else if (direction == "left"){
    myservo.write(0);
  }else if (direction == "straight"){
    myservo.write(90);
  }
}

void turn_robot(String direction){
  if(direction = "right"){

    //motor 1 direction
        digitalWrite(in1, HIGH);
        digitalWrite(in2, LOW);

        //motor 2 direction
        digitalWrite(in3, LOW);
        digitalWrite(in4, HIGH); 
    
  }else if(direction = "left"){

    //motor 1 direction
        digitalWrite(in1, LOW);
        digitalWrite(in2, HIGH);

        //motor 2 direction
        digitalWrite(in3, HIGH);
        digitalWrite(in4, LOW); 
    
    
  }

  while(pulsesL < pulse_required(10) && pulsesR < pulse_required(10)) // to calcultate properly for 90 degrees rotation
      { //this is not a good routines as it wont go in the straight line, i feel that

        if (pulsesL == pulsesR){
            analogWrite(enA , mSpeed);
            analogWrite(enB , mSpeed);
          }else if(pulsesL > pulsesR){
            analogWrite(enA , mSpeed);
            analogWrite(enB , 0); 
          }else if(pulsesR > pulsesL){
            analogWrite(enA , 0);
            analogWrite(enB , mSpeed);
            }

      }
            
        analogWrite(enA , 0); 
        analogWrite(enB , 0);  
}

bool obstacle()
{
  // Clears the trigPin condition
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  // Sets the trigPin HIGH (ACTIVE) for 10 microseconds
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);
  // Reads the echoPin, returns the sound wave travel time in microseconds
  duration = pulseIn(echoPin, HIGH);
  // Calculating the distance
  distance = duration * 0.034 / 2; // Speed of sound wave divided by 2 (go and back)

  if(distance >= 20){
    return true;
  }else{
    return false;
  }
}
void brake(){
  digitalWrite(in1, LOW);
  digitalWrite(in2, LOW);
  digitalWrite(in3, LOW);
  digitalWrite(in4, LOW);

  analogWrite(enA, 0);
  analogWrite(enB, 0);
}

long pulse_required(int distance){
  // Calculate target number of ticks
  int wheel_c = 10; //calculate this number in mm
  int counts_per_rev = 20; // double check
  float num_rev = (distance * 10) / wheel_c;  // Convert to mm
  unsigned long target_count = num_rev * counts_per_rev;

  return target_count;
}

void move_straight(String direction, int distance)
{
  bool inf_travel = 0;
  if(distance > 0)
  {
    inf_travel = false;
  }else
  {
    inf_travel = true;
  }

  if (direction == "forward")
      {
        //motor 1 direction
        digitalWrite(in1, HIGH);
        digitalWrite(in2, LOW);

        //motor 2 direction
        digitalWrite(in3, HIGH);
        digitalWrite(in4, LOW);    
      }else
      {
        pulsesR, pulsesL = 0; //as its backwards for a fix distance we will make encoder values to 0
        //motor 1 direction
        digitalWrite(in1, LOW);
        digitalWrite(in2, HIGH);

        //motor 2 direction
        digitalWrite(in3, LOW);
        digitalWrite(in4, HIGH);
      }


if(inf_travel)
  { if (pulsesL == pulsesR){
    analogWrite(enA , mSpeed);
    analogWrite(enB , mSpeed);
  }else if(pulsesL > pulsesR){
    analogWrite(enA , mSpeed);
    analogWrite(enB , 0); 
  }else if(pulsesR > pulsesL){
    analogWrite(enA , 0);
    analogWrite(enB , mSpeed);
    }
  }else
  {
    ///to move till a desired distance
    while(pulsesL < pulse_required(distance) && pulsesR < pulse_required(distance))
      { //this is not a good routines as it wont go in the straight line, i feel that

        if (pulsesL == pulsesR){
            analogWrite(enA , mSpeed);
            analogWrite(enB , mSpeed);
          }else if(pulsesL > pulsesR){
            analogWrite(enA , mSpeed);
            analogWrite(enB , 0); 
          }else if(pulsesR > pulsesL){
            analogWrite(enA , 0);
            analogWrite(enB , mSpeed);
            }

      }
            
        analogWrite(enA , 0); 
        analogWrite(enB , 0);  
    
  }
     
  
}

void setup() {
  // put your setup code here, to run once:
  pinMode(trigPin, OUTPUT); // Sets the trigPin as an OUTPUT
  pinMode(echoPin, INPUT); // Sets the echoPin as an INPUT
  
}

void loop() {
  // put your main code here, to run repeatedly:
  Routine();

}
