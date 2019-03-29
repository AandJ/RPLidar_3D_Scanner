/*
 * Stepper motor control code
 * Code causes stepper to move by 1 degree everytime data is published to the "/scan" topic
 */

#include <ros.h>
#include <std_msgs/Int16.h>
#include <Stepper.h>

const int stepsPerRevolution = 180;  

// initialize the stepper library on pins 8 through 13:
Stepper myStepper(stepsPerRevolution, 12, 13);

int pwmA = 3;
int pwmB = 11;
int brakeA = 9;
int brakeB = 8;
int dirA = 12;
int dirB = 13;
 
int x = 0;

void CallBack(const std_msgs::Int16& control);

ros::NodeHandle  nh;
ros::Subscriber<std_msgs::Int16> sub("/control", &CallBack);

void loop()
{
  nh.spinOnce();
  delay(50);
  printf("loop");

}

void setup()
{
  Serial.begin(9600);   //initialise the serial port
  
  pinMode(pwmA, OUTPUT);
  pinMode(pwmB, OUTPUT);
  pinMode(brakeA, OUTPUT);
  pinMode(brakeB, OUTPUT);
  digitalWrite(pwmA, HIGH);
  digitalWrite(pwmB, HIGH);
  digitalWrite(brakeA, LOW);
  digitalWrite(brakeB, LOW);
  
  // set the speed at 60 rpm:
  myStepper.setSpeed(10);
  
  nh.initNode();
  nh.subscribe(sub);
}

void CallBack(const std_msgs::Int16& control)
{
  if(control.data == 1) {
    myStepper.step(1);
  } else {
    myStepper.step(-1);
  }
}
