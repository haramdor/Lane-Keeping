//This code suppose to work on Arduino DUE with python code that build on Raspberry pi

#include <Servo.h>

Servo steeringwheel;  // create servo object to control a servo
Servo myspeed;        // create servo object to define the vertical speed

int servo_angle;
int SpeedValue;
const int DCMotorPin = 6;   //Assigned pin for DC motor
const int ServoPin = 5;     //Assigned pin for Servo steeringwheel motor
String value = "";
String command = "";

void setup() {
  steeringwheel.attach(ServoPin);   // attaches the servo on pin 5 to the servo object
  pinMode(ServoPin, OUTPUT);        //Servo will receive data
  myspeed.attach(DCMotorPin);       //DC motor assigned to pin 6
  pinMode(DCMotorPin, OUTPUT);      //DC motor will receive data
  Serial.begin(115200);             //Set the baudrate
  Serial.flush();                   //Waits for the transmission of outgoing serial data to complete.
  Serial.setTimeout(500);           //Waits for 500 miliseconds
}

void loop() {

  if (Serial.available()) {
    Serial.readStringUntil('<');            //Search for start marker '<'
    command = Serial.readStringUntil('>');  //Search for end marker '>'
    
    if (command.indexOf("an") >= 0) {       //If the message starts at '<an'
      value = command.substring(2);         //Put in 'value' the 2 integers that comes after the '<an'
      servo_angle = value.toInt();          //Converts a valid String to an integer
      steeringwheel.write(servo_angle);     //write the recieved angle to the servo object 'servo_angle'
      SpeedValue = 1615;                    //Good and slow speed we can see at SpeedValue ~ 1615
      myspeed.write(SpeedValue);            //write the recieved angle to the servo object 'servo_angle'
      Serial.println("srv = " + String(servo_angle));
      delay(10);                            //Waits for 10 miliseconds
    }

    if (command.indexOf("quit") >= 0) {   //If 'q' is pressed on the Raspberry pi
      steeringwheel.write(82);            //Make the wheels go straight
      myspeed.write(1500);                //And the DC motor stops working
    }
  }
}
