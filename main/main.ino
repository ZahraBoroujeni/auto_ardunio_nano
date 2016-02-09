/*
 Controlling a servo position using a serial port command
 by zahra boroujeni 
 modified on 30 Nov 2015
*/

#include <Servo.h>

Servo myservo;  // create servo object to control a servo

int potpin = 0;  // analog pin used to connect the potentiometer
int val;    // variable to read the value from the analog pin
String inputString = "";         // a string to hold incoming data
boolean stringComplete = false;  // whether the string is complete
#define servo_pin 10
#define POWER_STATE_D0 8
#define POWER_STATE_D1 9
#define BUTTON_GREEN 4
#define BUTTON_RED 5


void setup() {
  myservo.attach(servo_pin);  // attaches the servo on pin 9 to the servo object
  // initialize serial:
  Serial.begin(115200);
  // reserve 200 bytes for the inputString:
  inputString.reserve(200);

  //power board control
  pinMode(POWER_STATE_D0, OUTPUT);
  pinMode(POWER_STATE_D1, OUTPUT);
  pinMode(BUTTON_GREEN, INPUT);
  pinMode(BUTTON_RED, INPUT);
}

void loop() {
  //Serial.println("start");
  powerBoard();
  serialEvent(); //call the function
  // print the string when a newline arrives:
  if (stringComplete) {
    if (inputString=="en\r")
    {
      myservo.attach(servo_pin);
      //Serial.println("enable");
      //Serial.println(inputString);
    }
    else if (inputString=="di\r")
    {
      myservo.detach();
      //Serial.println("detach");
      //Serial.println(inputString);
    }
    else
    {
      //Serial.println(inputString);
      val=inputString.toInt();
      val = map(val, 0, 180, 900, 1900);     // scale it to use it with the servo (value between 0 and 180)
      myservo.writeMicroseconds(val);  
    }
    // clear the string:
    inputString = "";
    stringComplete = false;
  }
  
}

/* Control power board*/
void powerBoard(){
  if (digitalRead(BUTTON_RED)==0) //disable the red button to trun off everything
  {
    digitalWrite(POWER_STATE_D0, HIGH);
    digitalWrite(POWER_STATE_D1, HIGH);
  }
  else if (digitalRead(BUTTON_GREEN)==0)//press red button to turn on 5 volt for arduino and odroid
  {
    digitalWrite(POWER_STATE_D0, LOW);
    digitalWrite(POWER_STATE_D0, LOW); 
  } else //press also green button for traction motor supply
  {
    digitalWrite(POWER_STATE_D0, HIGH);
    digitalWrite(POWER_STATE_D0, LOW); 
  }
}
/*
  SerialEvent occurs whenever a new data comes in the
 hardware serial RX.  This routine is run between each
 time loop() runs, so using delay inside loop can delay
 response.  Multiple bytes of data may be available.
 */
void serialEvent() {
  while (Serial.available()) 
  {
    //Serial.println("here");
    // get the new byte:
    char inChar = (char)Serial.read();
    //Serial.println(inChar);
    // add it to the inputString:
    inputString += inChar;
    // if the incoming character is a newline, set a flag
    // so the main loop can do something about it:
    if (inChar == '\r') 
    {
      stringComplete = true;
       //Serial.println("true");
    }
  }
}
 
