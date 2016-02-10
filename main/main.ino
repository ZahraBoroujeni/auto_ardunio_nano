/*
 Controlling a servo position using a serial port command
 by zahra boroujeni 
 modified on 30 Nov 2015
*/

#include <Servo.h>
#include <MsTimer2.h>

Servo myservo;  // create servo object to control a servo
int state =0;

int potpin = 0;  // analog pin used to connect the potentiometer
int val;    // variable to read the value from the analog pin
String inputString = "";         // a string to hold incoming data
boolean stringComplete = false;  // whether the string is complete
boolean interrupt_flag=false;
boolean servo_control=false;
boolean ledState=false;

#define servo_pin 10
#define POWER_STATE_D0 8
#define POWER_STATE_D1 9
#define BUTTON_GREEN 4
#define BUTTON_RED 5


#define LED_STATUS_EN 3   //LED_STATUS_EN
#define LED_BRACK 11  //LED_LIGHTS_B0
#define LED_PARK_TAIL 13  //LED_LIGHTS_B1
#define LED_TURN_LEFT A0  //LED_LIGHTS_C0
#define LED_TURN_RIGHT A1  //LED_LIGHTS_C1
#define LED_BACKUP A2  //LED_LIGHTS_C2
#define LED_HEAD A3  //LED_LIGHTS_C3

#define ADC6_BAT_VOLT_LEVEL A6
#define ADC7_BAT_VOLT A7
#define AIN0_BAT_VOLT_LEVEL 9
#define AIN1_BAT_VOLT 10

volatile uint8_t *port_to_pcmask[] = {
  &PCMSK0,
  &PCMSK1,
  &PCMSK2
};

static int PCintMode[24];

typedef void (*voidFuncPtr)(void);

volatile static voidFuncPtr PCintFunc[24] = { 
  NULL };

volatile static uint8_t PCintLast[3];



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
  pinMode(13, OUTPUT);
  digitalWrite(POWER_STATE_D0, HIGH);
  digitalWrite(POWER_STATE_D1, HIGH); 
 // attachInterrupt(1,count,RISING);
  PCattachInterrupt(4, count, FALLING );
}

void loop() {
  //control Power board 5v arduino,5v arduino + 5 volt odroid, 5v arduino + 5 volt odroid + 12 volt motor
 if (interrupt_flag==true)
    powerBoard();
  serialEvent(); //read serial port commands from odroid
  if (stringComplete) 
  {
    servo_control=false;
    lightControl(); // control lights of the car, this function should call befor control motor
    if (servo_control==true)
      servoControl(); // control servo motor
    inputString = "";
    stringComplete = false;
  }
  // clear the string:
 
}
/* Control power board*/
void powerBoard(){
  MsTimer2::set(3000, turnOffCar); // 3 s period
  MsTimer2::start();
  interrupt_flag=false;
  if (state%3==0)//Press green button for first time
  {
    digitalWrite(POWER_STATE_D0, LOW);
    digitalWrite(POWER_STATE_D1, HIGH); 

  } else if (state%3==1) //Press green button for second time
  {
    digitalWrite(POWER_STATE_D0, LOW);
    digitalWrite(POWER_STATE_D1, LOW);
  } else if (state%3==2)//Press green button for third time
  {
    digitalWrite(POWER_STATE_D0, HIGH);
    digitalWrite(POWER_STATE_D0, LOW); 
  }
}
void count()
{
  state++;
  interrupt_flag=true;
}
void turnOffCar()
{
  int carShutdown=digitalRead(BUTTON_GREEN);
  if (carShutdown==0)
  {
    digitalWrite(POWER_STATE_D0, HIGH);
    digitalWrite(POWER_STATE_D0, HIGH); 
  }
  MsTimer2::stop();
}
/* Control lights*/
void servoControl(){
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
   
} 
/* Control lights*/
void lightControl(){ 
  resetLights();
  if (inputString=="enL\r")
    digitalWrite(LED_STATUS_EN, HIGH);
  else if (inputString=="diL\r")
    digitalWrite(LED_STATUS_EN, LOW);
  else if (inputString=="le\r")
  {
    MsTimer2::set(500, flashLeftLight); // 0.5 s period
    MsTimer2::start();
  }  
  else if (inputString=="ri\r")
  {
    MsTimer2::set(500, flashRightLight); // 0.5 s period
    MsTimer2::start();
  } 
  else if (inputString=="stop\r")
    digitalWrite(LED_BRACK, HIGH);
  else if (inputString=="pa\r")
    digitalWrite(LED_PARK_TAIL, HIGH);
  else if (inputString=="ta\r")
    digitalWrite(LED_PARK_TAIL, HIGH);
  else if (inputString=="re\r")
    digitalWrite(LED_BACKUP, HIGH);
  else if (inputString=="fo\r")
    digitalWrite(LED_HEAD, HIGH);
  else
    servo_control=true; 
} 
void resetLights()
{
  MsTimer2::stop(); //stop flashing lights if it is on
  digitalWrite(LED_TURN_LEFT, LOW);
  digitalWrite(LED_TURN_RIGHT, LOW);
  digitalWrite(LED_BRACK, LOW);
  digitalWrite(LED_PARK_TAIL, LOW);
  digitalWrite(LED_BACKUP, LOW);
  digitalWrite(LED_HEAD, LOW);
}
void flashLeftLight()
{
  ledState=!ledState;
  digitalWrite(LED_TURN_LEFT, ledState);
}
void flashRightLight()
{
  ledState=!ledState;
  digitalWrite(LED_TURN_RIGHT, ledState);
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

/////////////////////////////////////////////////////////////////////////////////
/*
 * attach an interrupt to a specific pin using pin change interrupts.
 */
 void PCattachInterrupt(uint8_t pin, void (*userFunc)(void), int mode) {
  uint8_t bit = digitalPinToBitMask(pin);
  uint8_t port = digitalPinToPort(pin);
  uint8_t slot;
  volatile uint8_t *pcmask;

  // map pin to PCIR register
  if (port == NOT_A_PORT) {
    return;
  } 
  else {
    port -= 2;
    pcmask = port_to_pcmask[port];
  }

// -- Fix by Baziki. In the original sources it was a little bug, which cause analog ports to work incorrectly.
  if (port == 1) {
     slot = port * 8 + (pin - 14);
  }
  else {
     slot = port * 8 + (pin % 8);
  }
// --Fix end
  PCintMode[slot] = mode;
  PCintFunc[slot] = userFunc;
  // set the mask
  *pcmask |= bit;
  // enable the interrupt
  PCICR |= 0x01 << port;
}

void PCdetachInterrupt(uint8_t pin) {
  uint8_t bit = digitalPinToBitMask(pin);
  uint8_t port = digitalPinToPort(pin);
  volatile uint8_t *pcmask;

  // map pin to PCIR register
  if (port == NOT_A_PORT) {
    return;
  } 
  else {
    port -= 2;
    pcmask = port_to_pcmask[port];
  }

  // disable the mask.
  *pcmask &= ~bit;
  // if that's the last one, disable the interrupt.
  if (*pcmask == 0) {
    PCICR &= ~(0x01 << port);
  }
}

// common code for isr handler. "port" is the PCINT number.
// there isn't really a good way to back-map ports and masks to pins.
static void PCint(uint8_t port) {
  uint8_t bit;
  uint8_t curr;
  uint8_t mask;
  uint8_t pin;

  // get the pin states for the indicated port.
  curr = *portInputRegister(port+2);
  mask = curr ^ PCintLast[port];
  PCintLast[port] = curr;
  // mask is pins that have changed. screen out non pcint pins.
  if ((mask &= *port_to_pcmask[port]) == 0) {
    return;
  }
  // mask is pcint pins that have changed.
  for (uint8_t i=0; i < 8; i++) {
    bit = 0x01 << i;
    if (bit & mask) {
      pin = port * 8 + i;
      // Trigger interrupt if mode is CHANGE, or if mode is RISING and
      // the bit is currently high, or if mode is FALLING and bit is low.
      if ((PCintMode[pin] == CHANGE
          || ((PCintMode[pin] == RISING) && (curr & bit))
          || ((PCintMode[pin] == FALLING) && !(curr & bit)))
          && (PCintFunc[pin] != NULL)) {
        PCintFunc[pin]();
      }
    }
  }
}


SIGNAL(PCINT0_vect) {
  PCint(0);
}
SIGNAL(PCINT1_vect) {
  PCint(1);
}
SIGNAL(PCINT2_vect) {
  PCint(2);
}

