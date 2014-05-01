
/*

Title : BotB27-Basic-Controls
Authors : Chris DL1CB www.Darc.de,  
And of Course you are going to extend this CODE......and take over then world!


*/

// ATMEL ATMEGA8 & ATMEGA168 / ARDUINO
//
//                      +-\/-+
//                PC6  1|    |28  PC5 (AI 5)
//          (D 0) PD0  2|    |27  PC4 (AI 4)
//          (D 1) PD1  3|    |26  PC3 (AI 3)
// Ping in  (D 2) PD2  4|    |25  PC2 (AI 2)
// PWM+     (D 3) PD3  5|    |24  PC1 (AI 1)
//          (D 4) PD4  6|    |23  PC0 (AI 0)
//                VCC  7|    |22  GND
//                GND  8|    |21  AREF
//                PB6  9|    |20  AVCC
//                PB7 10|    |19  PB5 (D 13)
// PWM+     (D 5) PD5 11|    |18  PB4 (D 12)
// PWM+     (D 6) PD6 12|    |17  PB3 (D 11) 
//          (D 7) PD7 13|    |16  PB2 (D 10) PWM
//          (D 8) PB0 14|    |15  PB1 (D 9)  PWM
//                      +----+
//
// (PWM+ indicates the additional PWM pins on the ATmega168.  We are using the ATmega8 it does not have PWM+)

// What are the different pins used for on the BotB27
// Pin 1  Reset - This is the microprocessor reset, by briefly setting it to the ground (-) it will cause a reset.
// Pin 2  TX    - Sends Text Messages to the computer terminal and used to program the processor
// Pin 3  RX    - Receives Text Messages fom the computer terminal and used to program the processor
// Pin 4  
// Pin 5
// Pin 6
// Pin 7  VCC   - This is a opower pin it needs 3.5 - 5.0 Volts i.e. the positive (+) from the battery, usually the red wire
// Pin 8  GND   - This is a ground pin, it needs 0 Volts, i.e the negative (-) from the battery, usually the black wire
// Pin 9  Xtal  - This pin connects to the crystal that supports the internal ocilator (clock) to tick at 16Mhz
// Pin 10 Xtal
// Pin 11 leftLED
// Pin 12
// Pin 13 MotorDirectionLeft
// Pin 14 MotorDirectionRight
// Pin 15 MotorSpeedRight
// Pin 16 MotorSpeedLeft
// Pin 17
// Pin 18
// Pin 19 rightLED
// Pin 20 AVCC - This is a power pin it needs 3.5 - 5.0 Volts
// Pin 21
// Pin 22
// Pin 23 analogEyeRightPin
// Pin 24 analogEyeLeftPin
// Pin 25
// Pin 26
// Pin 27
// Pin 28

/*
      
      Hier ein IR  emphaenger, was signalen aus pin 1 ergibt.
      Der IR emphaenger signalisiert wenn ein Infrarot licht 
      mit win träger frequez fon 32kHz ankommt.
       ___
      [_O_]
      | | |
      | | |
      | | |  
      1 2 3
      
      IR Pin 1 data  = Pin 2 (chip 4)
      IR Pin 2 +5V   = Pin 3 (chip 5)
      IR Pin 3 (GND) = Pin 4 (chip 6)
      
      Jeder IR fernbedienung nutztseing 
      eigene protokolle, Dazu ist es am besten 
      ein universal fernbedinung zu nuzten!
            
*/




// Define Varaibles and Constants

unsigned long time;  // Just a global variable to keep track of time

// Here are the constants and the varibales for the left Eye.....well its an infrared photo diode....and it sees light and dark
const int analogEyeLeftPin = 1;  // The output of the photo diode is fed into the Microporcessors Analog input port. 
int analogEyeLeft;               // In this integer variable we keep the calibrated value of the photo diode. It has a range 0..255
int analogEyeLeftMin = 1023;     // minimum sensor value ,This will be modified by the analogEye Calibration 
int analogEyeLeftMax = 0;        // maximum sensor value ,This will be modified by the analogEye Calibration

// Here are the constants and the varibales for the right Eye. It's set up the same way as the left eye, described above
const int analogEyeRightPin = 0; // 
int analogEyeRight;              // 
int analogEyeRightMin = 1023;    // 
int analogEyeRightMax = 0;       // 

// Define the contants and variables for the Motors

const int motorSpeedLeftPin = 10;      // Out of thid pin comes the Pulse Width Modulated (PWM) signal that controls how much power the motor receives 
const int motorSpeedRightPin = 9;    // 
const int motorDirectionLeftPin = 7;  // This pin controls the direction of the left pin together with the phase of the pwm modulation. 
const int motorDirectionRightPin = 8; // 

byte motorSpeedLeft = 0;              // The motorSpeed has a range of 0 - 255 , 0 means stop , 255 means full power      
byte motorSpeedRight = 0;             //

const boolean forward = LOW;          // Just Some Constants to make this code more readable
const boolean reverse = HIGH;         //

// Define the constants and variables for the LED's
const int leftLED =  5;    //- The Green LED
const int rightLED = 13;  //- The Red LED

// Define the pins used for the IR receiver
int pwr_high = 4;   //Sensor pin 1  +5v
int pwr_low = 3;    //Sensor pin 2  0v
int ir_pin = 2;     //Sensor pin 3, der DATA pin

// Define the variables for the IR receiver
volatile word IrData = 0x0000;
volatile boolean IrAvailable = false;

// flag indigating if the line is to be followed or not
volatile boolean followLine = false;



// CALIBRATE EYES
// The robot owner has a few seconds to set the eye's on a
// light and dark surfaces, that the range of linght and dark can be calibrated
// This is then used by the leftEyeValue() and rightEyeValue() functiond for calibrated constants
void calibrateEyes(){

  time = millis();
  // calibrate during the next 3 seconds 
  while (millis() < time + 3000) {
 
 
     // Calibrate the leftEye   
    digitalWrite(leftLED,HIGH);
    digitalWrite(rightLED,LOW);
    
    analogEyeLeft = analogRead(analogEyeLeftPin);

    // record the maximum sensor value
    if (analogEyeLeft > analogEyeLeftMax) {
        analogEyeLeftMax = analogEyeLeft;
    }

    // record the minimum sensor value
    if (analogEyeLeft < analogEyeLeftMin) {
      analogEyeLeftMin = analogEyeLeft;
    }
    
    delay(50);
    
    // Calibrate the righEye    
    digitalWrite(leftLED,LOW);
    digitalWrite(rightLED,HIGH);
    
    analogEyeRight = analogRead(analogEyeRightPin);

    // record the maximum sensor value
    if (analogEyeRight > analogEyeRightMax) {
       
        analogEyeRightMax = analogEyeRight;
    }

    // record the minimum sensor value
    if (analogEyeRight < analogEyeRightMin) {
    
      analogEyeRightMin = analogEyeRight;
    }
    
    delay(50);
    
    
  } // end while loop

} // end calibrate eyes


// READ LEFT EYE VALUE

int leftEyeValue(){

  // read the sensor:
  analogEyeLeft = analogRead(analogEyeLeftPin);
  // apply the calibration to the sensor reading
  analogEyeLeft = map(analogEyeLeft, analogEyeLeftMax, analogEyeLeftMin, 255, 0);
  // in case the sensor value is outside the range seen during calibration
  analogEyeLeft = constrain(analogEyeLeft, 0, 255);

  return analogEyeLeft;

}

// READ RIGHT EYE VALUE

int rightEyeValue(){

  // read the sensor:
  analogEyeRight = analogRead(analogEyeRightPin);
  // apply the calibration to the sensor reading
  analogEyeRight = map(analogEyeRight, analogEyeRightMax, analogEyeRightMin, 255, 0);
  // in case the sensor value is outside the range seen during calibration
  analogEyeRight = constrain(analogEyeRight, 0, 255);

  return analogEyeRight;

}




// CONTROL LEFT MOTOR SPEED AND DIRECTION
// I control the speed and diection of the LEFT motor , by setting the direction control pins and setting the pulse width modulation.
void setMotorLeft(byte motorSpeed, boolean direction){

  if (direction == forward){
    digitalWrite(motorDirectionLeftPin,forward);
    analogWrite(motorSpeedLeftPin,motorSpeed);
  }

  if (direction == reverse){
    digitalWrite(motorDirectionLeftPin,reverse);
    analogWrite(motorSpeedLeftPin,255-motorSpeed);
  }

}

// CONTROL RIGHT MOTOR SPEED AND DIRECTION
void setMotorRight(byte motorSpeed, boolean direction){

  if (direction == forward){
    digitalWrite(motorDirectionRightPin,forward);
    analogWrite(motorSpeedRightPin,motorSpeed);
  }

  if (direction == reverse){
    digitalWrite(motorDirectionRightPin,reverse);
    analogWrite(motorSpeedRightPin,255-motorSpeed);
  }

}


// ONE TIME SETUP
// setup is a program life cycle routine it is called first, but only once 
void setup()   {                
 

  // initialise the analog motor speed pin as output:
  pinMode(motorSpeedLeftPin, OUTPUT);    
  pinMode(motorSpeedRightPin, OUTPUT);
  
  // initialize the digital pin as an output:
  pinMode(motorDirectionLeftPin, OUTPUT);    
  pinMode(motorDirectionRightPin, OUTPUT);  

  // set the output pins off
  digitalWrite(motorDirectionLeftPin, forward);    
  digitalWrite(motorDirectionRightPin,forward);
  
  // stop both motors
  //setMotorLeft(0,forward);
  //setMotorRight(0,forward);
   
  // set the output pins of the LED .... to output!
  pinMode(leftLED, OUTPUT);
  pinMode(rightLED, OUTPUT);


  // Initialize the Serial port with a speed 9600 bits per second
  Serial.begin(9600); 
  Serial.println("Initializing"); 

  //calibrateEyes();
    
  // Set Pin Modes for IR receiver
  pinMode(pwr_high, OUTPUT);    
  pinMode(pwr_low, OUTPUT);
  pinMode(ir_pin, INPUT);
  
  // Give the IR receiver power
  digitalWrite(pwr_high, HIGH);    //Sensor pin 1 always high
  digitalWrite(pwr_low, LOW);      //Sensor pin 2 always low
  
  delay(1); // let the IR receiver settle, otherwise the next interrupt could be un-nesseralily fired.
  
  // Listen for INT0 on teh ir_pin and fire off the function readInfrared
  attachInterrupt(0, readInfrared, LOW); // listen for infrared interrupts
  

   
}


// RUN LOOP 
void loop(){
    
    // Read in the calibrated eye values
    int leftValue = leftEyeValue();
    int rigthValue = rightEyeValue();
    
    /*
    delay(200);   
    Serial.print( leftValue, DEC);
    Serial.print("          ");
    Serial.println( rigthValue, DEC);
    */
    
 
    if(leftValue > 200){
      digitalWrite(leftLED,HIGH);
    }else{
      digitalWrite(leftLED,LOW);
    }
    
    if(rigthValue > 200){
      digitalWrite(rightLED,HIGH);
    }else{
      digitalWrite(rightLED,LOW);
    }
 
  
   if(followLine){
       setMotorLeft(rigthValue,forward);
       setMotorRight(leftValue,forward);
   }
   
  // handle IR data  
  if(IrAvailable){
    
    switch (IrData) {
        case 43368:
         
          // follow the line
          followLine = !followLine;
           if(followLine){
             setMotorLeft(rigthValue,forward);
             setMotorRight(leftValue,forward);
          }else{
             setMotorLeft(0,forward);
             setMotorRight(0,forward);
          }
      
          Serial.print("Follow Line ");
          Serial.println(followLine,BIN); 
          break;
        case 39592:
            // forward
            setMotorLeft(250,forward);
            setMotorRight(250,forward);
 
            Serial.println("Forward");
          break;
        case 39588:
            // reverse
            setMotorLeft(250,reverse);
            setMotorRight(250,reverse);
 
            Serial.println("Reverse");
          break;
        case 43364:
           // stop
           followLine = false;
           setMotorLeft(0,forward);
           setMotorRight(0,forward);
           
          Serial.println("Stop");
          break;
        case 42664:
           // right turn
           setMotorLeft(250,forward); 
           setMotorRight(0,forward);
          Serial.println("Right Turn");
          break;
        case 42660:
           // left turn
           setMotorLeft(0,forward); 
           setMotorRight(250,forward);
          Serial.println("Left Turn");
          break;
        case 38312:
          // stop motors
          followLine = false;
          setMotorLeft(0,forward); 
          setMotorRight(0,forward);
          Serial.println("Calibrate Eyes");
          calibrateEyes();
          break;  
          
        //default: 
          //Serial.println("Error");
    
    } // end swithc   

      delay(100);
      IrAvailable = false; // clear the IrAvailable flag
    
    
  } // end if  IrAvailanble

 
    
} // end of main loop


// ISR : Interrupt Service Routine : read infrared reciever
void readInfrared(){
  
  // This is to prevent over writing the last received value
  if(IrAvailable == false){
      // make sure we read the data in the middle  of the data pulse width
      delayMicroseconds(416);
      IrData = 0;
      // read in 28 bits.. only the last 16 bits are of interest
      for (int p = 28; p > 0; p--)  {
         // clock the 28 bits into IrData[ 16 bit register] by sh
         IrData = IrData | ((!digitalRead(2)) << p);
         // 833uS is the duty cycle of a bit
         delayMicroseconds(833);   
      }
      // got data ? set the data available flag
      if(IrData > 0){
        IrAvailable = true;
      }
      
   }
    
}// end ISR
