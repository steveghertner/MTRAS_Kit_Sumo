//MTRAS SUMO robot kit example code.
#include <QTRSensors.h>
#include <Servo.h>
#define NUM_SENSORS 6 // number of sensors used
#define TIMEOUT 2500 // waits for 2500 microseconds for sensor outputs to go low
#define EMITTER_PIN 2 // emitter is controlled by digital pin 2


// Odd numbered sensors 1 through 11 are connected to digital pins 3 through 8, respectively
QTRSensorsRC qtrrc((unsigned char[]) {3, 4, 5, 6, 7, 8},
NUM_SENSORS, TIMEOUT, EMITTER_PIN);
unsigned int sensorValues[NUM_SENSORS];
Servo Leftservo; // create servo object to control left servo
Servo Rightservo; // create servo object to control right servo

#include <NewPing.h>
#include <Servo.h>


#define TRIGGER_PIN  12      // Wire the sensor from the Trig pin on the Sensor to pin 8 on the S rail. 
#define ECHO_PIN         9  //Wire the sensor from Echo on the Sensor to pin 9 on the S rail.
#define MAX_DISTANCE 75 //Max distance you want the ultrasonic sensor to read in cm. 


NewPing sonar(TRIGGER_PIN, ECHO_PIN, MAX_DISTANCE);


unsigned int time;
int distance;
int triggerDistance = 50; //Adjust this value to whatever you want the range detection of your robot to be. Measured in cm.
int fDistance;
int buttonState = 0;         // variable for reading the pushbutton status.
const int buttonPin = A0;     // Pin number of the pushbutton pin.


void setup(){
Serial.begin(9600); // open the serial port at 9600 bps:


pinMode(13, OUTPUT); //Setting Pin 13 as an output as it has a built in LED. 

    Serial.println("Calibrating");       // print as an ASCII-encoded decimal


digitalWrite(13, HIGH);    // turn on Arduino's LED to indicate we are in calibration mode
  for (int i = 0; i < 250; i++)  // make the calibration take about 5 seconds
  {
    qtrrc.calibrate();
  }
  digitalWrite(13, LOW);     // turn off Arduino's LED to indicate we are through with calibration


  pinMode(buttonPin, INPUT);

ButtonStart();
}  //Ends setup routine. 


void loop()
{
// read calibrated sensor values and obtain a measure of the line position from 0 to 5000
unsigned int position = qtrrc.readLine(sensorValues);
// If the position variable is less than 2000, the sensor is right of center
if (position < 2000||position > 3000){
backup();
}
else{
    Serial.println("Forward");       // print as an ASCII-encoded decimal

Leftservo.write(125); // Give a value between 0 and 180. Theoretically 90 should stop both servos.But it varies slightly.
Rightservo.write(25); // 0-80 controls the servo in one direction and 90 - 180 the other. The closer to90 the slower and vice versa.
  }
}

void forward(){
Serial.println("Forward");       // print as an ASCII-encoded decimal

Leftservo.write(125); // Give a value between 0 and 180. Theoretically 90 should stop both servos.But it varies slightly.
Rightservo.write(25); // 0-80 controls the servo in one direction and 90 - 180 the other. The closer to90 the slower and vice versa.me to turned passed the point where it saw something and further to the
}

void backup(){
    Serial.println("Backing Up");       // print as an ASCII-encoded decimal

Leftservo.write(25);
Rightservo.write(125);
delay(1000);
turn();
}

void turn(){
    Serial.println("Turning");       // print as an ASCII-encoded decimal

Leftservo.write(120);
Rightservo.write(120);
scan(); //Go to scan so as we are turning we can see if an opponent is in front of us. 
}

void scan() {                       //This portion of code uses the NewPing library to make our lives easier.
    Serial.println("Scanning and Turning");       // print as an ASCII-encoded decimal

  time = sonar.ping();
  distance = time / US_ROUNDTRIP_CM;
  if (distance == 0) {
    distance = 100;
  }
  delay(10);
  fDistance = distance;             //Taking the variable given from the scan result and assigning it to "fDistance".
  if (fDistance < triggerDistance) { // The robot saw something within the trigger distance.
   forward();
  }
else scan();
}

void ButtonStart(){
    Serial.println("Press Start Button");       // print as an ASCII-encoded decimal

  // read the state of the pushbutton value:
  buttonState = digitalRead(buttonPin);

  // check if the pushbutton is pressed. If it is, the buttonState is HIGH:
  if (buttonState == HIGH) {
        Serial.println("Button Pressed");       // print as an ASCII-encoded decimal

        delay(1000); //Wait 5 seconds before beginning main loop. 
        Serial.println("5");       // print as an ASCII-encoded decimal
        delay(1000); //Wait 5 seconds before beginning main loop. 
        Serial.println("4");       // print as an ASCII-encoded decimal
        delay(1000); //Wait 5 seconds before beginning main loop. 
        Serial.println("3");       // print as an ASCII-encoded decimal
        delay(1000); //Wait 5 seconds before beginning main loop. 
        Serial.println("2");       // print as an ASCII-encoded decimal
        delay(1000); //Wait 5 seconds before beginning main loop. 
        Serial.println("1");       // print as an ASCII-encoded decimal
        delay(1000); //Wait 5 seconds before beginning main loop. 
    Leftservo.attach(10); // attaches the servo on pin 10 to the servo object
    Rightservo.attach(11); // attaches the servo on pin 11 to the servo object
   loop(); //Goto main loop. 
  } else {
   ButtonStart(); //If button not pressed go back to ButtonStart
  }
}
