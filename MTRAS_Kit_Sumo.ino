/*
MTRAS SUMO robot kit example code.
Base code for SUMO kit build Jan 2020

Libraries Required: 
  QTRSensors pololu library version 4
  NewPing
*/
#include <QTRSensors.h>
#include <Servo.h>
#include <NewPing.h>

#define NUM_SENSORS 6 // number of sensors used
#define TIMEOUT 2500 // waits for 2500 microseconds for sensor outputs to go low
#define EMITTER_PIN 2 // emitter is controlled by digital pin 2


// Odd numbered sensors 1 through 11 are connected to digital pins 3 through 8, respectively
QTRSensors qtrrc;

Servo Leftservo; // create servo object to control left servo
Servo Rightservo; // create servo object to control right servo

#define TRIGGER_PIN  12      // Wire the sensor from the Trig pin on the Sensor to pin 8 on the S rail. 
#define ECHO_PIN      9  //Wire the sensor from Echo on the Sensor to pin 9 on the S rail.
#define MAX_DISTANCE 75 //Max distance you want the ultrasonic sensor to read in cm. 
#define RING_BORDER_THRESHOLD 250 //can go to about 1000

NewPing sonar(TRIGGER_PIN, ECHO_PIN, MAX_DISTANCE); //instantiant instance of ping sensor

//variables used for ping sensor
unsigned int pingTime; //time it take for roundtrip read of ultrasonic signal
int distance;
int triggerDistance = 50; //Adjust this value to whatever you want the range detection of your robot to be. Measured in cm.
boolean objectFound=false; //used to determine if opponent is found
//variables used for pushbutton
int buttonState = 0;         // variable for reading the pushbutton status.
const int buttonPin = A0;     // Pin number of the pushbutton pin.
//variables used for line sensing
unsigned int sensorValues[NUM_SENSORS];

void setup(){
  
  Serial.begin(9600); // open the serial port at 9600 bps:
  
  qtrrc.setTypeRC(); //setup qt line sensor type, pin #s
  qtrrc.setSensorPins((const uint8_t[]) {3, 4, 5, 6, 7, 8},NUM_SENSORS);
  qtrrc.setEmitterPin(EMITTER_PIN);
  
  //set output pins
  pinMode(13, OUTPUT); //Setting Pin 13 as an output as it has a built in LED. 
  pinMode(buttonPin, INPUT_PULLUP);  //setup pushbutton pin with internal pullup
  
  digitalWrite(13, HIGH);    // turn on Arduino's LED to indicate we are in 5 sec
                              //wait time before battle
  ButtonStart(); //wait for PB press and do countdown
  digitalWrite(13, LOW);     // turn off Arduino's LED to indicate ready for battle
}  //Ends setup routine. 


void loop(){
  //variables used to average sensor values
  int sumOfReadings=0;
  int avgReading=0;
  // read calibrated sensor values and obtain a measure of the line position from 0 to 5000
  
  //int position=0;
  // read  sensor values values between 0 and XXXX
  qtrrc.read(sensorValues); //read sensor values
  
  for(int i=0; i<NUM_SENSORS;i++){ //calculate average reading for all 6 sensors
    sumOfReadings+=sensorValues[i]; 
  }
  avgReading=sumOfReadings/NUM_SENSORS;
  //Serial.println(avgReading);
  if(avgReading<RING_BORDER_THRESHOLD){  //if at ring border, turn then and go forward
    backup();
    delay(750);
    turn();
    delay(750); //time to allow turn, change time to make turn longer/shorter
    forward();  //go foward for some time to clear border
    //delay(500);
  }else{ //if not at ring border then
  if(objectFound){//if see oponent, then go forward
    forward();
   }else{
    turn(); // if object no found then turn
   }
    //delay(100); //movement interval time before scanning again
    scan(); //scan to see if see opponent  
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
  //delay(1000);
  //turn();
}

void turn(){
  //could break this into two separate functions to
  //turn right or left based on which sensors saw line first
  Serial.println("Turning");       // print as an ASCII-encoded decimal
  Leftservo.write(120); //turn left
  Rightservo.write(120);
  //Leftservo.write(25);  //turn right
  //Rightservo.write(25);
  //scan(); //Go to scan so as we are turning we can see if an opponent is in front of us. 
}

void scan() {                       
  //This portion of code uses the NewPing library to determine distance
  //using ultrasonic sensor
  Serial.println("Scanning");       // print as an ASCII-encoded decimal
  pingTime = sonar.ping(); //read time for ultrasonic ping roundtrip
  distance = pingTime / US_ROUNDTRIP_CM; //calc distance in CM
  if (distance == 0) {      //if null value set to very long
    distance = 100;
  }
  Serial.println(distance);
  delay(10);         //dealy before moving on
  //fDistance = distance;             //Taking the variable given from the scan result and assigning it to "fDistance".
  if (distance < triggerDistance) { // The robot saw something within the trigger distance.
   //forward();
   objectFound=true;  //set flag that opponent is found
  }else{
    objectFound=false;
  }
//else scan();
  
}

void ButtonStart(){
    int buttonState=HIGH; //set default button state to not pushed
    Serial.println("Press Start Button");       // print as an ASCII-encoded decimal
    digitalWrite(13, HIGH); //turn on LED while waiting for PB Press 
  // check if the pushbutton is pressed. If it is, the buttonState is HIGH:
  while (buttonState == HIGH) {  //wait for pushbutton press
        // read the state of the pushbutton value:
        buttonState = digitalRead(buttonPin);
        delay(10); //debounce time
        //buttonState = digitalRead(buttonPin);
  }      
        Serial.println("Button Pressed");       // print as an ASCII-encoded decimal
        digitalWrite(13, LOW); //blink light each second
        delay(1000); //Wait 5 seconds before beginning main loop. 
        Serial.println("4"); // print as an ASCII-encoded decimal
        digitalWrite(13, HIGH); //blink light each second
        delay(1000); //Wait 5 seconds before beginning main loop. 
        Serial.println("3"); // print as an ASCII-encoded decimal
        digitalWrite(13, LOW); //blink light each second
        delay(1000); //Wait 5 seconds before beginning main loop. 
        Serial.println("2");  // print as an ASCII-encoded decimal
        digitalWrite(13, HIGH); //blink light each second
        delay(1000); //Wait 5 seconds before beginning main loop. 
        Serial.println("1");       // print as an ASCII-encoded decimal
        delay(1000); //Wait 5 seconds before beginning main loop. 
        digitalWrite(13, LOW); //blink light each second
        Serial.println("0-- GO");       // print as an ASCII-encoded decimal
        //attach servo inputs now
        Leftservo.attach(10);  // attaches the servo on pin 10 to the servo object
        Rightservo.attach(11);  // attaches the servo on pin 11 to the servo object

}
