#include <WiFi.h>


#include <PID_v1.h>
#include<vector>

const char* ssid = ""; #SSID
const char* password = ""; #Password

const char* host = ""; #host ip address
const uint16_t port = 80; #port

int ttime=0;
int stime=0;
int cdect=0;
int c=0;
int cx=0;

const int encoderPin1 = 25;  // Encoder pin 1
const int encoderPin2 = 26;  // Encoder pin 2
const int encoderPin3 = 32;  // Encoder pin 3  // ,push_back() , pop_back()
const int encoderPin4 = 33;  // Encoder pin 4


const int ir = 21;

int irValue=0;



// Variables
volatile long leftEncoderCount = 0;   // Left encoder count
volatile long rightEncoderCount = 0;  // Right encoder count
float wheelRadius = 4.3;              // Wheel radius in cm
float axleLength = 16.1;              // Axle length in cm
float x = 0.0;                        // X coordinate
float y = 0.0;                        // Y coordinate
float theta = 0.0;     // Orientation in radians
int nc=0;

float x21=0.0;
float y21=0.0;


float x2 = 0.0;                        // X coordinate
float y2 = 0.0;                        // Y coordinate


 String vx = "A";



// Function prototypes
void leftEncoderISR();
void rightEncoderISR();
void updateOdometry();






// Define the pins for the ultrasonic sensor
int x1=0;
const int trigPin = 22;
const int echoPin = 23;
long duration;  
float distance;  
float d1;
int a=1;

// Define the pins for the motor driver

const int leftMotorPin1 = 17;
const int leftMotorPin2 = 16;


const int rightMotorPin1 = 5;
const int rightMotorPin2 = 18;                    

// Define the PID parameters
double Kp = 6.0;
double Ki = 0;
double Kd = 0;
// Define the setpoint and input variables
double setpoint = 15.0; // Target distance from the wall
double input, output, error;

// Create a PID object
PID pid(&input, &output, &setpoint, Kp, Ki, Kd, DIRECT);


WiFiClient client;


void setup() {
  ttime=millis();
  pinMode(leftMotorPin1, OUTPUT);
  pinMode(leftMotorPin2, OUTPUT);
  pinMode(rightMotorPin1, OUTPUT);
  pinMode(rightMotorPin2, OUTPUT);


  
  analogWrite(leftMotorPin1, 0);
  analogWrite(rightMotorPin1, 0);
  // Initialize the serial communication
  Serial.begin(9600);

  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
    delay(1000);
    Serial.println("Connecting to WiFi...");
  }

  Serial.println("Connected to WiFi");

  while (true){
  if (client.connect(host, port)) {
    Serial.println("Connected to server");
    break;
  } else {
    Serial.println("Connection to server failed");
    delay(1000);
  }
  }

  // Set the motor driver pins as output


  pinMode(encoderPin1, INPUT);
  pinMode(encoderPin2, INPUT);
  pinMode(encoderPin3, INPUT);
  pinMode(encoderPin4, INPUT);

  // Attach interrupts for encoder pins
  attachInterrupt(digitalPinToInterrupt(encoderPin1), leftEncoderISR, CHANGE);
  attachInterrupt(digitalPinToInterrupt(encoderPin3), rightEncoderISR, CHANGE);

  pinMode(trigPin, OUTPUT); // Sets the trigPin as an Output  
  pinMode(echoPin, INPUT); // Sets the echoPin as an Input  
  // Set the PID tuning parameters
  pid.SetMode(AUTOMATIC);
  pid.SetOutputLimits(-125, 125);
  delay(5000);

}

void loop() {
  if (a==1){

  analogWrite(leftMotorPin1, 150);
  digitalWrite(leftMotorPin2, LOW);

  analogWrite(rightMotorPin1, 150);
  digitalWrite(rightMotorPin2, LOW);
  a=0;
  }







  
  // Measure the distance from the wall using the ultrasonic sensor
 digitalWrite(trigPin, LOW);  
 delayMicroseconds(2);  
 // Sets the trigPin on HIGH state for 10 micro seconds  
 digitalWrite(trigPin, HIGH);  
 delayMicroseconds(10);  
 digitalWrite(trigPin, LOW);  
 // Reads the echoPin, returns the sound wave travel time in microseconds  
 duration = pulseIn(echoPin, HIGH);  
 // Calculating the distance  
 distance= duration*0.0344/2;  

  // Calculate the error
  
  input=distance;
  error = setpoint - distance;
  if (error<-20){
    cdect++;
    error=-20;
  }
    if (error>0){
      if (cdect>=0){
      cdect--;
      cdect--;
      }
    error=error*4;
  }



  // Compute the PID output
  pid.Compute();
  //output=0;//////////////////////////////////////////////////////////////////
  
  ttime=millis();

  //CORNERRRRRRRRRRRRRRRRRRRRRRRRRRRRRRRRRRRRRRRRRRRRRRRRRRRRRRRRRRRRRRRRRRRRRRRRRRRRRRRRRRRRRRRRRRRRRRRRRRRRRRRRRRRRRRRRRRRRRRRRRRRRRRRRRRRRRRRRRRRRRRRRRRRRRR
  if (distance>25){
    //corner detecteddddddddddddddddddd
    stime=millis();
    c++;
    cdect=0;

    vx=vx+" "+String(x21)+" "+String(y21);
    

  }


  updateOdometry();


  /*Serial.print("X: ");
  Serial.print(x);
  Serial.print(" cm\t");
  Serial.print("Y: ");
  Serial.print(y);
  Serial.print(" cm\t");
  Serial.print("Theta: ");
  Serial.print(theta*180/PI);
  Serial.println(" degree");*/
  
  // Set the motor speeds based on the PID output



  
  
  analogWrite(leftMotorPin1, 125+output);
  analogWrite(rightMotorPin1, 125-output);

  





//////////////////////////////////////////
  if (distance<=18){
  if (WiFi.status() == WL_CONNECTED && client.connected()) {
    

    String data = String(x2) +','+String(y2)+',';
    Serial.print(x2);
    Serial.print(' ');
    Serial.println(y2);
    client.print(data);

    Serial.println("Data sent to server");


   


  } else {
    // If the client is disconnected, attempt to reconnect
    if (!client.connected()) {
      Serial.println("Disconnected from server. Reconnecting...");
      client.connect(host, port);
    }

    // Wait for the connection to establish
    analogWrite(leftMotorPin1, 0);
    analogWrite(rightMotorPin1, 0);

    delay(1000);
  }
  }
  /////////////////////////

  // Print the debug information

//////////////////////////////////////////////
  if (stime>5000){
    if( x<4 && x>2 && y<10 && y>-10){
      cx++;
      if (cx==1){
      analogWrite(leftMotorPin1, 0);
      analogWrite(rightMotorPin1, 0);
      delay(1500);
      client.print(vx);
      delay(1000);
      client.print("STOP");
      
      delay(1000000);
      }
      
    }
  }

  if (theta>=2.5*PI){
      analogWrite(leftMotorPin1, 0);
      analogWrite(rightMotorPin1, 0);
      delay(1500);
      client.print(vx);
      delay(1000);
      client.print("STOP");
      
      delay(1000000);
  }


  irValue=digitalRead(ir);

  /*if (irValue==0){
    nc++;
  }
  else{
    if (nc<0){
      nc=0;
    }
    else {
      nc--;
      nc--;
    }
  }*/

  if (nc >=6){
      analogWrite(leftMotorPin1, 0);
      analogWrite(rightMotorPin1, 0);
      delay(2000);
      client.print("NC");
      
      delay(1000000);
    
  }

  // Wait for some time before the next loop
  delay(25);
}



void leftEncoderISR() {
  if (digitalRead(encoderPin2) == digitalRead(encoderPin1))
    leftEncoderCount++;
  else
    leftEncoderCount--;
}

void rightEncoderISR() {
  if (digitalRead(encoderPin4) == digitalRead(encoderPin3))
    rightEncoderCount++;
  else
    rightEncoderCount--;

}

void updateOdometry() {
  // Calculate distance traveled by each wheel
  float leftDistance = (PI * wheelRadius * leftEncoderCount) / 1788;
  float rightDistance = (PI * wheelRadius * rightEncoderCount) / 1788;

  // Calculate change in heading and displacement
  float deltaHeading = (rightDistance - leftDistance) / axleLength;
  float displacement = (leftDistance + rightDistance) / 2.0;

  // Update orientation and position
  theta += deltaHeading;
  x += displacement * cos(theta);
  y += displacement * sin(theta);

  if (distance<=25){
    x21=x2;
    y21=y2;
    x2=(x-distance*sin(theta));
    y2=(y+distance*cos(theta));


    
  }

  // Reset encoder counts
  leftEncoderCount = 0;
  rightEncoderCount = 0;
}
