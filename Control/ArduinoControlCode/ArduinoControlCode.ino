#include <Servo.h>
#include <Wire.h>
#include <LIS3MDL.h>
#include <LSM6.h>

LIS3MDL mag;
LSM6 imu;

LIS3MDL::vector<int16_t> m_min = { -6366,	-2594,	1633};
LIS3MDL::vector<int16_t> m_max = {-2597,	762,	2086};

Servo myservo;

int servoPin = 3;       // Pin that the servomotor is connected to
int solenoidPin = 2;    // Pin that the mosfet is conected to
int switchPin = 4;      // Pin that the switch is conected to
int pos = 0;            // variable to store the servo position
int switchState;        // variable that stores the Reed switch state
int servoDir = 0;       // variable that stores the direction the motor is turning in the demo program
int solenoidState = LOW;  // variable that stores if solenoid is on or off         
unsigned long previousMillis = 0;        // will store last time solenoid was updated
const long interval = 1000;           // interval at which to turn solenoid on and off (milliseconds)
LIS3MDL::vector<float> times;

float dist = 0;
float distanceStartTurning; //point at which robot is ready to start turning, determined based on position
bool starting = true;
bool turnReady = false; //bool to determine if the robot is in position to turn
bool lookingDownTrench = false; //bool to determine if robot is looking in the correct direction
float desiredHeading = 339; //heading down the trench
float currentHeading; // heading updated every loop
float maxTurning = 0; //value associated with robot's maximum turning radius
float maxTurningRadius = 0; //turning radius asscoaited with max turning input
float desTurnDistance = 0; // distance needed to be associated with end of open loop turn
float rawHeading;
float input;
float prevTime = 0;
float filteredSignal_previous = 0; 
int prevflow = 0;

int startingPosition = 1; //front = 1; middle = 2; back = 3;
bool leftOrRight = false; //left = false; right = true;

float Kp = 1;
float Kd = 1;
float filterStrength = 0.9;


float frontDistance = 7.875;
float backDistance = 5.375;
float betweenDistance  = 7;


void setup() {
  myservo.attach(servoPin);               // attaches the servo on pin 9 to the servo object
  pinMode(solenoidPin, OUTPUT);           //Sets the pin as an output
  pinMode(switchPin, INPUT_PULLUP);       //Sets the pin as an input_pullup
  Serial.begin(9600);                     // starts serial communication @ 9600 bps
  Wire.begin();

  if (!mag.init())
  {
    Serial.println("Failed to detect and initialize LIS3MDL magnetometer!");
    while (1);
  }
  mag.enableDefault();

  if (!imu.init())
  {
    Serial.println("Failed to detect and initialize LSM6 IMU!");
    while (1);
  }
  imu.enableDefault();

// actuate solenoid once

digitalWrite(solenoidPin, HIGH);

startingParam(startingPosition);

findDesiredTurningDistance();


}

void loop() {
  // put your main code here, to run repeatedly:

  actuatePiston();
  //Call impulse data printing function
  getImpulse();

  mag.read(); //reads magnotometer
  imu.read();
  rawHeading = computeHeading();
  currentHeading = averagingFilter(rawHeading, filterStrength);


  startingParam(startingPosition);

//tells the robot to go forward and once it has covered its starting position distance it will activate the initial turn
  if (starting && dist >= distanceStartTurning) {
    turnReady = true;
    starting = false;
  }
// tells robot to turn at max distance depending on if the current heading is less than or greater than the desired
  if (turnReady) {
    if (currentHeading < desiredHeading) {
      myservo.write(maxTurning);
    }
    else {
      myservo.write(-maxTurning);
    }
// when the robot is close enough to desired heading it moves to the next section
    if(dist >= desTurnDistance) {
      myservo.write(0);
      turnReady = false;
      lookingDownTrench = true;
    }
  }

//closed loop control law to keep the robot straight while going down the trench
  if(lookingDownTrench) {
    input = -Kp * (desiredHeading - currentHeading) + Kd * (desiredHeading - currentHeading) / (millis() - prevTime);
    if (input > -maxTurning && input < maxTurning) {
      myservo.write(input);
    }
    else if (input > maxTurning) {
      myservo.write(maxTurning);
    }
    else if (input < -maxTurning) {
      myservo.write(-maxTurning);
    }
  }



  prevTime = millis();

 
}




//heading function for magnetometer

template <typename T> float computeHeading(LIS3MDL::vector<T> from)
{
  LIS3MDL::vector<int32_t> temp_m = {mag.m.x, mag.m.y, mag.m.z}; //creates vector for magnotometer readings

  // copy acceleration readings from LSM6::vector into an LIS3MDL::vector
  LIS3MDL::vector<int16_t> a = {imu.a.x, imu.a.y, imu.a.z};

  // subtract offset (average of min and max) from magnetometer readings
  temp_m.x -= ((int32_t)m_min.x + m_max.x) / 2;
  temp_m.y -= ((int32_t)m_min.y + m_max.y) / 2;
  temp_m.z -= ((int32_t)m_min.z + m_max.z) / 2;

  // compute E and N
  LIS3MDL::vector<float> E; // declares vector E
  LIS3MDL::vector<float> N; // declares vector N
  LIS3MDL::vector_cross(&temp_m, &a, &E); // goes into address of mag vector and acceleration vector and inserts into address of E
  LIS3MDL::vector_normalize(&E); 
  LIS3MDL::vector_cross(&a, &E, &N); // goes into address of a vector and E vector and inserts into address of N
  LIS3MDL::vector_normalize(&N);

  // compute heading
  float heading = atan2(LIS3MDL::vector_dot(&E, &from), LIS3MDL::vector_dot(&N, &from)) * 180 / PI;
  if (heading < 0) heading += 360; // keeps values in range of 0 to 360
  return heading;
}

/*
Returns the angular difference in the horizontal plane between a
default vector (the +X axis) and north, in degrees.
*/
float computeHeading()
{
  return computeHeading((LIS3MDL::vector<int>){1, 0, 0});
}





/*
function to actuate the piston for one cycle
  activate piston
  delay for 200 ms
  retract piston
  delay for 500 ms
*/
void actuatePiston() {
  unsigned long currentMillis = millis();
  if (currentMillis - previousMillis >= interval) {
    previousMillis = currentMillis;
    if (solenoidState == LOW) {
      //solenoidState = HIGH;
    } else {
      solenoidState = LOW;
    }
    digitalWrite(solenoidPin, solenoidState);    //Switch Solenoid ON/oFF
  }
}

/*
Function to get the current time and current distance everytime the limit switch is flicked.
*/
void getImpulse() {
  int flow = digitalRead(switchPin);
  if (prevflow != flow) {  //helps to ensure that the button only captures the first millisecond that the switch is flicked
    if (flow == 0){
      dist = dist + (3.14 * 2.75 / 5); //calculates distance in inches
      Serial.print(millis()); //prints out the data of time into serial 
      Serial.print("\t");
      Serial.println(dist); //prints out distance right after before going to new line
    }
  }
  prevflow = flow; //resets the previous flow number to what it is now so that there are no repeats for the same switch flick
}

/*
Function to set the starting parameters based on position
*/
void startingParam(int startingPos) {
  switch(startingPos) {
    case 1:
      distanceStartTurning = 0;
    case 2:
      distanceStartTurning = 0;
    case 3:
      distanceStartTurning = 0;
    default:
      distanceStartTurning = 0;
  }
}

/*
Function to filter the measured signal
*/
float averagingFilter(float measuredSignal, float filterStrength){   
  float filterOutput = (1-filterStrength)*measuredSignal + 
    filterStrength*filteredSignal_previous;    
  filteredSignal_previous = filterOutput;   
  return filterOutput; 
}

/*
Function to help find the desired distance the wheel must move in order to complete the 90 degree turn into the trench.
*/
void findDesiredTurningDistance() {
  desTurnDistance = distanceStartTurning;
  if (leftOrRight) {
    desTurnDistance = desTurnDistance + ((3.14/2)*sqrt(pow(betweenDistance, 2) + pow(maxTurningRadius + (frontDistance/2), 2)));
  }
  else {
    desTurnDistance = desTurnDistance + ((3.14/2)*sqrt(pow(betweenDistance, 2) + pow(maxTurningRadius - (frontDistance/2), 2)));
  }
}
