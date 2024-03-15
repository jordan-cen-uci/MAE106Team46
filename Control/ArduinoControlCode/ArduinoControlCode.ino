#include <Servo.h>
#include <Wire.h>
#include <LIS3MDL.h>
#include <LSM6.h>

LIS3MDL mag;
LSM6 imu;

LIS3MDL::vector<int16_t> m_min = {-4929,	-1724,	1425};
LIS3MDL::vector<int16_t> m_max = {-2182,	651,	1822};

Servo myservo;

int servoPin = 3;       // Pin that the servomotor is connected to
int solenoidPin = 2;    // Pin that the mosfet is conected to
int switchPin = 4;      // Pin that the switch is conected to
int pos = 0;            // variable to store the servo position
int switchState;        // variable that stores the Reed switch state
int servoDir = 0;       // variable that stores the direction the motor is turning in the demo program
int solenoidState = LOW;  // variable that stores if solenoid is on or off         
unsigned long previousMillis = 0;        // will store last time solenoid was updated
const long interval = 750;           // interval at which to turn solenoid on and off (milliseconds)
LIS3MDL::vector<float> times;

float dist = 0;
float distanceStartTurning = 0; //point at which robot is ready to start turning, determined based on position
bool starting = false;
bool turnReady = false; //bool to determine if the robot is in position to turn
bool lookingDownTrench = true; //bool to determine if robot is looking in the correct direction
float desiredHeading = 330; //heading down the trench
float currentHeading; // heading updated every loop
float maxTurning = 45; //value associated with robot's maximum turning radius
float maxTurningRadius = 13.75; //turning radius asscoaited with max turning input
float desTurnDistance = 0; // distance needed to be associated with end of open loop turn
float rawHeading;
float input;
float prevTime = 0;
float filteredSignal_previous = 0; 
int prevflow = 0;
int count = 1;

int startingPosition = 0; //front = 1; middle = 2; back = 3;
bool leftOrRight = true; //left = false; right = true;

float Kp = 3;
float Kd = 0.5;
float filterStrength = 0.95;


float frontDistance = 7.875;
float backDistance = 5.375;
float betweenDistance  = 7;


void setup() {
  myservo.attach(servoPin);               // attaches the servo on pin 9 to the servo object
  pinMode(solenoidPin, OUTPUT);           //Sets the pin as an output
  pinMode(switchPin, INPUT_PULLUP);       //Sets the pin as an input_pullup
  Serial.begin(115200);                     // starts serial communication @ 9600 bps
  Wire.begin();
// setup for magnotometer
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
myservo.write(0.7 * (90));
actuatePiston();

startingParam(startingPosition); //given position, will setup the initial distance before turning

findDesiredTurningDistance(); //function to calculate how much the counting wheel should turn before finsihing the turn; based on constants that may change through testing process
Serial.println("Setup Complete");

}

void loop() {
  // put your main code here, to run repeatedly:
  //
  actuatePiston();

  //Call impulse data printing function
  getImpulse();

  mag.read(); //reads magnotometer
  imu.read();
  rawHeading = computeHeading(); //executes the template to computer heading from the magnotometer readings
  currentHeading = averagingFilter(rawHeading, filterStrength); //calls filter to put heading data through a low pass filter to make up for external noise
  Serial.println(currentHeading);

//tells the robot to go forward and once it has covered its starting position distance it will activate the initial turn
  if ((starting) && (dist >= distanceStartTurning)) {
    turnReady = true;
    starting = false;
  }
// tells robot to turn at max distance depending on which side the robot starts on
  if (turnReady) {

    if (leftOrRight) {
    //center position is located at calibrated 90 degrees;
      myservo.write(0.7 * (90 + maxTurning)); //if on right side of field, turns hard right
    }
    else {
      myservo.write(0.7 * (90 - maxTurning)); //if on left side of field, turns hard left
    }
// when the robot finishes a quarter of the arc of its turning radius it resets its steering to straight
    if(dist >= desTurnDistance) {
      Serial.println("Stopping Initial Turn");
      if (leftOrRight) {
      //to account for slack in linkages, servo to go straight is overcompensated for and then rewritten back to 90
        myservo.write(0.7 * 80); //overcompensated reset from right to center
        myservo.write(0.7 * 90); //reset to straight
      }
      else {
        myservo.write(0.7 * 105); //overcompensated reset from left to center
        myservo.write(0.7 * 90); //reset to straight
      }

      turnReady = false;
      lookingDownTrench = true;
    }
  }

//closed loop control law to keep the robot straight while going down the trench
  if(lookingDownTrench) {
    float error = desiredHeading - currentHeading;
    //wrap function in case it crosses point of 0 or 360
    if (error > 180) {
      error = desiredHeading - (360 + currentHeading);
    }
    else if (error < -180) {
      error = desiredHeading + (360 - currentHeading);
    }

    input = -Kp * (error) + (Kd * (error / (millis() - prevTime))); //control law for closed loop down the trench
    if (input > -maxTurning && input < maxTurning) {
      myservo.write(0.7 * (90 - input));
    }
    else if (input > maxTurning) {
      myservo.write(0.7 * (90 - maxTurning));
    }
    else if (input < -maxTurning) {
      myservo.write(0.7 * (90 + maxTurning));
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
      solenoidState = HIGH;
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
  //delay(10);
}

/*
Function to set the starting parameters based on position
*/
void startingParam(int startingPos) {
  switch(startingPos) {
    case 1:
      distanceStartTurning = 116;
      Serial.println("startingParam done");
      break;
    case 2:
      distanceStartTurning = 132;
      break;
    case 3:
      distanceStartTurning = 164.5;
      break;
    default:
      distanceStartTurning = 0;
      break;
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
    desTurnDistance = desTurnDistance + ((3.14/2)*sqrt(pow(betweenDistance, 2) + pow(maxTurningRadius - (frontDistance/2), 2)));
  }
  else {
    desTurnDistance = desTurnDistance + ((3.14/2)*sqrt(pow(betweenDistance, 2) + pow(maxTurningRadius + (frontDistance/2), 2)));
  }
}
