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
int startingPosition = 1; //front = 1; middle = 2; back = 3;
float distanceStartTurning; //point at which robot is ready to start turning, determined based on position
bool turnReady = false; //bool to determine if the robot is in position to turn
bool lookingDownTrench = false; //bool to determine if robot is looking in the correct direction
float desiredHeading = 0; //heading down the trench
float currentHeading; // heading updated every loop
float maxTurning = 0; //value associated with robot's maximum turning radius
float Kp = 1;
float Kd = 1;
float filterStrength = 0.9;
float rawHeading;
float input;
float filteredSignal_previous = 0; 
int prevflow = 0;


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
  if (dist >= distanceStartTurning) {
    turnReady = true;
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
    if(abs(currentHeading - desiredHeading) < 0.05) {
      myservo.write(0);
      turnReady = false;
      lookingDownTrench = true;
    }
  }

//closed loop control law to keep the robot straight while going down the trench
  if(lookingDownTrench) {
    
  }


  ////////////// SERVOMOTOR ///////////////////////////////////////////////////

  if(servoDir == 0)
  {
    pos++;
    if(pos >= 180)
    {
      servoDir = 1;
    }
  }
  else
  {
    pos--;
    if(pos <= 0)
    {
      servoDir = 0;
    }
  }

  myservo.write(pos);              // tell servo to go to position in variable 'pos'
  delay(10); 

////////////// MAGNETOMETER ///////////////////////////////////////////////////

  mag.read();
  imu.read();

  float heading = computeHeading();


////////////// SOLENOID VALVE ///////////////////////////////////////////////////

actuatePiston();

////////////// REED SWITCH ///////////////////////////////////////////////////
  switchState = digitalRead(switchPin);

////////////// Serial Print  ///////////////////////////////////////////////////
  //Serial.print("Reed Switch: ");
  //Serial.print(switchState);
  Serial.print("   Magnetometer: ");
  Serial.println(heading);

  //delay(100);
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
  get the current data
  initial steering function (won't do anything if the condition in the function isn't met)
  correction steering function

  actuate piston for one cycle

  estimate position
  */


/*
function for estimating distance traveled (impulse response)
  will take inputs as:
    -theta (servo direction relative to robot)
    -phi (robot direction)
    -time
    -index counter
  will ouput:
    -current position
    -current angle
  Notes:
  pass in struct or array of sampled data
  as a struct or some array in order to return both values
  take the circumference (or portion of circumference) divide by the current time - previous time to get impulse
  use the set loop frequency's time to predict the distance covered by the robot in that time
*/

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

float averagingFilter(float measuredSignal, float filterStrength){   
  float filterOutput = (1-filterStrength)*measuredSignal + 
    filterStrength*filteredSignal_previous;    
  filteredSignal_previous = filterOutput;   
  return filterOutput; 
}
/*
function for initial steer (maybe only activate once)
  if target 1 (located outside the trench) is satisfied
    begin turning at 40 degrees
    if magnotometer is not pointed down the trench yet (keyword yet, as in this should be as lng as its less than or more than a certain value)
      keep turning

concerns: need to figure out how to turn it on or off without a for loop and allowing the piston to continue to actuate
*/

/*
function for correction steering
  if robot is not headed toward the target (some range of values to be considered the middle)
    if robot is too far right
      adjust left
    if robot is too far left
      adjust right
    else
      stay straight
  may have to consider a right triangle of the robot angle from the target being 
  theta and the adjacent side to calculate the opposite side (POV distance from target) 
  to judge if it is straight enough, however angle could be enough
*/

/*
function for data sampling
  grabs data from each working component:
  - magnotometer
  - switch
  - servo motor
  - piston (maybe not?)
  - time from arduino for every wheel turn
  To calculate the impulse response we need to see how far and fast the robot goes after one piston fire. 
  The switch is activated every wheel turn and each each time the switch is on the time will be taken
*/




/*
Q/A

3) how to seperate piston activation and steering
4) what would the code look like for data sampling and implementing the gain correction
*/