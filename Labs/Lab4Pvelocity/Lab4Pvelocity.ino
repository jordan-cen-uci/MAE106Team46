//---------------------------------------------------------------------------

/*
 * Code to run a Pololu Dual MC33926 motor driver with an encoder
 * Encoder has 'aCPR' counts per revolution of the output shaft. 
 * This code includes a PI velocity controller 
 *
 * Authors:  Joan Aguilar Mayans + Brenad Smith
 * Date:   4/15/16
 */

//---------------------------------------------------------------------------
//-----------------Experimental Parameters (Change These)--------------------
//---------------------------------------------------------------------------

float Kp = 1; // proportional controller gain
boolean constantVelocity = true; // true or false
float desVel_amplitude = 60; // magnitude of constant desVel or amplitude of sinusoidal desVel
float desVel_frequency = 1;

//---------------------------------------------------------------------------
//-------------Initialize variable names (Don't Change These)----------------
//---------------------------------------------------------------------------


// If using interrupts, it only works with motor 2 (unless the motor driver is remapped).
const int D2 = 4;  // D2 pin, disables the motors if low
const int M2DIR = 8;  // Motor 2 direction 
const int M2PWM = 10;  // Motor 2 input (PWM)
const int M2EN  = 12;  // Motor 2 Diag
const int M2FB  = 15;  // (Analog1 pin)  Motor 2 current sense
const float CPR = 464.1;  // Encoder counts per revolution

// Encoder/interrupt pins
const int encoderApin = 2;
const int encoderBpin = 3;

// Encoder status
volatile int encoderA;
volatile int encoderB;

// Time variables
float currentMicros;  // time in microseconds
float oldMicros;  // previous measurement of time in microseconds
float loopDuration = 1000; // microseconds; sets the duration of each loop
int printcounter = 0; // how many samples since the last print to screen

// Position variables
const int origPos = 0;  // Original position
volatile long rawPos;  // Actual position
float Pos;
float oldPos;  // Old position value

// Velocity variables
float Vel;
float desVel;

float v[5] = {0.0};



// Controller variables
float u;
float motorAmps;
float ampsError;

// Oscilloscope
int numSampsWait;
float delayToTakeData;
int maxNumSamps = 100;
int numSampsSaved = 0;
int waitforOscope = 0;
float oscopeChan1[100] = {0.0};
float oscopeChan2[100] = {0.0};
float oscopeChan3[100] = {0.0};

//---------------------------------------------------------------------------
//-------------Setup() runs one time when the Arduino turns on---------------
//---------------------------------------------------------------------------

void setup() {
  
  // Enable motors
  pinMode(D2, OUTPUT);
  digitalWrite(D2, HIGH);
  
  // Enable motor direction pin
  pinMode(M2DIR, OUTPUT);
  
  // Enable current sense
  pinMode(M2FB, INPUT);
  
  // Enable another analog sensor input
  pinMode(A2, INPUT);  // A2 is defined in the environment to correspond to the # that specifies Analog Input 2
  pinMode(A3, OUTPUT); // We can use analog output 3 to power our sensor if we set it as an output
  digitalWrite(A3,HIGH); // This sets analog pint 3 to provide 5 volts
  
  // Enable encoder
  pinMode(encoderApin, INPUT);
  pinMode(encoderBpin, INPUT);
  
  // Read encoder status
  encoderA = digitalRead(encoderApin);
  encoderB = digitalRead(encoderBpin);
  
  // Attach interrupt service routines for each possible interrupt
  attachInterrupt(0, encoderAchange, CHANGE);
  attachInterrupt(1, encoderBchange, CHANGE);
  
  currentMicros = (float)micros();
  oldMicros = currentMicros;
  
  rawPos = origPos;  // Set raw position  
  Pos = 2.0*PI*((float)rawPos)/CPR;  // Set position
  oldPos = Pos;  // Set old position

  Vel = 0;
  desVel = 0;
  v[0] = 0;
  v[1]= 0;
  v[2] = 0;
  v[3] = 0;
  v[4] = 0;
   
  ampsError = 0;
  
  Serial.begin(115200); // Open serial port
}

//---------------------------------------------------------------------------
//------After setup runs, this loop runs over and over forever---------------
//---------------------------------------------------------------------------

void loop() {
  currentMicros = (float)micros(); // Current time in microseconds, obtained from Ardunio clock via function micros()
  
  if (((currentMicros - oldMicros) >= loopDuration)&&(numSampsSaved < maxNumSamps)) { // wait for loopDuration (1000) microseconds to elapse so we sample at 1/loopDuration (1000 Hz)
    // Desired position
    desVel = desiredVelocity(currentMicros/1000000.);
    // calculate position from rawPos
    Pos = 2.0*PI*((float)rawPos)/CPR; // rawPos is the number of encoder ticks, convert to radians
    // calculate velocity by taking the difference between two last encoder readigns and dividing by time elapsed
    Vel = 1000000.0*(Pos - oldPos)/(currentMicros - oldMicros); 
    // Filter the velocity
    Vel = filter(Vel);
    // Use the controller law to calculate the control input
    u = -Kp*(Vel - desVel);
    // Send the control input to the motor
    runMotor(u);
  
    // Update values
    oldPos = Pos;
    oldMicros = currentMicros;
    
  // Oscilloscope:
  // Because writing to the serial port slows down the controller we instead write to an array 
  //   (which we call "the oscilloscope"). Then, after the controller runs for a while (as determined 
  //   by maxNumSamps) we stop the motor and dump the data to the serial port.
  // Note that we write to the background oscilloscope every numSampsWait samples.
  // Thus, if the control sample rate is 1000 Hz, and I write to my oscilloscope every numSampsWait = 20 samples
  //   then the oscilloscope will show data sampled at 50 Hz
  // Note that the oscilloscope can only save 100 samples for each channel
  //   because the Arudino does not have very much memory
        
    if (currentMicros/1000000.0 > delayToTakeData) {  
      waitforOscope++;
    }
    
    if (waitforOscope >= numSampsWait) {
      if (numSampsSaved < maxNumSamps) {
        oscopeChan1[numSampsSaved] = (float)currentMicros/1000000.0 - delayToTakeData; // channel 1 is time in microseconds
        oscopeChan2[numSampsSaved] = Vel;  // channel 2 is velocity
        oscopeChan3[numSampsSaved ] = desVel; // channel 3 is desired velocity
        numSampsSaved++;
      }
      waitforOscope = 0;
    }
  }
    
  // Check if oscilloscope is full, and if so, stop the motor and print the oscilloscope contents to the serial port
  if ((printcounter < numSampsSaved) && (numSampsSaved == maxNumSamps)) { 
    runMotor(0); // stops the motor when we are done collectiing data
    Serial.flush();
    Serial.print(oscopeChan1[printcounter], 4);
    Serial.print("\t");
    Serial.print(oscopeChan2[printcounter], 3);
    Serial.print("\t");
    Serial.println(oscopeChan3[printcounter], 3);
    printcounter++;
  }
}

//---------------------------------------------------------------------------
//-------Encoder Functions that run when Interupt Detected (i.e. ISRs)-------
//---------------------------------------------------------------------------


void encoderAchange() {
  if (digitalRead(encoderApin) == digitalRead(encoderBpin)){
    rawPos --;
  }
  else if (digitalRead(encoderApin) != digitalRead(encoderBpin)){
    rawPos ++;
  }
}

void encoderBchange() {
  if (digitalRead(encoderApin) == digitalRead(encoderBpin)){
    rawPos ++;
  }
  else if (digitalRead(encoderApin) != digitalRead(encoderBpin)){
    rawPos --;
  }
}

//---------------------------------------------------------------------------
//-----------Function that sends the command 'u' to the Motor----------------
//---------------------------------------------------------------------------

void runMotor(float u) {
  boolean forward = (u > 0.0);
  
  // we use Pulse-Width-Modulation to talk to the motor
  if (forward) {
    digitalWrite(M2DIR, LOW);
    if (u > 255.0) {analogWrite(M2PWM, 255);}
    else {analogWrite(M2PWM, (int)u);}
  }
  else {
    digitalWrite(M2DIR, HIGH);
    if (u < -255.0) {analogWrite(M2PWM, 255);}
    else {analogWrite(M2PWM, -(int)u);} 
  }
}


//---------------------------------------------------------------------------
//-------------Function that defines PID Controller---------------------------
//---------------------------------------------------------------------------

float desiredVelocity(float t) {
  if (constantVelocity){
    delayToTakeData = 0.0;
    numSampsWait = 1; // 1 = collect data as fast as possible
    return desVel_amplitude;
  }
  else {
    delayToTakeData = 2.0;
    numSampsWait = max(1,20.0/desVel_frequency); // this causes the oscope to collect 2 periods of data (rounds down to integer below)
    return desVel_amplitude * sin(2*PI*desVel_frequency * t);
  }
}

//---------------------------------------------------------------------------
//-------------Filter for cleaning up the derivative of the Encoder----------
//---------------------------------------------------------------------------

float filter(float x) {
  float output;
  int filterorder;
  int filtercutoff;
  
// implements low pass filter
// designed using http://www.schwietering.com/jayduino/filtuino/

filterorder = 1;  // 0 = no filter, 1 = 1st order, 4 = 4th order 
// 4th order filters smooth nicely but add more lag
filtercutoff = 100; // 20, 50, or 100 Hz
switch (filterorder) {
    case 0:
       output = x;
       break;
    case 1:
        v[0] = v[1];
        switch (filtercutoff) {
          case 20:
            v[1] = (5.919070381841e-2 * x) + (0.8816185924 * v[0]); 
            break;
          case 50:
            v[1] = (1.367287359973e-1 * x) + (0.7265425280 * v[0]); 
            break;
          case 100:
            v[1] = (2.452372752528e-1 * x) + (0.5095254495 * v[0]);
            break;
          default: 
            output = x; // no filter if  you don't chose a viable cutoff option
        }
        output =  (v[0] + v[1]);
        break;
      
    case 4: 
        v[0] = v[1];
        v[1] = v[2];
        v[2] = v[3];
        v[3] = v[4];
       switch (filtercutoff) {
         case 20:
          v[4] = (6.146562100506e-5 * x) + (-0.5517688291 * v[0]) + (2.5441192827 * v[1]) + (-4.4173936876 * v[2]) + (3.4240597840 * v[3]);
          break;
        case 50:
          v[4] = (4.165992044066e-4 * x) + (-0.4382651423 * v[0]) + (2.1121553551 * v[1]) + (-3.8611943490 * v[2]) + (3.1806385489 * v[3]);
          break;
        case 100:
          v[4] = (4.824343357716e-3 * x) + (-0.1873794924 * v[0]) + (1.0546654059 * v[1]) + (-2.3139884144 * v[2]) + (2.3695130072 * v[3]);
         break;
        default: 
          output = x;// no filter if  you don't chose a viable cutoff option
        }
      output =  (v[0] + v[4]) + 4*(v[1] + v[3]) + 6*v[2];
    break;

    case 3:
      v[0] = v[1];
      v[1] = 0.9* v[0] + 0.1 * x;
      output = v[1];
    break;
    
    }
  return output;
}