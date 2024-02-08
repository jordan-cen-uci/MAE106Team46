void setup() {
  // put your setup code here, to run once:

}

void loop() {
  // put your main code here, to run repeatedly:

  /*
  get the current data
  initial steering function (won't do anything if the condition in the function isn't met)
  correction steering function

  actuate piston for one cycle

  estimate position
  */

}

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