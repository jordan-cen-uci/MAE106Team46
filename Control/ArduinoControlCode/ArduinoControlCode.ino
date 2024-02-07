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
function for estimating distance traveled
  will take inputs as:
    -theta (servo direction relative to robot)
    -phi (robot direction)
    -previous positions
    -previous angle??
    -time
  will ouput:
    -current position
    -current angle
    as a struct or some array in order to return both values
*/

/*
function to actuate the piston for one cycle
  activate piston
  delay for 200 ms
  retract piston
  delay for 500 ms

concerns: subject to change since retraction relies on the spring which cannot be controlled and will always be pulling the piston back
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
  may have to consider a right triangle of the robot angle from the target being theta and the adjacent side to calculate the opposite side (POV distance from target) to judge if it is straight enough, however angle could be enough
*/

/*
function for data sampling
  grabs data from each working component:
  - magnotometer
  - switch
  - servo motor
  - piston (maybe not?)

*/




/*
Q/A
1) where do i set the sampling rate
2) how important is knwoing impulse response
3) how to seperate piston activation and steering
4) what would the code look like for data sampling and implementing the gain correction
*/