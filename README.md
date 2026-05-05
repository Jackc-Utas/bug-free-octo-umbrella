/*
  KAA109 Engineering Problem Solving and Data Analysis
  Assignment 3: Arduino Autonomous Vehicle
  Team Number: 17
  Team Members: Jack Connell, Caleb Reynolds, Romy Di Ubaldo
  
  This programme implements the following funcitonality:
   * 
   *  
   * 
*/

// Enable debugging:
// If defined then output will be printed through Serial.print
#define DEBUG

// Tinkercad or physical system selection:
// If TINKERCAD is defined, then the RGB LEDs will use different
// values for enabling and setting their red, green and blue outputs.
//#define TINKERCAD

#ifdef DEBUG
#define log(x) Serial.print(x)
#define logln(x) Serial.println(x)
#else
#define log(x)
#define logln(x)
#endif

// Drive pin definitions
#define DRIVE_RIGHT_PIN_1 9   // PWM/analogWrite(0-255)
#define DRIVE_RIGHT_PIN_2 8
#define DRIVE_LEFT_PIN_1  10  // PWM/analogWrite(0-255)
#define DRIVE_LEFT_PIN_2  12

// Encoder pin definitions:
#define LEFT_ENCODER_PIN  2
#define RIGHT_ENCODER_PIN 3

// Ultrasonic sensor pin and value definitions:
#define LEFT_TRIG_PIN     A1
#define LEFT_ECHO_PIN     A0

#define RIGHT_TRIG_PIN    A2
#define RIGHT_ECHO_PIN    A3

// RGB LED pin definitions:
#define RED_LED_PIN           5
#define GREEN_LED_PIN         11
#define BLUE_LED_PIN          6
#define LED_LEFT_EN_PIN       4
#define LED_RIGHT_EN_PIN      7

// 'On' brightness of RGB LEDs (max 255):
#ifdef TINKERCAD
#define BRIGHT_PWM        255
#else
#define BRIGHT_PWM        16
#endif

// Left and right enable output values:
// Hint: Make sure you enable the left and right LEDs
//       if you want them to be on.
// Hint: To set red, green and blue outputs use the
//       LED_PWM_VALUE(x) macro to get their brightness.
#ifdef TINKERCAD
#define LED_LEFT_ENABLED   LOW
#define LED_RIGHT_ENABLED  LOW
#define LED_LEFT_DISABLED  HIGH
#define LED_RIGHT_DISABLED HIGH
#define LED_PWM_VALUE(pwm) pwm
#else
#define LED_LEFT_ENABLED   HIGH
#define LED_RIGHT_ENABLED  HIGH
#define LED_LEFT_DISABLED  LOW
#define LED_RIGHT_DISABLED LOW
#define LED_PWM_VALUE(pwm) (255-pwm)
#endif

//START OF CODE:

// Who Did what Component
/*
Romy - Sensors and Input Code
  
Caleb - Movement and Hardware Control Code

Jack - Navigation and Obstacle Detection Code

*/
// Group setup component


// set up Millis components for loop timing
unsigned long startStartTime = 0;
unsigned long flashStartTime = 0;
unsigned long obstacleClearTime = 0;

// The const long below sets the time that an obstacle can be seen before repeating
const unsigned long CLEAR_CONFIRM_MS = 200; 

// Sets up a true or false statement to define right and left 
int lockedTurnDirection = 1; // 1 = right, -1 = left

// Turning timing variables
const unsigned long TURN_TIME_MS = 400;
const unsigned long STOP_PAUSE_MS = 100;

// Flash rate for LED's
const unsigned long FLASH_INTERVAL_MS = 250;


//Navigation and Obstacle Detection setup constants ect.
// enum defines values for the individual states of the robot with a number 0-9 to determine which case the robot is in.
enum ROBOTSTATE {
 STATE_START,
 STATE_REORIENT,
 STATE_MOVE_FORWARD,
 STATE_AVOID_OBSTACLE,
 STATE_TURN_LEFT,
 STATE_TURN_RIGHT,
 STATE_BYPASS_DRIVE,
 STATE_STATIONARY,
 STATE_COMPLETED,
};

// Sets up variables for changing states in the state machine switch by defining a variable  for the ROBOTSTATE to set a state and thus it's number representation.
ROBOTSTATE CURRENT_STATE = STATE_START;
ROBOTSTATE NEW_STATE = STATE_START;

// Distance parameters
const float pi = 3.14;
// cm this is the acceptable Distance from the obstacle that the robot can be, this means after this point the robot will stop.
const float object_threshold = 20.0; 
//cm position of the robot in x
const float target_x = 200.0; 
//cm position of the robot in y
const float target_y = 0.0; 
//This is the target tolerance from an object before the robot stops moving and switches states 
const float target_tolerance = 20.0; // cm

// Timing parameters
long State_Start_Time = 0;


//Setup Movement and Hardware Control Code
//Restrict wheel speeds (0 to 255)
#define DRIVE_SPEED 160
#define TURN_SPEED 120
#define REVERSE_SPEED 160

//Define the pin modes (output or input) for each Pin used on the Robot
void setup(){
  pinMode(DRIVE_RIGHT_PIN_1, OUTPUT);
  pinMode(DRIVE_RIGHT_PIN_2, OUTPUT);
  pinMode(DRIVE_LEFT_PIN_1, OUTPUT);
  pinMode(DRIVE_LEFT_PIN_2, OUTPUT);
  
  pinMode(LED_LEFT_EN_PIN, OUTPUT);
  pinMode(LED_RIGHT_EN_PIN, OUTPUT);
  pinMode(RED_LED_PIN, OUTPUT);
  pinMode(GREEN_LED_PIN, OUTPUT);
  pinMode(BLUE_LED_PIN, OUTPUT);

  pinMode(LEFT_TRIG_PIN, OUTPUT);
  pinMode(LEFT_ECHO_PIN, INPUT);
  pinMode(RIGHT_TRIG_PIN, OUTPUT);
  pinMode(RIGHT_ECHO_PIN, INPUT);

// Sets the left and right encoders as increasing values (rising)
  attachInterrupt(digitalPinToInterrupt(LEFT_ENCODER_PIN), Left_Encoder_ISR, RISING);
  attachInterrupt(digitalPinToInterrupt(RIGHT_ENCODER_PIN), Right_Encoder_ISR, RISING);

// Ensures that the robot is set to stop with no LED output once the robot starts by calling the below functions
  Stop_Motors();
  Disable_Both_LEDs();
}


//Setup Sensors and Input Code
// Wheel Distance values from the wheel encoders
const float Wheel_Circumference = pi * 13;
const float Turn_Radius = 13.0 / 2.0;

const float CM_PER_PULSE = (pi * 6.7) / 20;
const float DEGREES_PER_PULSE = (CM_PER_PULSE / Wheel_Circumference) * 360;

// Position Values
float Left_Distance = 0; 
float Right_Distance = 0; 

float x_position = 0; 
float y_position = 0; 
float Heading_Deg = 0; 
float Avoid_Target_Heading = 0;

// Obstacle logic definitions
bool Obstacle_Left = false; 
bool Obstacle_Right = false; 

// Encoder value Definitions
// Current encoder count
volatile long Left_Encoder_Count = 0; 
volatile long Right_Encoder_Count = 0; 

// Previous_ encoder count
long Previous_Left_Encoder_Count = 0; 
long Previous_Right_Encoder_Count = 0; 

// sets a time for the robot to ignore obstacle detection later in the code
const unsigned long REORIENT_OBSTACLE_SUPPRESS_MS = 600;


//Functions for changing/representing robot states
//uses the wheel encoder knowledge to find the displacement of the robot from the end point(0,200)
float Distance_To_Target () 
{
  float dx = target_x - x_position; // compare the point in space the robot is in refernece to the target in the x-axis
  float dy = target_y - y_position; // compare the point in space the robot is in refernece to the target in the y-axis
  return sqrt(dx * dx + dy * dy); // use pythagoras theorem to determine the Distance in a straight line to the target
}

// true or false statement switch about the position of the robot in space compared to end point.
bool Target_Reached () 
{
  return Distance_To_Target() < target_tolerance;
}

// detects if the sensor data about the left and right of the robot is less then the target tolerance (Distance allowed from an object) 
bool Obstacle_Detected ()
{
  return (Left_Distance > 0 && Left_Distance < object_threshold) || 
  (Right_Distance > 0 && Right_Distance < object_threshold);
}

// chooses based upon the Distance from the sensor to dictate how the robot should act
int Direction_of_Turning ()
{
  // obstacle only on right side, turn left
  if (Obstacle_Right && !Obstacle_Left) return -1;
  
  // obstacle only on left side, turn right
  if (Obstacle_Left && !Obstacle_Right) return 1;
  
  // both sensors or neither — default turn right
  return 1;
}

// State change aiding the final loop
void Changestate(ROBOTSTATE NEW_STATE) 
{
  CURRENT_STATE = NEW_STATE;
  State_Start_Time = millis ();
}


// Movement and Hardware Control Functions
//Set the left and right drive pins to high/low to make the car drive forward and set the the high pins to the drive speed in an analogue write.
void Drive_Forwards ()
{
  digitalWrite(DRIVE_RIGHT_PIN_1, LOW);
  analogWrite(DRIVE_RIGHT_PIN_2, DRIVE_SPEED);
  digitalWrite(DRIVE_LEFT_PIN_1, LOW);
  analogWrite(DRIVE_LEFT_PIN_2, DRIVE_SPEED);
}

//Set all drive pins to 0 as some pins are set to an analogue value, ensuring the motors stop reliably 
void Stop_Motors ()
{
  analogWrite(DRIVE_RIGHT_PIN_1, 0);
  analogWrite(DRIVE_RIGHT_PIN_2, 0);
  analogWrite(DRIVE_LEFT_PIN_1, 0);
  analogWrite(DRIVE_LEFT_PIN_2, 0);
}

//Set the right drive pins to rotate forwards and set the left drive pins to rotate in reverse
void Turn_Left ()
{
  analogWrite(DRIVE_RIGHT_PIN_1, TURN_SPEED);
  digitalWrite(DRIVE_RIGHT_PIN_2, LOW);
  digitalWrite(DRIVE_LEFT_PIN_1, LOW);
  analogWrite(DRIVE_LEFT_PIN_2, TURN_SPEED);
}

//Set the right drive pins to rotate in reverse and set the left dive pins to rotate forwards
void Turn_Right ()
{
  digitalWrite(DRIVE_RIGHT_PIN_1, LOW);
  analogWrite(DRIVE_RIGHT_PIN_2, TURN_SPEED);
  analogWrite(DRIVE_LEFT_PIN_1, TURN_SPEED);
  digitalWrite(DRIVE_LEFT_PIN_2, LOW);
}

// Turns on only the Left LED to recieve a colour value
void Enable_Left_LED ()
{
  digitalWrite(LED_LEFT_EN_PIN, LED_LEFT_ENABLED);
}

// Turns off only the Left LED
void Disable_Left_LED ()
{
  digitalWrite(LED_LEFT_EN_PIN, LED_LEFT_DISABLED);
}

// Turns on only the right LED to receive a colour
void Enable_Right_LED ()
{
  digitalWrite(LED_RIGHT_EN_PIN, LED_RIGHT_ENABLED);
}

// Turns off only the right LED
void Disable_Right_LED ()
{
  digitalWrite(LED_RIGHT_EN_PIN, LED_RIGHT_DISABLED);
}

// Turns on both LEDs to recieve a colour value
void Enable_Both_LEDs ()
{
  Enable_Left_LED();
  Enable_Right_LED();
}

//  Turns off both LEDs
void Disable_Both_LEDs ()
{
  Disable_Left_LED();
  Disable_Right_LED();
}

// Sets up a universal function such that the led colour can be defined by it later on
void Set_LED_Colour(int red, int green, int blue)
{
  analogWrite(RED_LED_PIN, LED_PWM_VALUE(red));
  analogWrite(GREEN_LED_PIN, LED_PWM_VALUE(green));
  analogWrite(BLUE_LED_PIN, LED_PWM_VALUE(blue));
}

// Sets both LEDs on and to a green LED
void Driving_LEDs ()
{
  Enable_Both_LEDs();
  Set_LED_Colour(0, BRIGHT_PWM, 0);
}

// Turn on the left and ensure the right is disabled and set the colour to blue
void Left_Turn_LED ()
{
  Enable_Left_LED();
  Disable_Right_LED();
  Set_LED_Colour(0, 0, BRIGHT_PWM);
}

// Turn on the right and ensure the left is disabled and set the colour to blue
void Right_Turn_LED ()
{
  Enable_Right_LED();
  Disable_Left_LED();
  Set_LED_Colour(0, 0, BRIGHT_PWM);
}

// Sets both LEDs to red
void Set_Red_LED ()
{
  Enable_Both_LEDs ();
  Set_LED_Colour(BRIGHT_PWM, 0, 0);
}

// Calls the earlier drive forward function and set a time variable dependent version
void Drive_Forwards (unsigned long timeMs)
{
  Drive_Forwards();
  delay(timeMs);
  Stop_Motors();
}

// Calls the earlier Turn left function and set a time variable dependent version
void Turn_LeftFor(unsigned long timeMs)
{
  Turn_Left();
  delay(timeMs);
  Stop_Motors();
}

// Calls the earlier Turn right function and set a time variable dependent version
void Turn_Right_For (unsigned long timeMs)
{
  Turn_Right();
  delay(timeMs);
  Stop_Motors();
}

// Sets a universal stop function for stopping the motors and disabling the LEDs
void Stop()
{
  Stop_Motors();
  Disable_Both_LEDs();
}

// Uses encoder values to determine the point the robot is in space relative to the target 2m from the start point
void Update_Heading (int Turn_Sign) 
{
  long Left_Delta  = Left_Encoder_Count  - Previous_Left_Encoder_Count; // change in left Distance from the heading
  long Right_Delta = Right_Encoder_Count - Previous_Right_Encoder_Count; // change in right Distance from the heading
  
  Previous_Left_Encoder_Count  = Left_Encoder_Count;
  Previous_Right_Encoder_Count = Right_Encoder_Count;
  
  float AVERAGE_Delta = (Left_Delta + Right_Delta) / 2.0;
  Heading_Deg += Turn_Sign * AVERAGE_Delta * DEGREES_PER_PULSE;
  
  while (Heading_Deg < -180) Heading_Deg += 360;
  while (Heading_Deg >  180) Heading_Deg -= 360;
}


// Sensors and Input Functions
/* Sets up a function which signals the ultrasonic sensor to send a pulse and recieve any returning waves. using this, the function returns Distance in cm and filters out invalid Distances, those being duration 0 or less than 0 Distance or over 400cm Distance values.
*/
float Read_Ultrasonic_Distance(int Trig_Pin, int Echo_Pin)
{
  digitalWrite(Trig_Pin, LOW);
  delayMicroseconds(2);
  digitalWrite(Trig_Pin, HIGH);
  delayMicroseconds(10);
  digitalWrite(Trig_Pin, LOW);

  unsigned long Duration = pulseIn(Echo_Pin, HIGH, 30000); // timeout after 30ms

  if (Duration == 0)
  {
    return -1;  // Invalid reading, return error value
  }

  float Distance = Duration / 58.0;  // Convert pulse Duration to cm

  // Check if the Distance is within a reasonable range (0 - 400 cm)
  if (Distance <= 0 || Distance > 400)
  {
    return -1;  // Invalid Distance, return error
  }

  return Distance;  // Return valid Distance in cm
}

// Updates left and right Distance values 
void Update_Distance_Readings () 
{ 
 Left_Distance = Read_Ultrasonic_Distance(LEFT_TRIG_PIN, LEFT_ECHO_PIN); 
 Right_Distance = Read_Ultrasonic_Distance(RIGHT_TRIG_PIN, RIGHT_ECHO_PIN); 
} 

// Updates obstacle logic
void UpdateObstacleFlags() 
{ 
 if(Left_Distance > 0 && Left_Distance < object_threshold) 
 { 
 	Obstacle_Left = true; 
 } 
 	else 
 { 
 	Obstacle_Left = false;
 } 
 	if(Right_Distance > 0 && Right_Distance < object_threshold) 
 {
		Obstacle_Right = true
 }
	else
 { 			
	Obstacle_Right = false;
 } 
} 

// Left Encoder value increases for every encoder pass
void Left_Encoder_ISR() 
{ 
 Left_Encoder_Count++; 
} 

// Right Encoder value increases for every encoder pass
void Right_Encoder_ISR() 
{ 
 Right_Encoder_Count++; 
} 

// This function uses the left encoder and right encoder counts both past and present to determine the robots position relative to the target by utilising previous functions and defined values.
void UpdatePosition() 
{ 
 long Left_Change = Left_Encoder_Count - Previous_Left_Encoder_Count; 
 long Right_Change = Right_Encoder_Count - Previous_Right_Encoder_Count; 

 Previous_Left_Encoder_Count = Left_Encoder_Count; 
 Previous_Right_Encoder_Count = Right_Encoder_Count; 

 float AVG_Change = (Left_Change + Right_Change) / 2.0;  // use 2.0 to avoid integer division
 float Distance_Cm = AVG_Change * CM_PER_PULSE;

 float Heading_in_Radians = Heading_Deg * pi / 180.0;

 x_position += Distance_Cm * cos(Heading_in_Radians); 
 y_position += Distance_Cm * sin(Heading_in_Radians);

} 

// determines the angle from the centreline for the robot
float angle_to_target () 
{
float dx = target_x - x_position;
float dy = target_y - y_position;
return atan2(dy, dx) *(180 / pi);
}

// the following function calls all of the sensor update functions  
void Update_Sensors_And_Inputs() 
{ 
 Update_Distance_Readings(); 
 UpdateObstacleFlags(); 
} 



// Start of Main Loop
void loop () {
// if the the statement below, the robot has reached it's goal, thus exit loop() immediately and nothing else runs
if (CURRENT_STATE == STATE_COMPLETED) {
  Stop_Motors();
  Enable_Both_LEDs();
  Set_LED_Colour(0, 0, BRIGHT_PWM);
  return; 
}

  // update sensor values and input values
  Update_Sensors_And_Inputs ();  
  // define current time for the robot, this being the time that the robot has been turned on for
  unsigned long Current_Time = millis ();
  
   switch (CURRENT_STATE) 
   {
	case STATE_START:
    	Stop_Motors (); 
    	Disable_Both_LEDs ();
		Heading_Deg = angle_to_target();
    	if (Target_Reached ()) 
		{  
     	 	Changestate(STATE_COMPLETED);
    	}
    	else 
		{  
     		Changestate (STATE_MOVE_FORWARD); 
    	}
   	 	break;
    // This case is for driving to the target
    case STATE_MOVE_FORWARD:
   	{ 	
     	UpdatePosition(); 
      	Driving_LEDs (); // turns on both LEDs to green
    	if (Target_Reached ()) // if the robot is at x = 0, y = 200, the robot stops
        {
        	Stop_Motors (); 
        	Changestate(STATE_COMPLETED);
        }
    	else if (Obstacle_Detected()) 
        { 
        	Stop_Motors (); 
        	flashStartTime = Current_Time;
        	Changestate(STATE_AVOID_OBSTACLE);
        }
      // checks to see if the robot is heading off course
    	else 
   		{
        	float targetAngle = angle_to_target ();
        	float Angle_Diff = targetAngle - Heading_Deg;
        	while (Angle_Diff < -180) Angle_Diff += 360;
        	while (Angle_Diff > 180)  Angle_Diff -= 360;
        	if (abs(Angle_Diff) > 15) 
        	{
        		Stop_Motors ();
        		Changestate (STATE_REORIENT);
    		}
        	else 
    		{
        		Drive_Forwards ();
    		}
		}
	}
    break;
    case STATE_TURN_LEFT:
	{
		Left_Turn_LED();
		float Angle_Diff = Avoid_Target_Heading - Heading_Deg;
		while (Angle_Diff < -180) Angle_Diff += 360;
		while (Angle_Diff >  180) Angle_Diff -= 360;
		if (abs(Angle_Diff) > 5) 
		{
			Turn_Left();
			Update_Heading(-1);
		}
  		else 
		{
    		Stop_Motors();
    		Changestate(STATE_BYPASS_DRIVE);
  		}
  	break;
	}
	case STATE_TURN_RIGHT:
	{
	Right_Turn_LED();
	float Angle_Diff = Avoid_Target_Heading - Heading_Deg;
	while (Angle_Diff < -180) Angle_Diff += 360;
	while (Angle_Diff >  180) Angle_Diff -= 360;
	if (abs(Angle_Diff) > 5) 
	{
    	Turn_Right();
    	Update_Heading(+1);
	}
	else 
	{
    	Stop_Motors();
    	Changestate(STATE_BYPASS_DRIVE);
	}
  	break;
	}
	case STATE_REORIENT:
  {
    unsigned long timeInState = Current_Time - State_Start_Time;

    // briefly pause so encoders reset
  if (timeInState < 200)
  {
    Stop_Motors();
    Previous_Left_Encoder_Count  = Left_Encoder_Count;
    Previous_Right_Encoder_Count = Right_Encoder_Count;
    break;
  }


  if (Obstacle_Detected() && timeInState > REORIENT_OBSTACLE_SUPPRESS_MS)
  {
    Stop_Motors();
    Changestate(STATE_AVOID_OBSTACLE);
    break;
  }
  Driving_LEDs ();

  float targetAngle = angle_to_target ();
  float Angle_Diff = targetAngle - Heading_Deg;

  // normalise the angles to -180 and +180 degrees values
  while (Angle_Diff < -180) Angle_Diff += 360;
  while (Angle_Diff > 180) Angle_Diff -= 360;

  //checks if the robot is within an acceptable degree dispacement from the target.
  if (abs(Angle_Diff) < 5) {
    Stop_Motors ();
    Changestate (STATE_MOVE_FORWARD);
  }
  else if (Angle_Diff > 0) {
  Turn_Right();
  Update_Heading(+1);
}
else {
  Turn_Left();
  Update_Heading(-1);
}
  }
  break;

  case STATE_AVOID_OBSTACLE:
  Stop_Motors();

  if ((Current_Time / FLASH_INTERVAL_MS) % 2 == 0) Set_Red_LED();
  else Disable_Both_LEDs();

  if (Current_Time - State_Start_Time >= FLASH_INTERVAL_MS * 4) {
    lockedTurnDirection = Direction_of_Turning();
    Avoid_Target_Heading = Heading_Deg + (lockedTurnDirection * 90.0);
    while (Avoid_Target_Heading < -180) Avoid_Target_Heading += 360;
    while (Avoid_Target_Heading >  180) Avoid_Target_Heading -= 360;
    if (lockedTurnDirection < 0) Changestate(STATE_TURN_LEFT);
    else                         Changestate(STATE_TURN_RIGHT);
  }
  break;

  case STATE_BYPASS_DRIVE:
{
  Driving_LEDs();

  unsigned long timeInState = Current_Time - State_Start_Time;
  const unsigned long MIN_BYPASS_MS =670;

  if (timeInState < MIN_BYPASS_MS) {
    Drive_Forwards();
  }
  else if (!Obstacle_Detected()) {
    Stop_Motors ();
    Changestate (STATE_REORIENT);
  }
  else {
    // obstacle is not detected any longer
    Drive_Forwards ();
  }
  break;
}

    case STATE_COMPLETED:
    
    	Stop_Motors ();
     
     	Enable_Both_LEDs ();
     
     	Set_LED_Colour (0, 0, BRIGHT_PWM);
    
    	break;
  }
}
