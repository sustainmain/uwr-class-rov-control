/*
   Underwater Robotics Class Arduino ROV Control Program
   
   This program reads the position of two joysticks and uses that data
   to move two horizontal thrusters and a vertical thruster. The
   thrusters this program moves are meant to be controlled using Bipolar
   MOSFET Amplifiers (BMAs).
   
   The circuit:
     - X axis of first analog joystick connected to analog input 1
     - Y axis of first analog joystick connected to analog input 0
     - Y axis of second analog joystick connected to analog input 2
     - Positive signal wire of left BMA connected to digital pin 9
     - Negative signal wire of left BMA connected to digital pin 10
     - Positive signal wire of right BMA connected to digital pin 5
     - Negative signal wire of right BMA connected to digital pin 6
     - Positive signal wire of vertical BMA connected to digital pin 11
     - Negative signal wire of vertical BMA connected to digital pin 3
   
   created 02 March 2020
   by Josh Marusek
*/



// ===== TUNING =====

#define DEADZONE 5      // The distance joystick values must be from
                        // center (0) in order to move the thrusters

#define DELAY_TIME 100  // The number of microseconds the Arduino delays
                        // between each time the joystick is read. Lower
                        // numbers mean more precision (up to a point).

// ==================


// ===== JOYSTICK VALUES =====

float joystickLeftXValue = 0;
float joystickLeftYValue = 0;
float joystickRightYValue = 0;

// ===========================


// ===== PINS =====

// Inputs
#define PIN_LEFT_JOYSTICK_X A1
#define PIN_LEFT_JOYSTICK_Y A0
#define PIN_RIGHT_JOYSTICK_Y A2

// Outputs
#define PIN_LEFT_THRUSTER_1 9
#define PIN_LEFT_THRUSTER_2 10
#define PIN_RIGHT_THRUSTER_1 5
#define PIN_RIGHT_THRUSTER_2 6
#define PIN_VERTICAL_THRUSTER_1 11
#define PIN_VERTICAL_THRUSTER_2 3

// ================


// Make an array to store the 3 thruster values
byte servoValues[3];




void setup()
{
  // Set the thruster pins to OUTPUT mode. This allows you to use them
  // as thrusters instead of sensors.
  pinMode(PIN_LEFT_THRUSTER_1, OUTPUT);
  pinMode(PIN_LEFT_THRUSTER_2, OUTPUT);
  pinMode(PIN_RIGHT_THRUSTER_1, OUTPUT);
  pinMode(PIN_RIGHT_THRUSTER_2, OUTPUT);
  pinMode(PIN_VERTICAL_THRUSTER_1, OUTPUT);
  pinMode(PIN_VERTICAL_THRUSTER_2, OUTPUT);

  // Initiate a serial connection to the computer for diagnostics
  Serial.begin(9600);
}

void loop()
{
  // Read joystick values (each one will be between 0 and 1023)
  joystickLeftXValue = analogRead(PIN_LEFT_JOYSTICK_X);
  joystickLeftYValue = analogRead(PIN_LEFT_JOYSTICK_Y);
  joystickRightYValue = analogRead(PIN_RIGHT_JOYSTICK_Y);

  // calculateServoValues automatically stores servo values in the
  // global servoValues array
  calculateServoValues(joystickLeftXValue, joystickLeftYValue, joystickRightYValue);

  
  Serial.flush();  // Wait until all earlier Serial data is done sending

  // Show the thruster values (this is for diagnostic purposes)
  Serial.print("L: " + String(servoValues[0]));
  Serial.print("  R: " + String(servoValues[1]));
  Serial.println("  V: " + String(servoValues[2]));
  Serial.println();
  

  // Send signals to thrusters
  writeThruster(PIN_LEFT_THRUSTER_1, PIN_LEFT_THRUSTER_2, servoValues[0]);
  writeThruster(PIN_RIGHT_THRUSTER_1, PIN_RIGHT_THRUSTER_2, servoValues[1]);
  writeThruster(PIN_VERTICAL_THRUSTER_1, PIN_VERTICAL_THRUSTER_2, servoValues[2]);

  delay(DELAY_TIME);  // Wait before taking the next sample and updating
                      // thrusters
}


void calculateServoValues(float jsX, float jsY, float jsV)
{
  /* Calculates left, right, and vertical servo values.

  Reads the left joystick's x and y axes to calculate the PWM value to
  give the left and right thrusters. Also reads the right joystick's y
  axis and scales it to determine the vertical thruster's PWM value.

  Parameters:
    jsX (float): the value of the left joystick's x-axis (between 0 and 1023)
    jsY (float): the value of the left joystick's y-axis (between 0 and 1023)
    jsV (float): the value of the right joystick's y-axis (between 0 and 1023)

  Returns:
    servoValues[0] (global byte): the PWM value of the left thruster
    servoValues[1] (global byte): the PWM value of the right thruster
    servoValues[2] (global byte): the PWM value of the vertical thruster
  */

  // Converts joystick values between 0 and 1023 to values between -1.0 and 1.0
  // and inverts axes as necessary
  float x = -1 * ((jsX - 511.5) / 511.5);
  float y = ((jsY - 511.5) / 511.5);
  float v = -1 * ((jsV - 511.5) / 511.5);

  /*
  // Show the joystick values (this is for diagnostic purposes)
  Serial.flush();
  Serial.print("x: " + String(x));
  Serial.print("  y: " + String(y));
  Serial.println("  v: " + String(v));
  Serial.println();
  */

  // Rescale either x or y to allow maximum power
  if (abs(x) > abs(y))
  {
    if (x > 0)
      x += (x - abs(y));
    else if (x < 0)
      x += (x + abs(y));
  }
  else if (abs(x) < abs(y))
  {
    if (y > 0)
      y += (y - abs(x));
    else if (y < 0)
      y += (y + abs(x));
  }

  // The servo turn values account for how imbalanced we want to the
  // thrusters to be. Any imbalance results in a turning effect.
  float leftServoTurn = x;
  float rightServoTurn = -1 * leftServoTurn;

  // The servo forward/backward directional values accounts for the
  // motion we want other than just rotation. Any F/B directional value
  // other than 0 results in some directional motion.
  float leftServoFBDirectional = y;
  float rightServoFBDirectional = leftServoFBDirectional;

  // Each relative servo value is the average of the servo's respective
  // turn value and directional value. The relative servo values account
  // for how much rotation the thrusters should cause as well as how
  // much directional motion they should cause.
  float leftServoRelative = (leftServoTurn + leftServoFBDirectional) / 2;
  float rightServoRelative = (rightServoTurn + rightServoFBDirectional) / 2;

  // Each servo value is the final value that is sent to the servos.
  // They're scaled up to be in the servo range (255-0).
  byte leftServoValue = 127.5 + (-127.5 * leftServoRelative);
  byte rightServoValue = 127.5 + (-127.5 * rightServoRelative);
  byte verticalServoValue = 127.5 + (-127.5 * v);

  // Store all the calculated servo values
  servoValues[0] = leftServoValue;
  servoValues[1] = rightServoValue;
  servoValues[2] = verticalServoValue;
}

void writeThruster(int pin1, int pin2, byte value)
{
  /* Sets a thruster's power level using PWM.

  Takes a PWM value (255-0) and writes it to a specified pin1 of a BMA.
  The PWM signal is negated and written to a specified pin2 of the BMA.

  Parameters:
    pin1 (int): the positive signal pin of the BMA to write to
    pin2 (int): the negative signal pin of the BMA to write to
    value (byte): the PWM value (255-0) to write to the thruster's BMA
  */

  // Move the motors only if the joystick is out of the deadzone
  if (abs(value - 127) > DEADZONE)
  {
    // Write to the pins using PWM
    analogWrite(pin1, value);
    analogWrite(pin2, 255 - value);
  }
  else
  {
    // Turn the thruster off by switching both pins to ground
    digitalWrite(pin1, LOW);
    digitalWrite(pin2, LOW);
  }
}
