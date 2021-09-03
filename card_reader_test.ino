/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
 * Written by: 
 *    Zechariah Neak
 *  
 * Description:
 *    This Arduino code is intended to drive a stepper motor with 4 shaft arms, each with a card fixed at its
 *    end. The cards are meant to be spun into place right above a card reader, which is the device to be
 *    tested. This card reader must be able to successfully scan each card a set number of times (e.g. 10000)
 *    without ever once failing. 
 *    
 *    It is assumed that each of the 4 cards have a different required scanning time, and off time (an interval
 *    at which the card must not be rescanned).
 *    
 *    This code makes possible basic serial commands that a user or script can utilize to control the motor.
 *    Such commands include:
 *    
 *              pos              - Shows the current position relative to origin.
 *              
 *              limit [int]      - Sets the max number of full revolutions the motor will spin.
 *                                 If no arg is specified, prints the current number instead.
 *                             
 *              maxSpeed [int]   - Sets the max speed of the motor in steps/sec.
 *                                 If no arg is specified, prints the current max speed instead.
 *                             
 *              speed [int]      - Sets the current speed of the motor in steps/sec.
 *                                 If no arg is specified, prints the current speed instead.
 *                             
 *              origin           - Sets the origin (i.e. position 0) of the motor. 
 *                                 For consistency, this must be done before any sequence is run.
 *                             
 *              move [int]       - If the motor is not in runmode, moves the motor by the specified
 *                                 amount of pulses. 
 *                                 A negative value will cause the motor to spin backwards.
 *                                 
 *              goto [int]       - If the motor is not in runmode, moves the motor to the specified
 *                                 position, relative to the origin.
 *                                 [TODO: Use degrees instead of pulse values?]
 *                                 
 *              go               - Enables runmode, and starts the motor running sequence, whether 
 *                                 halted, paused, or stopped.
 *              
 *              halt             - Freezes the motor in place immediately, and disables runmode.
 *              
 *              pause            - Stops the motor only after its current revolution is complete, then 
 *                                 disables runmode.
 *                                 
 *              stop             - Same as pause, but also resets the revolution counter.
 *              
 *              restart          - Performs a stop on the motor, then restarts the running sequence again
 *                                 from the beginning.
 *
 * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */


#include <AccelStepper.h>

// Define stepper motor connections and interface
#define dirPin 2
#define stepPin 3
#define motorInterfaceType 1    // for using a stepper driver

AccelStepper stepper = AccelStepper(motorInterfaceType, stepPin, dirPin);

// Depends on stepper motor model and configuration (Default: 800 for NEMA 23)
int pulsesPerRev = 800;

// For counting number of motor revolutions
int revLimit = 10;
int revNum = 0;

// For reading Serial string data
const byte numChars = 32;
char charData[numChars] = {};
bool commandReady = false;

// Affects and tracks control of motor
/*  Note that runningSpeed tracks 'runmode' speed at all times. This is opposed to stepper.speed(),
 *  which because of a quirk of stepper.setCurrentPosition(), will be set to 0 after every 
 *  full revolution.
 */
int runningSpeed = 0;
int tempSpeed = 0;              // tracks a previous or new speed val to be used after a halt
bool runMode = false;
bool isHalted = false;
 

/* ==================================================================================================== */

void setup() {
  Serial.begin(9600);

  // Sets initial position to 0. Also implicitly sets current speed to 0.
  // (Note: Please manually ensure that no card is over the reader before starting the Arduino.)
  Serial.println("<Set current position as origin>");
  stepper.setCurrentPosition(0);

  Serial.println("<Set max speed to 1600 steps/sec>");
  stepper.setMaxSpeed(1600);

  Serial.println("<Set running speed to 800 steps/sec>");
  runningSpeed = 800;
}

void loop() {
  
  if(runMode && revNum != revLimit) {
    runMotor();
  }
  else if (Serial.available() > 0) { 
    serialCommandEvent();
  }
}

/* ==================================================================================================== */

/* -------------------------------------------------------------------
 * Spin the stepper motor for 1 full revolution
 *    Adapted from sample code found on: 
 * https://www.makerguides.com/a4988-stepper-motor-driver-arduino-tutorial/ 
 * ------------------------------------------------------------------- */
bool runMotor() {
  revNum++;

  Serial.println("<Rev " + String(revNum) + ">\n");
  while(abs(stepper.currentPosition()) != abs(pulsesPerRev))
  {
    // Check for user commands via serial first
    if (Serial.available() > 0) serialCommandEvent();

    // Freezing the current rev, then possibly displacing position, will
    // interrupt the sequence, so better restart last iteration
    if (runningSpeed != 0 && isHalted) {
      revNum--;
      break;
    }

    stepper.setSpeed(runningSpeed);
    stepper.runSpeed();
  }
  if (!isHalted)
    stepper.setCurrentPosition(0);      // reset origin for next rev
  
  if(revNum == revLimit) {
    Serial.println("STATUS: " + (String)revNum + " revolutions done. Stopping motor...\n");
    runMode = false;
    revNum = 0;
    delay(1000);
  }
  delay(800);
}


/* -------------------------------------------------------------------
 * Reads Serial inputs as they appear
 * Taken from: https://forum.arduino.cc/t/serial-input-basics/278284
 * ------------------------------------------------------------------- */
void serialCommandEvent() {
  static byte i = 0;
  char newChar;

  while (Serial.available() > 0 && commandReady == false) {
    newChar = Serial.read();

    if (newChar != '\n') {
      charData[i] = newChar;
      i++;

      if (i >= numChars) {
        Serial.println("<Warning: Index has exceeded string length>");
        i = numChars - 1;
      }
    }
    else {
      charData[i] = '\0';   // terminate string
      i = 0;
      commandReady = true;
    }
  }
  processCommand();
}


/* -------------------------------------------------------------------------------------
 * Parse Serial commands and run them
 *    Adapted from: 
 * https://forum.arduino.cc/t/serial-input-basics/278284/2#parsing-the-received-data-3
 * ------------------------------------------------------------------------------------- */
void processCommand() {
  char *token;
  char commandMessage[numChars] = {};
  int commandArg = -1;
  
  if (commandReady == true) {
    token = strtok(charData, " ");                            // isolate command
    strcpy(commandMessage, token);                  
    token = strtok(NULL, "\n");                               // isolate argument

    if (token != NULL) {
      if (token[0] == '-') commandArg = 0 - atoi(token+1);    // account for negative val
      else commandArg = atoi(token);
    }
    //else Serial.println("ERROR: Invalid input.\n  Try: [command] [arg]\n");


    // Match command and execute it
    // TODO: Create an alphabetically sorted array of string-function pairs and use binary search instead
    if (strcmp(commandMessage, "pos") == 0) showCurrentPosition();
    
    else if (strcmp(commandMessage, "limit") == 0 && token == NULL) showRevLimit();
    
    else if (strcmp(commandMessage, "maxSpeed") == 0  && token == NULL) showMaxSpeed();
    
    else if (strcmp(commandMessage, "speed") == 0  && token == NULL) showCurrentSpeed();
// -----------------------------------------------------------------------------------------------------
    
    else if (strcmp(commandMessage, "limit") == 0) updateRevLimit(commandArg);
    
    else if (strcmp(commandMessage, "maxSpeed") == 0) updateMaxSpeed(commandArg);
    
    else if (strcmp(commandMessage, "speed") == 0) updateSpeed(commandArg);
    
    else if (strcmp(commandMessage, "origin") == 0) updateOrigin();
// -----------------------------------------------------------------------------------------------------
    
    else if (strcmp(commandMessage, "move") == 0) displaceTo(false, commandArg);
      
    else if (strcmp(commandMessage, "goto") == 0) displaceTo(true, commandArg);
// -----------------------------------------------------------------------------------------------------
    
    else if (strcmp(commandMessage, "go") == 0) startMotor();
    
    else if (strcmp(commandMessage, "halt") == 0) freezeMotor();
    
    else if (strcmp(commandMessage, "pause") == 0) stopMotor(false);
    
    else if (strcmp(commandMessage, "stop") == 0) stopMotor(true);
    
    else if (strcmp(commandMessage, "restart") == 0) restartMotor();

    else Serial.println("ERROR: Please input a valid command.");
    
    commandReady = false;
  }
}


/* ----------------------------------------------------------------------------------
 * Functions to be called by Serial commands, either via Serial Monitor or a script
 * ---------------------------------------------------------------------------------- */
 
 /* 'pos' */
void showCurrentPosition() {
  Serial.println("GET: Current motor position is " + String(stepper.currentPosition()) + " steps from origin");
}

/* 'limit' */
void showRevLimit() {
  Serial.println("GET: Motor will run up to " + String(revLimit) + " revolutions");
}

/* 'maxSpeed' */
void showMaxSpeed() {
  Serial.println("GET: Max motor speed is " + String(int(stepper.maxSpeed())) + " steps/s");
}

/* 'speed' */
void showCurrentSpeed() {
  Serial.println("GET: Current motor speed is " + String(runningSpeed) + " steps/s");
}
// -----------------------------------------------------------------------------------------------------

/* 'limit [max # revolutions]' */
void updateRevLimit(int newLimit) {
  revLimit = newLimit;
  Serial.println("UPDATE: motor will now spin up to " + String(revLimit) + " revolutions");
}

/* 'maxSpeed [steps per sec]' */
void updateMaxSpeed(int newMaxSpeed) {
  if (newMaxSpeed < runningSpeed) {
    Serial.println("UPDATE: adjusting running speed to match new max speed");
    runningSpeed = newMaxSpeed;
  }
  
  stepper.setMaxSpeed(newMaxSpeed);
  Serial.println("UPDATE: max motor speed is now " + String(int(stepper.maxSpeed())) + " steps/s");
}

/* 'speed [steps per sec]' */
void updateSpeed(int newSpeed) {
  if (newSpeed == 0) freezeMotor();
  
  else if (newSpeed > stepper.maxSpeed()) {
    Serial.println("UPDATE: requested speed is higher than maxSpeed of " + 
                    String(int(stepper.maxSpeed())) + " steps/s");
    newSpeed = int(stepper.maxSpeed());
    runningSpeed = newSpeed;
  }
  else if (isHalted) tempSpeed = newSpeed;
  else runningSpeed = newSpeed;
  
  Serial.println("UPDATE: motor running speed is now " + String(newSpeed) + " steps/s");
}

/* 'origin' */
void updateOrigin() {
  stepper.setCurrentPosition(0);
  Serial.println("UPDATE: origin has been reset to current position");
}
// -----------------------------------------------------------------------------------------------------

/*  'move [# steps]' or 'goto [steps from origin]' 
 *  
 *  If 'isAbsolute' is false, then displace the motor by the specified # of steps from 
 *  the current position.
 *  
 *  Otherwise, displace the motor towards the position that is at that # of steps from
 *  the set origin.
*/
void displaceTo(bool isAbsolute, int steps) {
  if (runMode) {
    Serial.println("ERROR: motor must first not be running");
  }
  else if (isAbsolute && (0 > steps || steps > pulsesPerRev)) {
    Serial.println("ERROR: target position must be between 0 and " + String(pulsesPerRev));
  }
  else {
    if (isHalted) runningSpeed = tempSpeed;

    // Set new target position, whether relative to current position or origin
    if (!isAbsolute) {
      Serial.println("DISPLACE: " + String(steps) + " steps");
      stepper.move(steps);
    }
    else {
      Serial.println("DISPLACE: " + String(steps) + " steps from origin");
      stepper.moveTo(steps);
    }

    while(stepper.currentPosition() != stepper.targetPosition()) {
      stepper.setSpeed(runningSpeed);
      stepper.runSpeedToPosition();
    }
    
    // 'True' modulo operation (Python's result) to keep position value within bounds
    stepper.setCurrentPosition(((stepper.currentPosition() % pulsesPerRev)
                               + pulsesPerRev) % pulsesPerRev);
  }
}

// -----------------------------------------------------------------------------------------------------

/*  'go' 
 *  
 *  Starts or resumes the running sequence.
*/
void startMotor() {
  if (runMode && runningSpeed != 0) Serial.println("ERROR: motor is already running");
  
  else if (revNum != revLimit) {

    if (isHalted) {
      runningSpeed = tempSpeed;
      isHalted = false;
    }   
    runMode = true;
    Serial.println("STATUS: motor is now running");
  }
  else Serial.println("ERROR: limit has been reached, cannot resume without restarting");
}

/*  'halt' 
 *  
 *  Immediately freezes the motor in place without finishing the revolution.
*/
void freezeMotor() {
  if (isHalted) Serial.println("ERROR: motor is already frozen");

  else {
    isHalted = true;
    runMode = false;
    tempSpeed = runningSpeed;
    runningSpeed = 0;
    stepper.stop();
    Serial.println("STATUS: motor is now frozen");
  }
}

/*  'pause' or 'stop'
 *  
 *  Finishes the current revolution, then halts the motor at the origin.
 *  If [reset] is true, then also resets the revolution counter.
*/
void stopMotor(bool reset) {
  if (!runMode && !isHalted) Serial.println("ERROR: motor is already paused/stopped");
  
  else { 
    if (isHalted) startMotor();           // finish current revolution
    
    runMode = false;
  
    if (!reset) Serial.println("STATUS: motor is now pausing...");
    else {
      revNum = 0;
      Serial.println("STATUS: motor is now stopped. Counter has been reset."); 
    }
  }
}

/*  'restart'
 *  
 *  Same behavior as 'stop', but also starts the motor up again afterwards. 
*/
void restartMotor() {
  if (isHalted) startMotor();           // finish current revolution
  
  revNum = 0;
  runMode = true;
  Serial.println("STATUS: motor is now restarting...");
  delay(1500);
}
