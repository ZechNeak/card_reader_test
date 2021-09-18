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
 *              pos              - Shows the current microstep position relative to origin.
 *              
 *              maxPulses [int]  - Sets the number of pulses per full revolution of the motor.
 *                                 This must match the microstep setting of the accompanying stepper driver.
 *                                 If no arg is specified, prints the current number instead.
 *
 *              limit [int]      - Sets the max number of full revolutions the motor will spin.
 *                                 If no arg is specified, prints the current number instead.
 *
 *              maxSpeed [int]   - Sets the max speed of the motor in microsteps/sec.
 *                                 If no arg is specified, prints the current max speed instead.
 *                             
 *              speed [int]      - Sets the current speed of the motor in microsteps/sec.
 *                                 If no arg is specified, prints the current speed instead.
 *                             
 *              origin           - Sets the origin (i.e. microstep position 0) of the motor. 
 *                                 For consistency, this must be done before any sequence is run.
 *                             
 *              move [int]       - If the motor is not in runmode, moves the motor by the specified
 *                                 amount of microsteps. 
 *                                 A negative value will cause the motor to spin backwards.
 *                                 
 *              goto [int]       - If the motor is not in runmode, moves the motor to the specified
 *                                 microstep position, relative to the origin.
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

// Serial print statements stored in flash memory instead of SRAM
const char intro_origin[] PROGMEM = "<Set current position as origin>\n";
const char intro_maxspeed[] PROGMEM = "<Set max speed to 1600 microsteps/sec>\n";
const char intro_speed[] PROGMEM = "<Set running speed for all cards to 800 microsteps/sec>\n";
const char rev_count1[] PROGMEM = "<Rev ";
const char rev_count2[] PROGMEM = ">\n";
const char revs_done1[] PROGMEM = "STATUS: ";
const char revs_done2[] PROGMEM = " revolutions done. Stopping motor...";
const char serial_warning[] PROGMEM = "<Warning: Index has exceeded string length>\n";
const char error_invalid[] PROGMEM = "ERROR: Please input a valid command.\n";
const char pos_get1[] PROGMEM = "GET: Current motor position is ";
const char pos_get2[] PROGMEM = " microsteps from origin";
const char maxpulses_get1[] PROGMEM = "GET: Each full revolution requires ";
const char limit_get1[] PROGMEM = "GET: Motor will run up to ";
const char maxspeed_get1[] PROGMEM = "GET: Max motor speed is ";
const char speed_get1[] PROGMEM = "GET: Current motor speed is ";
const char maxpulses_set1[] PROGMEM = "GET: Each full revolution now requires ";
const char limit_set1[] PROGMEM = "UPDATE: motor will now spin up to ";
const char autoadjust_speed[] PROGMEM = "UPDATE: adjusting running speed to match new max speed\n";
const char maxspeed_set1[] PROGMEM = "UPDATE: max motor speed is now ";
const char throttle_speed1[] PROGMEM = "UPDATE: requested speed is higher than maxSpeed of ";
const char speed_set1[] PROGMEM = "UPDATE: motor running speed is now ";
const char origin_set[] PROGMEM = "UPDATE: origin has been reset to current position\n";
const char error_running_a[] PROGMEM = "ERROR: motor must first not be running\n";
const char error_running_b[] PROGMEM = "ERROR: motor is already running\n";
const char error_bounds[] PROGMEM = "ERROR: target position must be between 0 and ";
const char displace_by2[] PROGMEM = " microsteps";
const char displace_to2[] PROGMEM = " microsteps from origin";
const char status_running[] PROGMEM = "STATUS: motor is now running\n";
const char error_limit[] PROGMEM = "ERROR: limit has been reached, cannot resume without restarting\n";
const char error_frozen[] PROGMEM = "ERROR: motor is already frozen\n";
const char status_frozen[] PROGMEM = "STATUS: motor is now frozen\n";
const char error_stopped[] PROGMEM = "ERROR: motor is already paused/stopped\n";
const char status_pausing[] PROGMEM = "STATUS: motor is now pausing...\n";
const char status_stopped[] PROGMEM = "STATUS: motor is now stopped. Counter has been reset.\n";
const char status_restarting[] PROGMEM = "STATUS: motor is now restarting...\n";
const char pulse_count[] PROGMEM = " pulses, or microsteps";
const char rev_amount[] PROGMEM = " revolutions";
const char steps_per_sec[] PROGMEM = " microsteps/s";
const char displace_msg[] PROGMEM = "DISPLACE: ";

AccelStepper stepper = AccelStepper(motorInterfaceType, stepPin, dirPin);

// Depends on stepper motor model and configuration (Default: 1600 for NEMA 23)
int pulsesPerRev = 1600;

// For counting number of motor revolutions
int revLimit = 10;
int revNum = 0;

// For reading Serial string data
const byte numChars = 32;
char charData[numChars] = {};
bool commandReady = false;

// Affects and tracks control of motor
int runningSpeed = 0;
int tempSpeed = 0;                        // previous or new speed val to be used after a halt
bool runMode = false;
bool isHalted = false;
 

/* ==================================================================================================== */

void setup() {
  Serial.begin(9600);

  // Sets initial position to 0. Also implicitly sets current speed to 0.
  // (Note: Please manually ensure that no card is over the reader before starting the Arduino.)
  printFromFlash(intro_origin);
  stepper.setCurrentPosition(0);

  printFromFlash(intro_maxspeed);
  stepper.setMaxSpeed(1600);

  printFromFlash(intro_speed);
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

  printFromFlashAndMore(rev_count1, revNum, rev_count2);
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
    printFromFlashAndMore(revs_done1, revNum, revs_done2);
    runMode = false;
    revNum = 0;
    delay(1000);
  }
  delay(800);
}


/* -------------------------------------------------------------------
 * Reads Serial print statements stored in flash memory via PROGMEM,
 * and variable values if provided
 * ------------------------------------------------------------------- */
void printFromFlashAndMore(const char* statement1, int val, const char* statement2) {
  printFromFlash(statement1);
  Serial.print(String(val));
  printFromFlash(statement2);
  Serial.println();
}
 
void printFromFlash(const char* statement) {
  char currChar;
  
  for (byte i = 0; i < strlen_P(statement); i++) {
    currChar = pgm_read_byte_near(statement + i);
    Serial.print(currChar);
  }
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
        printFromFlash(serial_warning);
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

    else if (strcmp(commandMessage, "maxPulses") == 0 && token == NULL) showPulsesPerRev();
    
    else if (strcmp(commandMessage, "limit") == 0 && token == NULL) showRevLimit();
    
    else if (strcmp(commandMessage, "maxSpeed") == 0  && token == NULL) showMaxSpeed();
    
    else if (strcmp(commandMessage, "speed") == 0  && token == NULL) showCurrentSpeed();
// -----------------------------------------------------------------------------------------------------

    else if (strcmp(commandMessage, "maxPulses") == 0) updatePulsesPerRev(commandArg);
    
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

    else printFromFlash(error_invalid);
    
    commandReady = false;
  }
}


/* ----------------------------------------------------------------------------------
 * Functions to be called by Serial commands, either via Serial Monitor or a script
 * ---------------------------------------------------------------------------------- */
 
 /* 'pos' */
void showCurrentPosition() {
  printFromFlashAndMore(pos_get1, stepper.currentPosition(), pos_get2);
}

 /* 'maxPulses' */
void showPulsesPerRev() {
  printFromFlashAndMore(maxpulses_get1, pulsesPerRev, pulse_count);
}

/* 'limit' */
void showRevLimit() {
  printFromFlashAndMore(limit_get1, revLimit, rev_amount);
}

/* 'maxSpeed' */
void showMaxSpeed() {
  printFromFlashAndMore(maxspeed_get1, int(stepper.maxSpeed()), steps_per_sec);
}

/* 'speed' */
void showCurrentSpeed() {
  printFromFlashAndMore(speed_get1, runningSpeed, steps_per_sec);
}
// -----------------------------------------------------------------------------------------------------

/* 'maxPulses [# pulses per revolution]' */
void updatePulsesPerRev(int newConfig) {
  pulsesPerRev = newConfig;
  printFromFlashAndMore(maxpulses_set1, pulsesPerRev, pulse_count);
}

/* 'limit [max # revolutions]' */
void updateRevLimit(int newLimit) {
  revLimit = newLimit;
  printFromFlashAndMore(limit_set1, revLimit, rev_amount);
}

/* 'maxSpeed [microsteps per sec]' */
void updateMaxSpeed(int newMaxSpeed) {
  if (newMaxSpeed < runningSpeed) {
    printFromFlash(autoadjust_speed);
    runningSpeed = newMaxSpeed;
  }
  
  stepper.setMaxSpeed(newMaxSpeed);
  printFromFlashAndMore(maxspeed_set1, int(stepper.maxSpeed()), steps_per_sec);
}

/* 'speed [microsteps per sec]' */
void updateSpeed(int newSpeed) {
  if (newSpeed == 0) freezeMotor();
  
  else if (newSpeed > stepper.maxSpeed()) {
    printFromFlashAndMore(throttle_speed1, int(stepper.maxSpeed()), steps_per_sec);
    newSpeed = int(stepper.maxSpeed());
    runningSpeed = newSpeed;
  }
  else if (isHalted) tempSpeed = newSpeed;
  else runningSpeed = newSpeed;
  
  printFromFlashAndMore(speed_set1, newSpeed, steps_per_sec);
}

/* 'origin' */
void updateOrigin() {
  stepper.setCurrentPosition(0);
  printFromFlash(origin_set);
}
// -----------------------------------------------------------------------------------------------------

/*  'move [# microsteps]' or 'goto [microsteps from origin]' 
 *  
 *  If 'isAbsolute' is false, then displace the motor by the specified # of microsteps from 
 *  the current position.
 *  
 *  Otherwise, displace the motor towards the position that is at that # of microsteps from
 *  the set origin.
*/
void displaceTo(bool isAbsolute, int steps) {
  if (runMode) {
    printFromFlash(error_running_a);
  }
  else if (isAbsolute && (0 > steps || steps > pulsesPerRev)) {
    printFromFlashAndMore(error_bounds, pulsesPerRev, "");
  }
  else {
    if (isHalted) runningSpeed = tempSpeed;

    // Set new target position, whether relative to current position or origin
    if (!isAbsolute) {
      printFromFlashAndMore(displace_msg, steps, displace_by2);
      stepper.move(steps);
    }
    else {
      printFromFlashAndMore(displace_msg, steps, displace_to2);
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
  if (runMode && runningSpeed != 0) printFromFlash(error_running_b);
  
  else if (revNum != revLimit) {

    if (isHalted) {
      runningSpeed = tempSpeed;
      isHalted = false;
    }   
    runMode = true;
    printFromFlash(status_running);
  }
  else printFromFlash(error_limit);
}

/*  'halt' 
 *  
 *  Immediately freezes the motor in place without finishing the revolution.
*/
void freezeMotor() {
  if (isHalted) printFromFlash(error_frozen);

  else {
    isHalted = true;
    runMode = false;
    tempSpeed = runningSpeed;
    runningSpeed = 0;
    stepper.stop();
    printFromFlash(status_frozen);
  }
}

/*  'pause' or 'stop'
 *  
 *  Finishes the current revolution, then halts the motor at the origin.
 *  If [reset] is true, then also resets the revolution counter.
*/
void stopMotor(bool reset) {
  if (!runMode && !isHalted) printFromFlash(error_stopped);
  
  else { 
    if (isHalted) startMotor();           // finish current revolution
    
    runMode = false;
  
    if (!reset) printFromFlash(status_pausing);
    else {
      revNum = 0;
      printFromFlash(status_stopped);
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
  printFromFlash(status_restarting);
  delay(1500);
}
