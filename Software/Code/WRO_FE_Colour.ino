#include <Evo.h>
#include <SoftwareSerial.h>

// hardware setup
EVOX1 evo;
EvoBNO055 bno(I2C2);
EvoMotor driveMotor(M1, EV3MediumMotor, true);
EvoMotor steerMotor(M2, EV3MediumMotor, true);
EvoVL53L0X leftSensor(I2C3);
EvoVL53L0X rightSensor(I2C4);
EvoHuskyLens hl(I2C1);  // huskylens variable
HUSKYLENSResult result;  // huskylens result

// constants
const int STEER_SPEED = 4000;
const int DRIVE_SPEED = -1200;
const int THRESHOLD = 1800;  // threshold before considered gap
const int GAP_THRESHOLD = 1750;  // threshold for gaps
const int MAX_STEER_ANGLE = 65;
const float KP = 5.0f;  //increased for agressive heading tracking
const int POST_TURN_DISTANCE = 1000;
const unsigned long MIN_TURN_INTERVAL = 1500;
const int TOTAL_TURNS = 12;  // 3 complete squares
const int MIN_TRAVEL_DISTANCE = 100; // minimum distance travelled before returning
const int ACCEPTABLE_INCREASE = 600; 

// color detection constants
const int RED_ID = 2;    // red is ID 2 
const int GREEN_ID = 1;  // green is ID 1 
const int MIN_COLOR_SIZE = 100;  // minimum object size to consider

// averaging system constants
const int BUFFER_SIZE = 10;  // 10ms interval between averages
const int READINGS_PER_AVERAGE = 5;  // average of 5 readings

// state variables
float heading = 0;
float targetHeading = 0;
int phase = 0;  // 0=straight, 1=turning, 2=forward after turn, 3=color turn, 4=color return, 5=color back
int turnsCompleted = 0;
long forwardStartPos = 0;
unsigned long lastTurnTime = 0;

// color detection state variables
bool colorOverrideActive = false;
unsigned long colorOverrideStartTime = 0;
int colorOverrideDirection = 0;  // -1 for left (green), +1 for right (red)
bool colorTurnCompleted = false;
int lastDetectedColorID = 0;

// distance tracking variables for color avoidance
long outwardStartPos = 0;  // motor position when starting outward travel
long outwardDistance = 0;  // distance traveled away
long returnStartPos = 0;   // motor position when starting return travel

// sensor averaging buffers
struct SensorBuffer {
  int leftReadings[BUFFER_SIZE];
  int rightReadings[BUFFER_SIZE];
  int index;
  int count;
  unsigned long timestamps[BUFFER_SIZE];
  
  SensorBuffer() : index(0), count(0) {}
  
  void addReading(int left, int right) {
    leftReadings[index] = left;
    rightReadings[index] = right;
    timestamps[index] = millis();
    index = (index + 1) % BUFFER_SIZE;
    if (count < BUFFER_SIZE) count++;
  }
  
  // get average of readings from last second
  void getOneSecondAverage(int& leftAvg, int& rightAvg) {
    unsigned long currentTime = millis();
    unsigned long oneSecondAgo = currentTime - 1000;
    
    long leftSum = 0, rightSum = 0;
    int validCount = 0;
    
    // sum readings from the last 1 second
    for (int i = 0; i < count; i++) {
      int bufferIndex = (index - 1 - i + BUFFER_SIZE) % BUFFER_SIZE;
      if (timestamps[bufferIndex] >= oneSecondAgo) {
        leftSum += leftReadings[bufferIndex];
        rightSum += rightReadings[bufferIndex];
        validCount++;
      } else {
        break;  // older readings stop here
      }
    }
    
    if (validCount > 0) {
      leftAvg = leftSum / validCount;
      rightAvg = rightSum / validCount;
    } else {
      // fallback to most recent reading
      int recentIndex = (index - 1 + BUFFER_SIZE) % BUFFER_SIZE;
      leftAvg = leftReadings[recentIndex];
      rightAvg = rightReadings[recentIndex];
    }
  }
  
  // get smoothed reading 
  void getCurrentSmoothed(int& leftSmooth, int& rightSmooth) {
    int samplesToAverage = min(READINGS_PER_AVERAGE, count);
    if (samplesToAverage == 0) {
      leftSmooth = 1000;
      rightSmooth = 1000;
      return;
    }
    
    long leftSum = 0, rightSum = 0;
    for (int i = 0; i < samplesToAverage; i++) {
      int bufferIndex = (index - 1 - i + BUFFER_SIZE) % BUFFER_SIZE;
      leftSum += leftReadings[bufferIndex];
      rightSum += rightReadings[bufferIndex];
    }
    
    leftSmooth = leftSum / samplesToAverage;
    rightSmooth = rightSum / samplesToAverage;
  }
};

SensorBuffer sensorBuffer;

// utility functions
float normalize(float angle) {
  while (angle < 0) angle += 360;
  while (angle >= 360) angle -= 360;
  return angle;
}

float headingError(float current, float target) {
  float error = target - current;
  if (error > 180) error -= 360;
  if (error < -180) error += 360;
  return error;
}

// improved sensor reading with outlier rejection
int cleanSensorReading(int rawReading, int lastGoodReading) {
  // handle sensor errors and timeouts
  if (rawReading <= 0) return lastGoodReading;
  if (rawReading >= 8000) return 8000;
  
  // reject extreme outliers (noise)
  if (lastGoodReading > 0) {
    int difference = abs(rawReading - lastGoodReading);
    if (difference > 1500 && rawReading < 100) {
      // likely noise spike, use last accurate reading
      return lastGoodReading;
    }
  }
  
  return rawReading;
}

// color detection function
void checkColorDetection() {
  // only check colors at straight phase
  if (phase != 0) return;
  
  //get color detection from huskylens
  bool colorDetected = false;
  if (hl.requestBlocks(result)) {
    //check if detected object big enough
    int objectSize = result.width * result.height;
    
    if (objectSize >= MIN_COLOR_SIZE) {
      if (result.ID == RED_ID || result.ID == GREEN_ID) {
        //detect colors below y level 160 (above this value is too far)
        if (result.yCenter <= 160) {
          colorDetected = true;
          lastDetectedColorID = result.ID;
        }
      }
    }
  }
  
  //if currently in a color override and no longer see the color
  if (colorOverrideActive && !colorDetected) {
    //check if we've traveled minimum distance
    long currentPos = driveMotor.getAngle();
    outwardDistance = abs(currentPos - outwardStartPos);
    
    if (outwardDistance >= MIN_TRAVEL_DISTANCE) {
      colorOverrideActive = false;
      
      //set target to -45° (opposite direction) to return
      if (colorOverrideDirection == 1) {
        //was turning right, now turn left to return
        targetHeading = normalize(targetHeading - 90);  //from +45° to -45°
      } else {
        // was turning left, now turn right to return
        targetHeading = normalize(targetHeading + 90);  //from -45° to +45°
      }
      
      returnStartPos = currentPos;  //start of return journey
      phase = 4;  //color return phase
    }
    return;
  }
  
  //if detect a new color and not alr in color mode
  if (colorDetected && !colorOverrideActive) {
    colorOverrideActive = true;
    colorOverrideStartTime = millis();
    outwardStartPos = driveMotor.getAngle();  //start of outward travel
    
    if (lastDetectedColorID == RED_ID) {
      //red detected - turn right 45°
      colorOverrideDirection = 1;   //right turn
      targetHeading = normalize(targetHeading + 45);
    } else if (lastDetectedColorID == GREEN_ID) {
      //green detected - turn left 45°
      colorOverrideDirection = -1;  //left turn
      targetHeading = normalize(targetHeading - 45);
    }
    
    phase = 3;  // color turn phase
    lastTurnTime = millis();  //update turn time to prevent gap detection interference
  }
}

void setup() {
  evo.begin();
  bno.begin();
  driveMotor.begin();
  steerMotor.begin();
  leftSensor.begin();
  rightSensor.begin();
  hl.begin();  // initialize huskylens
  
  steerMotor.flipEncoderDirection(true);
  steerMotor.resetAngle();
  driveMotor.resetAngle();
  
  //setup huskylens for color recognition
  hl.setMode(ALGORITHM_COLOR_RECOGNITION);
  
  evo.writeLineToDisplay("3 Rounds + Color", 0, true, true);
  evo.writeLineToDisplay("Enhanced Detection", 1, true, false);
  delay(1000);
  evo.waitForBump();
  
  targetHeading = 0;
  phase = 0;
  lastTurnTime = 0;
  turnsCompleted = 0;
  colorOverrideActive = false;
  colorTurnCompleted = false;
  lastDetectedColorID = 0;
  outwardStartPos = 0;
  outwardDistance = 0;
  returnStartPos = 0;
}

void loop() {
  //check for color detection first
  checkColorDetection();
  
  // read sensors with improved error handling
  static int lastLeftReading = 1000;
  static int lastRightReading = 1000;
  
  int rawLeft = leftSensor.getDistance();
  int rawRight = rightSensor.getDistance();
  
  int leftDist = cleanSensorReading(rawLeft, lastLeftReading);
  int rightDist = cleanSensorReading(rawRight, lastRightReading);
  
  lastLeftReading = leftDist;
  lastRightReading = rightDist;
  
  // add readings to buffer
  sensorBuffer.addReading(leftDist, rightDist);
  
  //get smoothed current readings and 1-second averages
  int leftSmooth, rightSmooth;
  int leftAvg, rightAvg;
  sensorBuffer.getCurrentSmoothed(leftSmooth, rightSmooth);
  sensorBuffer.getOneSecondAverage(leftAvg, rightAvg);
  
  //get heading with basic error protection
  float x, y, z;
  static unsigned long lastValidReading = 0;
  static float lastValidHeading = 0;
  
  bno.getEuler(&x, &y, &z);
  
  // check if reading seems valid (not NaN or extreme values)
  if (!isnan(x) && x >= 0 && x < 360) {
    heading = normalize(x);
    lastValidHeading = heading;
    lastValidReading = millis();
  } else {
    //use last valid heading if sensor gives invalid data
    heading = lastValidHeading;
    // if sensor has been failing for too long, reset to 0
    if (millis() - lastValidReading > 2000) {
      heading = 0;
      lastValidHeading = 0;
      lastValidReading = millis();
    }
  }
  
  float error = headingError(heading, targetHeading);

  // add protection - reset if stuck in one phase too long
  static unsigned long phaseStartTime = 0;
  static int lastPhase = -1;
  
  if (phase != lastPhase) {
    phaseStartTime = millis();
    lastPhase = phase;
  }
  
  //if stuck in any phase for too long, reset to straight
  if ((phase == 1 || phase == 2 || phase == 3 || phase == 4 || phase == 5) && (millis() - phaseStartTime > 30000)) {
    phase = 0;  // back to straight mode
    targetHeading = heading;  // accept current heading as target
    colorOverrideActive = false;  // clear color override
  }
  
  // state machine
  if (phase == 0) {
    //straight phase (not michael)
    // simple steering based on heading error
    float steerAngle = -error * KP;
    steerAngle = constrain(steerAngle, -MAX_STEER_ANGLE, MAX_STEER_ANGLE);
    steerMotor.runTarget(STEER_SPEED, (int)steerAngle, MotorStop::HOLD, false);
    
    // drive forward
    driveMotor.run(DRIVE_SPEED);
    
    // gap detection (only if not overridden by color)
    if (!colorOverrideActive) {
      // enhanced gap detection using both current smoothed and averaged readings
      bool leftGapCurrent = leftSmooth > THRESHOLD;
      bool rightGapCurrent = rightSmooth > THRESHOLD;
      bool leftGapAverage = leftAvg > GAP_THRESHOLD;
      bool rightGapAverage = rightAvg > GAP_THRESHOLD;
      
      // early detection: trigger on current readings, confirm with averages
      bool leftGap = leftGapCurrent || leftGapAverage;
      bool rightGap = rightGapCurrent || rightGapAverage;
      
      // additional early detection: look for increasing distance trend
      bool leftIncreasing = (leftSmooth > lastLeftReading + ACCEPTABLE_INCREASE);
      bool rightIncreasing = (rightSmooth > lastRightReading + ACCEPTABLE_INCREASE);
      
      bool timeOK = (millis() - lastTurnTime) > MIN_TURN_INTERVAL;
      
      // turn if gap detected (with trend consideration) and enough time has passed
      if (((leftGap || leftIncreasing) || (rightGap || rightIncreasing)) && timeOK) {
        
        // use 1-second averages for turn direction decision
        if ((leftAvg > GAP_THRESHOLD || leftIncreasing) && !(rightAvg > GAP_THRESHOLD)) {
          targetHeading = normalize(targetHeading - 90);  // turn left toward left gap
        } else if ((rightAvg > GAP_THRESHOLD || rightIncreasing) && !(leftAvg > GAP_THRESHOLD)) {
          targetHeading = normalize(targetHeading + 90);  // turn right toward right gap
        } else {
          // both gaps or both increasing - turn toward larger average gap
          if (leftAvg > rightAvg) {
            targetHeading = normalize(targetHeading - 90);  //turn left
          } else {
            targetHeading = normalize(targetHeading + 90);  // turn right
          }
        }
        
        phase = 1;  // switch to turning
        lastTurnTime = millis();  // record turn time
      }
    }
    
  } else if (phase == 1) {
    //turning phase (90° turns for square)
    // turn gently
    float steerAngle = -error * KP;
    steerAngle = constrain(steerAngle, -MAX_STEER_ANGLE, MAX_STEER_ANGLE);
    steerMotor.runTarget(STEER_SPEED, (int)steerAngle, MotorStop::HOLD, false);
    
    // keep driving while turning (slower)
    driveMotor.run(DRIVE_SPEED);
    
    // check if turn is complete
    if (abs(error) < 8) {  //buffer
      turnsCompleted++;
      
      // straighten wheels - non-blocking
      steerMotor.runTarget(STEER_SPEED, 0, MotorStop::HOLD, false);
      
      // check if 3 squares done
      if (turnsCompleted >= TOTAL_TURNS) {
        driveMotor.coast();
        steerMotor.runTarget(STEER_SPEED, 0, MotorStop::HOLD, false);
        evo.clearDisplay();
        evo.writeLineToDisplay("3 ROUNDS DONE!", 0, true, true);
        
        // non-blocking end - just stop motors and continue display updates
        while(true) {
          driveMotor.coast();
          steerMotor.coast();
          delay(100);
        }
      }
      
      //start forward movement to clear the gap area
      forwardStartPos = driveMotor.getAngle();
      phase = 2;
    }
    
  } else if (phase == 2) {
    //forward after gap
    long currentPos = driveMotor.getAngle();
    long distanceTraveled = abs(currentPos - forwardStartPos);
    
    // simple heading hold
    float steerAngle = -error * KP;
    steerAngle = constrain(steerAngle, -MAX_STEER_ANGLE, MAX_STEER_ANGLE);
    steerMotor.runTarget(STEER_SPEED, (int)steerAngle, MotorStop::HOLD, false);
    
    // drive forward
    driveMotor.run(DRIVE_SPEED);
    
    // check if far enough
    if (distanceTraveled >= POST_TURN_DISTANCE) {
      phase = 0;  // back to straight
    }
    
  } else if (phase == 3) {
    // color turn (45° turn)
    // turn gently to 45° angle
    float steerAngle = -error * KP;
    steerAngle = constrain(steerAngle, -MAX_STEER_ANGLE, MAX_STEER_ANGLE);
    steerMotor.runTarget(STEER_SPEED, (int)steerAngle, MotorStop::HOLD, false);
    
    //keep driving while turning
    driveMotor.run(DRIVE_SPEED);
    
    //check if 45° turn is complete
    if (abs(error) < 8) {
      // 45° turn completed, continue checking for color while driving at angle
      phase = 0;  // return to straight mode but maintain 45° heading
    }
    
  } else if (phase == 4) {
    // colour return (turning to -45° to return)
    // turn opposite 45° angle to return
    float steerAngle = -error * KP;
    steerAngle = constrain(steerAngle, -MAX_STEER_ANGLE, MAX_STEER_ANGLE);
    steerMotor.runTarget(STEER_SPEED, (int)steerAngle, MotorStop::HOLD, false);
    
    // keep driving while turning to return angle
    driveMotor.run(DRIVE_SPEED);
    
    // check if return turn is complete
    if (abs(error) < 8) {
      // turn completed, now travel back the same distance
      phase = 5;  // switch to return travel phase
    }
    
  } else if (phase == 5) {
    // return to straight (traveling back the outward distance)
    // drive straight
    float steerAngle = -error * KP;
    steerAngle = constrain(steerAngle, -MAX_STEER_ANGLE, MAX_STEER_ANGLE);
    steerMotor.runTarget(STEER_SPEED, (int)steerAngle, MotorStop::HOLD, false);
    
    // drive forward
    driveMotor.run(DRIVE_SPEED);
    
    // check if we've traveled back the same distance we went out
    long currentPos = driveMotor.getAngle();
    long returnDistance = abs(currentPos - returnStartPos);
    
    if (returnDistance >= outwardDistance) {
      // traveled back the same distance, now return to straight (0°)
      targetHeading = 0;
      phase = 0;  // return directly to straight mode
      colorOverrideDirection = 0;
    }
  }
  
  //better display with color detection info
  evo.clearDisplay();
  evo.writeToDisplay("L:", 0, 0);   evo.writeToDisplay(leftSmooth, 15, 0);
  evo.writeToDisplay("/", 35, 0);   evo.writeToDisplay(leftAvg, 42, 0);
  evo.writeToDisplay("R:", 65, 0);  evo.writeToDisplay(rightSmooth, 80, 0);
  evo.writeToDisplay("/", 100, 0);  evo.writeToDisplay(rightAvg, 107, 0);
  evo.writeToDisplay("H:", 0, 16);  evo.writeToDisplay((int)heading, 20, 16);
  evo.writeToDisplay("T:", 60, 16); evo.writeToDisplay((int)targetHeading, 80, 16);
  
  //show turns completed or distance info for color phases
  if (phase == 3 && colorOverrideActive) {
    //show outward distance
    long currentPos = driveMotor.getAngle();
    long currentOutward = abs(currentPos - outwardStartPos);
    evo.writeToDisplay("Out:", 0, 32);  evo.writeToDisplay((int)currentOutward, 30, 32);
    evo.writeToDisplay("/", 60, 32);    evo.writeToDisplay(MIN_TRAVEL_DISTANCE, 65, 32);
  } else if (phase == 5) {
    //show return distance vs target
    long currentPos = driveMotor.getAngle();
    long currentReturn = abs(currentPos - returnStartPos);
    evo.writeToDisplay("Ret:", 0, 32);  evo.writeToDisplay((int)currentReturn, 30, 32);
    evo.writeToDisplay("/", 60, 32);    evo.writeToDisplay((int)outwardDistance, 65, 32);
  } else {
    evo.writeToDisplay("Turns:", 0, 32);  evo.writeToDisplay(turnsCompleted, 50, 32);
    evo.writeToDisplay("/12", 70, 32);
  }
  
  // show phase and color status
  evo.writeToDisplay("P:", 0, 48);  
  if (phase == 0) evo.writeToDisplay("STR", 20, 48);
  else if (phase == 1) evo.writeToDisplay("TRN", 20, 48);
  else if (phase == 2) evo.writeToDisplay("FWD", 20, 48);
  else if (phase == 3) evo.writeToDisplay("C45", 20, 48);
  else if (phase == 4) evo.writeToDisplay("RTN", 20, 48);
  else if (phase == 5) evo.writeToDisplay("BCK", 20, 48);
  
  //show color override status
  if (colorOverrideActive) {
    evo.writeToDisplay("C:", 50, 48);
    if (colorOverrideDirection == -1) evo.writeToDisplay("GRN", 65, 48);
    else evo.writeToDisplay("RED", 65, 48);
  }
  
  evo.drawDisplay();
  
  delay(10);
}