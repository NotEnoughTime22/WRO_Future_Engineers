#include <Evo.h>
#include <SoftwareSerial.h>

// hardware setup
EVOX1 evo;
EvoBNO055 bno(I2C2);
EvoMotor driveMotor(M1, EV3MediumMotor, true);
EvoMotor steerMotor(M2, EV3MediumMotor, true);
EvoVL53L0X leftSensor(I2C3);
EvoVL53L0X rightSensor(I2C4);

//constants
const int STEER_SPEED = 4000;
const int DRIVE_SPEED = -2000;
const int THRESHOLD = 700;  //reduced 
const int GAP_THRESHOLD = 900;  //additional threshold as backup
const int MAX_STEER_ANGLE = 65;
const float KP = 3.0f;
const int POST_TURN_DISTANCE = 1500;
const unsigned long MIN_TURN_INTERVAL = 1500;
const int TOTAL_TURNS = 12;  //3 squares

// averaging system constants
const int BUFFER_SIZE = 100;  // store readings at 10ms intervals
const int READINGS_PER_AVERAGE = 10;  //average 10 readings for smoothing

//state variables
float heading = 0;
float targetHeading = 0;
int phase = 0;  //0=straight, 1=turning, 2=forward after turn
int turnsCompleted = 0;
long forwardStartPos = 0;
unsigned long lastTurnTime = 0;

//sensor averaging buffers
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
  
  //get average of readings from the last 1 second
  void getOneSecondAverage(int& leftAvg, int& rightAvg) {
    unsigned long currentTime = millis();
    unsigned long oneSecondAgo = currentTime - 1000;
    
    long leftSum = 0, rightSum = 0;
    int validCount = 0;
    
    //sum readings from the last 1 second
    for (int i = 0; i < count; i++) {
      int bufferIndex = (index - 1 - i + BUFFER_SIZE) % BUFFER_SIZE;
      if (timestamps[bufferIndex] >= oneSecondAgo) {
        leftSum += leftReadings[bufferIndex];
        rightSum += rightReadings[bufferIndex];
        validCount++;
      } else {
        break;  //older readings
      }
    }
    
    if (validCount > 0) {
      leftAvg = leftSum / validCount;
      rightAvg = rightSum / validCount;
    } else {
      //fallback to most recent reading
      int recentIndex = (index - 1 + BUFFER_SIZE) % BUFFER_SIZE;
      leftAvg = leftReadings[recentIndex];
      rightAvg = rightReadings[recentIndex];
    }
  }
  
  //get current smoothed reading
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

//utility functions
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

//improved sensor reading with outlier rejection
int cleanSensorReading(int rawReading, int lastGoodReading) {
  // Handle sensor errors and timeouts
  if (rawReading <= 0) return lastGoodReading;
  if (rawReading >= 8000) return 2000;
  
  //reject extreme outliers (likely noise)
  if (lastGoodReading > 0) {
    int difference = abs(rawReading - lastGoodReading);
    if (difference > 1500 && rawReading < 100) {
      //noise spike, use last good reading
      return lastGoodReading;
    }
  }
  
  return rawReading;
}

void setup() {
  evo.begin();
  bno.begin();
  driveMotor.begin();
  steerMotor.begin();
  leftSensor.begin();
  rightSensor.begin();
  
  steerMotor.flipEncoderDirection(true);
  steerMotor.resetAngle();
  driveMotor.resetAngle();
  
  evo.writeLineToDisplay("3 Rounds Square", 0, true, true);
  evo.writeLineToDisplay("Enhanced Detection", 1, true, false);
  delay(1000);
  evo.waitForBump();
  
  targetHeading = 0;
  phase = 0;
  lastTurnTime = 0;
  turnsCompleted = 0;
}

void loop() {
  //improved error handling
  static int lastLeftReading = 1000;
  static int lastRightReading = 1000;
  
  int rawLeft = leftSensor.getDistance();
  int rawRight = rightSensor.getDistance();
  
  int leftDist = cleanSensorReading(rawLeft, lastLeftReading);
  int rightDist = cleanSensorReading(rawRight, lastRightReading);
  
  lastLeftReading = leftDist;
  lastRightReading = rightDist;
  
  //add readings to buffer
  sensorBuffer.addReading(leftDist, rightDist);
  
  //get smoothed readings
  int leftSmooth, rightSmooth;
  int leftAvg, rightAvg;
  sensorBuffer.getCurrentSmoothed(leftSmooth, rightSmooth);
  sensorBuffer.getOneSecondAverage(leftAvg, rightAvg);
  
  //get heading with basic error protection
  float x, y, z;
  static unsigned long lastValidReading = 0;
  static float lastValidHeading = 0;
  
  bno.getEuler(&x, &y, &z);
  
  //check if reading valid (not extreme values)
  if (!isnan(x) && x >= 0 && x < 360) {
    heading = normalize(x);
    lastValidHeading = heading;
    lastValidReading = millis();
  } else {
    //use last valid heading if sensor gives invalid data
    heading = lastValidHeading;
    //if sensor has been failing for too long, reset to 0
    if (millis() - lastValidReading > 2000) {
      heading = 0;
      lastValidHeading = 0;
      lastValidReading = millis();
    }
  }
  
  float error = headingError(heading, targetHeading);

  //reset if stuck in one phase too long
  static unsigned long phaseStartTime = 0;
  static int lastPhase = -1;
  
  if (phase != lastPhase) {
    phaseStartTime = millis();
    lastPhase = phase;
  }
  
  //if stuck in turning or forward phase for too long, reset to straight
  if ((phase == 1 || phase == 2) && (millis() - phaseStartTime > 10000)) {
    phase = 0;  // Force back to straight mode
    targetHeading = heading;  // Accept current heading as target
  }
  
  //state machine
  if (phase == 0) {
    //straight phase
    //simple steering based on heading error
    float steerAngle = -error * KP;
    steerAngle = constrain(steerAngle, -MAX_STEER_ANGLE, MAX_STEER_ANGLE);
    steerMotor.runTarget(STEER_SPEED, (int)steerAngle, MotorStop::HOLD, false);
    
    //drive forward
    driveMotor.run(DRIVE_SPEED);
    
    // enhanced gap detection using both current smoothed and averaged readings
    bool leftGapCurrent = leftSmooth > THRESHOLD;
    bool rightGapCurrent = rightSmooth > THRESHOLD;
    bool leftGapAverage = leftAvg > GAP_THRESHOLD;
    bool rightGapAverage = rightAvg > GAP_THRESHOLD;
    
    //early detection: trigger on current readings, confirm with averages
    bool leftGap = leftGapCurrent || leftGapAverage;
    bool rightGap = rightGapCurrent || rightGapAverage;
    
    //additional early detection: look for increasing distance trend
    bool leftIncreasing = (leftSmooth > lastLeftReading + 100);
    bool rightIncreasing = (rightSmooth > lastRightReading + 100);
    
    bool timeOK = (millis() - lastTurnTime) > MIN_TURN_INTERVAL;
    
    //turn if gap detected and time ok
    if (((leftGap || leftIncreasing) || (rightGap || rightIncreasing)) && timeOK) {
      
      //use 1-second averages for turn direction decision
      if ((leftAvg > GAP_THRESHOLD || leftIncreasing) && !(rightAvg > GAP_THRESHOLD)) {
        targetHeading = normalize(targetHeading - 90);  //turn left toward left gap
      } else if ((rightAvg > GAP_THRESHOLD || rightIncreasing) && !(leftAvg > GAP_THRESHOLD)) {
        targetHeading = normalize(targetHeading + 90);  //turn right toward right gap
      } else {
        //both gaps or both increasing - turn toward larger average gap
        if (leftAvg > rightAvg) {
          targetHeading = normalize(targetHeading - 90);  // turn left
        } else {
          targetHeading = normalize(targetHeading + 90);  // turn right
        }
      }
      
      phase = 1;  //switch to turning
      lastTurnTime = millis();  //record turn time
    }
    
  } else if (phase == 1) {
    // turning phase
    //turn gently
    float steerAngle = -error * KP;
    steerAngle = constrain(steerAngle, -MAX_STEER_ANGLE, MAX_STEER_ANGLE);
    steerMotor.runTarget(STEER_SPEED, (int)steerAngle, MotorStop::HOLD, false);
    
    //keep driving while turning (slower)
    driveMotor.run(DRIVE_SPEED);
    
    // check if turn is complete
    if (abs(error) < 8) {  // Slightly more lenient
      turnsCompleted++;
      
      // straighten wheels (non blocking)
      steerMotor.runTarget(STEER_SPEED, 0, MotorStop::HOLD, false);
      
      //check if 3 squares are done
      if (turnsCompleted >= TOTAL_TURNS) {
        driveMotor.coast();
        steerMotor.runTarget(STEER_SPEED, 0, MotorStop::HOLD, false);
        evo.clearDisplay();
        evo.writeLineToDisplay("3 ROUNDS DONE!", 0, true, true);
        
        // non blocking end. stop motors and continue display updates
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
    //forward after turn
    long currentPos = driveMotor.getAngle();
    long distanceTraveled = abs(currentPos - forwardStartPos);
    
    //heading hold
    float steerAngle = -error * KP;
    steerAngle = constrain(steerAngle, -MAX_STEER_ANGLE, MAX_STEER_ANGLE);
    steerMotor.runTarget(STEER_SPEED, (int)steerAngle, MotorStop::HOLD, false);
    
    // drive forward
    driveMotor.run(DRIVE_SPEED);
    
    // check if we've gone far enough
    if (distanceTraveled >= POST_TURN_DISTANCE) {
      phase = 0;  // back to straight
    }
  }
  
  // display with averaged info
  evo.clearDisplay();
  evo.writeToDisplay("L:", 0, 0);   evo.writeToDisplay(leftSmooth, 15, 0);
  evo.writeToDisplay("/", 35, 0);   evo.writeToDisplay(leftAvg, 42, 0);
  evo.writeToDisplay("R:", 65, 0);  evo.writeToDisplay(rightSmooth, 80, 0);
  evo.writeToDisplay("/", 100, 0);  evo.writeToDisplay(rightAvg, 107, 0);
  evo.writeToDisplay("H:", 0, 16);  evo.writeToDisplay((int)heading, 20, 16);
  evo.writeToDisplay("T:", 60, 16); evo.writeToDisplay((int)targetHeading, 80, 16);
  evo.writeToDisplay("Turns:", 0, 32);  evo.writeToDisplay(turnsCompleted, 50, 32);
  evo.writeToDisplay("/12", 70, 32);
  evo.writeToDisplay("P:", 0, 48);  
  if (phase == 0) evo.writeToDisplay("STR", 20, 48);
  else if (phase == 1) evo.writeToDisplay("TRN", 20, 48);
  else evo.writeToDisplay("FWD", 20, 48);
  evo.drawDisplay();
  
  delay(10);
}