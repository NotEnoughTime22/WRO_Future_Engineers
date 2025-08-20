#include <Evo.h>
#include <SoftwareSerial.h>

// Hardware setup
EVOX1 evo;
EvoBNO055 bno(I2C2);
EvoMotor driveMotor(M1, EV3MediumMotor, true);
EvoMotor steerMotor(M2, EV3MediumMotor, true);
EvoVL53L0X leftSensor(I2C3);
EvoVL53L0X rightSensor(I2C4);

// Constants
const int STEER_SPEED = 4000;
const int DRIVE_SPEED = -2000;
const int THRESHOLD = 700;  // Reduced for earlier detection
const int GAP_THRESHOLD = 900;  // Additional threshold for gap detection
const int MAX_STEER_ANGLE = 65;
const float KP = 3.0f;
const int POST_TURN_DISTANCE = 1500;
const unsigned long MIN_TURN_INTERVAL = 1500;
const int TOTAL_TURNS = 12;  // 3 complete squares

// Averaging system constants
const int BUFFER_SIZE = 100;  // Store 1 second of readings at ~10ms intervals
const int READINGS_PER_AVERAGE = 10;  // Average every 10 readings for smoothing

// State variables
float heading = 0;
float targetHeading = 0;
int phase = 0;  // 0=straight, 1=turning, 2=forward after turn
int turnsCompleted = 0;
long forwardStartPos = 0;
unsigned long lastTurnTime = 0;

// Sensor averaging buffers
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
  
  // Get average of readings from the last 1 second
  void getOneSecondAverage(int& leftAvg, int& rightAvg) {
    unsigned long currentTime = millis();
    unsigned long oneSecondAgo = currentTime - 1000;
    
    long leftSum = 0, rightSum = 0;
    int validCount = 0;
    
    // Sum readings from the last 1 second
    for (int i = 0; i < count; i++) {
      int bufferIndex = (index - 1 - i + BUFFER_SIZE) % BUFFER_SIZE;
      if (timestamps[bufferIndex] >= oneSecondAgo) {
        leftSum += leftReadings[bufferIndex];
        rightSum += rightReadings[bufferIndex];
        validCount++;
      } else {
        break;  // Older readings, stop here
      }
    }
    
    if (validCount > 0) {
      leftAvg = leftSum / validCount;
      rightAvg = rightSum / validCount;
    } else {
      // Fallback to most recent reading
      int recentIndex = (index - 1 + BUFFER_SIZE) % BUFFER_SIZE;
      leftAvg = leftReadings[recentIndex];
      rightAvg = rightReadings[recentIndex];
    }
  }
  
  // Get current smoothed reading (average of last few readings)
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

// Utility functions
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

// Improved sensor reading with outlier rejection
int cleanSensorReading(int rawReading, int lastGoodReading) {
  // Handle sensor errors and timeouts
  if (rawReading <= 0) return lastGoodReading;
  if (rawReading >= 8000) return 2000;
  
  // Reject extreme outliers (likely noise)
  if (lastGoodReading > 0) {
    int difference = abs(rawReading - lastGoodReading);
    if (difference > 1500 && rawReading < 100) {
      // Likely a noise spike, use last good reading
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
  // Read sensors with improved error handling
  static int lastLeftReading = 1000;
  static int lastRightReading = 1000;
  
  int rawLeft = leftSensor.getDistance();
  int rawRight = rightSensor.getDistance();
  
  int leftDist = cleanSensorReading(rawLeft, lastLeftReading);
  int rightDist = cleanSensorReading(rawRight, lastRightReading);
  
  lastLeftReading = leftDist;
  lastRightReading = rightDist;
  
  // Add readings to buffer
  sensorBuffer.addReading(leftDist, rightDist);
  
  // Get smoothed current readings and 1-second averages
  int leftSmooth, rightSmooth;
  int leftAvg, rightAvg;
  sensorBuffer.getCurrentSmoothed(leftSmooth, rightSmooth);
  sensorBuffer.getOneSecondAverage(leftAvg, rightAvg);
  
  // Get heading with basic error protection
  float x, y, z;
  static unsigned long lastValidReading = 0;
  static float lastValidHeading = 0;
  
  bno.getEuler(&x, &y, &z);
  
  // Check if reading seems valid (not NaN or extreme values)
  if (!isnan(x) && x >= 0 && x < 360) {
    heading = normalize(x);
    lastValidHeading = heading;
    lastValidReading = millis();
  } else {
    // Use last valid heading if sensor gives invalid data
    heading = lastValidHeading;
    // If sensor has been failing for too long, reset to 0
    if (millis() - lastValidReading > 2000) {
      heading = 0;
      lastValidHeading = 0;
      lastValidReading = millis();
    }
  }
  
  float error = headingError(heading, targetHeading);

  // Add watchdog protection - reset if stuck in one phase too long
  static unsigned long phaseStartTime = 0;
  static int lastPhase = -1;
  
  if (phase != lastPhase) {
    phaseStartTime = millis();
    lastPhase = phase;
  }
  
  // If stuck in turning or forward phase for too long, reset to straight
  if ((phase == 1 || phase == 2) && (millis() - phaseStartTime > 10000)) {
    phase = 0;  // Force back to straight mode
    targetHeading = heading;  // Accept current heading as target
  }
  
  // State machine
  if (phase == 0) {
    // STRAIGHT PHASE
    // Simple steering based on heading error
    float steerAngle = -error * KP;
    steerAngle = constrain(steerAngle, -MAX_STEER_ANGLE, MAX_STEER_ANGLE);
    steerMotor.runTarget(STEER_SPEED, (int)steerAngle, MotorStop::HOLD, false);
    
    // Drive forward
    driveMotor.run(DRIVE_SPEED);
    
    // Enhanced gap detection using both current smoothed and averaged readings
    bool leftGapCurrent = leftSmooth > THRESHOLD;
    bool rightGapCurrent = rightSmooth > THRESHOLD;
    bool leftGapAverage = leftAvg > GAP_THRESHOLD;
    bool rightGapAverage = rightAvg > GAP_THRESHOLD;
    
    // Early detection: trigger on current readings, confirm with averages
    bool leftGap = leftGapCurrent || leftGapAverage;
    bool rightGap = rightGapCurrent || rightGapAverage;
    
    // Additional early detection: look for increasing distance trend
    bool leftIncreasing = (leftSmooth > lastLeftReading + 100);
    bool rightIncreasing = (rightSmooth > lastRightReading + 100);
    
    bool timeOK = (millis() - lastTurnTime) > MIN_TURN_INTERVAL;
    
    // Turn if gap detected (with trend consideration) and enough time has passed
    if (((leftGap || leftIncreasing) || (rightGap || rightIncreasing)) && timeOK) {
      
      // Use 1-second averages for turn direction decision
      if ((leftAvg > GAP_THRESHOLD || leftIncreasing) && !(rightAvg > GAP_THRESHOLD)) {
        targetHeading = normalize(targetHeading - 90);  // Turn left toward left gap
      } else if ((rightAvg > GAP_THRESHOLD || rightIncreasing) && !(leftAvg > GAP_THRESHOLD)) {
        targetHeading = normalize(targetHeading + 90);  // Turn right toward right gap
      } else {
        // Both gaps or both increasing - turn toward larger average gap
        if (leftAvg > rightAvg) {
          targetHeading = normalize(targetHeading - 90);  // Turn left
        } else {
          targetHeading = normalize(targetHeading + 90);  // Turn right
        }
      }
      
      phase = 1;  // Switch to turning
      lastTurnTime = millis();  // Record turn time
    }
    
  } else if (phase == 1) {
    // TURNING PHASE
    // Turn gently
    float steerAngle = -error * KP;
    steerAngle = constrain(steerAngle, -MAX_STEER_ANGLE, MAX_STEER_ANGLE);
    steerMotor.runTarget(STEER_SPEED, (int)steerAngle, MotorStop::HOLD, false);
    
    // Keep driving while turning (slower)
    driveMotor.run(DRIVE_SPEED);
    
    // Check if turn is complete
    if (abs(error) < 8) {  // Slightly more lenient
      turnsCompleted++;
      
      // Straighten wheels - NON-BLOCKING
      steerMotor.runTarget(STEER_SPEED, 0, MotorStop::HOLD, false);
      
      // Check if 3 complete squares are done
      if (turnsCompleted >= TOTAL_TURNS) {
        driveMotor.coast();
        steerMotor.runTarget(STEER_SPEED, 0, MotorStop::HOLD, false);
        evo.clearDisplay();
        evo.writeLineToDisplay("3 ROUNDS DONE!", 0, true, true);
        
        // Non-blocking end - just stop motors and continue display updates
        while(true) {
          driveMotor.coast();
          steerMotor.coast();
          delay(100);
        }
      }
      
      // Start forward movement to clear the gap area
      forwardStartPos = driveMotor.getAngle();
      phase = 2;
    }
    
  } else if (phase == 2) {
    // FORWARD AFTER TURN PHASE
    long currentPos = driveMotor.getAngle();
    long distanceTraveled = abs(currentPos - forwardStartPos);
    
    // Simple heading hold
    float steerAngle = -error * KP;
    steerAngle = constrain(steerAngle, -MAX_STEER_ANGLE, MAX_STEER_ANGLE);
    steerMotor.runTarget(STEER_SPEED, (int)steerAngle, MotorStop::HOLD, false);
    
    // Drive forward
    driveMotor.run(DRIVE_SPEED);
    
    // Check if we've gone far enough
    if (distanceTraveled >= POST_TURN_DISTANCE) {
      phase = 0;  // Back to straight
    }
  }
  
  // Enhanced display with averaging info
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