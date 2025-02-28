// Robot Tour 2025 Science Olympiad
//Joshua Ford & Aliza Azhar

#include <Arduino.h>
#include <Wire.h>
#include <LiquidCrystal_I2C.h>

#define VERSION           "D4 2.1"

#define DISPLAY_PRESENT        0  // set to 1 if the 20x4 I2C Display is present & wired (do not do this otherwise, the code will break)


//  Board: Ardunio Uno R3
//  I/O PIN CONNECTIONS BELOW (DO NOT CHANGE)

#define PIN_MTR1_ENCA          2
#define PIN_MTR2_ENCA          3
#define PIN_PB_START           4
#define PIN_MTR1_DIR_FWD       5
#define PIN_MTR1_DIR_REV       6
#define PIN_MTR2_DIR_FWD       7
#define PIN_MTR2_DIR_REV       8
#define PIN_MTR1_PWM           9
#define PIN_MTR2_PWM          10
#define PIN_SONIC_PULSE       11
#define PIN_SONIC_TRIGGER     12
#define PIN_LED               13


//Continue to fine tune these values (maybe with new motor?)

#define ENCODER_COUNTS_PER_REV  540   // Number of encoder pulses per wheel revolution
#define MM_PER_REV              283   // Number of mm per wheel revolution (Diameter * Pi)
#define ENCODER_COUNTS_90_DEG   270   // Number of encoder pulses to make a 90 degree turn
#define SPEED_MIN               120    // Minimum speed (pulses/second) use at the end of individual moves

//#define RIGHT_ENCODER_SCALE 1.17161733572
#define RIGHT_ENCODER_SCALE 1.20
volatile float rightEncoderAccum = 0.0;
int rightTurnCount = 0;
int leftTurnCount = 0;
//Above three lines are the current solution for the problematic right motor encoder, a new motor should fix this issue
//If new motor has working encoder, remove rightEncoderAccum


LiquidCrystal_I2C display(0x27,20,4);

unsigned long usLast;
long usecElapsed;
long usScanLong;
int  usLongResetCount;
long usScanAvg;
long timerusScan;
int scanCount;

long timerRunTime;
int speedFwd;
int speedTurn;
int flagTimeRun;
int flagLastMoveFwd;

int flagLED;

float sonicDistance;

int printStep;
int printLastCmd;
unsigned long msTimerPrint;

unsigned long msTimerMPU;
unsigned long timerSonicRange;

unsigned long timerPBStartOn;
unsigned long timerPBStartOff;

unsigned long timerDelay;

#define MAX_COMMANDS    60

class Command
{
  private:
    int start;
    int end;
    int last_cmd;
    int list[MAX_COMMANDS];
    int p1[MAX_COMMANDS];
    int flagFirstScan;

  public:
    inline int   current()  { return list[start]; };
    inline int   getParameter1() { return p1[start]; };
    inline int   empty()    { if (start==end) return 127; else return 0; };
    inline int   last()     { return last_cmd; };
    inline int   firstScan() { int flag = flagFirstScan; flagFirstScan = 0; return flag; } 
  
    Command() {
      clear();
    }
  
    void clear() {
      start = 0;
      end = 0;
      last_cmd = 0;
      flagFirstScan = 0;
      int n = 0;
      for (n=0;n<MAX_COMMANDS;n++) { list[n] = 0;  p1[n] = 0; }
    }

    void add(int cmd) {
      add(cmd,0);
    }

    void add(int cmd,int par1) {
      list[end] = cmd;
      p1[end] = par1;
      end++;
      if (end >= MAX_COMMANDS) end = 0;
    }
  
    int next() {
      flagFirstScan = 127;
      if (empty()) return 0;
      last_cmd = list[start];   // save last command
      start++;
      if (start >= MAX_COMMANDS) start = 0;
      return list[start];
    }

};

Command cmdQueue;


//  List of possible vehicle motion commands
//   -- Additional motion commands can be added which will require code to execute
#define VEHICLE_START_WAIT      1       // Wait for the start button to be pressed
#define VEHICLE_START           2       // First motion command after button press
#define VEHICLE_FORWARD         10
#define VEHICLE_BACKWARD        20      // Added this command (reverse of VEHICLE_FORWARD)
#define VEHICLE_TURN_RIGHT      40
#define VEHICLE_TURN_LEFT       50
#define VEHICLE_SET_MOVE_SPEED  101
#define VEHICLE_SET_TURN_SPEED  102
#define VEHICLE_SET_ACCEL       105
#define VEHICLE_FINISHED        900     // Must be at the end of the command list
#define VEHICLE_STOP            1000
#define VEHICLE_ABORT           2000    // Used to abort the current movement list and stop the robot

// Loads the command queue with the robots commands to be executed during a run

void loadCommandQueue() {

  cmdQueue.clear();
  cmdQueue.add(VEHICLE_START_WAIT);     // DO NOT REMOVE
  cmdQueue.add(VEHICLE_START);          // DO NOT REMOVE

  // Define robot movement speeds

      cmdQueue.add(VEHICLE_SET_MOVE_SPEED, 1500);     // Speed used for forward movements (not sure is 1500 is the max)
      cmdQueue.add(VEHICLE_SET_TURN_SPEED, 600);     // Speed used for left or right turns
      cmdQueue.add(VEHICLE_SET_ACCEL, 700);         // smaller is softer; larger is quicker and less accurate moves

      //THIS IS WHERE THE CODE IS UPDATED!

      //Distance is in units
      //10ish units per centimeter

      // 1/2 Square - 300 Units (someone is wrong with the encoder but this is the 1/2 block value)
      // 1 Square - 500 Units
      // 2 Squares - 1000 Units
      // 3 Squares - 1500 Units
      // x squares - 500*x Units

      //cmdQueue.add(VEHICLE_FORWARD, 500); *1 Full Block Forward (500 per block)*
      //cmdQueue.add(VEHICLE_BACKWARD, 500); *1 Full Block Backward*
      //cmdQueue.add(VEHICLE_FORWARD, 300); *Starting Command (roughly half a block forward to enter the track)*

      //cmdQueue.add(VEHICLE_FORWARD, 400); *Command to enter gate (could be lowered, but don't go under like 300)*
      //cmdQueue.add(VEHICLE_BACKWARD, 400); *Command to reverse out of gate (MUST HAVE THE SAME VALUE AS CORRESPONDING PAIR)*

      //cmdQueue.add(VEHICLE_TURN_RIGHT); *Turn right*
      //cmdQueue.add(VEHICLE_TURN_LEFT); *Turn left*

      cmdQueue.add(VEHICLE_FORWARD, 300);

      // This MUST be the last command.  
      cmdQueue.add(VEHICLE_FINISHED);

}


// Distance is encoder pulses
// Speed is encoder pulses per second
// Accel is encoder pulses per second^2

class MotionLogic
{
  private:
    long timeAccel;
    long timeAtSpeed;
    long timeDecel;

    long timeRunning;
    long timeRunningLast;
    long timerUpdate;
    
    int pinPWM;
    int outputPWM;
    int outputFwd;
    int outputRev;

    int running;
    int flagStopped;
    int counterStopped;

    int pwmLoopI;
    int pwmLoopP; 

    int debugPrint;

  public:
    long accelRate;
    long decelRate;
    long position;
    long posProfile;

    int speedTarget;
    int speedProfile;
    int speedActual;
    int speedMinimum;

    int speedAtDecel;

    long countEncoder;
    long countEncoderLast;

    inline int getOutputPWM() { return outputPWM; };
    inline int getOutputFwd() { return outputFwd; };
    inline int getOutputRev() { return outputRev; };

    inline void incrEncoder() { countEncoder++; };

    inline void debugOn() { debugPrint = 1; };   // used this function to turn on debug print statements
                                                 
    inline int debugState() { return debugPrint; };

    MotionLogic() {
      outputPWM = 0;
      outputFwd = 0;
      outputRev = 0;

      countEncoder = 0;
      countEncoderLast = 0;

      debugPrint = 0;

      accelRate = 200;
      decelRate = 200;

      posProfile = 0;

      speedActual = 0;
      speedTarget = 0;
      speedAtDecel = -10000;
      timeRunning = 0;
      timeRunningLast = 0;
      flagStopped = 0;
      counterStopped = 0;
      running = 0;
    }

    int isStopped() {
      if (flagStopped) return 1;
      return 0;
    }

    void setParams(long accel,int spdMin,int pPWM) {
      accelRate = accel;
      decelRate = accel;
      speedMinimum = spdMin;
      pinPWM = pPWM;
    }

    void setAccel(long accel) {
      accelRate = accel;
      decelRate = accel;
    }

    void startMove(int pos, int spd) {

      if (spd > 0) {
        outputFwd = 1;
        outputRev = 0;
      } else {
        outputFwd = 0;
        outputRev = 1;
      }

      speedTarget = abs(spd);
      position = abs(pos);
      if (debugPrint) { Serial.print(F("Motion - speed       = ")); Serial.println(speedTarget); }
      if (debugPrint) { Serial.print(F("Motion - position    = ")); Serial.println(position); }

      float tAccel = (float) speedTarget / (float) accelRate;
      float tDecel = (float) speedTarget / (float) decelRate;
      float distAccel = (float) speedTarget / 2.0 * tAccel;
      float distDecel = (float) speedTarget / 2.0 * tDecel;
      float distAtSpeed = (float) position - distAccel - distDecel;
      if (distAtSpeed < 0.0) {  // current written as accel and decel same
        // Serial.println("Motion - Short move logic");
        distAccel = (float) position / 2.0;
        distDecel = (float) position / 2.0;
        distAtSpeed = 0.0;

        tAccel = sqrt(distAccel * 2.0 / (float) accelRate);
        tDecel = sqrt(distDecel * 2.0 / (float) decelRate);
      }
      float tAtSpeed = distAtSpeed / (float) speedTarget;

        // times are in microseconds
      timeAccel = 0;
      timeAtSpeed = tAccel * 1000000;
      timeDecel = timeAtSpeed + tAtSpeed * 1000000;


      pwmLoopI = 0;
      pwmLoopP = 0; 
      countEncoder = 0;
      countEncoderLast = 0;
      posProfile = 0;
      speedAtDecel = -10000;
      timeRunning = 0;
      timeRunningLast = 0;
      timerUpdate = 0;
      flagStopped = 0;
      counterStopped = 0;
      running = 1;
    }

    void stop() {
      outputPWM = 0;
      outputFwd = 0;
      outputRev = 0;
      analogWrite(pinPWM,outputPWM);
      running = 0;
      posProfile = 0;
      speedProfile = 0;
      speedTarget = 0;
    }

    void updateMotion(long usecElapsed) {
      int flagUpdate = 0;

      timerUpdate += usecElapsed;
      if (timerUpdate >= 30000) {
        long delta = countEncoder - countEncoderLast;
        speedActual = delta * 1000000L / timerUpdate;
        countEncoderLast = countEncoder;
        timerUpdate = 0;
        flagUpdate = 1;
        if (running == 0 && speedActual < 4) {
          if (!flagStopped) counterStopped++;
          if (counterStopped > 2) flagStopped = 1;
        }
      }

      if (running == 0) {
        outputPWM = 0;
        outputFwd = 0;
        outputRev = 0;
        pwmLoopP = 0;
        pwmLoopI = 0;
        return;
      }

      if (countEncoder >= (position - 2)) {
        if (debugPrint) { 
          Serial.print("STOPPED,");
          Serial.print("timeRunning:");
          Serial.print(timeRunning);
          Serial.print(",encoder:");
          Serial.print(countEncoder);
          Serial.print(",speedActual:");
          Serial.print(speedActual);
          Serial.print(",pwm:");
          Serial.println(outputPWM);
        }

        stop();
        return;
      }

      timeRunning += usecElapsed;
      if (flagUpdate == 0) return;

      float speed;
      if (timeRunning < timeAtSpeed) {
        speed = (float) timeRunning / 1000000.0 * (float) accelRate;
        speedProfile = (int) speed;
        //if (debugPrint) { Serial.print(" - Accel speedProfile = "); Serial.println(speedProfile); }
      } else if (timeRunning < timeDecel) {
        speedProfile = speedTarget;
        //if (debugPrint) { Serial.print(" - At Speed speedProfile = "); Serial.println(speedProfile); }
      } else {
        if (speedAtDecel <= -10000) speedAtDecel = speedProfile;
        speed = (float) (timeRunning - timeDecel) / 1000000.0 * (float) decelRate;
        speedProfile = speedAtDecel - (int) speed;
        if (speedProfile < speedMinimum) speedProfile = speedMinimum;
        //if (debugPrint) { Serial.print(" - Decel speedProfile = "); Serial.println(speedProfile); }
      }

      posProfile += (speedProfile * (timeRunning - timeRunningLast)) / 1000000;
      timeRunningLast = timeRunning;

      long perror = posProfile - countEncoder;
      int serror = speedProfile - speedActual;
    
          // PI loop to control speed
          // LOOP IS NOT TUNED YET (THIS MUST BE DONE SOON)

      pwmLoopI += serror / 4;
      pwmLoopP = serror / 2; 
        
      outputPWM = pwmLoopP + pwmLoopI;
      if (outputPWM < 0) outputPWM = 0;
      if (outputPWM > 254) outputPWM = 254;

      if (debugPrint) { 
        Serial.print("timeRunning:");
        Serial.print(timeRunning);
        Serial.print(",encoder:");
        Serial.print(countEncoder);
        Serial.print(",speedProfile:");
        Serial.print(speedProfile);
        Serial.print(",speedActual:");
        Serial.print(speedActual);
        Serial.print(",pos_error:");
        Serial.print(perror);
        Serial.print(",speed_error:");
        Serial.print(serror);
        Serial.print(",loopI:");
        Serial.print(pwmLoopI);
        Serial.print(",loopP:");
        Serial.print(pwmLoopP);
        Serial.print(",pwm:");
        Serial.println(outputPWM);
      }

      analogWrite(pinPWM,outputPWM);
    }

};

MotionLogic mtrLeft;
MotionLogic mtrRight;

// Interupt function for counting left motor encoder pulses
void encoderIntLeft()  { 
  mtrLeft.incrEncoder();
}


// Interupt function for counting right motor encoder pulses
void encoderIntRight()  { 
  // This function uses the rightEncoderScale to *fix* the right motor encoder
  // This should be updated to match the left encoder when the left motor arrives
  rightEncoderAccum += RIGHT_ENCODER_SCALE;
  long scaledCount = (long)(rightEncoderAccum + 0.5);
  mtrRight.countEncoder = scaledCount;
}


void setMotorOutputs() {
  if (mtrLeft.getOutputFwd()) digitalWrite(PIN_MTR1_DIR_FWD, HIGH);   
  else                        digitalWrite(PIN_MTR1_DIR_FWD, LOW); 
  if (mtrLeft.getOutputRev()) digitalWrite(PIN_MTR1_DIR_REV, HIGH);   
  else                        digitalWrite(PIN_MTR1_DIR_REV, LOW); 
  if (mtrRight.getOutputFwd()) digitalWrite(PIN_MTR2_DIR_FWD, HIGH);   
  else                         digitalWrite(PIN_MTR2_DIR_FWD, LOW); 
  if (mtrRight.getOutputRev()) digitalWrite(PIN_MTR2_DIR_REV, HIGH);   
  else                         digitalWrite(PIN_MTR2_DIR_REV, LOW); 
}

// Sonic Range Finder Function
// Will look at this later, sensor is not yet installed but we have it

void triggerRangeFinder() {
    digitalWrite(PIN_SONIC_TRIGGER, LOW);
    delayMicroseconds(20);
    digitalWrite(PIN_SONIC_TRIGGER, HIGH);
    delayMicroseconds(10);
    digitalWrite(PIN_SONIC_TRIGGER, LOW);
    int pcount = pulseIn(PIN_SONIC_PULSE, HIGH);
    //0.343 is the speed of sound in air divided by 2 for the round trip
    sonicDistance = float(pcount) * 0.343 / 2.0;
    //Serial.print(F("Sonic pcount = "));
    //Serial.print(pcount);
    //Serial.print(F("  mm = "));
    //Serial.println(sonicDistance);
}

// Used to display fault codes on the built-in LED
void faultCodeLED(int count) {
  int i = -1;
  flagLED = true;
  toggleLED();
  while (-1) {
    i = count;
    delay(2000);

    while (i > 0) {
      toggleLED();
      delay(300);
      toggleLED();
      delay(300);
      i--;
    }
  }
}

// Toggle the Ardunio built in LED each time this function is executed
void toggleLED() {
  if (flagLED) {                
    digitalWrite(PIN_LED,LOW);
    flagLED = false;
  } else {
    digitalWrite(PIN_LED,HIGH);
    flagLED = true;    
  }
}

// This function initializes the display but doesn't work yet since the screen is not wired

void initDisplay() {

  // Display is 20 characters wide by 4 lines

  // 01234567890123456789
  // Cmd:
  //   
  // Rng: xxxx.x cm
  // Time: xx.xxx  v.v.vv

  display.clear();
  display.setCursor(0,0);
  display.print(F("Cmd:"));

  display.setCursor(0,2);
  display.print(F("Rng:"));

  display.setCursor(0,3);
  display.print(F("Time:"));

}

// This functions updates the display (once every 20 seconds?)
void updateDisplay() {
  int cmd;
  char buff[12];
  int i;
  int itmp;
  float f;

  switch (printStep) {
    case 0 :
        cmd = cmdQueue.current();
        if (printLastCmd != cmd) {
          display.setCursor(4,0);
          switch (cmd) {
            case VEHICLE_START_WAIT :
                            // 4567890123456789
              display.print(F("WAIT START     "));
              break;
            case VEHICLE_START :
              display.print(F("WAIT RELEASE   "));
              break;
            case VEHICLE_FORWARD :
              display.print(F("FORWARD        "));
              break;
            case VEHICLE_BACKWARD :
              display.print(F("BACKWARD       "));
              break;
            case VEHICLE_TURN_RIGHT :
              display.print(F("TURN RIGHT     "));
              break;
            case VEHICLE_TURN_LEFT :
              display.print(F("TURN LEFT      "));
              break;
            case VEHICLE_FINISHED :
              display.print(F("FINISHED       "));
              break;
            case VEHICLE_STOP :
              display.print(F("STOP           "));
              break;
            case VEHICLE_ABORT :
              display.print(F("ABORT          "));
              break;
            default :
              display.print(F("***unknown**"));
              break;            
          }
          printLastCmd = cmd;
        }
        break;
    case 1 :
        break;
    case 2 :
        break;
    case 3 :
        display.setCursor(4,2);
        display.print(sonicDistance,1);
        //display.print("cm  ");
        //Probably in mm but need to check
        display.print("mm  ");
        break;
    case 4 :
        display.setCursor(6,3);
        f = (float) timerRunTime / 1000000.0; 
        display.print(f,3);
        break;
    default :
        printStep = -1;
        break;
  }

  printStep++;  // increment the print step value
  
  toggleLED();
}

void setup() {

  rightEncoderAccum = 0.0;
  mtrRight.countEncoder = 0;
  rightTurnCount = 0;
  leftTurnCount = 0;

  pinMode(PIN_LED, OUTPUT);
  flagLED = false;

  Serial.begin(115200); // ADDED
  delay(50); // ADDED
  Serial.println(F("Setup()...")); // ADDED
  //Above was for debugging the right motor encoder, can probably be removed now?

  // Only uncomment one motor at a time to use the Serial Plotter function to tune the PI loop
  //mtrLeft.debugOn();
  //mtrRight.debugOn();
  if (mtrLeft.debugState() || mtrRight.debugState()) {
    Serial.begin(115200);
    Serial.println(F("Setup()..."));
  }

  if (DISPLAY_PRESENT) {
    //Serial.println(F("Display init()"));
    display.init();  //initialize the lcd
    display.backlight();  //open the backlight 
    display.clear();
    display.setCursor(0,0);
    display.print(F("Start Up....."));
  }

  pinMode(PIN_SONIC_TRIGGER,OUTPUT);
  pinMode(PIN_SONIC_PULSE,INPUT);

  pinMode(PIN_PB_START, INPUT_PULLUP);

  pinMode(PIN_MTR1_PWM,OUTPUT);
  pinMode(PIN_MTR2_PWM,OUTPUT);

  pinMode(PIN_MTR1_ENCA, INPUT_PULLUP);
  pinMode(PIN_MTR2_ENCA, INPUT_PULLUP);

  sonicDistance = 0.0;
  timerSonicRange = 0;

  timerDelay = 0;

  attachInterrupt(digitalPinToInterrupt(PIN_MTR1_ENCA), encoderIntLeft, RISING);
  attachInterrupt(digitalPinToInterrupt(PIN_MTR2_ENCA), encoderIntRight, RISING);

  pinMode(PIN_MTR1_DIR_FWD,OUTPUT);
  pinMode(PIN_MTR1_DIR_REV,OUTPUT);
  pinMode(PIN_MTR2_DIR_FWD,OUTPUT);
  pinMode(PIN_MTR2_DIR_REV,OUTPUT);
  digitalWrite(PIN_MTR1_DIR_FWD, LOW);
  digitalWrite(PIN_MTR1_DIR_REV, LOW);
  digitalWrite(PIN_MTR2_DIR_FWD, LOW);
  digitalWrite(PIN_MTR2_DIR_REV, LOW);

  speedFwd = 100;   
  speedTurn = 100;

  int speedAccel = 100;
  int speedMin  = SPEED_MIN;
  mtrLeft.setParams(speedAccel, speedMin, PIN_MTR1_PWM);
  mtrRight.setParams(speedAccel, speedMin, PIN_MTR2_PWM);

  //Serial.println(F("Initialize Display with background text"));
  if (DISPLAY_PRESENT) { initDisplay(); }
  printStep = 0;

  loadCommandQueue();
  usScanLong = 0;
  usScanAvg = 0;
  timerusScan = 0;
  scanCount = 0;
  usLongResetCount = 0;
  //Serial.println(F("....End Setup"));

  usLast = micros();

}


//Be careful with this function as it manages the motors and movement
//Do NOT add any delays to this function ever

void loop() {
  long distance;
  int speed;
  float fspd;
  long ldelta;
  long delayWait;
  int itmp;

    // this block calculates the number microseconds since this function's last execution
  unsigned long current = micros();
  usecElapsed = current - usLast;
  usLast = current;
  if (usecElapsed > usScanLong) usScanLong = usecElapsed;
  timerusScan += usecElapsed;
  scanCount++;
  if (timerusScan > 1000000) {
    usScanAvg = timerusScan / scanCount;
    timerusScan = 0;
    scanCount = 0;
    usLongResetCount++;
    if (usLongResetCount > 10) {
      usScanLong = 0;
      usLongResetCount = 0;
    }
  }

  // update motor speed and status
  mtrLeft.updateMotion(usecElapsed);
  mtrRight.updateMotion(usecElapsed);

    // updates the display every 100000us or 0.1 seconds
  if (msTimerPrint > 100000) {
    if (DISPLAY_PRESENT) { updateDisplay(); }

    //--- ADDED: Print the speeds of each motor to Serial
    /*Serial.print(F("PWMLOOPP: "));
    Serial.print(mtrLeft.pwmLoopP);
    Serial.print(F("  |  PWMLOOPP: "));
    Serial.println(mtrRight.pwmLoopP);*/
    //--- END ADDED
    //Right motor encoder debugging

    msTimerPrint = 0;
  }
  msTimerPrint += usecElapsed;

  int newCmd = false;
  if (cmdQueue.firstScan()) {
    newCmd = true;
    //Serial.print(F("New Vehicle Cmd = "));
    //Serial.println(cmdQueue.current());
  }

  int pbStart = !digitalRead(PIN_PB_START);

  if (pbStart) {
    timerPBStartOn  += usecElapsed;
    timerPBStartOff = 0;
  } else {
    timerPBStartOn = 0;
    timerPBStartOff += usecElapsed;
  }
  
  if (cmdQueue.current() > VEHICLE_START && cmdQueue.current() < VEHICLE_ABORT) {
    if (timerPBStartOn > 100000) {
      cmdQueue.clear();
      cmdQueue.add(VEHICLE_ABORT);
      mtrLeft.stop();
      mtrRight.stop();
      setMotorOutputs();
    }
  }

  if (flagTimeRun) timerRunTime += usecElapsed;

  switch (cmdQueue.current()) {
    case VEHICLE_START_WAIT :
      if (timerPBStartOn > 100000) {
        cmdQueue.next();
      }
      timerSonicRange += usecElapsed;
      if (timerSonicRange > 1000000) {
        timerSonicRange = 0;
        triggerRangeFinder();
      }
      break;
    case VEHICLE_START :
      timerRunTime = 0;
      if (timerPBStartOff > 100000) {
        cmdQueue.next();
        flagTimeRun = 1;
        flagLastMoveFwd = 0;
      }
      break;
    case VEHICLE_FORWARD :
      if (newCmd) {
        distance = ((long) cmdQueue.getParameter1() * (long) ENCODER_COUNTS_PER_REV / (long) MM_PER_REV);
        speed = speedFwd;
        mtrLeft.startMove(distance,speed);
        mtrRight.startMove(distance,speed);
        setMotorOutputs();
      }
      
      if (mtrLeft.isStopped() || mtrRight.isStopped()) {
        mtrLeft.stop();
        mtrRight.stop();
        rightEncoderAccum = 0;
        mtrRight.countEncoder = 0;
        setMotorOutputs();
        cmdQueue.next();
      }
      break;

    case VEHICLE_BACKWARD : 
    if (newCmd) {
      distance = ((long) cmdQueue.getParameter1() * (long) ENCODER_COUNTS_PER_REV / (long) MM_PER_REV);
      speed = speedFwd;
      mtrLeft.startMove(distance,speed * -1);
      mtrRight.startMove(distance,speed * -1);
      setMotorOutputs();
    }

    if (mtrLeft.isStopped() || mtrRight.isStopped()) {
      mtrLeft.stop();
      mtrRight.stop();
      rightEncoderAccum = 0;
      mtrRight.countEncoder = 0;
      setMotorOutputs();
      cmdQueue.next();
    }
    break;

    case VEHICLE_TURN_RIGHT :
      if (newCmd) {
        //distance = (ENCODER_COUNTS_90_DEG + (turnCount * 3)); // change this(#turnCount#)
        distance = ENCODER_COUNTS_90_DEG + 4;
        rightTurnCount++;
        if (rightTurnCount >= 3 && rightTurnCount % 2 == 0) {
          distance = distance + (rightTurnCount * 1);
        }
        speed = speedTurn;
        mtrLeft.startMove(distance,speed);
        mtrRight.startMove(distance,speed * -1);
        setMotorOutputs();
      }

      if (mtrLeft.isStopped() && mtrRight.isStopped()) {
        mtrLeft.stop();
        mtrRight.stop();
        rightEncoderAccum = 0;
        mtrRight.countEncoder = 0;
        setMotorOutputs();
        cmdQueue.next();
      }      
      break;
    case VEHICLE_TURN_LEFT :
      if (newCmd) {
        //distance = (ENCODER_COUNTS_90_DEG - (turnCount * 5) - 15); //change this(#turnCount#)
        distance = ENCODER_COUNTS_90_DEG - 16;
        leftTurnCount++;
        if (leftTurnCount >= 3 && leftTurnCount % 2 == 0) {
          distance = distance - (leftTurnCount * 1.1);
        }
        speed = speedTurn;
        mtrLeft.startMove(distance,speed * -1);
        mtrRight.startMove(distance,speed);
        setMotorOutputs();
      }

      if (mtrLeft.isStopped() && mtrRight.isStopped()) {
        mtrLeft.stop();
        mtrRight.stop();
        rightEncoderAccum = 0;
        mtrRight.countEncoder = 0;
        setMotorOutputs();
        cmdQueue.next();
      }      
      break;
    case VEHICLE_SET_MOVE_SPEED :
      speedFwd = cmdQueue.getParameter1();
      cmdQueue.next();
      break;
    case VEHICLE_SET_TURN_SPEED :
      speedTurn = cmdQueue.getParameter1();
      cmdQueue.next();
      break;
    case VEHICLE_SET_ACCEL :
      mtrLeft.setAccel(cmdQueue.getParameter1());
      mtrRight.setAccel(cmdQueue.getParameter1());
      cmdQueue.next();
      break;
    default :
    case VEHICLE_FINISHED :
      if (newCmd) {
        mtrLeft.stop();
        mtrRight.stop();
        setMotorOutputs();
      }
      flagTimeRun = 0;
      break;
    case VEHICLE_ABORT :
      mtrLeft.stop();
      mtrRight.stop();
      setMotorOutputs();
      flagTimeRun = 0;
      if (timerPBStartOff > 200000) {
        loadCommandQueue();
      }
      break;
  }

}