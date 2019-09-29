/* INCLUDES ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~*/
#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps20.h"
#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
    #include "Wire.h"
#endif

/* PIN DEFINITIONS ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~*/
#define ledPin 8     // On-board LED
#define mpuIntPin A1 // MPU6050 interrupt pin

/* GLOBAL VARIABLES ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~*/
bool is_balancing = false; // Disables or enables robot movement 

float Kp, Ki, Kd;                  // PID gain values
float* PID[3];                     // Array for looping over PID values
float err, err_prev, err_d, err_i; // Error terms for controller
float setpoint;                    // Desired robot pitch to maintain
unsigned long timer;               // Controller loop counter

float pitch;    // Pitch angle of the balancing robot
float velocity; // Requested angular velocity of wheels

int step_prd_cur, step_prd_cur_raw; // Stepper motor pulse period control
int step_prd_target;
int prd_counter; 

/* GLOBAL VARIABLES - MPU6050 Library ~~~~~~~~~~~~~~~~~~~~~~~~~*/
MPU6050 mpu;            // MPU control/status vars
bool dmpReady = false;  // set true if DMP init was successful
uint8_t mpuIntStatus;   // holds actual interrupt status byte from MPU
uint8_t devStatus;      // return status after each device operation (0 = success, !0 = error)
uint16_t packetSize;    // expected DMP packet size (default is 42 bytes)
uint16_t fifoCount;     // count of all bytes currently in FIFO
uint8_t fifoBuffer[64]; // FIFO storage buffer
volatile bool mpuInterrupt = false; // indicates whether MPU interrupt pin has gone high

Quaternion q;           // [w, x, y, z]         quaternion container
VectorInt16 aa;         // [x, y, z]            accel sensor measurements
VectorInt16 aaReal;     // [x, y, z]            gravity-free accel sensor measurements
VectorInt16 aaWorld;    // [x, y, z]            world-frame accel sensor measurements
VectorFloat gravity;    // [x, y, z]            gravity vector
float euler[3];         // [psi, theta, phi]    Euler angle container
float ypr[3];           // [yaw, pitch, roll]   yaw/pitch/roll container and gravity vector

/* CONSTANTS ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~*/
const int ZERO_PRD = 10000; // Large integer to stop steppers
const int SWAP_PRD = 15000; // Target value for stopping motor, larger than ZERO_PRD

const int TICKS_PER_SEC = 46875; // Number of timer ticks per second
const int STEPS_PER_REV = 2000;  // Stepper motor number of pulses per revolution

const unsigned long T_LOOP = 10000; // PID control loop period in microseconds
const unsigned int  HZ_ACC = 1000;  // Acceleration counter frequency 
const unsigned int  CC0    = TICKS_PER_SEC/HZ_ACC - 1; // Register for counter overflow

const float BALANCE_ANGLE = 89.3; // Angle at which robot is naturally balanced
const int   FAILURE_ANGLE = 40;   // Angle at which to abort balancing and shutdown robot

const float MAX_VELOCITY = 2.6*M_PI; // Maximum velocity permitted

const int   F_inc = 50; // Frequency (PPS) increment per acceleration interpolation step
const float BETA = (float)F_inc/TICKS_PER_SEC; // Constant for acc calc

/* SETUP FUNCTION ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~*/
void setup() 
{
  pinMode(ledPin, OUTPUT);
  pinMode(mpuIntPin, INPUT);

  SerialUSB.begin(115200);
  while (!SerialUSB); // Wait until serial port ready

  setupSteppers(); // Setup stepper pins and ISR
  setupMPU6050();  // Start MPU6050 readings
  setupPID();      // Initialise PID values

  // Prompt for initial user command
  SerialUSB.println(F("Enter \"s\" to start, \"x\" to stop."));
  emptySerialBuffer();
  waitForSerialData();
}

/* Simple integer sign function ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~*/
static inline int sign(int value)
{
  return (value > 0) - (value < 0);
}

/* Simple float sign function ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~*/
static inline int signf(float value)
{
  return (value > 0) - (value < 0);
}

void setupPID()
{
  Kp = 0.0; // Set default PID values
  Ki = 0.0;
  Kd = 0.0;

  pitch = BALANCE_ANGLE; // Assume starting vertical
  setpoint = BALANCE_ANGLE; // Controller target setpoint
  
  PID[0]=&Kp;  PID[1]=&Ki;  PID[2]=&Kd; // Set PID pointer array values
}

void setupSteppers()
{
  // Set stepper pins as outputs
  REG_PORT_DIR1 |= PORT_PB09; // STEP_R
  REG_PORT_DIR0 |= PORT_PA05; // STEP_L
  REG_PORT_DIR0 |= PORT_PA04; // DIR_R
  REG_PORT_DIR0 |= PORT_PA10; // DIR_L

  int step_dir = 1; // Initialise stepper motor directions forward
  setStepperDirections(step_dir);
  
  setupTimerISR(); // Set timer TC4 to call TC4_Handler at 46.876 kHz (every ~21.33us)

  prd_counter      = 0;
  step_prd_cur     = 0;
  step_prd_cur_raw = 0;
  step_prd_target  = 0;  
}

void setupTimerISR()
{
  // Set up the generic clock (GCLK4) used to clock timers
  REG_GCLK_GENDIV = GCLK_GENDIV_DIV(1) |          // Divide the 48MHz clock source by divisor 1: 48MHz/1=48MHz
                    GCLK_GENDIV_ID(4);            // Select Generic Clock (GCLK) 4
  while (GCLK->STATUS.bit.SYNCBUSY);              // Wait for synchronization

  REG_GCLK_GENCTRL = GCLK_GENCTRL_IDC |           // Set the duty cycle to 50/50 HIGH/LOW
                     GCLK_GENCTRL_GENEN |         // Enable GCLK4
                     GCLK_GENCTRL_SRC_DFLL48M |   // Set the 48MHz clock source
                     GCLK_GENCTRL_ID(4);          // Select GCLK4
  while (GCLK->STATUS.bit.SYNCBUSY);              // Wait for synchronization

  // Feed GCLK4 to TC4 and TC5
  REG_GCLK_CLKCTRL = GCLK_CLKCTRL_CLKEN |         // Enable GCLK4 to TC4 and TC5
                     GCLK_CLKCTRL_GEN_GCLK4 |     // Select GCLK4
                     GCLK_CLKCTRL_ID_TC4_TC5;     // Feed the GCLK4 to TC4 and TC5
  while (GCLK->STATUS.bit.SYNCBUSY);              // Wait for synchronization

  // TC4 at 46.875 kHz for step period
  REG_TC4_COUNT16_CC0 = 0;                      // Set the TC4 CC0 register as the TOP value in match frequency mode
  while (TC4->COUNT16.STATUS.bit.SYNCBUSY);       // Wait for synchronization

  // TC5 at 1000 Hz for acceleration
  REG_TC5_COUNT16_CC0 = CC0;                        // Set the TC5 CC0 register as the TOP value in match frequency mode
  while (TC5->COUNT16.STATUS.bit.SYNCBUSY);       // Wait for synchronization

  NVIC_SetPriority(TC4_IRQn, 0);    // Set the Nested Vector Interrupt Controller (NVIC) priority for TC4 to 0 (highest)
  NVIC_EnableIRQ(TC4_IRQn);         // Connect TC4 to Nested Vector Interrupt Controller (NVIC)

  NVIC_SetPriority(TC5_IRQn, 1);    // Set the Nested Vector Interrupt Controller (NVIC) priority for TC5 to 1
  NVIC_EnableIRQ(TC5_IRQn);         // Connect TC5 to Nested Vector Interrupt Controller (NVIC)

  REG_TC4_INTFLAG |= TC_INTFLAG_OVF;              // Clear the interrupt flags
  REG_TC4_INTENSET = TC_INTENSET_OVF;             // Enable TC4 interrupts

  REG_TC5_INTFLAG |= TC_INTFLAG_OVF;              // Clear the interrupt flags
  REG_TC5_INTENSET = TC_INTENSET_OVF;             // Enable TC5 interrupts
 
  REG_TC4_CTRLA |= TC_CTRLA_PRESCALER_DIV1024 |   // Set prescaler to 1024, 48MHz/1024 = 46.875kHz
                   TC_CTRLA_WAVEGEN_MFRQ |        // Put the timer TC4 into match frequency (MFRQ) mode
                   TC_CTRLA_ENABLE;               // Enable TC4
  while (TC4->COUNT16.STATUS.bit.SYNCBUSY);       // Wait for synchronization

  REG_TC5_CTRLA |= TC_CTRLA_PRESCALER_DIV1024 |   // Set prescaler to 1024, 48MHz/1024 = 46.875kHz
                   TC_CTRLA_WAVEGEN_MFRQ |        // Put the timer TC5 into match frequency (MFRQ) mode
                   TC_CTRLA_ENABLE;               // Enable TC5
  while (TC5->COUNT16.STATUS.bit.SYNCBUSY);       // Wait for synchronization
}

void setupMPU6050()
{
  #if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
      Wire.begin();
      Wire.setClock(400000);
  #elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
      Fastwire::setup(400, true);
  #endif

  mpu.initialize(); // Initialise IMU
  SerialUSB.println(mpu.testConnection() ? F("MPU6050 connection successful.") \
    : F("MPU6050 connection failed."));
    
  devStatus = mpu.dmpInitialize(); // Initialise DMP
  if (devStatus == 0)
    SerialUSB.println(F("Enabling DMP."));
  else
    SerialUSB.println(F("DMP Initialization failed."));
  
  // Accelerometer and gyrometer offsets.
  mpu.setXGyroOffset(-1535);
  mpu.setYGyroOffset(-255);
  mpu.setZGyroOffset(36);
  mpu.setXAccelOffset(-2947); 
  mpu.setYAccelOffset(-5297);
  mpu.setZAccelOffset(2365);
  
  mpu.setDMPEnabled(true); // Enable DMP

  // Enable Arduino interrupt detection
  attachInterrupt(digitalPinToInterrupt(mpuIntPin), dmpDataReady, RISING);
  mpuIntStatus = mpu.getIntStatus();

  dmpReady = true; // Set DMP Ready flag so other functions know it is ready
  packetSize = mpu.dmpGetFIFOPacketSize(); // Get expected DMP packet size  
}

void emptySerialBuffer()
{
  while (SerialUSB.available() && SerialUSB.read()); // Empty buffer
}

void waitForSerialData()
{
  while (!SerialUSB.available()); // Wait for data
}

/* LOOP FUNCTION ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~*/
void loop() 
{
  checkForSerialCommands(); // Check for user input

  if (!dmpReady) // If DMP programming failed, abort balancing.
    is_balancing = false;
  
  if (is_balancing) // Balance mode active
  {
    loopIMU(); // Read pitch angle from MPU6050

    if (fabs(pitch - BALANCE_ANGLE) > FAILURE_ANGLE) // Abort balancing if robot falls over too far
    {
      SerialUSB.println(F("Failure angle exceeded. Aborting balancing and deactivating robot."));
      is_balancing = false;
      return;
    }

    velocity = getPIDControlSignal(); // Desired wheel angular velocity
    SerialUSB.println(velocity);

    if (fabs(velocity) > MAX_VELOCITY) // Constrain maximum velocity
      velocity = signf(velocity)*MAX_VELOCITY;

    step_prd_target = velocityToStepPeriod(velocity); // Convert velocity to step period
    
    digitalWrite(ledPin, HIGH); // Light up on-board LED
  }
  else // Robot deactivated
  {
    step_prd_target = 0; // Don't move motors
    step_prd_cur = 0;
    step_prd_cur_raw = 0;
    digitalWrite(ledPin, LOW); // Turn off on-board LED
  }  

  while (timer > micros()) {}; // Wait until loop elapses correct amount of time
  timer += T_LOOP;             // Increment timer
}

/* Convert velocity to required step period ~~~~~~~~~~~~~~*/
float velocityToStepPeriod(float value)
{
  if (fabs(value) < 0.01) // Avoid division by zero
    return signf(value)*ZERO_PRD;
  else
    return round(TICKS_PER_SEC/((value/(2*M_PI))*STEPS_PER_REV));
}

/* Calculate acceleration interpolation ~~~~~~~~~~~~~~~~~~~~~*/
int accelerate(int p_cur, int p_des)
{
  return round((float)p_cur/(1 - sign(p_des - p_cur)*BETA*p_cur));
}

/* Reads data from the MPU6050 ~~~~~~~~~~~~~~~~~~~~~~~~~~~~*/
void loopIMU()
{  
  // Wait for MPU6050 reading to be available
  while (!mpuInterrupt && fifoCount < packetSize) 
  {
    if (mpuInterrupt && fifoCount < packetSize) 
      fifoCount = mpu.getFIFOCount(); // Try to escape while loop 
  }

  mpuInterrupt = false;              // Reset interrupt flag
  mpuIntStatus = mpu.getIntStatus(); // Get INT_STATUS byte
  fifoCount = mpu.getFIFOCount();    // Get current FIFO count

  // Check for overflow
  if ((mpuIntStatus & bit(MPU6050_INTERRUPT_FIFO_OFLOW_BIT)) || fifoCount >= 1024) 
  {
      mpu.resetFIFO(); // Reset to continue cleanly
      fifoCount = mpu.getFIFOCount();
  }
  // Check for DMP data ready interrupt
  else if (mpuIntStatus & bit(MPU6050_INTERRUPT_DMP_INT_BIT)) 
  {
    // Wait for correct available data length (very quick)
    while (fifoCount < packetSize) fifoCount = mpu.getFIFOCount();
    mpu.getFIFOBytes(fifoBuffer, packetSize); // Read a packet from FIFO
    
    // Track FIFO count, in case more than 1 packet is available
    // (allows immediate reading of further packets without an interrupt)
    fifoCount -= packetSize;

    mpu.dmpGetQuaternion(&q, fifoBuffer);
    mpu.dmpGetGravity(&gravity, &q);
    mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
    pitch = ypr[1] * 180/M_PI;
  }
}

/* NOTE: Assumes main loop will maintain consistent loop timing.
 */
float getPIDControlSignal()
{  
  err    = setpoint - pitch; // Calculate error term
  err_i += err;              // Estimate integral of error
  err_d  = err - err_prev;   // Estimate derivative of error

  err_prev = err; // Store current error to use in next calc
  
  return Kp*err + Ki*err_i - Kd*err_d; // Return control signal
}

/* Reset variables when starting balancing again ~~~~~~~~~~*/
void startBalancing()
{
  timer = micros(); // Update loop timer

  err_prev = 0; // Reset PID error terms
  err_i = 0;

  pitch = setpoint; // Assume starting vertical

  step_prd_cur = 0;
  step_prd_cur_raw = ZERO_PRD;
  step_prd_target  = ZERO_PRD; 
}

/* Stepper motor interrupt service routine ~~~~~~~~~~~~~~~~*/
void TC4_Handler() // Interrupt Service Routine (ISR) for timer TC4
{     
  // Check for overflow
  if (TC4->COUNT16.INTFLAG.bit.OVF && TC4->COUNT16.INTENSET.bit.OVF)             
  {    
    prd_counter++; // Increment pulse period counter
        
    // Period has expired
    if (prd_counter >= step_prd_cur)
    {
      prd_counter = 0; // Reset counter for next pulse      
    }
    else if (prd_counter == 1)
    {
      setStepperDirections(step_prd_cur_raw); // Set stepper directions
  
      // Pulse high at start of period
      REG_PORT_OUT1 |= PORT_PB09; // STEP_R high
      REG_PORT_OUT0 |= PORT_PA05; // STEP_L high
    }
    else if (prd_counter == 2) // Finish pulse with low
    {      
      REG_PORT_OUT1 &= ~PORT_PB09; // STEP_R low
      REG_PORT_OUT0 &= ~PORT_PA05; // STEP_L low
    }
    
    REG_TC4_INTFLAG = TC_INTFLAG_OVF; // Clear the OVF interrupt flag
  }
}

/* Acceleration interrupt service routine ~~~~~~~~~~~~~~~~*/
void TC5_Handler() // Interrupt Service Routine (ISR) for timer TC5
{      
  // Perform ACCELERATION interpolation upon counter overflow
  if (TC5->COUNT16.INTFLAG.bit.OVF && TC5->COUNT16.INTENSET.bit.OVF)             
  {       
    // Check which side of period-velocity profile we need to get to
    
    if (sign(step_prd_target) != sign(step_prd_cur_raw)) // OPPOSITE side of profile
    {
      if (abs(step_prd_cur_raw) < ZERO_PRD) // If we haven't slowed to zero yet
      {
        // Slow the stepper to zero
        step_prd_cur_raw = accelerate(step_prd_cur_raw, sign(step_prd_cur_raw)*SWAP_PRD);
        step_prd_cur = abs(step_prd_cur_raw);
      }
      else // Slowed to zero already
      {
        // Swap side of period-velocity profile and dip below the zero threshold
        step_prd_cur_raw = -sign(step_prd_cur_raw)*(ZERO_PRD - 1); 
      }
    }
    else // SAME side of profile
    {
      int p_new = accelerate(step_prd_cur_raw, step_prd_target); // Get acc interp
      
      // Avoid overshooting the target period
      if (abs(p_new - step_prd_cur_raw) > abs(step_prd_target - step_prd_cur_raw))
      {
        step_prd_cur_raw = step_prd_target; // Use original target period if overshoot
        step_prd_cur = abs(step_prd_cur_raw);
      }
      else
      {
        step_prd_cur_raw = p_new; // Use interpolated period
        step_prd_cur = abs(step_prd_cur_raw);
      }      
    }
   
    REG_TC5_INTFLAG = TC_INTFLAG_OVF; // Clear the OVF interrupt flag
  }
}

void setStepperDirections(int &step_prd_raw)
{
  if (step_prd_raw > 0)      // Set target direction FORWARDS
  {
    REG_PORT_OUT0 |= PORT_PA04;  // DIR_R high
    REG_PORT_OUT0 &= ~PORT_PA10; // DIR_L low
  }
  else if (step_prd_raw < 0) // Set target direction BACKWARDS
  {    
    REG_PORT_OUT0 &= ~PORT_PA04; // DIR_R low
    REG_PORT_OUT0 |= PORT_PA10;  // DIR_L high
  }
}

void checkForSerialCommands()
{
  if (SerialUSB.available() > 0) 
  {
    char cmd = SerialUSB.read(); // Read first byte of data
    if (cmd == 's')              // Activate balancing
    {
      bool use_defaults = false;
      if (SerialUSB.available() > 0) // Check any request for default values
      {
        cmd = SerialUSB.read(); // Read next byte of data
        if (cmd == 'd') use_defaults = true;
      }

      if (!use_defaults) // Parse PID values from serial input
      {
        SerialUSB.println(F("Enter comma separated PID values."));
        emptySerialBuffer();
        waitForSerialData();
        
        int k = 0; // Counter for while loop
        while (SerialUSB.available() > 0) 
        {
          if (k == 3) break; // Read only 3 floats
          
          *PID[k] = SerialUSB.parseFloat(); // Parse PID values from serial input
          k++;
        } 

        SerialUSB.print("P: "); SerialUSB.print(Kp); // Print back values to confirm
        SerialUSB.print("\tI:"); SerialUSB.print(Ki);
        SerialUSB.print("\tD:"); SerialUSB.println(Kd);

        SerialUSB.println(F("Enter setpoint value."));
        emptySerialBuffer();
        waitForSerialData();
        
        setpoint = SerialUSB.parseFloat(); // Parse setpoint from serial input

        SerialUSB.print("Setpoint: "); SerialUSB.println(setpoint);
      }  
      
      SerialUSB.println(F("Enabling balance mode."));
      is_balancing = true;
      startBalancing(); // Run any needed resetting code
    }
    else if (cmd == 'x') // Stop balancing
    {
      SerialUSB.println(F("Disabling balance mode."));
      is_balancing = false;
    }

    emptySerialBuffer();
  }
}

/* MPU6050 interrupt service routine ~~~~~~~~~~~~~~~~~~~~~*/
void dmpDataReady() 
{
    mpuInterrupt = true;
}
