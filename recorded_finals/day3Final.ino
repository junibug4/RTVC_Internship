#include <Servo.h>


//Main .ino file

#include <PID_v1.h> 
#include <I2Cdev.h>
#include <SPI.h>
#include <SD.h>
#include <MPU6050_6Axis_MotionApps20.h>
const int chipSelect = 4;

MPU6050 mpu;
Servo myservo;

#define OUTPUT_READABLE_YAWPITCHROLL
#define INTERRUPT_PIN 2
#define LED_PIN 13
#define REVERSE  1

#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
#include <Wire.h>
#endif

bool blinkState = false;          // MPU6050 start
bool dmpReady = false;
uint8_t mpuIntStatus;
uint8_t devStatus;
uint16_t packetSize;
uint16_t fifoCount;
uint8_t fifoBuffer[64];

Quaternion q;
VectorFloat gravity;
float ypr[3];
volatile bool mpuInterrupt = false;
double pitchCopy;


void dmpDataReady() {
  mpuInterrupt = true;
}                                // MPU6050 end

#define ACCEL_IN 4
#define SERVO_OUT 9
double setpoint, pitch, servo_angle,  fan_angle;    
double Kp = 30 , Ki = 0. , Kd = 0.0; 
PID myPID(&pitch, &servo_angle, &setpoint, Kp, Ki, Kd, DIRECT);   

void setup() // #########################  SETUP  ############################ //
{
  Serial.begin(115200);
  
  myservo.attach(9);

  Serial.begin(115200);
    while (!Serial);
    Serial.print("Initializing SD card...");
    if (!SD.begin(chipSelect)) {
        Serial.println("Card failed, or not present");
        while (1);
    }
    Serial.println("Card initialized.");



  myPID.SetMode(AUTOMATIC);    // initialising myPID
  myPID.SetTunings(Kp, Ki, Kd);   
  myPID.SetOutputLimits(-70,70);

  Wire.begin();             // Start of copied code. Attributed to Phil Lightfoot
    Wire.setClock(400000);
    Wire.setWireTimeout(3000, true);

    Serial.println(F("Initializing I2C devices..."));
    mpu.initialize();
    pinMode(INTERRUPT_PIN, INPUT);

    Serial.println(F("Initializing DMP..."));
    devStatus = mpu.dmpInitialize();

    mpu.setXGyroOffset(220);
    mpu.setYGyroOffset(76);
    mpu.setZGyroOffset(-85);
    mpu.setZAccelOffset(1788);

    //mpu.setRate(3);     // Sets the sample rate divisor meaning that it reduces the rate at which the MPU6050 produces data. 
    if (devStatus == 0) {
        mpu.CalibrateAccel(6);
        mpu.CalibrateGyro(6);
        mpu.PrintActiveOffsets();
        Serial.println(F("Enabling DMP..."));
        mpu.setDMPEnabled(true);
        attachInterrupt(digitalPinToInterrupt(INTERRUPT_PIN), dmpDataReady, RISING);
        mpuIntStatus = mpu.getIntStatus();
        dmpReady = true;
        packetSize = mpu.dmpGetFIFOPacketSize();
        mpu.resetFIFO(); // Clear FIFO on startup
    } else {
        Serial.print(F("DMP Initialization failed (code "));
        Serial.print(devStatus);
        Serial.println(F(")"));
    }
    pinMode(LED_PIN, OUTPUT);
                                    //end of copied code
    
    setpoint = 0;
    
  //analogWrite(SERVO_OUT,90);
  
  myservo.write(90);
    
}            // ######################  END OF SETUP  ######################## //                                 


void loop() { // #########################  LOOP  ############################ //
  if (!dmpReady) return;
  while (!mpuInterrupt && fifoCount < packetSize) {
    fifoCount = mpu.getFIFOCount();
  }


    mpuInterrupt = false;
    mpuIntStatus = mpu.getIntStatus();
    fifoCount = mpu.getFIFOCount();

    if ((mpuIntStatus & (0x01 << MPU6050_INTERRUPT_FIFO_OFLOW_BIT)) || fifoCount >= 128) {      // adjust the value of fifoCount from 128 to 1024 depending on the delay set in order to force reset of FIFO overflow
        mpu.resetFIFO();
        //Serial.println(F("FIFO overflow detected! Clearing buffer..."));   //Uncomment this line if you want to see when the FIFO overflow is being cleared. You want the FIFO overflow to happen as that resets it. Otherwise the overflow produces stupid angle readings
        fifoCount = mpu.getFIFOCount(); // Reset FIFO count
        return; // Exit loop early to prevent processing invalid data
    }
    
    if (mpuIntStatus & (0x01 << MPU6050_INTERRUPT_DMP_INT_BIT)) {
      while (fifoCount >= packetSize) {
        mpu.getFIFOBytes(fifoBuffer, packetSize);
        fifoCount -= packetSize;
      }


      mpu.dmpGetQuaternion(&q, fifoBuffer);
      mpu.dmpGetGravity(&gravity, &q);
      mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
      /*
      Serial.print("ypr\t");
      Serial.print(ypr[0] * 180/M_PI);
      Serial.print("\t");
      Serial.print(ypr[1] * 180/M_PI);
      Serial.print("\t");
      Serial.println(ypr[2] * 180/M_PI); 
      */

      blinkState = !blinkState;
      digitalWrite(LED_PIN, blinkState);
        
    }
  pitch = ypr[1] * 180/M_PI;

  File dataFile = SD.open("datalog.txt", FILE_WRITE);
  if (dataFile) {
    dataFile.println(pitch);
    dataFile.close();
    pitchCopy = pitch;
    // Serial.print("pitch = ");
    // Serial.print(pitch);
  } else {
    Serial.println("error opening datalog.txt");  
  }
  //Serial.println(ypr[1])

  // ##############################  PITCH FOUND  ############################ //

  // PID changes to be added here
  
  // #############################  START OF PID  ############################ //
  
  pitch = ypr[1];
  myPID.Compute();
    double servo_out = servo_angle + 90;    // setpoint = 0 so for fan to be downwards, servo must be at 90 deg

  if (servo_angle < 20)
  {servo_angle = 20;}
  if (servo_angle > 160)
  {servo_angle = 160;}

  myservo.write(servo_out);
  // myservo.write(pitchCopy + 90);
  Serial.print("pitchCopy = ");
  Serial.print(pitchCopy);
  Serial.print("    ,    servo_out = ");
  Serial.println(servo_out);
  
  delay(200);
}
