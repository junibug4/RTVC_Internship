//Main .ino file

#include <PID_v1.h> 
#include <I2Cdev.h>
#include <MPU6050_6Axis_MotionApps20.h>

#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
#include <Wire.h>
#endif

bool blinkState = false;
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

void dmpDataReady() {
  mpuInterrupt = true;
}

void setup() // #########################  SETUP  ############################ //
{
  Serial.begin(115200);

  myPID.SetMode(AUTOMATIC);    // initialising myPID
  myPID.SetTunings(Kp, Ki, Kd);   
  myPID.SetOutputLimits(-100,-100);

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
      Serial.print("ypr\t");
      Serial.print(ypr[0] * 180/M_PI);
      Serial.print("\t");
      Serial.print(ypr[1] * 180/M_PI);
      Serial.print("\t");
      Serial.println(ypr[2] * 180/M_PI);

      blinkState = !blinkState;
      digitalWrite(LED_PIN, blinkState);
        
    }

  
  
  Serial.println(ypr[1])

  // ##############################  PITCH FOUND  ############################ //

  // PID changes to be added here


  delay(50);
}
