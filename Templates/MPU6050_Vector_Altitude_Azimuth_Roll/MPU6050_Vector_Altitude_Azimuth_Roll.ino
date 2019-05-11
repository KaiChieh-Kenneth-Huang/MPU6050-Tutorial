// Created by Kai-Chieh (Kenneth) Huang
// <kaichieh.kenneth.huang@gmail.com>
// Date 2019-05-10

// =============  Author's Notes  ======================
// To use this sketch, you must first intall I2Cdev and MPU6050 as libraries
// These libraries should be found here: https://github.com/jrowberg/i2cdevlib
//
// This is how you should wire MPU6050 http://301o583r8shhildde3s0vcnh.wpengine.netdna-cdn.com/wp-content/uploads/2014/11/conn.png
// (the SCL, SDA, and INT pins on MPU6050 connects to pins A4, A5, and 2 pins on the Arduino Uno.) 
// (You will not find the connection declarations for the SCL and SDA pins on this sketch.)
//
// This sketch serves as a template for creating projects using the MPU6050.
// Three classes are defined in this code for acquiring the altitude, azimuth, and roll (https://en.wikipedia.org/wiki/Azimuth)
// of the MPU6050 with respect to a self-assigned vector.
//
// WARNING: Avoid using delay in the loop() function. 
// A long delay may cause an overflow on MPU6050's buffer, and you will not be able to read from it.
// If your code in the loop() function takes a long time to run,
// try calling the readMPUFIFOBuffer() function more than once in the loop() function.
//
// The initial output from the Digital Motion Processor(DMP) right after reset is not reliable.
// So it is advised to start using them after a little while.
//
// 

// =====  Improvements Over Previous Reference Sketches  =========
// I used the two refenence sketches below and made the following refinements:
// 1. It is only safe to access memory(buffers) on MPU6050 immediately after the MPU6050 signals that data is ready
//    via the interrupt pin. (As far as my testing goes, both RISING and FALLING signals indicate such an event.)
//    Therefore, I added a line of code to ensure this happens, which fixes freezing issues when using the reference sketches below.
// 2. I removed code that I determined redundant, making the sketch more readable. Please refer to the reference
//    sketches below if you feel that I removed something you need.

// =============  Attribution ==========================
// Provided by "HC" aka "zhomeslice" on forum.arduino.cc
// at https://forum.arduino.cc/index.php?PHPSESSID=h4c6487i42hbb7uh6rjk0eadp1&topic=446713.msg3073854#msg3073854
// The above work most likely is based off of Jeff Rowberg's <jeff@rowberg.net> MPU6050_DMP6
// at https://github.com/jrowberg/i2cdevlib/tree/master/Arduino/MPU6050
// =====================================================

/* ============================================
I2Cdev device library code is placed under the MIT license
Copyright (c) 2012 Jeff Rowberg

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in
all copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
THE SOFTWARE.
===============================================
*/

#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps20.h"
#include "Wire.h"
MPU6050 mpu;

#define DEBUG
#ifdef DEBUG
//#define DPRINT(args...)  Serial.print(args)             //OR use the following syntax:
#define DPRINTSTIMER(t)    for (static unsigned long SpamTimer; (unsigned long)(millis() - SpamTimer) >= (t); SpamTimer = millis())
#define  DPRINTSFN(StrSize,Name,...) {char S[StrSize];Serial.print("\t");Serial.print(Name);Serial.print(" "); Serial.print(dtostrf((float)__VA_ARGS__ ,S));}//StringSize,Name,Variable,Spaces,Percision
#define DPRINTLN(...)      Serial.println(__VA_ARGS__)
#else
#define DPRINTSTIMER(t)    if(false)
#define DPRINTSFN(...)     //blank line
#define DPRINTLN(...)      //blank line
#endif



#define LED_PIN 13

// You may use MPU6050_calibration.ino (https://github.com/Protonerd/DIYino/blob/master/MPU6050_calibration.ino)
// to find the offest values for your MPU6050. 
// If you require high precision, you may wish to fine tune it by printing out some parameters.
//                       XA      YA      ZA      XG      YG      ZG
int MPUOffsets[6] = {  -5153,  -840,    855,     69,     -9,     18};


// ================================================================
// ===                      i2c SETUP Items                     ===
// ================================================================
void i2cSetup() {
  // join I2C bus (I2Cdev library doesn't do this automatically)
#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
  Wire.begin();
  TWBR = 24; // 400kHz I2C clock (200kHz if CPU is 8MHz)
#elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
  Fastwire::setup(400, true);
#endif
}

// ================================================================
// ===               INTERRUPT DETECTION ROUTINE                ===
// ================================================================
volatile bool mpuInterrupt = false;     // indicates whether MPU interrupt pin has changed
void dmpDataReady() {
  mpuInterrupt = true;
}

// ================================================================
// ===                      MPU DMP SETUP                       ===
// ================================================================
// MPU control/status vars
uint8_t mpuIntStatus;   // holds actual interrupt status byte from MPU
uint8_t devStatus;      // return status after each device operation (0 = success, !0 = error)
uint16_t packetSize;    // expected DMP packet size (default is 42 bytes)
uint16_t fifoCount;     // count of all bytes currently in FIFO
uint8_t fifoBuffer[64]; // FIFO storage buffer

// orientation/motion vars
Quaternion q;           // [w, x, y, z]         quaternion container
VectorInt16 aa;         // [x, y, z]            accel sensor measurements
VectorInt16 aaReal;     // [x, y, z]            gravity-free accel sensor measurements
VectorInt16 aaWorld;    // [x, y, z]            world-frame accel sensor measurements
VectorFloat gravity;    // [x, y, z]            gravity vector
float euler[3];         // [psi, theta, phi]    Euler angle container
float ypr[3];           // [yaw, pitch, roll]   yaw/pitch/roll container and gravity vector

void MPU6050Connect() {
  static int MPUInitCntr = 0;
  // initialize device
  mpu.initialize();
  // load and configure the DMP
  devStatus = mpu.dmpInitialize();

  if (devStatus != 0) {
    // ERROR!
    // 1 = initial memory load failed
    // 2 = DMP configuration updates failed
    // (if it's going to break, usually the code will be 1)

    char * StatStr[5] { "No Error", "initial memory load failed", "DMP configuration updates failed", "3", "4"};

    MPUInitCntr++;

    Serial.print(F("MPU connection Try #"));
    Serial.println(MPUInitCntr);
    Serial.print(F("DMP Initialization failed (code "));
    Serial.print(StatStr[devStatus]);
    Serial.println(F(")"));

    if (MPUInitCntr >= 10) return; //only try 10 times
    MPU6050Connect(); // Lets try again
    return;
  }
  mpu.setXAccelOffset(MPUOffsets[0]);
  mpu.setYAccelOffset(MPUOffsets[1]);
  mpu.setZAccelOffset(MPUOffsets[2]);
  mpu.setXGyroOffset(MPUOffsets[3]);
  mpu.setYGyroOffset(MPUOffsets[4]);
  mpu.setZGyroOffset(MPUOffsets[5]);

  Serial.println(F("Enabling DMP..."));
  mpu.setDMPEnabled(true);
  // enable Arduino interrupt detection
  Serial.println(F("Enabling interrupt detection (Arduino external interrupt pin 2 on the Uno)..."));
  Serial.print("mpu.getInterruptDrive=  "); Serial.println(mpu.getInterruptDrive());
  attachInterrupt(4, dmpDataReady, CHANGE); // pin 2 on the Uno. Please check the online Arduino reference for more options for connecting this interrupt pin
  // get expected DMP packet size for later comparison
  packetSize = mpu.dmpGetFIFOPacketSize();
  mpu.resetFIFO(); // Clear fifo buffer
  mpuInterrupt = false; // wait for next interrupt
}

// ================================================================
// ===                    MPU DMP Get Data                      ===
// ================================================================
void readMPUFIFOBuffer() {
  mpuIntStatus = mpu.getIntStatus();
  fifoCount = mpu.getFIFOCount();
  
  // Check for overflow
  if((mpuIntStatus & _BV(MPU6050_INTERRUPT_FIFO_OFLOW_BIT)) || fifoCount >= 1024) {
    // reset so we can continue cleanly
    mpu.resetFIFO();
    Serial.println(F("FIFO overflow!"));
    digitalWrite(LED_PIN, LOW); // lets turn off the blinking light so we can see we are failing.
    
  // check for incorrect packet size
  }else if((!fifoCount) || (fifoCount % packetSize)){ // something's wrong. reset and try again.
    Serial.println(F("Wrong packet size!"));
    digitalWrite(LED_PIN, LOW); // lets turn off the blinking light so we can see we are failing.
    mpu.resetFIFO();// clear the buffer and start over
    
  // otherwise, check for DMP data ready interrupt (this should happen almost always)
  }else if(mpuIntStatus & _BV(MPU6050_INTERRUPT_DMP_INT_BIT)){
    // wait for correct available data length, should be a VERY short wait
    while (fifoCount < packetSize) fifoCount = mpu.getFIFOCount();
    // read a packet from FIFO
    mpu.getFIFOBytes(fifoBuffer, packetSize);
    // discard the rest of the values(packets) inside buffer to prevent overflow. We don't need every single packet MPU6050 gives us.
    mpu.resetFIFO();

    digitalWrite(LED_PIN, !digitalRead(LED_PIN)); // Blink the LED to indicate activity
  }else{
    // This usually never happens.
  }
}

// ================================================================
// ===            ALTITUDE AZIMUTH ROLL PARAMETERS              ===
// ================================================================

// reference vectors for Altitude, Azimuth, and Roll
// Altitude, azimuth, and roll angles will be returned according to the reference vectors defined here.
// For example, you would assign (0,1,0) to refVectAlt1 if you desire to acquire the altitude of the y+ direction of the MPU6050
VectorFloat refVectAlt1(0,1,0); // (0,1,0) means the positive y direction is used as reference
VectorFloat refVectAzm1(0,1,0); // (0,1,0) means the positive y direction is used as reference
VectorFloat refVectRol1(0,1,0); // (0,1,0) means the positive y direction is used as reference

// reference orientation reset controllers 
// These control the reseting of the reference orientation to the current MPU orientation.
bool resetAzm1 = true;
bool resetRol1 = true;

// ================================================================
// ===              ALTITUDE AZIMUTH ROLL CLASSES               ===
// ================================================================

class Altitude {
  private:
    MPU6050 *_MPUptr;
    VectorFloat _gravityVector;
    VectorFloat _refVector;
    Quaternion *_qPtr;
 
  public:
    Altitude(MPU6050 *MPUptr, Quaternion *qPtr, VectorFloat refVector) {
      _MPUptr = MPUptr;
      _qPtr = qPtr;
      _refVector = refVector;

      _refVector.normalize();
    }

    // Member functions
    float getAltitudeAngle() // returns a value between -90 and 90. A return value of 0 means that the reference vector and gravity vector are perpendicular.
    {
      _MPUptr -> dmpGetGravity(&_gravityVector, _qPtr); // call helper function to get gravity vector from the quaternion "q". 'q' is the quaternion generated by the MPU6050 updated whenever data is ready
      return 90.0 - angleBetweenVectors(&_refVector, &_gravityVector); // gravity vector points upwards.
    }
};

class Azimuth {
  private:
    MPU6050 *_MPUptr;
    VectorFloat _gravityVector;
    VectorFloat _refVector;
    VectorFloat _projectedRefVector;
    Quaternion *_qPtr;
    
    Quaternion _i; // reference quaternion
    
 
  public:
    Azimuth(MPU6050 *MPUptr, Quaternion *qPtr, VectorFloat refVector) {
      _MPUptr = MPUptr;
      _qPtr = qPtr;
      _refVector = refVector;

      _refVector.normalize();
    }

    // Member functions
    void setRefOrientation()
    { //Sets the current azimuth as 0
    // Get a vector perpendicular to gravity and set as reference vector
      float dProduct;
    
      _i = *_qPtr;
      _MPUptr -> dmpGetGravity(&_gravityVector, _qPtr); // call helper function to get gravity vector from the quaternion "q"
      // Project the reference vector onto the horizontal plane. 
      // projection(u) = u - (u dot n / abs(n)^2)*n, where n is the normal vector to the plane (which is the unit gravity vector in this case. This means abs(n)^2 = 1)
      dProduct = dotProduct(&_refVector, &_gravityVector);
      _projectedRefVector.x = _refVector.x - dProduct*_gravityVector.x; 
      _projectedRefVector.y = _refVector.y - dProduct*_gravityVector.y;
      _projectedRefVector.z = _refVector.z - dProduct*_gravityVector.z;
      _projectedRefVector.normalize();
      return;
    }
    
    float getAzimuthAngle()
    {
      Quaternion d; // rotation quaternion
      VectorFloat newVector, projectedVector;
      float dProduct, angle;
      
      // Get the rotation quaternion with respect to reference
      d = _i.getConjugate();
      d = d.getProduct(*_qPtr);
      // Rotate projected reference vector by the rotation quaternion
      newVector = _refVector.getRotated(&d);
      // Project the new vector onto the horizontal plane
      // projection(u) = u - (u dot n / abs(n)^2)*n, where n is the normal vector to the plane (which is the unit gravity vector in this case. This means abs(n)^2 = 1)
      dProduct = dotProduct(&newVector, &_gravityVector);
      projectedVector.x = newVector.x - dProduct*_gravityVector.x; 
      projectedVector.y = newVector.y - dProduct*_gravityVector.y;
      projectedVector.z = newVector.z - dProduct*_gravityVector.z;
      // Find the degree difference between the projected vector and the projected reference vector
      angle = angleBetweenVectors(&_projectedRefVector, &projectedVector);
      // Find the turning direction with cross product of _projectedRefVector and projectedVector then compare it to the direction of gravity vector
      if(angle != 180 && (_gravityVector.x * (_projectedRefVector.y * projectedVector.z - _projectedRefVector.z * projectedVector.y) > 0 || _gravityVector.y * (_projectedRefVector.z * projectedVector.x - _projectedRefVector.x * projectedVector.z) > 0 || _gravityVector.z * (_projectedRefVector.x * projectedVector.y - _projectedRefVector.y * projectedVector.x) > 0))
      {
        // Counter clock-wise when viewed from above
        return angle;
      }else{
        // Clock-wise when viewed from above
        return 360.0 - angle;
      }
    }
};

class Roll {
  private:
    MPU6050 *_MPUptr;
    VectorFloat _gravityVector;
    VectorFloat _refVector;
    VectorFloat _orthogonalRefVector; // a reference vector orthogonal to _refVector (a vector on a plane whose normal vector is _refVector)
    Quaternion *_qPtr;
    
    Quaternion _i; // reference quaternion
    
 
  public:
    Roll(MPU6050 *MPUptr, Quaternion *qPtr, VectorFloat refVector) {
      _MPUptr = MPUptr;
      _qPtr = qPtr;
      _refVector = refVector;

      _refVector.normalize();
    }

    // Member functions
    void setRefOrientation()
    { // Sets the current roll as 0
      float absX, absY, absZ;
      
      _i = *_qPtr;
      // Find a vector orthogonal to _refVector while ensuring that there's no singularities
      absX = abs(_refVector.x);
      absY = abs(_refVector.y);
      absZ = abs(_refVector.z);
      if(absX < absY && absX < absZ){ // _refVector(ai + bj + ck) dot _orthogonalRefVector(0i + cj - bk) = 0
        _orthogonalRefVector.x = 0;
        _orthogonalRefVector.y = _refVector.z;
        _orthogonalRefVector.z = -_refVector.y;
      }else if(absY < absX && absY < absZ){ // _refVector(ai + bj + ck) dot _orthogonalRefVector(ci + 0j - ak) = 0
        _orthogonalRefVector.x = _refVector.z;
        _orthogonalRefVector.y = 0;
        _orthogonalRefVector.z = -_refVector.x;
      }else{ // _refVector(ai + bj + ck) dot _orthogonalRefVector(bi - aj + 0k) = 0
        _orthogonalRefVector.x = _refVector.y;
        _orthogonalRefVector.y = -_refVector.x;
        _orthogonalRefVector.z = 0;
      }
      _orthogonalRefVector.normalize(); // make the absolute value of the vector 1
      return;
    }
   
    float getRollAngle()
    {
      Quaternion d; // rotation quaternion
      VectorFloat newVector, projectedVector;
      float dProduct, angle;
      
      // Get the rotation quaternion with respect to reference
      d = _i.getConjugate();
      d = d.getProduct(*_qPtr);
      // Rotate orthogonal reference vector by the rotation quaternion
      newVector = _orthogonalRefVector.getRotated(&d);
      // Project the new vector onto a plane perpendicular to the reference vector
      // projection(u) = u - (u dot n / abs(n)^2)*n, where n is the normal vector to the plane (which is the unit _refVector in this case. This means abs(n)^2 = 1)
      dProduct = dotProduct(&newVector, &_refVector);
      projectedVector.x = newVector.x - dProduct*_refVector.x; 
      projectedVector.y = newVector.y - dProduct*_refVector.y;
      projectedVector.z = newVector.z - dProduct*_refVector.z;
      // Find the degree difference between the projected vector and the orthogonal reference vector
      angle = angleBetweenVectors(&_orthogonalRefVector, &projectedVector);
      // Find the turning direction with cross product then compare it to reference vector
       if(angle != 180 && (_refVector.x * (_orthogonalRefVector.y * projectedVector.z - _orthogonalRefVector.z * projectedVector.y) > 0 || _refVector.y * (_orthogonalRefVector.z * projectedVector.x - _orthogonalRefVector.x * projectedVector.z ) > 0 || _refVector.z * (_orthogonalRefVector.x * projectedVector.y - _orthogonalRefVector.y * projectedVector.x) > 0)) // the second term should be > 0 as well... But for some reason the result is the reverse.
      {
        // Counter clock-wise
        return angle;
      }else{
        // clock-wise
        return 360.0 - angle;
      }
    }
};

// helper functions
float dotProduct(VectorFloat *v1, VectorFloat *v2) // helper_3dmath.h does not include a member function for VectorFloat dot product operation, thus I created my own
{
  return v1->x * v2->x + v1->y * v2->y + v1->z * v2->z;
}

float angleBetweenVectors(VectorFloat *v1, VectorFloat *v2) // helper_3dmath.h does not include a member function for VectorFloat dot product operation, thus I created my own
{
  //Formula of angle between two vectors: angle θ = arccosine((u dot v) / (abs(u)*abs(v)))
  float val = dotProduct(v1, v2) / (v1->getMagnitude() * v2->getMagnitude());
  if(val > 1){ // prevents value from accidentally going out of range
    return 0.0;
  }else if(val < -1){
    return 180.0;
  }else{
    return acos(val) * 180 / M_PI;
  }
}

// Object instantiation
// You can create multiple instances of each class with different reference vectors.
Altitude alt1(&mpu, &q, refVectAlt1);
Azimuth azm1(&mpu, &q, refVectAzm1);
Roll rol1(&mpu, &q, refVectRol1);

// ================================================================
// ===                    Output Functions                      ===
// ================================================================
// add these functions to your code as needed

// get quaternion components in a [w, x, y, z] format
// very useful when Euler and YPR angles cannot satisfy your application
// refer to https://en.wikipedia.org/wiki/Quaternion for more information
void getQuaternion()
{
  mpu.dmpGetQuaternion(&q, fifoBuffer);
}
void printQuaternion()
{
  // display quaternion values in easy matrix form: w x y z
  Serial.print("quat\t");
  Serial.print(q.w);
  Serial.print("\t");
  Serial.print(q.x);
  Serial.print("\t");
  Serial.print(q.y);
  Serial.print("\t");
  Serial.println(q.z);
}

// The Euler angles are in radians. Divide it by M_PI then multiply it by 180 to get angles in degrees.
// Note that Euler angles suffer from gimbal lock (for more info, see http://en.wikipedia.org/wiki/Gimbal_lock)
// Try calculating your parameters from quaternions if you experience gimbal lock.
void getEuler()
{
  mpu.dmpGetQuaternion(&q, fifoBuffer);
  mpu.dmpGetEuler(euler, &q);
}
void printEuler()
{
  // display Euler angles in degrees
  Serial.print("euler\t");
  Serial.print(euler[0] * 180/M_PI);
  Serial.print("\t");
  Serial.print(euler[1] * 180/M_PI);
  Serial.print("\t");
  Serial.println(euler[2] * 180/M_PI);
}

// Yaw, pitch, and roll angles are in radians. Divide it by M_PI then multiply it by 180 to get angles in degrees.
// Note that these angles suffer from gimbal lock (for more info, see http://en.wikipedia.org/wiki/Gimbal_lock)
// Try calculating your parameters from quaternions if you experience gimbal lock.
void getYawPitchRoll()
{
  mpu.dmpGetQuaternion(&q, fifoBuffer);
  mpu.dmpGetGravity(&gravity, &q);
  mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
}
void printYawPitchRoll()
{
  // display yaw, pitch, roll in degrees
  Serial.print("ypr\t");
  Serial.print(ypr[0] * 180/M_PI);
  Serial.print("\t");
  Serial.print(ypr[1] * 180/M_PI);
  Serial.print("\t");
  Serial.println(ypr[2] * 180/M_PI);
}

// Use this if you want to see acceleration
// components with gravity removed. This acceleration reference frame is
// not compensated for orientation, so +X is always +X according to the
// sensor, just without the effects of gravity. If you want acceleration
// compensated for orientation, use getWorldAccel() instead.
void getRealAccel()
{
  mpu.dmpGetQuaternion(&q, fifoBuffer);
  mpu.dmpGetAccel(&aa, fifoBuffer);
  mpu.dmpGetGravity(&gravity, &q);
  mpu.dmpGetLinearAccel(&aaReal, &aa, &gravity);
}
void printRealAccel()
{
  Serial.print("areal\t");
  Serial.print(aaReal.x);
  Serial.print("\t");
  Serial.print(aaReal.y);
  Serial.print("\t");
  Serial.println(aaReal.z);
}

// Use this if you want to see acceleration
// components with gravity removed and adjusted for the world frame of
// reference (yaw is relative to initial orientation, since no magnetometer
// is present in this case). Could be quite handy in some cases.
void getWorldAccel()
{
  mpu.dmpGetQuaternion(&q, fifoBuffer);
  mpu.dmpGetAccel(&aa, fifoBuffer);
  mpu.dmpGetGravity(&gravity, &q);
  mpu.dmpGetLinearAccel(&aaReal, &aa, &gravity);
  mpu.dmpGetLinearAccelInWorld(&aaWorld, &aaReal, &q);
}
void printWorldAccel()
{
  Serial.print("aworld\t");
  Serial.print(aaWorld.x);
  Serial.print("\t");
  Serial.print(aaWorld.y);
  Serial.print("\t");
  Serial.println(aaWorld.z);
}

// ================================================================
// ===                         Setup                            ===
// ================================================================
void setup() {
  Serial.begin(115200); //115200
  while (!Serial);
  Serial.println("i2cSetup");
  i2cSetup();
  Serial.println("MPU6050Connect");
  MPU6050Connect();
  Serial.println("Setup complete");
  pinMode(LED_PIN, OUTPUT);
}
// ================================================================
// ===                          Loop                            ===
// ================================================================
void loop() {
  // ---- read from the FIFO buffer ---- //
  // The interrupt pin could have changed for some time already. 
  // So set mpuInterrupt to false and wait for the next interrupt pin CHANGE signal.
  mpuInterrupt = false; 
  //  Wait until the next interrupt signal. This ensures the buffer is read right after the signal change.
  while(!mpuInterrupt){ /* If you have something that cannot wait and can be completed EXTREMELY quickly, you can put it here. Otherwise, leave it blank. */}
  mpuInterrupt = false;
  readMPUFIFOBuffer();
  
  // Calculate variables of interest using the acquired values from the FIFO buffer
  getQuaternion(); // Altitude, Azimuth, and Roll relies on quaternion for calculation. Thus we must update the quaternion every time

  // ==========  Your code goes here  ========= //
  
  // It takes some time for values from the MPU6050 to stabilize.
  // I have only added a 5 second wait here.
  // Feel free to extend, decrease, or remove the wait.
  if(millis() < 5000){
    // Waiting for MPU readings to stabilize...
    Serial.print("Reset reference orientation in ");
    Serial.print(5 - (int)millis()/1000);
    Serial.println("s");
    resetAzm1 = true;
    resetRol1 = true;
  }else if(millis() > 5000){
    if(resetAzm1){
      azm1.setRefOrientation();
      resetAzm1 = false;
    }
    if(resetRol1){
      rol1.setRefOrientation();
      resetRol1 = false;
    }
    Serial.print(alt1.getAltitudeAngle());
    Serial.print(',');
    Serial.print(azm1.getAzimuthAngle());
    Serial.print(',');
    Serial.println(rol1.getRollAngle());
  }
}
