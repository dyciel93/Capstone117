/*
By: Danny Loo & William Choi

The following code addresses up to 8 MPU6050 sensors at address 0x68 and displays the complimentary angles of each. A compplimentary filter was chosen as it is the easiest and gives similar results to a kalman filter.
A mux is used to cycle through each sensor.

*/

// I2Cdev and MPU6050 must be installed as libraries, or else the .cpp/.h files
// for both classes must be in the include path of your project
#include "I2Cdev.h"
#include "MPU6050.h"

// Arduino Wire library is required if I2Cdev I2CDEV_ARDUINO_WIRE implementation
// is used in I2Cdev.h
#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
#include "Wire.h"
#endif

//#define OUTPUT_READABLE_ACCELGYRO//Uncomment to display ax, ay, az values of sensors

MPU6050 accelgyro(0x68);//Declare memory address of sensor, all sensors will be at 0x68 and will be cycled through by the mux
const int sensorNum = 3;//Number of sensors connected, supports up to 8

//Declare variables needed
int16_t ax[sensorNum], ay[sensorNum], az[sensorNum], ax1[sensorNum], ay1[sensorNum], az1[sensorNum];
int16_t gx[sensorNum], gy[sensorNum], gz[sensorNum], gx1[sensorNum], gy1[sensorNum], gz1[sensorNum];

double gyroXangle[sensorNum], gyroYangle[sensorNum]; // Angle calculate using the gyro only
double compAngleX[sensorNum], compAngleY[sensorNum]; // Calculated angle using a complementary filter
double roll[sensorNum], pitch[sensorNum], gyroXrate[sensorNum], gyroYrate[sensorNum], dt[sensorNum];

uint32_t timer[sensorNum];

#define LED_PIN 13
bool blinkState = false;

//Used to cycle through sensors connected to mux. 0-7
void tcaselect(uint8_t i) {
  if (i > 7) return;
 
  Wire.beginTransmission(0x70);
  Wire.write(1 << i);
  Wire.endTransmission();  
}

void setup() {
    // join I2C bus (I2Cdev library doesn't do this automatically)
    #if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
        Wire.begin();
    #elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
        Fastwire::setup(400, true);
    #endif

    // initialize serial communication
    // 38400 chosen because it works as well at 8MHz as it does at 16MHz
    Serial.begin(38400);

    // initialize device
    Serial.println("Initializing I2C devices...");
    for(int i=0;i<sensorNum;i++){
      tcaselect(i);
      accelgyro.initialize();  
      Serial.println("Done Initializing I2C devices...");
  
      // verify connection
      Serial.println("Testing device connections...");   
      Serial.println(accelgyro.testConnection() ? "MPU6050 connection successful" : "MPU6050 connection failed");               
        
      // use the code below to change accel/gyro offset values
      /*
      Serial.println("Updating internal sensor offsets...");
      // -76	-2359	1688	0	0	0
      Serial.print(accelgyro.getXAccelOffset()); Serial.print("\t"); // -76
      Serial.print(accelgyro.getYAccelOffset()); Serial.print("\t"); // -2359
      Serial.print(accelgyro.getZAccelOffset()); Serial.print("\t"); // 1688
      Serial.print(accelgyro.getXGyroOffset()); Serial.print("\t"); // 0
      Serial.print(accelgyro.getYGyroOffset()); Serial.print("\t"); // 0
      Serial.print(accelgyro.getZGyroOffset()); Serial.print("\t"); // 0
      Serial.print("\n");
      accelgyro.setXGyroOffset(220);
      accelgyro.setYGyroOffset(76);
      accelgyro.setZGyroOffset(-85);
      Serial.print(accelgyro.getXAccelOffset()); Serial.print("\t"); // -76
      Serial.print(accelgyro.getYAccelOffset()); Serial.print("\t"); // -2359
      Serial.print(accelgyro.getZAccelOffset()); Serial.print("\t"); // 1688
      Serial.print(accelgyro.getXGyroOffset()); Serial.print("\t"); // 0
      Serial.print(accelgyro.getYGyroOffset()); Serial.print("\t"); // 0
      Serial.print(accelgyro.getZGyroOffset()); Serial.print("\t"); // 0
      Serial.print("\n");
      */    
        timer[i] = micros();
        // configure Arduino LED for
        pinMode(LED_PIN, OUTPUT);
    }
}

void loop() {
    // read raw accel/gyro measurements from device
    for(int i=0;i<sensorNum;i++){
      tcaselect(i);
      accelgyro.getMotion6(&ax[i], &ay[i], &az[i], &gx[i], &gy[i], &gz[i]);          
  
      //Calculate delta time
      dt[i] = (micros() - timer[i]) / 1000000; // Calculate delta time
      timer[i] = micros();
      
      // these methods (and a few others) are also available
      //accelgyro.getAcceleration(&ax, &ay, &az);
      //accelgyro.getRotation(&gx, &gy, &gz);

      #ifdef OUTPUT_READABLE_ACCELGYRO
      // display tab-separated accel/gyro x/y/z values    
        Serial.println("\t");
        Serial.print("ax");Serial.print(i);Serial.print(": ");
        Serial.print(ax[i]); Serial.print("\t");
        Serial.print("ay");Serial.print(i);Serial.print(": ");
        Serial.print(ay[i]); Serial.print("\t");
        Serial.print("az");Serial.print(i);Serial.print(": ");
        Serial.print(az[i]); Serial.println("\t");        
      #endif

      //Calculate roll and pitch, used in comp filter
      roll[i]  = atan2(ay[i], az[i]) * RAD_TO_DEG;
      pitch[i] = atan2(-ax[i], az[i]) * RAD_TO_DEG;
      gyroXrate[i] = gx[i] / 131.0; // Convert to deg/s
      gyroYrate[i] = gy[i] / 131.0; // Convert to deg/s
    
      // This fixes the transition problem when the accelerometer angle jumps between -180 and 180 degrees      
      if ((roll[i] < -90 && compAngleX[i] > 90) || (roll[i] > 90 && compAngleX[i] < -90)) {
        compAngleX[i] = roll[i];
        gyroXangle[i] = roll[i];
      } 
      
      // Calculate gyro angle without any filter
      gyroXangle[i] += gyroXrate[i] * dt[i]; 
      gyroYangle[i] += gyroYrate[i] * dt[i];

      //Calculate comp angles     
      compAngleX[i] = 0.93 * (compAngleX[i] + gyroXrate[i] * dt[i]) + 0.07 * roll[i]; // Calculate the angle using a Complimentary filter
      compAngleY[i] = 0.93 * (compAngleY[i] + gyroYrate[i] * dt[i]) + 0.07 * pitch[i];
    
      // Reset the gyro angle when it has drifted too much
      if (gyroXangle[i] < -180 || gyroXangle[i] > 180)
        gyroXangle[i] = compAngleX[i];
      if (gyroYangle[i] < -180 || gyroYangle[i] > 180)
        gyroYangle[i] = compAngleY[i];

      Serial.print("Comp X");Serial.print(i);Serial.print(": ");
      Serial.print(compAngleX[i]); Serial.print("\t");
      Serial.print("\t");
    
      Serial.print("Comp Y");Serial.print(i);Serial.print(": ");
      Serial.print(compAngleY[i]); Serial.print("\t");
      Serial.print("\n");
    }
 
    delay(25);
    // blink LED to indicate activity
    blinkState = !blinkState;
    digitalWrite(LED_PIN, blinkState);
}
