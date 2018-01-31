/*
By: Danny Loo & William Choi

The following code addresses two MPU6050 sensors at address 0x68 and 0x69 and displays the complimentary angles of each. A compplimentary filter was chosen as it is the easiest and gives similar results to a kalman filter.

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

#define OUTPUT_READABLE_ACCELGYRO//Uncomment to display ax, ay, az values of both sensors

MPU6050 accelgyro[2];
//MPU6050 accelgyro[0](0x68);
//MPU6050 accelgyro[1](0x69);
//accelgyro[0].setMemoryStartAddress(0x68);
//accelgyro[1].setMemoryStartAddress(0x69);
//MPU6050 accelgyro(0x68);
//MPU6050 accelgyro1(0x69);


int16_t ax[2], ay[2], az[2], ax1[2], ay1[2], az1[2];
int16_t gx[2], gy[2], gz[2], gx1[2], gy1[2], gz1[2];

double gyroXangle, gyroYangle; // Angle calculate using the gyro only
double compAngleX, compAngleY; // Calculated angle using a complementary filter

uint32_t timer;


#define LED_PIN 13
bool blinkState = false;

void setup() {
accelgyro[0].setSlave4Address(0x68);
accelgyro[1].setSlave4Address(0x69);
    // join I2C bus (I2Cdev library doesn't do this automatically)
    #if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
        Wire.begin();
    #elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
        Fastwire::setup(400, true);
    #endif

    // initialize serial communication
    // (38400 chosen because it works as well at 8MHz as it does at 16MHz, but
    // it's really up to you depending on your project)
    Serial.begin(38400);

    // initialize device
    Serial.println("Initializing I2C devices...");
    for(int i=0;i<2;i++){
      accelgyro[i].initialize();
    }
    Serial.println("Done Initializing I2C devices...");

    // verify connection
    Serial.println("Testing device connections...");    
    for(int i=0;i<2;i++){
      Serial.println(accelgyro[i].testConnection() ? "MPU6050 connection successful" : "MPU6050 connection failed");
    }
    
      
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
    
    timer = micros();
    // configure Arduino LED for
    pinMode(LED_PIN, OUTPUT);
}

void loop() {
    // read raw accel/gyro measurements from device
    for(int i=0;i<2;i++){
      accelgyro[i].getMotion6(&ax[i], &ay[i], &az[i], &gx[i], &gy[i], &gz[i]);
    }
    
    double dt = (double)(micros() - timer) / 1000000; // Calculate delta time
    timer = micros();
    
    // these methods (and a few others) are also available
    //accelgyro.getAcceleration(&ax, &ay, &az);
    //accelgyro.getRotation(&gx, &gy, &gz);

    #ifdef OUTPUT_READABLE_ACCELGYRO
        // display tab-separated accel/gyro x/y/z values
        Serial.println("\t");
        Serial.print("ax0:");
        Serial.print(ax[0]); Serial.print("\t");
        Serial.print("ay0:");
        Serial.print(ay[0]); Serial.print("\t");
        Serial.print("az0:");
        Serial.print(az[0]); Serial.println("\t");
        
        Serial.print("ax1:");
        Serial.print(ax[1]); Serial.print("\t");
        Serial.print("ay1:");
        Serial.print(ay[1]); Serial.print("\t");
        Serial.print("az1:");
        Serial.print(az[1]); Serial.println("\t");

        
        //Serial.print(gx); Serial.print("\t");
        //Serial.print(gy); Serial.print("\t");
        //Serial.println(gz);
    #endif


    double roll  = atan2(ay[0], az[0]) * RAD_TO_DEG;
    //double pitch = atan(-ax / sqrt(ay * ay + az * az)) * RAD_TO_DEG;
    double pitch = atan2(-ax[0], az[0]) * RAD_TO_DEG;
    
    double gyroXrate = gx[0] / 131.0; // Convert to deg/s
    double gyroYrate = gy[0] / 131.0; // Convert to deg/s

    // This fixes the transition problem when the accelerometer angle jumps between -180 and 180 degrees
    if ((roll < -90 && compAngleX > 90) || (roll > 90 && compAngleX < -90)) {
      compAngleX = roll;
      gyroXangle = roll;
    } 

    gyroXangle += gyroXrate * dt; // Calculate gyro angle without any filter
    gyroYangle += gyroYrate * dt;
  
    compAngleX = 0.93 * (compAngleX + gyroXrate * dt) + 0.07 * roll; // Calculate the angle using a Complimentary filter
    compAngleY = 0.93 * (compAngleY + gyroYrate * dt) + 0.07 * pitch;
  
    // Reset the gyro angle when it has drifted too much
    if (gyroXangle < -180 || gyroXangle > 180)
      gyroXangle = compAngleX;
    if (gyroYangle < -180 || gyroYangle > 180)
      gyroYangle = compAngleY;

  
//    Serial.print("Comp X: ");
//    Serial.print(compAngleX); Serial.print("\t");
//    Serial.print("\t");
//  
//    Serial.print("Comp Y: ");
//    Serial.print(compAngleY); Serial.print("\t");
//    Serial.print("\n");
  
      
    delay(25);
    // blink LED to indicate activity
    blinkState = !blinkState;
    digitalWrite(LED_PIN, blinkState);
}
