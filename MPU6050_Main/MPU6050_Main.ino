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

MPU6050 accelgyro(0x68);  //Declare memory address of sensor, all sensors will be at 0x68 and will be cycled through by the mux
const int sensorNum = 2;  //Number of sensors connected, supports up to 8
const int buttonPin = 2;  // the number of the pushbutton pin, used for calibration
const int switchPin = 3;  // the number of the switch pin, used for modes
int buttonState = 0;      // variable for reading the pushbutton status
int modeState = 0;      // variable for reading the pushbutton status
int degTime15, degTime5;

//Declare variables needed
int16_t ax[sensorNum], ay[sensorNum], az[sensorNum], ax1[sensorNum], ay1[sensorNum], az1[sensorNum];
int16_t gx[sensorNum], gy[sensorNum], gz[sensorNum], gx1[sensorNum], gy1[sensorNum], gz1[sensorNum];

double gyroXangle[sensorNum], gyroYangle[sensorNum]; // Angle calculate using the gyro only
double compAngleX[sensorNum], compAngleY[sensorNum]; // Calculated angle using a complementary filter
double roll[sensorNum], pitch[sensorNum], gyroXrate[sensorNum], gyroYrate[sensorNum], dt[sensorNum];
double offsetY[sensorNum], offsetX[sensorNum], angleX[sensorNum], angleY[sensorNum];

uint32_t derivativestartTime[sensorNum], startTime1, startTime2, startTimeSMA; //startTime 1 used for >15deg, startTime2 used for 5<deg<15, startTimeSMA used for actuating SMA

#define LED_PIN 13
bool blinkState = false;
bool finishCalibration=false; 
bool standing = false;
bool sitting = false;

//Used to cycle through sensors connected to mux. 0-7
void tcaselect(uint8_t i) {
  if (i > 7) return; 
  Wire.beginTransmission(0x70);
  Wire.write(1 << i);
  Wire.endTransmission();  
}

void setup() {
  //join I2C bus (I2Cdev library doesn't do this automatically)
  #if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
      Wire.begin();
  #elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
      Fastwire::setup(400, true);
  #endif

  //initialize serial communication
  //38400 chosen because it works as well at 8MHz as it does at 16MHz
  Serial.begin(38400);

  // initialize the pushbutton pin as an input:
  pinMode(buttonPin, INPUT);

  //initialize devices
  Serial.println("Initializing I2C devices...");
  
  //Cycle through each sensor in mux
  for(int i=0;i<sensorNum;i++){
    tcaselect(i);
    accelgyro.initialize();  
    Serial.print("Done Initializing I2C devices...");Serial.println(i);//individually init each sensor

    // verify connection
    Serial.print("Testing device connections..."); Serial.println(i);    
    Serial.println(accelgyro.testConnection() ? "MPU6050 connection successful" : "MPU6050 connection failed");               
  
    derivativestartTime[i] = micros();
  }
    Serial.println("\n\nPlease calibrate sensors once in a upright posture");
    Serial.println("Choose sitting mode(switch 1) or standing mode(switch 2)");
}

void loop() {
  // read raw accel/gyro measurements from device
  for(int i=0;i<sensorNum;i++){
    tcaselect(i);
    accelgyro.getMotion6(&ax[i], &ay[i], &az[i], &gx[i], &gy[i], &gz[i]);          

    //Calculate delta time
    dt[i] = (micros() - derivativestartTime[i]) / 1000000; // Calculate delta time
    derivativestartTime[i] = micros();

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
    angleX[i] = compAngleX[i]-offsetX[i];
    angleY[i] = compAngleY[i]-offsetY[i];
  
    // Reset the gyro angle when it has drifted too much
    if (gyroXangle[i] < -180 || gyroXangle[i] > 180)
      gyroXangle[i] = compAngleX[i];
    if (gyroYangle[i] < -180 || gyroYangle[i] > 180)
      gyroYangle[i] = compAngleY[i];

    //printAngles();

    //Calculate offset values when button pressed
    buttonState = digitalRead(buttonPin);
    if (buttonState == HIGH) {
      Serial.print("calibrating offset\n");
      calibration();
      }      

    //ditermine sitting or standing mode
    modeState = digitalRead(switchPin);
    if (modeState == HIGH) {
      //Serial.print("sitting\n");
      degTime15 = 30000;
      degTime5 = 120000;
      }
      
    if (modeState == LOW) {
      //Serial.print("standing\n");
      degTime15 = 60000;
      degTime5 = 300000;
      }    

    //If user's posture is >15degrees for 30seconds or more 
    if((abs(angleX[i]) > 15.0 || abs(angleY[i]) > 15.0) && finishCalibration){
      Serial.print(">15deg\n");
      if(((millis()-startTime1) >=degTime15)){
         Serial.print("Activating SMA for 30sec\n");
         activateSMA();
        }
      }    
    if(abs(angleX[i]) < 15.0 && abs(angleY[i]) < 15.0){
      startTime1=millis();
      //Serial.print("Start Counting1 \n");
      }   

    //If user's posture 5<deg<15 for 2mins or more
    if((abs(angleX[i]) > 5.0 || abs(angleY[i]) > 5.0) && (((abs(angleX[i]) < 15.0) && (abs(angleY[i]) < 15.0))) && finishCalibration){
      Serial.print("5<deg<15\n");
      if(((millis()-startTime2) >=degTime5)){
         Serial.print("Activating SMA for 30sec\n");
         activateSMA();         
        }
      }
    if(abs(angleX[i]) < 5.0 && abs(angleY[i]) < 5.0){
      startTime2=millis();
      Serial.print("<5deg\n");
      } 
        
  } 
  delay(25);//Small delay so sensors are not constantly polled 
  // blink LED to indicate activity
  blinkState = !blinkState;
  digitalWrite(LED_PIN, blinkState);

}

///////////////////////////////////   FUNCTIONS   ////////////////////////////////////

void calibration(){
  for(int i=0;i<sensorNum;i++){
    tcaselect(i);
    //Get offsets  
    offsetX[i]=compAngleX[i];
    offsetY[i]=compAngleY[i];  
    angleX[i] = compAngleX[i]-offsetX[i];
    angleY[i] = compAngleY[i]-offsetY[i];
    finishCalibration=true;
   }
}

void printAngles(){
  for(int i=0;i<sensorNum;i++){
    tcaselect(i);    
    Serial.print("Complimentary angle X");Serial.print(i);Serial.print(": ");
    Serial.print(angleX[i]); Serial.print("\t");
    Serial.print("\t");
  
    Serial.print("Complimentary angle Y");Serial.print(i);Serial.print(": ");
    Serial.print(angleY[i]); Serial.print("\t");
    Serial.print("\n");
  }
}

void printGryoValues(){
  for(int i=0;i<sensorNum;i++){
    tcaselect(i);     
      Serial.println("\t");
      Serial.print("ax");Serial.print(i);Serial.print(": ");
      Serial.print(ax[i]); Serial.print("\t");
      Serial.print("ay");Serial.print(i);Serial.print(": ");
      Serial.print(ay[i]); Serial.print("\t");
      Serial.print("az");Serial.print(i);Serial.print(": ");
      Serial.print(az[i]); Serial.println("\t");
  }
}

void elapsedTime(){
   
}

void activateSMA(){
   startTimeSMA = millis();
   while(millis()-startTimeSMA <= 30000){
     //Serial.print("Activate SMA for 30sec\n");  
   }
  startTime1=millis();
  startTime2=millis();
  Serial.print("Done activating\n");
}


