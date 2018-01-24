/*
 * Name: William Choi & Danny Loo
 * Version: Pseudo
 * Purpose: I2C address shifter for MPU
 * 
 * 
 * 
 */

int ind = 0;

 // Make arrays of digital pins to manipulate the AD0 pins of six IMU's 

int d[] = {2, 3, 4, 5, 6, 7};

void setup() {

Serial.begin(115200);
pinMode(d[0], OUTPUT);
pinMode(d[1], OUTPUT);
pinMode(d[2], OUTPUT);
pinMode(d[3], OUTPUT);
pinMode(d[4], OUTPUT);
pinMode(d[5], OUTPUT);


}

void loop() {
  digitalWrite(d[ind], HIGH);  //set the AD0 pin of the first IMU to high
  Serial.println(d[ind]);
  delay(200);

  
  digitalWrite(d[ind], LOW);
  ind = (ind+1)%6;
  


}

