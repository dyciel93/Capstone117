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

 int D[] = {D1, D2, D3, D4, D5, D6};

 while(1)
{
  D[ind] = 1;  //set the AD0 pin of the first IMU to high
  x = read.address(0x69);
  D[ind] = 0;
  ind = (ind+1)%7;
  


  
 }

