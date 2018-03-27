#ifndef CharliePlexing_h
#define CharliePlexing_h

#include <MsTimer2.h>
		
namespace CharliePlexing {
  extern volatile unsigned char uc_LED_Output_Image;
  extern volatile unsigned char uc_PortB_Input;
	
  extern volatile unsigned char uc_ButtonImageInput;
  extern volatile unsigned char uc_VerticalButtonCounter_B;
  extern volatile unsigned char uc_VerticalButtonCounter_A;
  extern volatile unsigned char uc_ButtonImageWorking;
  extern volatile unsigned char uc_ButtonImageOutput;
	
  extern volatile unsigned long ul_LeftEncoder_Count;
  extern volatile unsigned long ul_RightEncoder_Count;
	
  void set();
  void CharliePlex();
  void Write(unsigned char uc_LED_Number,unsigned char uc_On_Off);
}

#endif
