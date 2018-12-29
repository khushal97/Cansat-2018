#ifndef  Xbee_API_avr
#define  Xbee_API_avr


#include <string.h>
#include <SoftwareSerial.h>
#include <inttypes.h>
#include "Arduino.h"

#define XbeeSerial Serial3

class Xbee
{
	private:
		unsigned char Adrr[8];
		unsigned char Adrr_CheckSum;
		
	public:
		void SetUSART(int a);
		void SetAdrr(uint64_t Device);
		void SendData(uint8_t data[]);
		void SendAtCmnd(char para_value[], char atcmd[]);
		
		int i;
	
};


#endif
