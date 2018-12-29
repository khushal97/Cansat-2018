#include "Xbee_API_avr.h"
#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <string.h>

void Xbee::SetUSART(int a)
{
	/*if(a==1)
		#define XbeeSerial Serial1;
	else if(a==2)
		#define XbeeSerial Serial2;
	else if(a==3)
		#define XbeeSerial Serial3;
	else if(a==4)
		#define XbeeSerial Serial4;
	else if(a==5)
		#define XbeeSerial Serial5;
	else if(a==6)
		#define XbeeSerial Serial6;
	*/
}

void Xbee::SetAdrr(uint64_t Device)
{
	
	Adrr_CheckSum = 0;
	
	for (i=0;i<8;i++)
	{
		Adrr[i] = Device >> (8*(7-i));		//Add address to char array 
		Adrr_CheckSum += Adrr[i];
	}
}

void Xbee::SendData(uint8_t data[])
{
	//Serial1.begin(9600);
	//unsigned int i;
	static unsigned int ID;
	int s;
	s = strlen((const char*)data);
	unsigned int length;
	length = s + 18;				//Define Frame length
	unsigned char Frame[length];
	unsigned int CheckSum;
	
	CheckSum = 0;
	Frame[0] = 0x7E;
	Frame[1] = (length-4) >> 8;
	Frame[2] = length - 4;
	Frame[3] = 0x10;
	Frame[4] = ID++;
	Frame[13] = 0xFF;
	Frame[14] = 0xFE;
	Frame[15] = 0x00;
	Frame[16] = 0x00;
	
	for (i=0;i<(length-18);i++)			//Add data to Frame
	{
		Frame[17+i] = data[i];
		CheckSum += data[i];
	}
	
	for (i=0;i<8;i++)					//Add address to Frame
	{
		Frame[i+5] = Adrr[i];
	}
	
	CheckSum += 0x0D + Adrr_CheckSum + Frame[4];		//Calculate CheckSum
	CheckSum = 0xFF - CheckSum;
	Frame[length-1] = CheckSum;
	
	for(i=0;i<length;i++)
	{
		XbeeSerial.write(Frame[i]);
		//Serial.print(Frame[i],HEX);
		//Serial.print("  ");
		//USART_Transmitchar(Frame[i],usart);			//Send Whole Frame
	}
}


void Xbee::SendAtCmnd(char para_value[], char atcmd[])
{
	int frameid=0x01;
	int length;
	int checksum;

	checksum=0x00;
	length=15;
	int i,j;
	i = strlen(para_value);			//4
	length+=i;							//add message bytes
	unsigned char array[length+4];
	
	
	array[0]=0x7E;
	array[1]=(length) >> 8;
	array[2]=length;
	
	length += 4;
	
	array[3]=0x17;
	array[4]=frameid;
	
	for(j=0;j<=7;j++)								// Fill address
	{
		array[j+5]= Adrr[j];
		//checksum+= array[j+5];
	}
	
	array[13]=0xFF;
	array[14]=0xFE;									// 16 bit address
	array[15]=0x02;
	
	for(j=0;j<2;j++)
	{
		array[j+16]=atcmd[j];
		checksum+=array[j+16];
	}
	
	for(j=0;j<4;j++)
	{
		array[j+18]=para_value[j];				//18 bytes up to para values
		checksum+=array[j+18];
	}
	
	checksum=0xFF-(checksum + 0x17 + 0x01 + 0xFF + 0xFE + 0x02 + Adrr_CheckSum);
	array[length-1]=checksum;
	int x;
	for(x=0;x<4+19;x++)						//Transmit frame
	{
		XbeeSerial.print(array[x],HEX);
		//USART_Transmitchar(array[x],usart);
	}
	
}

