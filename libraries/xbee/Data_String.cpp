#include "Data_String.h"

void DataStr::Add_Int_to_String(int64_t Num)
{
	if(Num==0)
	{
		DataString[++Count] = '0';
	}
	else
	{
		if(Num<0)
		{
			Count++;
			DataString[Count]='-';
			Num = (-1)*Num;
		}
	
		Count = Count + (Get_Length((uint64_t)Num));
		DataString[Count] = Devide(Num,Count) + '0';
	}
	DataString[++Count] = ',';
}

uint8_t DataStr::Devide(int64_t Num , uint8_t length)
{
	if (Num<10)
	{
		return Num;
	}
	else
	{
		length = length - 1;
		DataString[length] = Devide(Num/10,length);
		DataString[length] += '0';
		return (Num%10);
	}
}

void DataStr::Add_Float_To_String(float Num)
{
	Add_Int_to_String(Num);
	DataString[Count] = '.';
	
	int temp;
	temp = ((Num - (int)Num))*10000.0;
	if(temp<0)
		temp *= -1;
	if(temp<1000)
	{
		DataString[++Count] = '0';
		if(temp<100)
		{
			DataString[++Count] = '0';
			if(temp<10)
			{
				DataString[++Count] = '0';
			}
		}
	}
	
	Add_Int_to_String(temp);
}

uint8_t DataStr::Get_Length(uint64_t Num)
{
	n=0;
	while(Num!=0)
	{
		Num = (uint64_t)Num/10;
		n = n+1;
	}
	return(n);
}

void DataStr::Reset_String(void)
{
	Count = 65535;
	int i;
	for(i=0;i<200;i++)
	{
		DataString[i] = 0;
	}
}

void DataStr::Add_String_To_String(char str[],uint8_t length)
{
	uint8_t i;
	for(i=0;i<length;i++)
	{
		DataString[++Count] = str[i];
	}
	DataString[++Count] = ',';
}

void DataStr::End_String(void)
{
	DataString[++Count] = 'X';
	DataString[++Count] = '\n';
	
}
