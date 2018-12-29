#ifndef Data_String
#define Data_String

#include <inttypes.h>
#include <string.h>
class DataStr
{
	public :
		void Add_Int_to_String(int64_t Num);
		void Add_Float_To_String(float Num);
		void Reset_String(void);
		void Add_String_To_String(char str[],uint8_t length);
		void End_String(void);
		
		unsigned char DataString[200];

	private :
		uint8_t Devide(int64_t Num , uint8_t length);
		uint8_t Get_Length(uint64_t Num);
		uint8_t n;
		uint16_t Count = 65535;
		
};
#endif
