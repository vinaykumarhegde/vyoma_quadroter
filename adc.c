#include "adc.h"
#include "LPC_2148.h"
#include "type.h"

/**
*******************************************************************************
  Function Name :init_adc0()

  Description :	Initialises the ADC0  

  Input :	None

  Output :	None

  Note : 
*******************************************************************************
*/	
void init_adc0(void)
{
  PINSEL1 = (PINSEL1 & ~(3 << 28)) | (1 << 28);
  //PINSEL1 = (PINSEL1 & ~(3 << 26)) | (1 << 26);
  //PINSEL1 = (PINSEL1 & ~(3 << 24)) | (1 << 24);
}



/**
*******************************************************************************
	Function Name : adc_read()

	Description   :

	Input         : adc number,channel

	Output        : 10 bit AD value

	Note          :
*******************************************************************************
*/

U16 adc_read(U8 adc_num, U8 ch)
{
  U32 i=0;
  
  switch(adc_num)
  {
    case 0:
      AD0CR = 0x00200D00 | (1<<ch);    // select channel
      AD0CR |= 0x01000000; // Start A/D Conversion
    
      do
      {
        i = AD0GDR; // Read A/D Data Register
      } while ((i & 0x80000000) == 0); // Wait for end of A/D Conversion
      break;
    
    case 1:
      
      AD1CR = 0x00200D00 | (1<<ch); // select channel
      AD1CR |= 0x01000000; // Start A/D Conversion
      do
      {
        i = AD1GDR; // Read A/D Data Register
      } while ((i & 0x80000000) == 0); // Wait for end of A/D Conversion
      
      break;
  }
 
  return (i >> 6) & 0x03FF; // bit 6:15 is 10 bit AD value
}









