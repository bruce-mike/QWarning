//===================================================================================
//   crc32.c
//===================================================================================


#include "shareddefs.h"


// 0xEDB88320L represents the coefficients of the CRC32 polynomial as 1's
// where there is an x^n term (except for the x^32 term, which is left out).
#define CRC32_POLY  0xEDB88320L



//=====================================
void
crc32Init(uint32_t *pCRC)
//=====================================
{
	*pCRC = 0xffffffff;
}
 
//===========================================
void
crc32AddByte(uint32_t *pCRC, uint8_t val8)
//===========================================
{
    uint32_t i, poly;
    uint32_t entry;
	uint32_t crcIn;
    uint32_t crcOut;

    crcIn = *pCRC;
    poly = CRC32_POLY;

    entry = (crcIn ^ ((uint32_t)val8)) & 0xFF;

    for (i=0; i<8; i++)
    {
	    if (entry & 1)
        {
	        entry = (entry >> 1) ^ poly;
        }
        else
		{
            entry >>= 1;
		}
   }

   crcOut = ((crcIn>>8) & 0x00FFFFFF) ^ entry;

   *pCRC = crcOut;
}


//======================================================
void
crc32AddBytes(uint32_t *pCRC, uint8_t *vals, uint32_t numBytes)
//======================================================
{
	while(numBytes)
	{
		crc32AddByte(pCRC, *vals);
		vals++;
		numBytes--;
	}
}	
		
 
//==============================
void
crc32End(uint32_t *pCRC)
//==============================
{
	*pCRC ^= 0xffffffff;
}
 
//===========================================
uint32_t
crc32Calculate(void *pBfr, uint32_t size)
//===========================================
{
	uint32_t crc32;
    uint8_t  *pu8;

    crc32Init(&crc32);
    pu8 = (uint8_t *) pBfr;

    while (size-- != 0)
    {
		crc32AddByte(&crc32, *pu8);
        pu8++ ;
    }

    crc32End(&crc32);

    return(crc32);
}
 
