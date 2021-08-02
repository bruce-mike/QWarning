//=========================================================================================
// iap.c
//=========================================================================================


#include <stddef.h>
#include "shareddefs.h"
#include <stdint.h>
#include "iap.h"
#include "lpc23xx.h"
#include <stdio.h>
#include <string.h>
#include "ProductID.h"


// do not unable the debug unless the functions associated with 
// printf() are retargeted - otherwise, software will crash
//#define ENABLE_IAP_DEBUG    1

// function prototypes:
void __swi(0xFE) disable_isr (void);
void __swi(0xFF) enable_isr (void);

/* pointer to reserved flash rom section for configuration data */
//__attribute((aligned(256))) char configmem[256] __attribute__((section(".configmem")));


//===================================================================
// CRTITCAL!!!
// this must be aligned on a 4 byte address or the IAP upgrade 
// will be broke !!!
//===================================================================
__align(4) static uint8_t blockData[FLASH_BLOCK_SIZE];
__align(4) static uint32_t blockUint32Data[64];
//===================================================================


__align(4) static uint32_t targetId;
__align(4) static uint32_t codeSize;
__align(4) static uint32_t codeCrc32;


static uint32_t enabledIntMask;
static BOOL isInterruptMaskSaved = FALSE;


static unsigned int iap_command[5];         // contains parameters for IAP command
static unsigned int iap_result[2];          // contains results
typedef void (*IAP)(unsigned int[], unsigned int[]);    // type definition for IAP entry function
IAP IAP_Entry;

#define _XTAL 4000000

#define INVALID_SECTOR_NUM  (0xFF)



#define UPGRADE_FLAG_LOC            0


uint32_t sectorAddresses[NUM_SECTORS] =
{
    0x0000,
    0x1000,
    0x2000,
    0x3000,
    0x4000,
    0x5000,
    0x6000,
    0x7000,
    0x8000,
    0x10000,
    0x18000,
    0x20000,
    0x28000,
    0x30000,
    0x38000
};



uint16_t swapUint16(uint16_t value) 
{
    uint16_t result;

    ((BYTE*)&result)[0] = ((BYTE*)&value)[1];
    ((BYTE*)&result)[1] = ((BYTE*)&value)[0];
    
    return result;
}

uint32_t swapUint32(uint32_t value)
{
    unsigned long result;
    
    ((BYTE*)&result)[0] = ((BYTE*)&value)[3];
    ((BYTE*)&result)[1] = ((BYTE*)&value)[2];
    ((BYTE*)&result)[2] = ((BYTE*)&value)[1];
    ((BYTE*)&result)[3] = ((BYTE*)&value)[0];
    
    return result;
}

uint32_t convertByteToUint32(uint8_t *bytes)
{
    uint32_t uint32Val;
    
    uint32Val = ((uint32_t)bytes[0] << 24) +
                ((uint32_t)bytes[1] << 16) +
                ((uint32_t)bytes[2] << 8) + bytes[3];

    return uint32Val;
}

//=======================================================
static void
disableInterrupts(void)
//=======================================================
{
    if(FALSE == isInterruptMaskSaved)
    {
        // save enabled interrupts
        enabledIntMask = VICIntEnable;
    
        // disable all (32) interrupts by writing 1's to 
        // Interrupt Enable Clear Register
        VICIntEnClr = 0xFFFFFFFF;
        isInterruptMaskSaved = TRUE;
    }
    
}

//=======================================================
static void
enableInterrupts(void)
//=======================================================
{
    if(TRUE == isInterruptMaskSaved)
    {
        // restore enabled interrupts
        VICIntEnable = enabledIntMask;
        isInterruptMaskSaved = FALSE;
    }
}

//==============================================================
uint32_t
getSectorAddress(BYTE sectorNum)
//==============================================================
{
    // return sectorAddresses[sectorNum];
    
    switch(sectorNum) {
        
        case 0:
            return 0x0;
        case 1:
            return 0x1000;
        case 2:
            return 0x2000;
        case 3:
            return 0x3000;
        case 4:
            return 0x4000;
        case 5:
            return 0x5000;
        case 6:
            return 0x6000;
        case 7:
            return 0x7000;
        case 8:
            return 0x8000;
        case 9:
           return 0x10000;
        case 10:
            return 0x18000;
        case 11:
           return 0x20000;
        case 12:
            return 0x28000;
        case 13:
           return 0x30000;
        case 14:
            return 0x38000;
    }
    
    return 0x40000;
}


//==============================================================
uint32_t
getSectorSize(uint8_t sectorNum)
//==============================================================
{
    if(sectorNum <= 9)
    {
        return 0x1000;
    }
    else if(sectorNum <= 14)
    {
        return 0x8000;
    }
    
    return 0;
}


//==============================================================
uint32_t
getSectorNumber(uint32_t adr)
//==============================================================
// sectors 0-7 are 4kB
// sectors 10-21 are 32kB
// sectors 22-27 are 4kB
// sector 28 is 8k boot sector
// addresses range from 0x0 to 0x7DFFF
//==============================================================
{
        if(adr < 0x1000)       return 0;
        else if(adr < 0x2000)  return 1;
        else if(adr < 0x3000)  return 2;
        else if(adr < 0x4000)  return 3;   
        else if(adr < 0x5000)  return 4;   
        else if(adr < 0x6000)  return 5;   
        else if(adr < 0x7000)  return 6;   
        else if(adr < 0x8000)  return 7;   
        else if(adr < 0x10000) return 8;   
        else if(adr < 0x18000) return 9;   
        else if(adr < 0x20000) return 10;   
        else if(adr < 0x28000) return 11;   
        else if(adr < 0x30000) return 12;   
        else if(adr < 0x88000) return 13;   
    
        return 14;
    
}
    


//========================================================
static uint32_t
iap(uint32_t code, uint32_t p1, uint32_t p2, uint32_t p3, uint32_t p4)
//========================================================
{
    iap_command[0] = code;      // set command code
    iap_command[1] = p1;        // set 1st param
    iap_command[2] = p2;        // set 2nd param
    iap_command[3] = p3;        // set 3rd param
    iap_command[4] = p4;        // set 4th param

    ((void (*)())0x7ffffff1)(iap_command, iap_result);      // IAP entry point
    return *iap_result;
}

//======================================================================
static uint32_t
prepare_sectors(uint32_t tmp_sect1, uint32_t tmp_sect2)
//======================================================================
// Description: This command must be executed before executing "Copy RAM to Flash" or "Erase Sector(s)"
//              command. Successful execution of the "Copy RAM to Flash" or "Erase Sector(s)" command causes
//              relevant sectors to be protected again. The boot sector can not be prepared by this command. To
//              prepare a single sector use the same "Start" and "End" sector numbers..
//              Command code: 50
//              Param0: Start Sector Number
//              Param1: End Sector Number: Should be greater than or equal to start sector number.
//
// Parameters:  long tmp_sect1:     Param0
//              long tmp_sect2:     Param1
//
// Return:      Code    CMD_SUCCESS |
//                      BUSY |
//                      INVALID_SECTOR
//======================================================================
{
    uint32_t retVal;
    
//    disableInterrupts();
    retVal = iap(PREPARE_SECTOR_FOR_WRITE_OPERATION, tmp_sect1, tmp_sect2, 0 , 0);
//    enableInterrupts();
    
    return retVal;
}


//==============================================================
static BYTE
flashWriteBlock(uint32_t *dst, const BYTE *src, WORD size)
//==============================================================
{
    char err;
    BYTE sec;
    
           
    (void) size; // unused
        

    sec = getSectorNumber((uint32_t) dst);
   
    if (sec == INVALID_SECTOR_NUM)
    {
        return FALSE;
    }   
    else if(sec < USER_CODE_FIRST_SECTOR || sec > UPGRADE_CODE_LAST_SECTOR)
    {
        if(UPGRADE_FLAG_SECTOR != sec)
        {
            return FALSE;
        }
    }
    
    // check sector 
    if (blank_check_sector(sec, sec) == SECTOR_NOT_BLANK)
    {
    }

    /* prepare sector */
    err = prepare_sectors(sec, sec);

    if (err)
    {
        return FALSE;
    }

    /* write flash */
    err = copy_ram_to_flash((uint32_t) dst, (uint32_t) src, FLASH_BLOCK_SIZE);

    if (err)
    {
        /* set interrupts back and return */
        //enableInterrupts();
        return FALSE;
    }
    
    /* check result */
    err = compare((uint32_t) dst, (uint32_t) src, FLASH_BLOCK_SIZE);

    if(err)
    {
        return FALSE;
    }
    
        
    return TRUE;
}


//==============================================================================
uint8_t
writeFlashData(uint32_t blockAddress, uint8_t *data, uint32_t bytes)
//==============================================================================
{    
    int i;
    
    memset(&blockData[0], 0xFF, FLASH_BLOCK_SIZE);
    
    for(i = 0; i < bytes; i++)
    {
        blockData[i] = data[i];
    }
    
    return(flashWriteBlock((uint32_t *)(blockAddress), &blockData[0], FLASH_BLOCK_SIZE));   
} 



//=======================================================
BYTE
flashrom_erase(BYTE *addr)
//=======================================================
{
    BYTE sec = getSectorNumber((uint32_t) addr);

    
    if (sec == INVALID_SECTOR_NUM)
    {
#ifdef ENABLE_IAP_DEBUG
        printf("Invalid address\n");
#endif
        return 0;
    }

    /* check sector */
    if (!blank_check_sector(sec, sec))
    {
#ifdef ENABLE_IAP_DEBUG
        printf("Sector already blank!\n");
#endif
        return 1;
    }

    /* prepare sector */
    if (prepare_sectors(sec, sec))
    {
#ifdef ENABLE_IAP_DEBUG
        printf("-- ERROR: PREPARE_SECTOR_FOR_WRITE_OPERATION --\n");
#endif
        return 0;
    }


    /* erase sector */
    if (erase_sectors(sec, sec))
    {
#ifdef ENABLE_IAP_DEBUG
        printf("-- ERROR: ERASE SECTOR --\n");
 #endif
        return 0;
    }


    /* check again */
    if (blank_check_sector(sec, sec))
    {
#ifdef ENABLE_IAP_DEBUG
        printf("-- ERROR: BLANK_CHECK_SECTOR\n");
#endif
        return 0;
    }
#ifdef ENABLE_IAP_DEBUG
    printf("Sector successfully erased.\n");
#endif
    return 1;
}

 
//==========================================================================================================
uint32_t 
blank_check_sector(uint32_t startSector, uint32_t endSector)
//==========================================================================================================
// Returns: CMD_SUCCESS | BUSY | SECTOR_NOT_BLANK | INVALID_SECTOR
//==========================================================================================================
{
    return iap(BLANK_CHECK_SECTOR, startSector, endSector, 0 , 0);
}

//==============================================================================
uint32_t 
copy_ram_to_flash(uint32_t tmp_adr_dst, uint32_t tmp_adr_src, uint32_t tmp_size)
//==============================================================================
// Param0: Destination Flash adress where data bytes are to be written.
//         This address should be a 512 byte boundary.
// Param1: (SRC) Source RAM adress from which data byre are to be read.
// Param2: Number of bytes to be written. Should be 256 | 512 | 1024 | 4096 | 8192.
// Param3: System Clock Frequency (CCLK) in KHz.
//
// Returns: CMD_SUCCESS |
//          SRC_ADDR_ERROR (Address not on word boundary) |
//          DST_ADDR_ERROR (Address not on correct boundary) |
//          SRC_ADDR_NOT_MAPPED |
//          DST_ADDR_NOT_MAPPED |
//          COUNT_ERROR (Byte count is not 512 | 1024 | 4096 | 8192) |
//          SECTOR_NOT_PREPARED_FOR_WRITE_OPERATION |
//          BUSY
//==============================================================================
{
    uint32_t retVal;
    
    disableInterrupts();
    retVal = iap(COPY_RAM_TO_FLASH, tmp_adr_dst, tmp_adr_src, tmp_size, _XTAL);
    enableInterrupts();
    
    return retVal;
}



//======================================================================
uint32_t
erase_sectors(uint32_t firstSector, uint32_t lastSector)
//======================================================================
// To erase a single sector make firstSector and lastSector the same
// Returns:  CMD_SUCCESS |
//           BUSY |
//           SECTOR_NOT_PREPARED_FOR_WRITE_OPERATION |
//           INVALID_SECTOR
//======================================================================
{
    uint32_t retVal;
    
    disableInterrupts();
    
    retVal = prepare_sectors(firstSector, lastSector);
    
    if(CMD_SUCCESS == retVal)
    {
        retVal = iap(ERASE_SECTOR, firstSector, lastSector, _XTAL, 0);
    }
    
    enableInterrupts();
    
    return retVal;
}

//=====================================================================================
uint32_t 
compare(uint32_t tmp_adr_dst, uint32_t tmp_adr_src, uint32_t tmp_size)
//=====================================================================================
// Compare result may not be correct when source or destination address contains
// any of the first 64 bytes starting from address zero. First 64 bytes can be
// re-mapped to RAM. Number of bytes compared should be a multile of 4
//
// Returns: CMD_SUCCESS | COMPARE_ERROR | COUNT_ERROR (Byte count is not multiple of 4)
//          | ADDR_ERROR | ADDR_NOT_MAPPED
//              
//======================================================================================
{
    return iap(COMPARE, tmp_adr_dst, tmp_adr_src, tmp_size, 0);
}


//==================================================
static uint32_t
readMemoryLocation(uint32_t flashAddress)
//==================================================
{
    uint32_t *memPtr = (uint32_t*)flashAddress;

    return (*memPtr);
}



//======================================================
uint32_t
readCodeCrc32()
//======================================================
{
    uint32_t crc32 = readMemoryLocation(CODE_CRC32_LOCATION);
        
    return swapUint32(crc32);
}


//======================================================
uint32_t
readCodeSize()
//======================================================
{
     uint32_t codeSize = readMemoryLocation(CODE_SIZE_LOCATION);
     
     return swapUint32(codeSize);
} 


//======================================================
uint8_t
readCodeUpgradeFlag()
//======================================================
{
    // have to read 4 bytes from flash   
    uint32_t flag;  
    
    flag = readMemoryLocation(SOFTWARE_UPGRADE_FLAG_LOC);
        
    return(flag & 0xFF);
}


//===================================================
uint8_t
readCodeDeviceId()
//===================================================
{
    uint32_t devId = readMemoryLocation(TARGET_ID_LOCATION);
    
    return (devId & 0xFF);
}


//================================================
uint32_t 
getUpgradeStatusFlag()
//================================================
{
    uint32_t* memPtr;
    
    memPtr = (uint32_t*)(SOFTWARE_UPGRADE_FLAG_LOC); 

    return(swapUint32(*memPtr));    
}


//================================================
BYTE
upgradeCodePresent(void)
//================================================
{    
    if(readCodeUpgradeFlag() == SOFTWARE_UPGRADE_FLAG_READY)
    {
        codeSize = readCodeSize();
        codeCrc32 = readCodeCrc32();
        targetId = readCodeDeviceId();
        
        if(targetId != PRODUCT_TARGET_ID)
        {
            // the binary file is not meant for this prodcut
            writeUpgradeCompleteFlag(SOFTWARE_UPGRADE_STAUS_TARGET_ERROR);
            return FALSE;
        }
               

#ifdef ENABLE_IAP_DEBUG    
printf("codeSize[%08x] codeSize[%u] codeCrc32[%08x]\r\n", codeSize, codeSize, codeCrc32);       
#endif
        return TRUE;
    }
    
    return FALSE;   
}    


//==============================================================================
uint8_t
writeUpgradeFlag(uint8_t targetId, uint32_t size, uint32_t crc32)
//==============================================================================
{
    eraseUpgradeFlagSector();

    memset(&blockData[0], 0xFF, FLASH_BLOCK_SIZE);
    
    blockData[UPGRADE_FLAG_LOC] = SOFTWARE_UPGRADE_FLAG_READY;
        
    blockData[UPGRADE_TARGET_ID_LOC] = targetId;
    
    blockData[UPGRADE_CODE_SIZE_LOC]   = (uint8_t)((size >> 24) & 0xFF);
    blockData[UPGRADE_CODE_SIZE_LOC+1] = (uint8_t)((size >> 16) & 0xFF);
    blockData[UPGRADE_CODE_SIZE_LOC+2] = (uint8_t)((size >> 8) & 0xFF);
    blockData[UPGRADE_CODE_SIZE_LOC+3] = (uint8_t)(size & 0xFF); 
 
    blockData[UPGRADE_CRC32_LOC]   = (uint8_t)((crc32 >> 24) & 0xFF);
    blockData[UPGRADE_CRC32_LOC+1] = (uint8_t)((crc32 >> 16) & 0xFF);
    blockData[UPGRADE_CRC32_LOC+2] = (uint8_t)((crc32 >> 8) & 0xFF);
    blockData[UPGRADE_CRC32_LOC+3] = (uint8_t)(crc32 & 0xFF);       
 

    return(flashWriteBlock((uint32_t *)(SOFTWARE_UPGRADE_FLAG_LOC), &blockData[0], FLASH_BLOCK_SIZE));   
} 


//===============================================================================
BYTE
writeUpgradeCompleteFlag(uint8_t flag)
//===============================================================================
{
    eraseUpgradeFlagSector();
           
    memset(&blockData[0], 0xFF, FLASH_BLOCK_SIZE);
    
    blockData[UPGRADE_FLAG_LOC] = flag;
    blockData[UPGRADE_TARGET_ID_LOC] = targetId;

       
    blockData[UPGRADE_CODE_SIZE_LOC] =   (uint8_t)((codeSize >> 24) & 0xFF);
    blockData[UPGRADE_CODE_SIZE_LOC+1] = (uint8_t)((codeSize >> 16) & 0xFF);
    blockData[UPGRADE_CODE_SIZE_LOC+2] = (uint8_t)((codeSize >> 8) & 0xFF);
    blockData[UPGRADE_CODE_SIZE_LOC+3] = (uint8_t)(codeSize & 0xFF); 
 
    blockData[UPGRADE_CRC32_LOC] = (uint8_t)((codeCrc32 >> 24) & 0xFF);
    blockData[UPGRADE_CRC32_LOC+1] = (uint8_t)((codeCrc32 >> 16) & 0xFF);
    blockData[UPGRADE_CRC32_LOC+2] = (uint8_t)((codeCrc32 >> 8) & 0xFF);
    blockData[UPGRADE_CRC32_LOC+3] = (uint8_t)(codeCrc32 & 0xFF);   

    return(flashWriteBlock((uint32_t *)(SOFTWARE_UPGRADE_FLAG_LOC), &blockData[0], FLASH_BLOCK_SIZE));  
}


//===============================================================================
void
writeTestSector()
//===============================================================================
{
    int i;
    uint32_t testSectorAddress = iapGetBlockAddress(11, 2);
       
    for(i=0; i < FLASH_BLOCK_SIZE; i++)
    {
        blockData[i] = i; 
    }

    for(i=0; i < 15; i++)
    {
        blockData[0] = i;
        testSectorAddress = iapGetBlockAddress(11, i);
        flashWriteBlock((uint32_t *)(testSectorAddress), &blockData[0], FLASH_BLOCK_SIZE);
    }        
}


//==============================================================================
uint8_t
copySectorData(uint32_t src, uint32_t dest)
//==============================================================================
{
    uint16_t i;
    uint32_t *memPtr;
 
    
    memPtr = (uint32_t*)(src);   
 
    for(i=0; i<64; i++)
    {
        blockUint32Data[i] = memPtr[i];
    }
        
	writeFlashData(dest, (uint8_t*)&blockUint32Data[0],  FLASH_BLOCK_SIZE);
    
    return TRUE;
}

    

//===============================================================================
void
eraseUpgradeFlagSector()
//===============================================================================
{
    erase_sectors(UPGRADE_FLAG_SECTOR, UPGRADE_FLAG_SECTOR);   
}


//=====================================================================================
BYTE
eraseUpgradeSectors()
//=====================================================================================
{	
	if(CMD_SUCCESS == erase_sectors(UPGRADE_CODE_FIRST_SECTOR, UPGRADE_CODE_LAST_SECTOR))
    {
        if(CMD_SUCCESS == erase_sectors(UPGRADE_FLAG_SECTOR, UPGRADE_FLAG_SECTOR))
        {
            return TRUE;
        }
    }
              
    return FALSE;
}

//=====================================================================================
BYTE
eraseCodeSectors()
//=====================================================================================
{
		BYTE retVal;
	
		retVal = erase_sectors(USER_CODE_FIRST_SECTOR, USER_CODE_LAST_SECTOR);

		if(CMD_SUCCESS == retVal)
		{
			return TRUE;
		}
		
		return FALSE;
}



//==============================================
uint8_t 
iapTestWriteApplicationCode()
//==============================================
{
    uint8_t retVal;
    uint16_t i;
    uint32_t testWriteSectorAddress = UPGRADE_CODE_BASE_ADDRESS;    
    
    erase_sectors(11, 11);
    
    memset(&blockData[0], 0xA5, FLASH_BLOCK_SIZE);
    
    
    for(i = 0; i < 128; i++)
    {
        blockData[0] = i;
        retVal = writeFlashData(testWriteSectorAddress, &blockData[0], FLASH_BLOCK_SIZE);
        testWriteSectorAddress += FLASH_BLOCK_SIZE;
    }
    
    return retVal;
}

//==================================================
uint32_t
iapGetBlockAddress(uint16_t sector, uint16_t block)
//==================================================
{
    uint32_t blockAddress;
    
    blockAddress = getSectorAddress(sector) + (block * FLASH_BLOCK_SIZE);
    
    return blockAddress;
}
