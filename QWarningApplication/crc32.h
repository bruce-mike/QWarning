#ifndef CRC32_H
#define CRC32_H


uint32_t crc32Calculate(void *pBfr, uint32_t size);
void crc32Init(uint32_t *pCRC);
void crc32AddByte(uint32_t *pCRC, uint8_t val8);
void crc32End(uint32_t *pCRC);
void crc32AddBytes(uint32_t *pCRC, uint8_t *vals, uint32_t numBytes);


#endif
