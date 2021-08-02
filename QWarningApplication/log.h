/*
 * log.h
 */
 
#ifndef __LOG_H 
#define __LOG_H

// Only one is allowed
#define LOG_PRINTF

extern void initLogging( void );
extern void logDoWork( void );

extern BOOL eraseLogging( void );
extern void turnOnLogging( void );
extern void turnOffLogging( void );
extern BOOL isLoggingEnabled( void );

extern char *getLogCharBuffer( void );
extern void printfLog( char *output );
extern BOOL writeLog( uint8_t *data, uint16_t length );
extern void dumpLog( uint32_t length );

#endif   // __LOG_H
