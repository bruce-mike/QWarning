//===========================================================
// rateLimiter.c
//
// rate limiter for asset tracker 'modem'
//
// start with 60 tokens
// if less than 60 tokens add 1 per minute
// modem will not be allowed to communicate if out of tokens
//============================================================



#include "shareddefs.h"
#include "sharedInterface.h"
#include "timer.h"
#include <stdio.h>


//#define DEBUG_LIM 1


#ifdef DEBUG_LIM
#define MAX_MODEM_COMM_TOKENS   10
#else
#define MAX_MODEM_COMM_TOKENS   60
#endif

static int modemCommTokens;

static TIMERCTL addModemTokenTimer;
// add one token per minute
#define ADD_MODEM_TOKEN_INTERVAL  60000

// keep data rate limited until 5 tokens are recovered
#define NUM_TOKENS_RESUME_COMM  10
// flag to indicate rate limitation is in effect
static BOOL isRateLimited = FALSE;


//==============================================
int
rateLimiterTokenCount()
//==============================================
{
   return modemCommTokens;
}

//==============================================
BOOL
rateLimiterIsActive()
//==============================================
{
   return isRateLimited;
}


//==============================================
BOOL
rateLimiterTokenAvailable(void)
//==============================================
// return FALSE if out of tokens
// otherwise, use a token and return TRUE
//==============================================
{
   BOOL retVal = FALSE;

   #ifdef DEBUG_LIM
   printf("modemCommTokens[%d]  isRateLimited[%d]\r\n", modemCommTokens, isRateLimited);
   #endif

   if( FALSE == isRateLimited )
   {
      // need to save one token in order to send Fleet Manager message that comm is
      // going to be interrupted
      if( modemCommTokens > 1 )
      {
         modemCommTokens--;

         #ifdef DEBUG_LIM
         printf("Token removed [%d]\r\n", modemCommTokens);
         #endif
         retVal = TRUE;
      }
      else
      {
         #ifdef DEBUG_LIM
         printf("Comm suspended [%d]\r\n", modemCommTokens);
         #endif
         isRateLimited = TRUE;
      }
   }

   #ifdef DEBUG_LIM
   printf("retVal[%d]\r\n", (int)retVal);
   #endif

   #ifdef JEFF_DEV_MODE
   retVal = TRUE;
   #endif

   return retVal;
}

int getTokens( void )
{
   return( modemCommTokens );
}
//===========================================
void
rateLimiterDoWork()
//===========================================
// add token every 60 seconds
//===========================================
{
   static BOOL firstTime = TRUE;


   if( TRUE == firstTime )
   {
      firstTime = FALSE;

      

      modemCommTokens = MAX_MODEM_COMM_TOKENS;

      #ifdef DEBUG_LIM
      printf("Initialize tokens [%d]\r\n", modemCommTokens);
      #endif

      initTimer(&addModemTokenTimer);
      startTimer(&addModemTokenTimer, ADD_MODEM_TOKEN_INTERVAL);
      printf("INIT: modemCommTokens[%d]  isRateLimited[%d]\r\n", modemCommTokens, isRateLimited);
      return;
   }


   if( isTimerExpired(&addModemTokenTimer) )
   {
      // add token each timer 'addModemTokenTimer' expiration
      if( modemCommTokens < MAX_MODEM_COMM_TOKENS )
      {
         modemCommTokens += 5;

         #ifdef DEBUG_LIM
         printf("Token added [%d]\r\n", modemCommTokens);
         #endif
      }

      if( TRUE == isRateLimited )
      {
         if( modemCommTokens > NUM_TOKENS_RESUME_COMM )
         {
            isRateLimited = FALSE;             //TRUE;
            #ifdef DEBUG_LIM
            printf("Comm resumed [%d]\r\n", modemCommTokens);
            #endif
         }
      }

      startTimer(&addModemTokenTimer, ADD_MODEM_TOKEN_INTERVAL);
   }

}
