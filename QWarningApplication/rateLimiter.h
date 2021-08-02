#ifndef _RATE_LIMITER
#define _RATE_LIMITER


int rateLimiterTokenCount(void);

void rateLimiterDoWork(void);

BOOL rateLimiterTokenAvailable(void);

BOOL rateLimiterIsActive(void);

int getTokens( void );

#endif

