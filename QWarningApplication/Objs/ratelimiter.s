; generated by Component: ARM Compiler 5.04 update 1 (build 49) Tool: ArmCC [5040049]
; commandline ArmCC [--c99 --debug -c --asm -o.\objs\ratelimiter.o --depend=.\objs\ratelimiter.d --apcs=interwork -Otime --diag_suppress=9931 -IC:\Keil\ARM\INC\Phillips -I..\SharedCode -IC:\Keil_v5\ARM\RV31\INC -IC:\Keil_v5\ARM\CMSIS\Include -IC:\Keil_v5\ARM\Inc\Philips --enum_is_int --omf_browse=.\objs\ratelimiter.crf rateLimiter.c]
        ARM
        REQUIRE8
        PRESERVE8

        AREA ||.text||, CODE, READONLY, ALIGN=2

        REQUIRE _printf_percent
        REQUIRE _printf_d
        REQUIRE _printf_int_dec
rateLimiterTokenCount PROC
        LDR      r0,|L1.256|
        LDR      r0,[r0,#4]  ; modemCommTokens
        BX       lr
        ENDP

rateLimiterIsActive PROC
        LDR      r0,|L1.256|
        LDRB     r0,[r0,#0]  ; isRateLimited
        BX       lr
        ENDP

rateLimiterTokenAvailable PROC
        LDR      r2,|L1.256|
        MOV      r0,#0
        LDRB     r1,[r2,#0]  ; isRateLimited
        CMP      r1,#0
        BXNE     lr
        LDR      r1,[r2,#4]  ; modemCommTokens
        CMP      r1,#1
        MOVLE    r1,#1
        STRBLE   r1,[r2,#0]  ; isRateLimited
        BXLE     lr
        SUB      r0,r1,#1
        STR      r0,[r2,#4]  ; modemCommTokens
        MOV      r0,#1
        BX       lr
        ENDP

getTokens PROC
        LDR      r0,|L1.256|
        LDR      r0,[r0,#4]  ; modemCommTokens
        BX       lr
        ENDP

rateLimiterDoWork PROC
        PUSH     {r4-r6,lr}
        LDR      r4,|L1.256|
        MOV      r6,#0xa60
        LDRB     r0,[r4,#1]  ; firstTime
        MOV      r5,#0
        ADD      r6,r6,#0xe000
        CMP      r0,#1
        BEQ      |L1.176|
        ADD      r0,r4,#8
        BL       isTimerExpired
        CMP      r0,#0
        POPEQ    {r4-r6,lr}
        BXEQ     lr
        LDR      r0,[r4,#4]  ; modemCommTokens
        CMP      r0,#0x3c
        ADDLT    r0,r0,#5
        STRLT    r0,[r4,#4]  ; modemCommTokens
        LDRB     r0,[r4,#0]  ; isRateLimited
        CMP      r0,#1
        BNE      |L1.240|
        B        |L1.228|
|L1.176|
        MOV      r0,#0x3c
        STRB     r5,[r4,#1]  ; firstTime
        STR      r0,[r4,#4]  ; modemCommTokens
        LDR      r0,|L1.260|
        BL       initTimer
        LDR      r0,|L1.260|
        MOV      r1,r6
        BL       startTimer
        LDRB     r2,[r4,#0]  ; isRateLimited
        LDR      r1,[r4,#4]  ; modemCommTokens
        POP      {r4-r6,lr}
        ADR      r0,|L1.264|
        B        __2printf
|L1.228|
        LDR      r0,[r4,#4]  ; modemCommTokens
        CMP      r0,#0xa
        STRBGT   r5,[r4,#0]  ; isRateLimited
|L1.240|
        MOV      r1,r6
        POP      {r4-r6,lr}
        LDR      r0,|L1.260|
        B        startTimer
|L1.256|
        DCD      ||.data||
|L1.260|
        DCD      ||.data||+0x8
|L1.264|
        DCB      "INIT: modemCommTokens[%d]  isRateLimited[%d]\r\n",0
        DCB      0
        ENDP


        AREA ||.arm_vfe_header||, DATA, READONLY, NOALLOC, ALIGN=2

        DCD      0x00000000

        AREA ||.data||, DATA, ALIGN=2

isRateLimited
        DCB      0x00
firstTime
        DCB      0x01,0x00,0x00
modemCommTokens
        DCD      0x00000000
addModemTokenTimer
        %        8

        EXPORT rateLimiterTokenCount [CODE]
        EXPORT rateLimiterIsActive [CODE]
        EXPORT rateLimiterTokenAvailable [CODE]
        EXPORT getTokens [CODE]
        EXPORT rateLimiterDoWork [CODE]

        IMPORT ||Lib$$Request$$armlib|| [CODE,WEAK]
        IMPORT initTimer [CODE]
        IMPORT startTimer [CODE]
        IMPORT _printf_percent [CODE]
        IMPORT _printf_d [CODE]
        IMPORT _printf_int_dec [CODE]
        IMPORT __2printf [CODE]
        IMPORT isTimerExpired [CODE]

        KEEP isRateLimited
        KEEP firstTime
        KEEP modemCommTokens
        KEEP addModemTokenTimer

        ATTR FILESCOPE
        ATTR SETVALUE Tag_ABI_PCS_wchar_t,2
        ATTR SETVALUE Tag_ABI_enum_size,2
        ATTR SETVALUE Tag_ABI_optimization_goals,2
        ATTR SETSTRING Tag_conformance,"2.06"
        ATTR SETVALUE AV,18,1

        ASSERT {ENDIAN} = "little"
        ASSERT {INTER} = {TRUE}
        ASSERT {ROPI} = {FALSE}
        ASSERT {RWPI} = {FALSE}
        ASSERT {IEEE_FULL} = {FALSE}
        ASSERT {IEEE_PART} = {FALSE}
        ASSERT {IEEE_JAVA} = {FALSE}
        END
