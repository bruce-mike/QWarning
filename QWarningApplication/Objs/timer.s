; generated by Component: ARM Compiler 5.04 update 1 (build 49) Tool: ArmCC [5040049]
; commandline ArmCC [--c99 --debug -c --asm -o.\objs\timer.o --depend=.\objs\timer.d --apcs=interwork -Otime --diag_suppress=9931 -IC:\Keil\ARM\INC\Phillips -I..\SharedCode -IC:\Keil_v5\ARM\RV31\INC -IC:\Keil_v5\ARM\CMSIS\Include -IC:\Keil_v5\ARM\Inc\Philips --enum_is_int --omf_browse=.\objs\timer.crf timer.c]
        ARM
        REQUIRE8
        PRESERVE8

        AREA ||.text||, CODE, READONLY, ALIGN=2

T0Handler PROC
|L1.0|
        PUSH     {r0-r5,r12,lr}
        MOV      r1,#0x4000
        MOV      r0,#8
        ADD      r1,r1,#0xe0000000
        STR      r0,[r1,#0]
        LDR      r4,|L1.1352|
        LDR      r0,[r4,#0xc]  ; nCurrentMilliseconds
        ADDS     r0,r0,#1
        STR      r0,[r4,#0xc]  ; nCurrentMilliseconds
        BNE      |L1.52|
        LDRB     r0,[r4,#0]  ; nCurrentEpoch
        ADD      r0,r0,#1
        STRB     r0,[r4,#0]  ; nCurrentEpoch
|L1.52|
        LDR      r0,[r4,#8]  ; nLEDCycles
        CMP      r0,#0
        BLT      |L1.92|
        SUB      r0,r0,#1
        CMP      r0,#0
        STR      r0,[r4,#8]  ; nLEDCycles
        BGT      |L1.92|
        BL       toggleGreenLED
        MOV      r0,#0x1f4
        STR      r0,[r4,#8]  ; nLEDCycles
|L1.92|
        MOV      r0,#0
        STR      r0,[r0,#-0x100]
        POP      {r0-r5,r12,lr}
        SUBS     pc,lr,#4
        ENDP

T1Handler PROC
|L1.108|
        PUSH     {r0,r1}
        MOV      r1,#0x8000
        MOV      r0,#8
        ADD      r1,r1,#0xe0000000
        STR      r0,[r1,#0]
        MOV      r0,#0
        STR      r0,[r0,#-0x100]
        POP      {r0,r1}
        SUBS     pc,lr,#4
        ENDP

initTimer PROC
        MOV      r1,#0
        STRB     r1,[r0,#4]
        STR      r1,[r0,#0]
        BX       lr
        ENDP

T0_SETUP_PERIODIC_INT PROC
        MOV      r2,#0x1fc000
        PUSH     {r4,lr}
        ADD      r2,r2,#0xe0000000
        LDR      r1,[r2,#0xc4]
        ORR      r1,r1,#2
        STR      r1,[r2,#0xc4]
        SUB      r4,r2,#0x1f8000
        LDR      r1,[r4,#0x14]
        ORR      r1,r1,#0x600
        STR      r1,[r4,#0x14]
        LDR      r1,|L1.1352|
        LDR      r1,[r1,#4]  ; ePlatformType
        CMP      r1,#0
        BEQ      |L1.328|
        CMP      r1,#1
        LDR      r1,[r2,#0x1a8]
        BIC      r1,r1,#0xc
        STR      r1,[r2,#0x1a8]
        LDR      r1,[r2,#0x1a8]
        ORR      r1,r1,#0xc
        STRNE    r1,[r2,#0x1a8]
        BEQ      |L1.344|
|L1.248|
        MOV      r1,#0x66000
        ADD      r1,r1,#0x2100000
        MUL      r0,r1,r0
        LDR      r1,|L1.1356|
        UMULL    r2,r0,r1,r0
        LSR      r0,r0,#8
        STR      r0,[r4,#0x24]
        MOV      r2,#0xf
        ADR      r1,|L1.0|
        MOV      r0,#4
        BL       install_irq
        MOV      r0,#2
        STR      r0,[r4,#4]
        MOV      r0,r0
        MOV      r0,#0
        STR      r0,[r4,#4]
        MOV      r0,#1
        STR      r0,[r4,#4]
        POP      {r4,lr}
        BX       lr
|L1.328|
        LDR      r1,[r2,#0x1a8]
        BIC      r1,r1,#0xc
        STR      r1,[r2,#0x1a8]
        LDR      r1,[r2,#0x1a8]
|L1.344|
        STR      r1,[r2,#0x1a8]
        B        |L1.248|
        ENDP

T1_SETUP_PERIODIC_INT PROC
        MOV      r1,#0x1fc000
        PUSH     {r4,lr}
        ADD      r1,r1,#0xe0000000
        LDR      r0,[r1,#0xc4]
        ORR      r0,r0,#4
        STR      r0,[r1,#0xc4]
        SUB      r4,r1,#0x1f4000
        LDR      r0,[r4,#0x14]
        ORR      r0,r0,#0x600
        STR      r0,[r4,#0x14]
        LDR      r0,|L1.1352|
        LDR      r0,[r0,#4]  ; ePlatformType
        CMP      r0,#0
        BEQ      |L1.444|
        CMP      r0,#1
        LDR      r0,[r1,#0x1a8]
        BIC      r0,r0,#0x30
        STR      r0,[r1,#0x1a8]
        LDR      r0,[r1,#0x1a8]
        ORR      r0,r0,#0x30
        STREQ    r0,[r1,#0x1a8]
        BEQ      |L1.464|
        B        |L1.460|
|L1.444|
        LDR      r0,[r1,#0x1a8]
        BIC      r0,r0,#0x30
        STR      r0,[r1,#0x1a8]
        LDR      r0,[r1,#0x1a8]
|L1.460|
        STR      r0,[r1,#0x1a8]
|L1.464|
        MVN      r0,#0
        STR      r0,[r4,#0x24]
        MOV      r2,#0xf
        ADR      r1,|L1.108|
        MOV      r0,#5
        BL       install_irq
        MOV      r0,#2
        STR      r0,[r4,#4]
        MOV      r0,r0
        MOV      r0,#0
        STR      r0,[r4,#4]
        MOV      r0,#1
        STR      r0,[r4,#4]
        POP      {r4,lr}
        BX       lr
        ENDP

timerShowLEDHeartbeat PROC
        LDR      r1,|L1.1352|
        MOV      r0,#0x1f4
        STR      r0,[r1,#8]  ; nLEDCycles
        BX       lr
        ENDP

timerStopLEDHeartbeat PROC
        LDR      r1,|L1.1352|
        MVN      r0,#0
        STR      r0,[r1,#8]  ; nLEDCycles
        BX       lr
        ENDP

getTimeNow PROC
        LDR      r0,|L1.1352|
        SUB      sp,sp,#8
        LDRB     r1,[r0,#0]  ; nCurrentEpoch
        STRB     r1,[sp,#4]
        LDR      r1,[r0,#0xc]  ; nCurrentMilliseconds
        STR      r1,[sp,#0]
        LDRB     r1,[sp,#4]
        LDRB     r2,[r0,#0]  ; nCurrentEpoch
        CMP      r1,r2
        LDRBNE   r1,[r0,#0]  ; nCurrentEpoch
        STRBNE   r1,[sp,#4]
        LDR      r1,[sp,#0]
        LDR      r2,[r0,#0xc]  ; nCurrentMilliseconds
        CMP      r1,r2
        BEQ      |L1.636|
|L1.616|
        LDR      r1,[r0,#0xc]  ; nCurrentMilliseconds
        STR      r1,[sp,#0]
        LDR      r2,[r0,#0xc]  ; nCurrentMilliseconds
        CMP      r1,r2
        BNE      |L1.616|
|L1.636|
        MOV      r0,r1
        ADD      sp,sp,#8
        BX       lr
        ENDP

hasTimedOut PROC
        LDR      r2,|L1.1352|
        SUB      sp,sp,#8
        LDRB     r3,[r2,#0]  ; nCurrentEpoch
        STRB     r3,[sp,#4]
        LDR      r3,[r2,#0xc]  ; nCurrentMilliseconds
        STR      r3,[sp,#0]
        LDRB     r3,[sp,#4]
        LDRB     r12,[r2,#0]  ; nCurrentEpoch
        CMP      r3,r12
        LDRBNE   r3,[r2,#0]  ; nCurrentEpoch
        STRBNE   r3,[sp,#4]
        LDR      r3,[sp,#0]
        LDR      r12,[r2,#0xc]  ; nCurrentMilliseconds
        CMP      r3,r12
        BEQ      |L1.728|
|L1.708|
        LDR      r3,[r2,#0xc]  ; nCurrentMilliseconds
        STR      r3,[sp,#0]
        LDR      r12,[r2,#0xc]  ; nCurrentMilliseconds
        CMP      r3,r12
        BNE      |L1.708|
|L1.728|
        SUB      r0,r3,r0
        CMP      r0,r1
        MOVCS    r0,#1
        MOVCC    r0,#0
        ADD      sp,sp,#8
        BX       lr
        ENDP

getTimer1Count PROC
        MOV      r0,#0x8000
        ADD      r0,r0,#0xe0000000
        LDR      r1,[r0,#8]
        SUB      sp,sp,#8
        STR      r1,[sp,#4]
        LDR      r1,[r0,#8]
        LDR      r2,[sp,#4]
        STR      r1,[sp,#0]
        SUB      r1,r1,r2
        CMP      r1,#0xa
        BLS      |L1.828|
|L1.796|
        LDR      r1,[r0,#8]
        STR      r1,[sp,#4]
        LDR      r1,[r0,#8]
        LDR      r2,[sp,#4]
        STR      r1,[sp,#0]
        SUB      r1,r1,r2
        CMP      r1,#0xa
        BHI      |L1.796|
|L1.828|
        LDR      r0,[sp,#0]
        ADD      sp,sp,#8
        BX       lr
        ENDP

startTimer PROC
        LDR      r2,|L1.1352|
        PUSH     {r4}
        LDRB     r3,[r2,#0]  ; nCurrentEpoch
        SUB      sp,sp,#8
        STRB     r3,[sp,#4]
        LDR      r3,[r2,#0xc]  ; nCurrentMilliseconds
        STR      r3,[sp,#0]
        LDRB     r3,[sp,#4]
        LDRB     r12,[r2,#0]  ; nCurrentEpoch
        CMP      r3,r12
        BEQ      |L1.900|
        LDRB     r3,[r2,#0]  ; nCurrentEpoch
        STRB     r3,[sp,#4]
        LDR      r3,[r2,#0xc]  ; nCurrentMilliseconds
        STR      r3,[sp,#0]
|L1.900|
        LDRB     r4,[sp,#4]
        STRB     r4,[r0,#4]
        LDR      r3,[sp,#0]
        LDR      r12,[r2,#0xc]  ; nCurrentMilliseconds
        CMP      r3,r12
        BEQ      |L1.944|
|L1.924|
        LDR      r3,[r2,#0xc]  ; nCurrentMilliseconds
        STR      r3,[sp,#0]
        LDR      r12,[r2,#0xc]  ; nCurrentMilliseconds
        CMP      r3,r12
        BNE      |L1.924|
|L1.944|
        CMP      r1,#0
        STR      r3,[r0,#0]
        ADDEQ    sp,sp,#8
        POPEQ    {r4}
        MOV      r2,r3
        BXEQ     lr
        ADD      r1,r1,r2
        STR      r1,[r0,#0]
        LDR      r2,[sp,#0]
        CMP      r1,r2
        ADDCC    r1,r4,#1
        STRBCC   r1,[r0,#4]
        ADD      sp,sp,#8
        POP      {r4}
        BX       lr
        ENDP

isTimerExpired PROC
        LDR      r1,|L1.1352|
        MOV      r2,r0
        LDRB     r3,[r1,#0]  ; nCurrentEpoch
        SUB      sp,sp,#8
        MOV      r0,#0
        STRB     r3,[sp,#0]
        LDR      r3,[r1,#0xc]  ; nCurrentMilliseconds
        STR      r3,[sp,#4]
        LDRB     r3,[sp,#0]
        LDRB     r12,[r1,#0]  ; nCurrentEpoch
        CMP      r3,r12
        BEQ      |L1.1068|
        LDRB     r3,[r1,#0]  ; nCurrentEpoch
        STRB     r3,[sp,#0]
        LDR      r1,[r1,#0xc]  ; nCurrentMilliseconds
        STR      r1,[sp,#4]
|L1.1068|
        LDRB     r1,[r2,#4]
        LDRB     r3,[sp,#0]
        CMP      r1,r3
        BCS      |L1.1096|
|L1.1084|
        MOV      r0,#1
        ADD      sp,sp,#8
        BX       lr
|L1.1096|
        LDRB     r3,[sp,#0]
        CMP      r1,r3
        ADDNE    sp,sp,#8
        BXNE     lr
        LDR      r1,[r2,#0]
        LDR      r2,[sp,#4]
        CMP      r1,r2
        BCC      |L1.1084|
        ADD      sp,sp,#8
        BX       lr
        ENDP

stopTimer PROC
        MOV      r1,#0
        STRB     r1,[r0,#4]
        STR      r1,[r0,#0]
        BX       lr
        ENDP

timerInit PROC
        LDR      r1,|L1.1352|
        PUSH     {r4,lr}
        STR      r0,[r1,#4]  ; ePlatformType
        MOV      r0,#0
        STR      r0,[r1,#0xc]  ; nCurrentMilliseconds
        STRB     r0,[r1,#0]  ; nCurrentEpoch
        MOV      r0,#1
        BL       T0_SETUP_PERIODIC_INT
        POP      {r4,lr}
        MOV      r0,#0xa
        B        T1_SETUP_PERIODIC_INT
        ENDP

delayMs PROC
        PUSH     {lr}
        SUB      sp,sp,#0x14
        MOV      r1,#0
        CMP      r0,#0
        MOVEQ    r0,#1
        STRB     r1,[sp,#4]
        STR      r1,[sp,#0]
        MOV      r1,r0
        MOV      r0,sp
        BL       startTimer
        LDRB     r0,[sp,#4]
        LDR      r1,|L1.1352|
|L1.1244|
        LDRB     r2,[r1,#0]  ; nCurrentEpoch
        STRB     r2,[sp,#8]
        LDR      r2,[r1,#0xc]  ; nCurrentMilliseconds
        STR      r2,[sp,#0xc]
        LDRB     r2,[sp,#8]
        LDRB     r3,[r1,#0]  ; nCurrentEpoch
        CMP      r2,r3
        BEQ      |L1.1292|
        LDRB     r2,[r1,#0]  ; nCurrentEpoch
        STRB     r2,[sp,#8]
        LDR      r2,[r1,#0xc]  ; nCurrentMilliseconds
        STR      r2,[sp,#0xc]
|L1.1292|
        LDRB     r2,[sp,#8]
        CMP      r0,r2
        ADDCC    sp,sp,#0x14
        POPCC    {lr}
        BXCC     lr
        LDRB     r2,[sp,#8]
        CMP      r0,r2
        BNE      |L1.1244|
        LDR      r2,[sp,#0]
        LDR      r3,[sp,#0xc]
        CMP      r2,r3
        BCS      |L1.1244|
        ADD      sp,sp,#0x14
        POP      {lr}
        BX       lr
        ENDP

|L1.1352|
        DCD      ||.data||
|L1.1356|
        DCD      0x10624dd3

        AREA ||.arm_vfe_header||, DATA, READONLY, NOALLOC, ALIGN=2

        DCD      0x00000000

        AREA ||.data||, DATA, ALIGN=2

nCurrentEpoch
        DCB      0x00,0x00,0x00,0x00
ePlatformType
        DCD      0x00000000
nLEDCycles
        DCD      0x00000000
nCurrentMilliseconds
        DCD      0x00000000

        EXPORT T0Handler [CODE]
        EXPORT T1Handler [CODE]
        EXPORT initTimer [CODE]
        EXPORT T0_SETUP_PERIODIC_INT [CODE]
        EXPORT T1_SETUP_PERIODIC_INT [CODE]
        EXPORT timerShowLEDHeartbeat [CODE]
        EXPORT timerStopLEDHeartbeat [CODE]
        EXPORT getTimeNow [CODE]
        EXPORT hasTimedOut [CODE]
        EXPORT getTimer1Count [CODE]
        EXPORT startTimer [CODE]
        EXPORT isTimerExpired [CODE]
        EXPORT stopTimer [CODE]
        EXPORT timerInit [CODE]
        EXPORT delayMs [CODE]

        IMPORT ||Lib$$Request$$armlib|| [CODE,WEAK]
        IMPORT toggleGreenLED [CODE]
        IMPORT install_irq [CODE]

        KEEP nCurrentEpoch
        KEEP ePlatformType
        KEEP nLEDCycles
        KEEP nCurrentMilliseconds

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
