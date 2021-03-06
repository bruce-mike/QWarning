; generated by Component: ARM Compiler 5.04 update 1 (build 49) Tool: ArmCC [5040049]
; commandline ArmCC [--c99 --debug -c --asm -o.\objs\sensor.o --depend=.\objs\sensor.d --apcs=interwork -Otime --diag_suppress=9931 -IC:\Keil\ARM\INC\Phillips -I..\SharedCode -IC:\Keil_v5\ARM\RV31\INC -IC:\Keil_v5\ARM\CMSIS\Include -IC:\Keil_v5\ARM\Inc\Philips --enum_is_int --omf_browse=.\objs\sensor.crf sensor.c]
        ARM
        REQUIRE8
        PRESERVE8

        AREA ||.text||, CODE, READONLY, ALIGN=2

        REQUIRE _printf_percent
        REQUIRE _printf_d
        REQUIRE _printf_int_dec
        REQUIRE _printf_s
        REQUIRE _printf_str
sensorIrqHandler PROC
|L1.0|
        PUSH     {r0-r5,r12,lr}
        MOV      r4,#0x28000
        ADD      r4,r4,#0xe0000000
        LDR      r0,[r4,#0x88]
        LDR      r5,|L1.436|
        TST      r0,#0x20000
        BEQ      |L1.52|
        BL       getTimer1Count
        STR      r0,[r5,#8]  ; sensorFallingEdgeTimeStamp
        LDR      r0,|L1.440|
        LDRB     r1,[r0,#0x114]  ; sensorInfo
        ORR      r1,r1,#1
        STRB     r1,[r0,#0x114]  ; sensorInfo
|L1.52|
        LDR      r0,[r4,#0x84]
        TST      r0,#0x20000
        BEQ      |L1.72|
        BL       getTimer1Count
        STR      r0,[r5,#4]  ; sensorRisingEdgeTimeStamp
|L1.72|
        MVN      r0,#0
        STR      r0,[r4,#0x8c]
        STR      r0,[r4,#0xac]
        MOV      r0,#0
        STR      r0,[r0,#-0x100]
        POP      {r0-r5,r12,lr}
        SUBS     pc,lr,#4
        ENDP

getSensorOccupancy PROC
        LDR      r12,|L1.444|
        MOV      r1,#0
        LSL      r3,r0,#8
        MOV      r2,r12
        MOV      r0,r1
        UMLAL    r2,r0,r3,r12
        LDR      r3,|L1.440|
        PUSH     {r4,r5}
        LDRB     r4,[r3,#0x10e]  ; sensorInfo
        LSR      r0,r0,#9
        MVN      r2,#0xff0000
        ADD      r0,r0,#0x80
        AND      r12,r2,r0,LSR #8
        SUB      r0,r4,r12
        AND      r0,r0,#0x7f
|L1.160|
        ADD      r2,r3,r0
        LDRB     r5,[r2,#0x8e]
        ADD      r2,r0,#1
        CMP      r5,#0
        MOVNE    r0,#1
        MOVEQ    r0,#0
        ADD      r0,r0,r1
        BIC      r1,r0,#0x10000
        AND      r0,r2,#0x7f
        CMP      r4,r0
        BNE      |L1.160|
        AND      r0,r12,#0xff
        POP      {r4,r5}
        ORR      r0,r0,r1,LSL #8
        BIC      r0,r0,#0xff0000
        BX       lr
        ENDP

trafficStoppedState PROC
        PUSH     {r4-r8,lr}
        LDR      r7,|L1.440|
        MOV      r4,#0x86
        ADD      r4,r4,#0x2400
        LDR      r0,[r7,#8]  ; sensorInfo
        SUB      sp,sp,#8
        MOV      r1,r4
        BL       hasTimedOut
        CMP      r0,#0
        ADDEQ    sp,sp,#8
        POPEQ    {r4-r8,lr}
        BXEQ     lr
        BL       getTimeNow
        STR      r0,[r7,#8]  ; sensorInfo
        MOV      r0,r4
        BL       getSensorOccupancy
        MOV      r4,r0
        AND      r1,r0,#0xff
        LSR      r5,r0,#8
        MOV      r0,#0x64
        MUL      r0,r5,r0
        BL       __aeabi_uidivmod
        MOV      r8,r0
        MOV      r0,#0xb
        BL       getAvgSpeed
        CMP      r8,#0x4b
        MOV      r6,r0
        ADDCS    sp,sp,#8
        POPCS    {r4-r8,lr}
        BXCS     lr
        CMP      r6,#0x10
        ADDLS    sp,sp,#8
        POPLS    {r4-r8,lr}
        BXLS     lr
        LDRB     r0,[r7,#0x114]  ; sensorInfo
        BIC      r0,r0,#0x20
        STRB     r0,[r7,#0x114]  ; sensorInfo
        BL       getTimeNow
        STR      r0,[sp,#4]
        AND      r2,r4,#0xff
        MOV      r3,r8
        MOV      r1,r5
        ADR      r0,|L1.448|
        STR      r6,[sp,#0]
        BL       __2printf
        ADR      r0,|L1.488|
        BL       __2printf
        LDR      r0,|L1.528|
        LDR      r1,|L1.436|
        STR      r0,[r1,#0]  ; sensorState
        ADD      sp,sp,#8
        POP      {r4-r8,lr}
        BX       lr
|L1.436|
        DCD      ||.data||
|L1.440|
        DCD      ||.bss||+0xc
|L1.444|
        DCD      0x46d987e3
|L1.448|
        DCB      "Sensor: %d / %d  %d%%  Speed %d  <%d>\r\n",0
|L1.488|
        DCB      "Switch to checkTrafficStoppedState\r\n",0
        DCB      0
        DCB      0
        DCB      0
|L1.528|
        DCD      checkTrafficStoppedState
        ENDP

checkTrafficStoppedState PROC
        PUSH     {r4-r8,lr}
        LDR      r7,|L1.440|
        MOV      r4,#0x86
        ADD      r4,r4,#0x2400
        LDR      r0,[r7,#8]  ; sensorInfo
        SUB      sp,sp,#8
        MOV      r1,r4
        BL       hasTimedOut
        CMP      r0,#0
        ADDEQ    sp,sp,#8
        POPEQ    {r4-r8,lr}
        BXEQ     lr
        BL       getTimeNow
        STR      r0,[r7,#8]  ; sensorInfo
        MOV      r0,r4
        BL       getSensorOccupancy
        MOV      r4,r0
        AND      r1,r0,#0xff
        LSR      r5,r0,#8
        MOV      r0,#0x64
        MUL      r0,r5,r0
        BL       __aeabi_uidivmod
        MOV      r8,r0
        MOV      r0,#0xb
        BL       getAvgSpeed
        CMP      r8,#0x4b
        MOV      r6,r0
        ADDLS    sp,sp,#8
        POPLS    {r4-r8,lr}
        BXLS     lr
        CMP      r6,#8
        ADDCS    sp,sp,#8
        POPCS    {r4-r8,lr}
        BXCS     lr
        LDRB     r0,[r7,#0x114]  ; sensorInfo
        ORR      r0,r0,#0x20
        STRB     r0,[r7,#0x114]  ; sensorInfo
        BL       getTimeNow
        STR      r0,[sp,#4]
        AND      r2,r4,#0xff
        MOV      r3,r8
        MOV      r1,r5
        ADR      r0,|L1.448|
        STR      r6,[sp,#0]
        BL       __2printf
        ADR      r0,|L1.744|
        BL       __2printf
        LDR      r0,|L1.776|
        LDR      r1,|L1.436|
        STR      r0,[r1,#0]  ; sensorState
        ADD      sp,sp,#8
        POP      {r4-r8,lr}
        BX       lr
|L1.744|
        DCB      "Switch to trafficStoppedState\r\n",0
|L1.776|
        DCD      trafficStoppedState
        ENDP

unknownState PROC
        LDR      r1,|L1.440|
        MOV      r0,#0
|L1.788|
        ADD      r2,r1,r0
        LDRB     r2,[r2,#0xe]
        CMP      r2,#0
        BEQ      |L1.824|
        ADD      r0,r0,#1
        AND      r0,r0,#0xff
        CMP      r0,#0x80
        BCC      |L1.788|
        BX       lr
|L1.824|
        LDR      r0,|L1.528|
        LDR      r1,|L1.436|
        STR      r0,[r1,#0]  ; sensorState
        ADR      r0,|L1.488|
        B        __2printf
        ENDP

sensorInit PROC
        PUSH     {r3-r5,lr}
        MOV      r5,r0
        LDR      r0,|L1.440|
        MOV      r1,#0x118
        BL       __aeabi_memclr4
        LDR      r1,|L1.1060|
        LDR      r2,|L1.436|
        LDR      r4,|L1.440|
        STR      r1,[r2,#0]  ; sensorState
        MOV      r2,#1
        MOV      r1,sp
        MOV      r0,#0x20
        STR      r5,[r4,#0x110]  ; sensorInfo
        BL       readFramData
        LDR      r0,[sp,#0]
        MOV      r2,#1
        SUB      r1,r0,#6
        CMP      r1,#0x7c
        MOVCS    r0,#0x18
        STRB     r0,[r4,#0xc]  ; sensorInfo
        MOV      r1,sp
        MOV      r0,#0x21
        BL       readFramData
        LDR      r0,[sp,#0]
        SUB      r1,r0,#7
        CMP      r1,#0x7c
        MOVCS    r0,#0x82
        STRB     r0,[r4,#0xd]  ; sensorInfo
        BL       getTimeNow
        STR      r0,[r4,#0]  ; sensorInfo
        MOV      r2,#0xf
        ADR      r1,|L1.0|
        MOV      r0,#0x11
        BL       install_irq
        MOV      r0,#0x28000
        MVN      r1,#0
        ADD      r0,r0,#0xe0000000
        STR      r1,[r0,#0xac]
        MOV      r1,#0x20000
        STR      r1,[r0,#0x90]
        STR      r1,[r0,#0x94]
        POP      {r3-r5,lr}
        BX       lr
        ENDP

getDistanceString PROC
        AND      r1,r0,#3
        ADD      r2,r1,r1,LSL #3
        ADD      r3,r2,r1,LSL #4
        LSR      r2,r0,#2
        PUSH     {r4,lr}
        LDR      r0,|L1.1076|
        ADR      r1,|L1.1064|
        BL       __2sprintf
        POP      {r4,lr}
        LDR      r0,|L1.1076|
        BX       lr
|L1.1060|
        DCD      unknownState
|L1.1064|
        DCB      "%d.%d Ft",0
        DCB      0
        DCB      0
        DCB      0
|L1.1076|
        DCD      ||.bss||
        ENDP

sensorDoWork PROC
        PUSH     {r4-r8,lr}
        LDR      r5,|L1.440|
        MOV      r1,#0x388
        LDR      r0,[r5,#0]  ; sensorInfo
        ADD      r1,r1,#0x1000
        BL       hasTimedOut
        CMP      r0,#0
        BEQ      |L1.1124|
        LDRB     r0,[r5,#0x114]  ; sensorInfo
        TST      r0,#2
        BEQ      |L1.1140|
|L1.1124|
        LDRB     r0,[r5,#0x114]  ; sensorInfo
        TST      r0,#1
        BNE      |L1.1212|
        B        |L1.1196|
|L1.1140|
        LDRB     r0,[r5,#0x114]  ; sensorInfo
        ORR      r0,r0,#2
        STRB     r0,[r5,#0x114]  ; sensorInfo
        BL       startUpUltraSonicTrigger
        BL       getTimeNow
        STR      r0,[r5,#0]  ; sensorInfo
        LDRB     r0,[r5,#0x114]  ; sensorInfo
        ORR      r0,r0,#0x10
        STRB     r0,[r5,#0x114]  ; sensorInfo
        BL       getTimeNow
        POP      {r4-r8,lr}
        MOV      r1,r0
        ADR      r0,|L1.1696|
        B        __2printf
|L1.1196|
        LDRB     r0,[r5,#0x114]  ; sensorInfo
        TST      r0,#0x10
        POPNE    {r4-r8,lr}
        BXNE     lr
|L1.1212|
        LDRB     r0,[r5,#0x114]  ; sensorInfo
        TST      r0,#1
        LDRBNE   r0,[r5,#0x114]  ; sensorInfo
        TSTNE    r0,#0x10
        BEQ      |L1.1644|
        LDRB     r0,[r5,#0x114]  ; sensorInfo
        BIC      r0,r0,#1
        STRB     r0,[r5,#0x114]  ; sensorInfo
        LDRB     r0,[r5,#0x114]  ; sensorInfo
        BIC      r0,r0,#2
        STRB     r0,[r5,#0x114]  ; sensorInfo
        LDR      r6,|L1.436|
        LDR      r0,[r6,#8]  ; sensorFallingEdgeTimeStamp
        LDR      r1,[r6,#4]  ; sensorRisingEdgeTimeStamp
        LDRB     r2,[r5,#0xc]  ; sensorInfo
        SUB      r0,r0,r1
        ADD      r1,r0,r0,LSL #4
        ADD      r0,r1,r0,LSL #5
        ADD      r0,r0,r0,LSL #9
        LSR      r4,r0,#24
        CMP      r2,r4
        LDRBCC   r2,[r5,#0xd]  ; sensorInfo
        LDR      r0,|L1.1728|
        MOV      r1,#0
        CMPCC    r4,r2
        LDRB     r2,[r5,#0x10e]  ; sensorInfo
        MOVCC    r1,#1
        LDR      r8,|L1.444|
        STRB     r1,[r2,r0]
        LDRB     r1,[r5,#0x10e]  ; sensorInfo
        LDR      r0,|L1.1732|
        MOV      r7,#0xa60
        STRB     r4,[r1,r0]
        LDRB     r0,[r5,#0x10e]  ; sensorInfo
        ADD      r7,r7,#0xe000
        MOV      r2,#0
        ADD      r0,r0,#1
        AND      r1,r0,#0x7f
        LSL      r12,r7,#8
        MOV      r3,r8
        MOV      r0,r2
        UMLAL    r3,r0,r12,r8
        LSR      r0,r0,#9
        MVN      r3,#0xff0000
        ADD      r0,r0,#0x80
        AND      r3,r3,r0,LSR #8
        SUB      r0,r1,r3
        AND      r0,r0,#0x7f
        STRB     r1,[r5,#0x10e]  ; sensorInfo
|L1.1408|
        ADD      r12,r5,r0
        LDRB     r12,[r12,#0xe]
        ADD      r0,r0,#1
        AND      r0,r0,#0x7f
        CMP      r1,r0
        ADD      r2,r2,r12
        BNE      |L1.1408|
        MOV      r1,r3
        MOV      r0,r2
        BL       __aeabi_uidivmod
        AND      r8,r0,#0xff
        LDRB     r0,[r5,#0xc]  ; sensorInfo
        CMP      r0,r8
        LDRB     r0,[r5,#0x114]  ; sensorInfo
        BLS      |L1.1600|
        TST      r0,#8
        BEQ      |L1.1576|
        LDR      r0,[r5,#4]  ; sensorInfo
        MOV      r1,r7
        BL       hasTimedOut
        CMP      r0,#0
        BEQ      |L1.1620|
        LDRB     r0,[r5,#0x114]  ; sensorInfo
        ORR      r0,r0,#4
        STRB     r0,[r5,#0x114]  ; sensorInfo
        LDRB     r0,[r5,#0x114]  ; sensorInfo
        TST      r0,#4
        BEQ      |L1.1620|
        BL       getTimeNow
        MOV      r7,r0
        AND      r0,r8,#3
        ADD      r1,r0,r0,LSL #3
        ADD      r3,r1,r0,LSL #4
        LDR      r0,|L1.1076|
        LSR      r2,r8,#2
        ADR      r1,|L1.1064|
        BL       __2sprintf
        LDR      r1,|L1.1076|
        MOV      r2,r7
        ADR      r0,|L1.1736|
        BL       __2printf
        B        |L1.1620|
|L1.1576|
        LDRB     r0,[r5,#0x114]  ; sensorInfo
        ORR      r0,r0,#8
        STRB     r0,[r5,#0x114]  ; sensorInfo
        BL       getTimeNow
        STR      r0,[r5,#4]  ; sensorInfo
        B        |L1.1620|
|L1.1600|
        BIC      r0,r0,#8
        STRB     r0,[r5,#0x114]  ; sensorInfo
        LDRB     r0,[r5,#0x114]  ; sensorInfo
        BIC      r0,r0,#4
        STRB     r0,[r5,#0x114]  ; sensorInfo
|L1.1620|
        BL       getTimeNow
        STR      r0,[r5,#0]  ; sensorInfo
        LDR      r1,[r6,#0]  ; sensorState
        MOV      r0,r4
        POP      {r4-r8,lr}
        BX       r1
|L1.1644|
        MOV      r1,#0x3a
        LDR      r0,[r5,#0]  ; sensorInfo
        ADD      r1,r1,#0x700
        BL       hasTimedOut
        CMP      r0,#0
        POPEQ    {r4-r8,lr}
        BXEQ     lr
        BL       setHighUltraSonicTrigger
        LDRB     r0,[r5,#0x114]  ; sensorInfo
        ORR      r0,r0,#0x10
        STRB     r0,[r5,#0x114]  ; sensorInfo
        POP      {r4-r8,lr}
        BX       lr
|L1.1696|
        DCB      "ERROR:Sensor Timed Out  <%d>\r\n",0
        DCB      0
|L1.1728|
        DCD      ||.bss||+0x9a
|L1.1732|
        DCD      ||.bss||+0x1a
|L1.1736|
        DCB      "ERROR:Sensor Blocked  Avg: %s <%d>\r\n",0
        DCB      0
        DCB      0
        DCB      0
        ENDP

saveSensorTriggerDistanceLow PROC
        PUSH     {r3,lr}
        STRB     r0,[sp,#0]
        SUB      r0,r0,#6
        CMP      r0,#0x7c
        MOVCS    r0,#0x18
        STRBCS   r0,[sp,#0]
        LDRB     r0,[sp,#0]
        LDR      r1,|L1.440|
        MOV      r2,#1
        STRB     r0,[r1,#0xc]  ; sensorInfo
        MOV      r1,sp
        MOV      r0,#0x20
        BL       writeFramData
        LDRB     r1,[sp,#0]
        AND      r0,r1,#3
        ADD      r2,r0,r0,LSL #3
        ADD      r2,r2,r0,LSL #4
        LSR      r1,r1,#2
        ADR      r0,|L1.1864|
        BL       __2printf
        POP      {r3,lr}
        BX       lr
|L1.1864|
        DCB      "Saving trigger distance low %d.%d feet\r\n",0
        DCB      0
        DCB      0
        DCB      0
        ENDP

saveSensorTriggerDistanceHigh PROC
        PUSH     {r3,lr}
        STRB     r0,[sp,#0]
        SUB      r0,r0,#7
        CMP      r0,#0x7c
        MOVCS    r0,#0x82
        STRBCS   r0,[sp,#0]
        LDRB     r0,[sp,#0]
        LDR      r1,|L1.440|
        MOV      r2,#1
        STRB     r0,[r1,#0xd]  ; sensorInfo
        MOV      r1,sp
        MOV      r0,#0x21
        BL       writeFramData
        LDRB     r1,[sp,#0]
        AND      r0,r1,#3
        ADD      r2,r0,r0,LSL #3
        ADD      r2,r2,r0,LSL #4
        LSR      r1,r1,#2
        ADR      r0,|L1.1996|
        BL       __2printf
        POP      {r3,lr}
        BX       lr
|L1.1996|
        DCB      "Saving trigger distance high %d.%d feet\r\n",0
        DCB      0
        DCB      0
        ENDP

getAverageDistance PROC
        LDR      r12,|L1.444|
        MOV      r0,#0
        MOV      r3,#0x3b00
        ADD      r3,r3,#0x70000
        MOV      r2,r12
        MOV      r1,r0
        UMLAL    r2,r1,r3,r12
        LDR      r3,|L1.440|
        PUSH     {r4,lr}
        LDRB     r12,[r3,#0x10e]  ; sensorInfo
        LSR      r1,r1,#9
        MVN      r2,#0xff0000
        ADD      r1,r1,#0x80
        AND      r1,r2,r1,LSR #8
        SUB      r2,r12,r1
        AND      r2,r2,#0x7f
|L1.2104|
        ADD      r4,r3,r2
        LDRB     r4,[r4,#0xe]
        ADD      r2,r2,#1
        AND      r2,r2,#0x7f
        CMP      r12,r2
        ADD      r0,r0,r4
        BNE      |L1.2104|
        BL       __aeabi_uidivmod
        POP      {r4,lr}
        AND      r0,r0,#0xff
        BX       lr
        ENDP

getSensorTriggerDistanceLow PROC
        LDR      r0,|L1.440|
        LDRB     r0,[r0,#0xc]  ; sensorInfo
        BX       lr
        ENDP

getSensorTriggerDistanceHigh PROC
        LDR      r0,|L1.440|
        LDRB     r0,[r0,#0xd]  ; sensorInfo
        BX       lr
        ENDP

getLasSensortReading PROC
        LDR      r0,|L1.440|
        LDRB     r1,[r0,#0x10e]  ; sensorInfo
        SUB      r1,r1,#1
        AND      r1,r1,#0x7f
        ADD      r0,r0,r1
        LDRB     r0,[r0,#0xe]
        BX       lr
        ENDP

getSensorOperatingStatus PROC
        LDR      r2,|L1.440|
        LDRB     r0,[r2,#0x114]  ; sensorInfo
        LDRB     r1,[r2,#0x114]  ; sensorInfo
        ANDS     r0,r0,#2
        MOVNE    r0,#8
        ANDS     r1,r1,#4
        MOVNE    r1,#4
        ORR      r1,r1,r0
        LDRB     r0,[r2,#0x114]  ; sensorInfo
        ANDS     r0,r0,#0x20
        MOVNE    r0,#0x800
        ORR      r0,r0,r1
        BX       lr
        ENDP


        AREA ||.arm_vfe_header||, DATA, READONLY, NOALLOC, ALIGN=2

        DCD      0x00000000

        AREA ||.bss||, DATA, NOINIT, ALIGN=2

feetString
        %        12
sensorInfo
        %        280

        AREA ||.data||, DATA, ALIGN=2

sensorState
        DCD      0x00000000
sensorRisingEdgeTimeStamp
        DCD      0x00000000
sensorFallingEdgeTimeStamp
        DCD      0x00000000

        EXPORT sensorIrqHandler [CODE]
        EXPORT getSensorOccupancy [CODE]
        EXPORT sensorInit [CODE]
        EXPORT getDistanceString [CODE]
        EXPORT sensorDoWork [CODE]
        EXPORT saveSensorTriggerDistanceLow [CODE]
        EXPORT saveSensorTriggerDistanceHigh [CODE]
        EXPORT getAverageDistance [CODE]
        EXPORT getSensorTriggerDistanceLow [CODE]
        EXPORT getSensorTriggerDistanceHigh [CODE]
        EXPORT getLasSensortReading [CODE]
        EXPORT getSensorOperatingStatus [CODE]

        IMPORT ||Lib$$Request$$armlib|| [CODE,WEAK]
        IMPORT getTimer1Count [CODE]
        IMPORT hasTimedOut [CODE]
        IMPORT getTimeNow [CODE]
        IMPORT __aeabi_uidivmod [CODE]
        IMPORT getAvgSpeed [CODE]
        IMPORT _printf_percent [CODE]
        IMPORT _printf_d [CODE]
        IMPORT _printf_int_dec [CODE]
        IMPORT __2printf [CODE]
        IMPORT __aeabi_memclr4 [CODE]
        IMPORT readFramData [CODE]
        IMPORT install_irq [CODE]
        IMPORT __2sprintf [CODE]
        IMPORT startUpUltraSonicTrigger [CODE]
        IMPORT _printf_s [CODE]
        IMPORT _printf_str [CODE]
        IMPORT setHighUltraSonicTrigger [CODE]
        IMPORT writeFramData [CODE]

        KEEP trafficStoppedState
        KEEP checkTrafficStoppedState
        KEEP unknownState
        KEEP feetString
        KEEP sensorInfo
        KEEP sensorState
        KEEP sensorRisingEdgeTimeStamp
        KEEP sensorFallingEdgeTimeStamp

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
