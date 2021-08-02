; generated by Component: ARM Compiler 5.04 update 1 (build 49) Tool: ArmCC [5040049]
; commandline ArmCC [--c99 --debug -c --asm -o.\objs\serial.o --depend=.\objs\serial.d --apcs=interwork -Otime --diag_suppress=9931 -IC:\Keil\ARM\INC\Phillips -I..\SharedCode -IC:\Keil_v5\ARM\RV31\INC -IC:\Keil_v5\ARM\CMSIS\Include -IC:\Keil_v5\ARM\Inc\Philips --enum_is_int --omf_browse=.\objs\serial.crf serial.c]
        ARM
        REQUIRE8
        PRESERVE8

        AREA ||.text||, CODE, READONLY, ALIGN=2

UART1_HANDLER PROC
        PUSH     {r0}
        MOV      r0,#0
        STR      r0,[r0,#-0x100]
        POP      {r0}
        SUBS     pc,lr,#4
        ENDP

UART3_HANDLER PROC
        PUSH     {r0-r5,r12}
        MOV      r12,#0x7c000
        ADD      r12,r12,#0xe0000000
        LDR      r0,[r12,#4]
        TST      r0,#1
        LDRNE    r0,[r12,#0x14]
        TSTNE    r0,#1
        BEQ      |L1.112|
        LDR      r3,|L1.736|
        LDR      r5,|L1.740|
        LDRH     r4,[r3,#0xa]
|L1.64|
        LDR      r0,[r12,#0]
        LDRH     r1,[r3,#8]  ; nRxRadarAddToIndex
        AND      r2,r0,#0xff
        ADD      r0,r1,#1
        CMP      r0,#0x18
        MOVCS    r0,#0
        CMP      r0,r4
        STRBNE   r2,[r5,r1]
        STRHNE   r0,[r3,#8]  ; nRxRadarAddToIndex
        LDR      r0,[r12,#0x14]
        TST      r0,#1
        BNE      |L1.64|
|L1.112|
        MOV      r0,#0
        STR      r0,[r0,#-0x100]
        POP      {r0-r5,r12}
        SUBS     pc,lr,#4
        ENDP

serialInitPort PROC
        MOV      r1,#0x2c000
        CMP      r0,#0
        MOV      r2,#1
        MOV      r3,#0
        MOV      r12,#0x13
        ADD      r1,r1,#0xe0000000
        PUSH     {r4}
        BEQ      |L1.200|
        CMP      r0,#1
        BEQ      |L1.256|
        CMP      r0,#2
        BEQ      |L1.328|
        CMP      r0,#3
        MVNNE    r2,#0
        BEQ      |L1.372|
|L1.188|
        POP      {r4}
        MOV      r0,r2
        BX       lr
|L1.200|
        MOV      r0,#0xc000
        ADD      r0,r0,#0xe0000000
        LDR      r4,[r0,#0xc]
        ORR      r4,r4,#0x83
        STR      r4,[r0,#0xc]
        STR      r3,[r0,#4]
        STR      r12,[r0,#0]
        LDR      r3,[r0,#0xc]
        BIC      r3,r3,#0x80
        STR      r3,[r0,#0xc]
        LDR      r0,[r1,#0]
        ORR      r0,r0,#0x50
        STR      r0,[r1,#0]
        B        |L1.188|
|L1.256|
        MOV      r0,#0x10000
        ADD      r0,r0,#0xe0000000
        LDR      r12,[r0,#0xc]
        ORR      r12,r12,#0x83
        STR      r12,[r0,#0xc]
        STR      r3,[r0,#4]
        MOV      r3,#0xe4
        STR      r3,[r0,#0]
        LDR      r3,[r0,#0xc]
        BIC      r3,r3,#0x80
        STR      r3,[r0,#0xc]
        LDR      r0,[r1,#0]
        ORR      r0,r0,#0x40000000
        STR      r0,[r1,#0]
        LDR      r0,[r1,#4]
        ORR      r0,r0,#1
        STR      r0,[r1,#4]
        B        |L1.188|
|L1.328|
        MOV      r0,#0x78000
        ADD      r0,r0,#0xe0000000
        LDR      r1,[r0,#0xc]
        ORR      r1,r1,#0x83
        STR      r1,[r0,#0xc]
        STR      r3,[r0,#4]
        STR      r12,[r0,#0]
        LDR      r1,[r0,#0xc]
        BIC      r1,r1,#0x80
        STR      r1,[r0,#0xc]
        B        |L1.188|
|L1.372|
        MOV      r0,#0x7c000
        ADD      r0,r0,#0xe0000000
        LDR      r1,[r0,#0xc]
        ORR      r1,r1,#0x83
        STR      r1,[r0,#0xc]
        MOV      r1,#7
        STR      r1,[r0,#4]
        MOV      r1,#0x20
        STR      r1,[r0,#0]
        LDR      r1,[r0,#0xc]
        BIC      r1,r1,#0x80
        STR      r1,[r0,#0xc]
        B        |L1.188|
        ENDP

sendchar PROC
        MOV      r1,#0xc000
        CMP      r0,#0xa
        ADD      r1,r1,#0xe0000000
        BNE      |L1.460|
|L1.440|
        LDR      r2,[r1,#0x14]
        TST      r2,#0x20
        BEQ      |L1.440|
        MOV      r2,#0xd
        STR      r2,[r1,#0]
|L1.460|
        LDR      r2,[r1,#0x14]
        TST      r2,#0x20
        BEQ      |L1.460|
        STR      r0,[r1,#0]
        BX       lr
        ENDP

RxDataReady PROC
        MOV      r0,#0xc000
        ADD      r0,r0,#0xe0000000
        LDR      r0,[r0,#0x14]
        AND      r0,r0,#1
        BX       lr
        ENDP

serialDisableInterface PROC
        PUSH     {r4-r6,lr}
        CMP      r0,#0
        BEQ      |L1.556|
        CMP      r0,#1
        MOV      r5,#6
        BEQ      |L1.584|
        CMP      r0,#2
        MOV      r6,#0xc1
        BEQ      |L1.632|
        CMP      r0,#3
        BEQ      |L1.680|
        POP      {r4-r6,lr}
        ADR      r0,|L1.744|
        B        __2printf
|L1.556|
        MOV      r0,#0xc000
        ADD      r0,r0,#0xe0000000
        LDR      r1,[r0,#4]
        BIC      r1,r1,#1
        STR      r1,[r0,#4]
        POP      {r4-r6,lr}
        BX       lr
|L1.584|
        MOV      r4,#0x10000
        MOV      r0,#0
        ADD      r4,r4,#0xe0000000
        STR      r0,[r4,#4]
        MOV      r0,#1
        BL       delayMs
        LDR      r0,[r4,#0x14]
        CMP      r0,#0x60
        LDRNE    r0,[r4,#0]
        STR      r5,[r4,#8]
        POP      {r4-r6,lr}
        BX       lr
|L1.632|
        MOV      r4,#0x78000
        ADD      r4,r4,#0xe0000000
        LDR      r0,[r4,#4]
        BIC      r0,r0,#1
        STR      r0,[r4,#4]
        MOV      r0,#1
        BL       delayMs
        LDR      r0,[r4,#0x14]
        CMP      r0,#0x60
        BEQ      |L1.720|
|L1.672|
        LDR      r0,[r4,#0]
        B        |L1.720|
|L1.680|
        MOV      r4,#0x7c000
        ADD      r4,r4,#0xe0000000
        LDR      r0,[r4,#4]
        BIC      r0,r0,#1
        STR      r0,[r4,#4]
        MOV      r0,#1
        BL       delayMs
        LDR      r0,[r4,#0x14]
        CMP      r0,#0x60
        BNE      |L1.672|
|L1.720|
        STR      r5,[r4,#8]
        STR      r6,[r4,#8]
        POP      {r4-r6,lr}
        BX       lr
|L1.736|
        DCD      ||.data||
|L1.740|
        DCD      ||.bss||+0x64
|L1.744|
        DCB      "serialDisableInterface() - unknown port\r\n",0
        DCB      0
        DCB      0
        ENDP

serialEnableInterface PROC
        CMP      r0,#0
        CMPNE    r0,#1
        BXEQ     lr
        PUSH     {r4-r6,lr}
        LDR      r4,|L1.736|
        CMP      r0,#2
        MOV      r1,#0
        BEQ      |L1.924|
        CMP      r0,#3
        POPNE    {r4-r6,lr}
        BXNE     lr
        MOV      r5,#0x7c000
        ADD      r5,r5,#0xe0000000
        STR      r1,[r5,#8]
        STRH     r1,[r4,#8]  ; nRxRadarAddToIndex
        STRH     r1,[r4,#0xa]  ; nRxRadarRemoveFromIndex
        STRH     r1,[r4,#0xc]  ; nTxRadarRemoveFromIndex
        STRH     r1,[r4,#4]  ; nTxRadarAddToIndex
        LDR      r0,|L1.740|
        MOV      r1,#0x18
        BL       __aeabi_memclr
        LDR      r0,|L1.1984|
        MOV      r1,#0x18
        BL       __aeabi_memclr
        LDR      r0,|L1.1988|
        MOV      r1,#0x64
        BL       __aeabi_memclr
        MVN      r0,#0
        STRH     r0,[r4,#0xe]  ; rxRadarPacketIndex
        LDR      r0,[r5,#4]
        ORR      r0,r0,#1
        STR      r0,[r5,#4]
        POP      {r4-r6,lr}
        BX       lr
|L1.924|
        MOV      r5,#0x78000
        ADD      r5,r5,#0xe0000000
        STR      r1,[r5,#8]
        STRH     r1,[r4,#6]  ; nTxAssetRemoveFromIndex
        STRH     r1,[r4,#2]  ; nTxAssetAddToIndex
        LDR      r0,|L1.1992|
        MOV      r1,#0x64
        BL       __aeabi_memclr
        LDR      r0,[r5,#4]
        ORR      r0,r0,#1
        STR      r0,[r5,#4]
        POP      {r4-r6,lr}
        BX       lr
        ENDP

serialSendByteToAsset PROC
        PUSH     {r4-r6}
        MOV      r4,#0x78000
        ADD      r4,r4,#0xe0000000
        LDR      r1,[r4,#4]
        MOV      r2,#0
        TST      r1,#2
        BEQ      |L1.1020|
        LDR      r1,[r4,#4]
        BIC      r1,r1,#2
        STR      r1,[r4,#4]
        MOV      r2,#1
|L1.1020|
        LDR      r5,|L1.736|
        LDR      r6,|L1.1992|
        LDRSH    r12,[r5,#2]  ; nTxAssetAddToIndex
        LDRH     r3,[r5,#6]  ; nTxAssetRemoveFromIndex
        ADD      r1,r12,#1
        CMP      r1,#0x64
        MOVCS    r1,#0
        CMP      r1,r3
        STRBNE   r0,[r6,r12]
        STRHNE   r1,[r5,#2]  ; nTxAssetAddToIndex
        CMP      r2,#0
        BEQ      |L1.1088|
|L1.1068|
        LDR      r0,[r4,#4]
        ORR      r0,r0,#2
        STR      r0,[r4,#4]
        POP      {r4-r6}
        BX       lr
|L1.1088|
        ADD      r0,r3,#1
        BIC      r0,r0,#0x10000
        LDRB     r1,[r6,r3]
        CMP      r0,#0x64
        STRH     r0,[r5,#6]  ; nTxAssetRemoveFromIndex
        MOVCS    r0,#0
        STRHCS   r0,[r5,#6]  ; nTxAssetRemoveFromIndex
        STR      r1,[r4,#0]
        B        |L1.1068|
        ENDP

switchUart2RxBuffers PROC
        MOV      r1,#0x78000
        ADD      r1,r1,#0xe0000000
        LDR      r0,[r1,#4]
        BIC      r0,r0,#1
        STR      r0,[r1,#4]
        LDR      r0,|L1.736|
        MOV      r2,#0
        LDRB     r12,[r0,#0]  ; uart2DataIndex
        MOV      r3,#1
        CMP      r12,#1
        STRBEQ   r2,[r0,#0]  ; uart2DataIndex
        STRBEQ   r3,[r0,#1]  ; uart2IsrIndex
        STRBNE   r3,[r0,#0]  ; uart2DataIndex
        STRBNE   r2,[r0,#1]  ; uart2IsrIndex
        LDR      r0,[r1,#4]
        ORR      r0,r0,#1
        STR      r0,[r1,#4]
        BX       lr
        ENDP

assetSerialDoWork PROC
        PUSH     {r4-r8,lr}
        MOV      r0,#0x78000
        ADD      r0,r0,#0xe0000000
        LDR      r1,[r0,#4]
        BIC      r1,r1,#1
        STR      r1,[r0,#4]
        LDR      r5,|L1.736|
        MOV      r7,#0
        LDRB     r2,[r5,#0]  ; uart2DataIndex
        MOV      r1,#1
        CMP      r2,#1
        STRBEQ   r7,[r5,#0]  ; uart2DataIndex
        STRBEQ   r1,[r5,#1]  ; uart2IsrIndex
        STRBNE   r1,[r5,#0]  ; uart2DataIndex
        STRBNE   r7,[r5,#1]  ; uart2IsrIndex
        LDR      r1,[r0,#4]
        ORR      r1,r1,#1
        STR      r1,[r0,#4]
        LDRB     r0,[r5,#0]  ; uart2DataIndex
        LDR      r6,|L1.1996|
        MOV      r4,#0
        ADD      r1,r0,r0,LSL #1
        ADD      r0,r1,r0,LSL #3
        ADD      r0,r6,r0,LSL #5
        LDRH     r0,[r0,#0]
        CMP      r0,#0
        BLS      |L1.1368|
|L1.1304|
        LDRB     r0,[r5,#0]  ; uart2DataIndex
        ADD      r1,r0,r0,LSL #1
        ADD      r0,r1,r0,LSL #3
        ADD      r0,r6,r0,LSL #5
        ADD      r0,r0,r4
        LDRB     r0,[r0,#2]
        BL       buildRxAssetPacket
        ADD      r0,r4,#1
        BIC      r4,r0,#0x10000
        LDRB     r0,[r5,#0]  ; uart2DataIndex
        ADD      r1,r0,r0,LSL #1
        ADD      r0,r1,r0,LSL #3
        ADD      r0,r6,r0,LSL #5
        LDRH     r0,[r0,#0]
        CMP      r0,r4
        BHI      |L1.1304|
|L1.1368|
        LDRB     r0,[r5,#0]  ; uart2DataIndex
        ADD      r1,r0,r0,LSL #1
        ADD      r0,r1,r0,LSL #3
        ADD      r0,r6,r0,LSL #5
        STRH     r7,[r0,#0]
        POP      {r4-r8,lr}
        BX       lr
        ENDP

UART2_HANDLER PROC
|L1.1396|
        PUSH     {r0-r5,r12}
        MOV      r2,#0x78000
        ADD      r2,r2,#0xe0000000
        LDR      r0,[r2,#4]
        LDR      r3,|L1.736|
        TST      r0,#1
        BEQ      |L1.1512|
        LDRB     r0,[r3,#1]  ; uart2IsrIndex
        LDR      r12,[r2,#0x14]
        ADD      r1,r0,r0,LSL #1
        ADD      r1,r1,r0,LSL #3
        LDR      r0,|L1.1996|
        ADD      r1,r0,r1,LSL #5
        LDRH     r0,[r1,#0]
        TST      r12,#1
        MOVNE    r12,#0x5e
        ADDNE    r12,r12,#0x100
        BEQ      |L1.1508|
|L1.1468|
        CMP      r12,r0
        BLS      |L1.1496|
        LDR      r4,[r2,#0]
        ADD      r5,r1,r0
        ADD      r0,r0,#1
        BIC      r0,r0,#0x10000
        STRB     r4,[r5,#2]
|L1.1496|
        LDR      r4,[r2,#0x14]
        TST      r4,#1
        BNE      |L1.1468|
|L1.1508|
        STRH     r0,[r1,#0]
|L1.1512|
        LDR      r0,[r2,#4]
        MOV      r4,#0
        TST      r0,#2
        LDRNE    r0,[r2,#0x14]
        TSTNE    r0,#0x20
        LDRSHNE  r1,[r3,#2]
        LDRNE    r12,|L1.1992|
        BEQ      |L1.1624|
|L1.1544|
        LDRH     r0,[r3,#6]  ; nTxAssetRemoveFromIndex
        CMP      r0,r1
        BEQ      |L1.1596|
        LDRB     r5,[r12,r0]
        STR      r5,[r2,#0]
        ADD      r0,r0,#1
        BIC      r0,r0,#0x10000
        CMP      r0,#0x64
        STRH     r0,[r3,#6]  ; nTxAssetRemoveFromIndex
        STRHCS   r4,[r3,#6]  ; nTxAssetRemoveFromIndex
        LDRH     r0,[r3,#6]  ; nTxAssetRemoveFromIndex
        CMP      r0,r1
        BNE      |L1.1612|
|L1.1596|
        LDR      r0,[r2,#4]
        BIC      r0,r0,#2
        STR      r0,[r2,#4]
        B        |L1.1624|
|L1.1612|
        LDR      r0,[r2,#0x14]
        TST      r0,#0x20
        BNE      |L1.1544|
|L1.1624|
        STR      r4,[r4,#-0x100]
        POP      {r0-r5,r12}
        SUBS     pc,lr,#4
        ENDP

serialSendByteToRadar PROC
        PUSH     {r4-r6}
        MOV      r4,#0x7c000
        ADD      r4,r4,#0xe0000000
        LDR      r1,[r4,#4]
        MOV      r2,#0
        TST      r1,#2
        BEQ      |L1.1680|
        LDR      r1,[r4,#4]
        BIC      r1,r1,#2
        STR      r1,[r4,#4]
        MOV      r2,#1
|L1.1680|
        LDR      r5,|L1.736|
        LDR      r6,|L1.1984|
        LDRSH    r12,[r5,#4]  ; nTxRadarAddToIndex
        LDRH     r3,[r5,#0xc]  ; nTxRadarRemoveFromIndex
        ADD      r1,r12,#1
        CMP      r1,#0x18
        MOVCS    r1,#0
        CMP      r1,r3
        STRBNE   r0,[r6,r12]
        STRHNE   r1,[r5,#4]  ; nTxRadarAddToIndex
        CMP      r2,#0
        BEQ      |L1.1748|
|L1.1728|
        LDR      r0,[r4,#4]
        ORR      r0,r0,#2
        STR      r0,[r4,#4]
        POP      {r4-r6}
        BX       lr
|L1.1748|
        ADD      r0,r3,#1
        BIC      r0,r0,#0x10000
        LDRB     r1,[r6,r3]
        CMP      r0,#0x18
        STRH     r0,[r5,#0xc]  ; nTxRadarRemoveFromIndex
        MOVCS    r0,#0
        STRHCS   r0,[r5,#0xc]  ; nTxRadarRemoveFromIndex
        STR      r1,[r4,#0]
        B        |L1.1728|
        ENDP

radarSerialDoWork PROC
        PUSH     {r4-r8,lr}
        MOV      r5,#0x7c000
        ADD      r5,r5,#0xe0000000
        LDR      r0,[r5,#4]
        MOV      r4,#0
        LDR      r7,|L1.740|
        LDR      r6,|L1.736|
        TST      r0,#1
        MOVNE    r4,#1
        MOV      r8,#0
|L1.1824|
        LDR      r0,[r5,#4]
        BIC      r0,r0,#1
        STR      r0,[r5,#4]
        LDRH     r0,[r6,#8]  ; nRxRadarAddToIndex
        LDRH     r1,[r6,#0xa]  ; nRxRadarRemoveFromIndex
        CMP      r0,r1
        BEQ      |L1.1912|
        LDRB     r0,[r7,r1]
        ADD      r1,r1,#1
        BIC      r1,r1,#0x10000
        CMP      r1,#0x18
        STRH     r1,[r6,#0xa]  ; nRxRadarRemoveFromIndex
        STRHCS   r8,[r6,#0xa]  ; nRxRadarRemoveFromIndex
        CMP      r4,#0
        BEQ      |L1.1896|
        LDR      r1,[r5,#4]
        ORR      r1,r1,#1
        STR      r1,[r5,#4]
|L1.1896|
        LDR      r2,|L1.2000|
        LDR      r1,|L1.1988|
        BL       buildRxRadarPacket
        B        |L1.1824|
|L1.1912|
        CMP      r4,#0
        POPEQ    {r4-r8,lr}
        BXEQ     lr
        LDR      r0,[r5,#4]
        ORR      r0,r0,#1
        STR      r0,[r5,#4]
        POP      {r4-r8,lr}
        BX       lr
        ENDP

serialWrite PROC
        CMP      r0,#0
        BEQ      |L1.1976|
        CMP      r0,#1
        CMPNE    r0,#2
        CMPNE    r0,#4
        BXEQ     lr
        ADR      r0,|L1.2004|
        B        __2printf
|L1.1976|
        ADR      r0,|L1.2060|
        B        __2printf
|L1.1984|
        DCD      ||.bss||+0x7c
|L1.1988|
        DCD      ||.bss||+0x94
|L1.1992|
        DCD      ||.bss||
|L1.1996|
        DCD      ||.bss||+0xf8
|L1.2000|
        DCD      ||.data||+0xe
|L1.2004|
        DCB      "serialWrite(Unknown eInterface) defaulting to wireless\n"
        DCB      0
|L1.2060|
        DCB      " ",0
        DCB      0
        DCB      0
        ENDP

serialInit PROC
        PUSH     {r4,lr}
        MOV      r0,#0
        BL       serialInitPort
        MOV      r0,#0
        BL       serialEnableInterface
        MOV      r0,#2
        BL       serialInitPort
        MOV      r0,#0x78000
        ADD      r0,r0,#0xe0000000
        LDR      r1,[r0,#4]
        BIC      r1,r1,#2
        STR      r1,[r0,#4]
        MOV      r2,#1
        ADR      r1,|L1.1396|
        MOV      r0,#0x1c
        BL       install_irq
        MOV      r0,#2
        BL       serialEnableInterface
        MOV      r0,#3
        BL       serialInitPort
        MOV      r0,#0x7c000
        ADD      r0,r0,#0xe0000000
        LDR      r1,[r0,#4]
        BIC      r1,r1,#2
        STR      r1,[r0,#4]
        LDR      r1,|L1.2192|
        MOV      r2,#1
        MOV      r0,#0x1d
        BL       install_irq
        POP      {r4,lr}
        MOV      r0,#3
        B        serialEnableInterface
        ENDP

|L1.2192|
        DCD      UART3_HANDLER

        AREA ||.arm_vfe_header||, DATA, READONLY, NOALLOC, ALIGN=2

        DCD      0x00000000

        AREA ||.bss||, DATA, NOINIT, ALIGN=1

txAssetFIFO
        %        100
rxRadarFIFO
        %        24
txRadarFIFO
        %        24
rxRadarPacket
        %        100
uart2RxBuffer
        %        704

        AREA ||.data||, DATA, ALIGN=1

uart2DataIndex
        DCB      0x00
uart2IsrIndex
        DCB      0x01
nTxAssetAddToIndex
        DCW      0xffff
nTxRadarAddToIndex
        DCW      0xffff
nTxAssetRemoveFromIndex
        DCB      0x00,0x00
nRxRadarAddToIndex
        DCB      0x00,0x00
nRxRadarRemoveFromIndex
        DCB      0x00,0x00
nTxRadarRemoveFromIndex
        DCB      0x00,0x00
rxRadarPacketIndex
        DCB      0x00,0x00

        EXPORT UART1_HANDLER [CODE]
        EXPORT UART3_HANDLER [CODE]
        EXPORT sendchar [CODE]
        EXPORT RxDataReady [CODE]
        EXPORT serialDisableInterface [CODE]
        EXPORT serialEnableInterface [CODE]
        EXPORT serialSendByteToAsset [CODE]
        EXPORT switchUart2RxBuffers [CODE]
        EXPORT assetSerialDoWork [CODE]
        EXPORT UART2_HANDLER [CODE]
        EXPORT serialSendByteToRadar [CODE]
        EXPORT radarSerialDoWork [CODE]
        EXPORT serialWrite [CODE]
        EXPORT serialInit [CODE]

        IMPORT ||Lib$$Request$$armlib|| [CODE,WEAK]
        IMPORT delayMs [CODE]
        IMPORT __2printf [CODE]
        IMPORT __aeabi_memclr [CODE]
        IMPORT buildRxAssetPacket [CODE]
        IMPORT buildRxRadarPacket [CODE]
        IMPORT install_irq [CODE]

        KEEP serialInitPort
        KEEP txAssetFIFO
        KEEP rxRadarFIFO
        KEEP txRadarFIFO
        KEEP rxRadarPacket
        KEEP uart2RxBuffer
        KEEP uart2DataIndex
        KEEP uart2IsrIndex
        KEEP nTxAssetAddToIndex
        KEEP nTxRadarAddToIndex
        KEEP nTxAssetRemoveFromIndex
        KEEP nRxRadarAddToIndex
        KEEP nRxRadarRemoveFromIndex
        KEEP nTxRadarRemoveFromIndex
        KEEP rxRadarPacketIndex

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