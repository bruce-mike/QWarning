; generated by Component: ARM Compiler 5.04 update 1 (build 49) Tool: ArmCC [5040049]
; commandline ArmCC [--c99 --debug -c --asm -o.\objs\watchdog.o --depend=.\objs\watchdog.d --apcs=interwork -Otime --diag_suppress=9931 -IC:\Keil\ARM\INC\Phillips -I..\SharedCode -IC:\Keil_v5\ARM\RV31\INC -IC:\Keil_v5\ARM\CMSIS\Include -IC:\Keil_v5\ARM\Inc\Philips --enum_is_int --omf_browse=.\objs\watchdog.crf watchdog.c]
        ARM
        REQUIRE8
        PRESERVE8

        AREA ||.text||, CODE, READONLY, ALIGN=2

lastResetType PROC
        MOV      r0,#0x1fc000
        ADD      r0,r0,#0xe0000000
        LDR      r0,[r0,#0x180]
        AND      r0,r0,#0xff
        BX       lr
        ENDP

watchdogFeed PROC
        MOV      r1,#0xaa
        MOV      r0,#0xe0000000
        STR      r1,[r0,#8]
        MOV      r1,#0x55
        STR      r1,[r0,#8]
        BX       lr
        ENDP

watchdogInit PROC
        MOV      r2,#0
        MOV      r1,#0xe0000000
        STR      r2,[r1,#0x10]
        MOV      r2,#0x3e8
        MUL      r0,r2,r0
        STR      r0,[r1,#4]
        LDR      r0,[r1,#0]
        ORR      r0,r0,#3
        STR      r0,[r1,#0]
        MOV      r0,#0xaa
        STR      r0,[r1,#8]
        MOV      r0,#0x55
        STR      r0,[r1,#8]
        BX       lr
        ENDP

watchdogReboot PROC
        MOV      r0,#0xff
        MOV      r1,#0xe0000000
        STR      r0,[r1,#4]
        MOV      r2,#0x120
        ADD      r2,r2,#0x7a000
        STR      r2,[r1,#4]
        MOV      r2,#0xaa
        STR      r2,[r1,#8]
        MOV      r2,#0x55
        STR      r2,[r1,#8]
|L1.140|
        STR      r0,[r1,#4]
        B        |L1.140|
        ENDP


        AREA ||.arm_vfe_header||, DATA, READONLY, NOALLOC, ALIGN=2

        DCD      0x00000000

        EXPORT lastResetType [CODE]
        EXPORT watchdogFeed [CODE]
        EXPORT watchdogInit [CODE]
        EXPORT watchdogReboot [CODE]

        IMPORT ||Lib$$Request$$armlib|| [CODE,WEAK]

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
