; generated by Component: ARM Compiler 5.04 update 1 (build 49) Tool: ArmCC [5040049]
; commandline ArmCC [--c99 --debug -c --asm -o.\objs\gpio.o --depend=.\objs\gpio.d --apcs=interwork -Otime --diag_suppress=9931 -IC:\Keil\ARM\INC\Phillips -I..\SharedCode -IC:\Keil_v5\ARM\RV31\INC -IC:\Keil_v5\ARM\CMSIS\Include -IC:\Keil_v5\ARM\Inc\Philips --enum_is_int --omf_browse=.\objs\gpio.crf gpio.c]
        ARM
        REQUIRE8
        PRESERVE8

        AREA ||.text||, CODE, READONLY, ALIGN=2

GPIO_CONFIG PROC
        MOV      r1,#0x1fc000
        MOV      r0,#0x21
        ADD      r1,r1,#0xe0000000
        STR      r0,[r1,#0x1a0]
        LDR      r0,|L1.196|
        SUB      r1,r1,#0x1d0000
        STR      r0,[r1,#0]
        MOV      r0,#1
        ADD      r0,r0,#0x54000
        STR      r0,[r1,#4]
        MOV      r2,#0
        SUB      r0,r0,r1,ROR #31
        STR      r2,[r0,#0]
        LDR      r3,|L1.200|
        STR      r3,[r0,#0]
        MVN      r3,#0
        STR      r3,[r0,#0x18]
        MOV      r12,#0x80000
        STR      r12,[r0,#0x1c]
        STR      r2,[r1,#8]
        STR      r2,[r1,#0xc]
        MOV      r12,#0x8100
        ADD      r12,r12,#0x30000
        STR      r12,[r0,#0x20]
        LDR      r12,[r1,#0x4c]
        BIC      r12,r12,#0xfc00000
        STR      r12,[r1,#0x4c]
        STR      r3,[r0,#0x38]
        MOV      r12,#0x20000
        STR      r12,[r0,#0x3c]
        MOV      r12,#0x100
        STR      r12,[r0,#0x3c]
        MOV      r12,#0xa0000
        STR      r12,[r1,#0x10]
        ADD      r12,r3,#0x104
        STR      r12,[r0,#0x40]
        STR      r3,[r0,#0x58]
        MOV      r3,#2
        STR      r3,[r0,#0x5c]
        STR      r2,[r1,#0x28]
        STR      r2,[r1,#0x1c]
        MOV      r3,#0x4000000
        STR      r3,[r0,#0x60]
        STR      r3,[r0,#0x7c]
        STR      r2,[r1,#0x24]
        MOV      r1,#0x30000000
        STR      r1,[r0,#0x80]
        STR      r1,[r0,#0x9c]
        BX       lr
        ENDP

|L1.196|
        DCD      0x400a805a
|L1.200|
        DCD      0x004882c5

        AREA ||.arm_vfe_header||, DATA, READONLY, NOALLOC, ALIGN=2

        DCD      0x00000000

        EXPORT GPIO_CONFIG [CODE]

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
