.syntax unified
.cpu cortex-m0
.thumb

.text
.global stub
stub:
    cpsid i
    ldr r4, =0x08004a89
    ldr r5, =0xE000E018
    ldr r6, =0x2000055c
    movs r7, #32

1:
    ldr r0, [r5]
    blx r4
    ldr r1, [r5]
    subs r2, r0, r1
    bcs 2f
    ldr r3, =0xBB80
    adds r2, r3
2:
    str r2, [r6]
    adds r6, #4
    subs r7, #1
    bne 1b

    ldr r0, =0x40012C20
    ldrh r1, [r0]
    ldr r2, =0x555
    bics r1, r2
    strh r1, [r0]
    bkpt #0
