#  muldiv.s version 4.0
.align 4
.section .text
.globl _start
_start:

# unsigned unsigned multiplication (5*4 = 20 = x0014)
addi x1, x1, 5
addi x2, x2, 4
mulhu x3, x1, x2
mul   x4, x1, x2

# signed unsigned multiplication (-1*4 = -4 = xFFFC)
add x5, x5, -1     # -1
mulhsu x6, x5, x2           # -1 * 4 = -4   (100)
mul x7, x5, x2 

# signed signed multiplication (-1*4 = -4 = xFFFC)
#                              (-1*-2 = 2 = x0002)
mulh x8, x5, x2     # -1 * 4 = -4
mul x9, x5, x2

add x10, x10, -2    # -2
mulh x11, x5, x10           # -1 * -2 = 2

add x0, x0, x0
add x0, x0, x0

divu x12, x1, x2 
remu x13, x1, x2

add x15, x15, 2
add x16, x16, 2
add x17, x17, 3



.section ".tohost"
.globl tohost
tohost: .dword 0
.section ".fromhost"
.globl fromhost
fromhost: .dword 0
