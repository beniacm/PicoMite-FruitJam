MODE 4
FRAMEBUFFER CREATE
CLS RGB(0,0,0)
h% = 240
w% = 320
DIM INTEGER mb%(140)
ASM mb%()
  PUSH {R4, R5, R6, R7, LR}
  MOV R4, R0
  VLDR S8, [R1]
  VLDR S9, [R1, #4]
  VLDR S10, [R1, #8]
  VLDR S11, [R1, #12]
  MOV R5, #w%
  MOV R6, #h%
  VMOV S14, R5
  VCVT.F32.S32 S14, S14
  VMOV S15, R6
  VCVT.F32.S32 S15, S15
  MOV R0, #4
  VMOV S12, R0
  VCVT.F32.S32 S12, S12
  MOV R0, #0
yy:
  CMP R0, R6
  BGE dn
  PUSH {R0}
  VMOV S1, R0
  VCVT.F32.S32 S1, S1
  VMUL.F32 S1, S11, S1
  VDIV.F32 S1, S1, S15
  VADD.F32 S1, S10, S1
  MOV R1, #0
xl:
  CMP R1, R5
  BGE xd
  PUSH {R1}
  VMOV S0, R1
  VCVT.F32.S32 S0, S0
  VMUL.F32 S0, S9, S0
  VDIV.F32 S0, S0, S14
  VADD.F32 S0, S8, S0
  MOV R3, #0
  VMOV S2, R3
  VCVT.F32.S32 S2, S2
  VMOV S3, S2
  MOV R7, #64
il:
  VMUL.F32 S5, S2, S2
  VMUL.F32 S6, S3, S3
  VADD.F32 S7, S5, S6
  VCMP.F32 S7, S12
  VMRS
  BHI id
  VMUL.F32 S7, S2, S3
  VADD.F32 S7, S7, S7
  VADD.F32 S3, S7, S1
  VSUB.F32 S5, S5, S6
  VADD.F32 S2, S5, S0
  ADD R3, R3, #1
  CMP R3, R7
  BLT il
id:
  CMP R3, R7
  BEQ bk
  ' R=(iter)&31, G=(iter*3)&31, B=(iter*7)&31
  ' Use 2-operand ORR (not 3-operand!)
  MOV R2, R3
  LSL R2, R2, #27
  LSR R2, R2, #17
  MOV R0, R3
  ADD R0, R0, R3
  ADD R0, R0, R3
  LSL R0, R0, #27
  LSR R0, R0, #22
  ORR R2, R0
  MOV R0, R3
  LSL R0, R0, #3
  SUBS R0, R0, R3
  LSL R0, R0, #27
  LSR R0, R0, #27
  ORR R2, R0
  B px
bk:
  MOV R2, #0
px:
  POP {R1}
  POP {R0}
  PUSH {R0}
  MOV R7, R5
  MUL R7, R0, R7
  ADD R7, R7, R1
  LSL R7, R7, #1
  ADD R7, R4, R7
  STRH R2, [R7]
  ADD R1, R1, #1
  B xl
xd:
  POP {R0}
  ADD R0, R0, #1
  B yy
dn:
  POP {R4, R5, R6, R7, PC}
END ASM
tx = -0.745 : ty = 0.186
zoom = 1.0
For frame = 1 To 150
  zoom = zoom * 0.96
  xr = 3.0 * zoom : yr = 2.5 * zoom
  cx = tx - xr / 2 : cy = ty - yr / 2
  FRAMEBUFFER WRITE F
  fb% = MM.INFO(WRITEBUF)
  CALL mb%(), fb%, cx, xr, cy, yr
  FRAMEBUFFER COPY F, N
Next frame
PAUSE 5000
FRAMEBUFFER CLOSE
MODE 1
Print "Zoom complete!"
