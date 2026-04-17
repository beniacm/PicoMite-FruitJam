MODE 5
' Set up a smooth rainbow palette (256 entries)
' Use Bernstein polynomial palette in BASIC for setup
For i = 0 To 63
  t = i / 64.0
  t1 = 1.0 - t
  r% = Int(9 * t1 * t * t * t * 255)
  g% = Int(15 * t1 * t1 * t * t * 255)
  b% = Int(8.5 * t1 * t1 * t1 * t * 255)
  If r% > 255 Then r% = 255
  If g% > 255 Then g% = 255
  If b% > 255 Then b% = 255
  MAP i = RGB(r%, g%, b%)
Next i
' Repeat palette with phase shift for entries 64-127
For i = 0 To 63
  t = i / 64.0
  t1 = 1.0 - t
  r% = Int(8.5 * t1 * t1 * t1 * t * 255)
  g% = Int(9 * t1 * t * t * t * 255)
  b% = Int(15 * t1 * t1 * t * t * 255)
  MAP i + 64 = RGB(r%, g%, b%)
Next i
' Third phase for 128-191
For i = 0 To 63
  t = i / 64.0
  t1 = 1.0 - t
  r% = Int(15 * t1 * t1 * t * t * 255)
  g% = Int(8.5 * t1 * t1 * t1 * t * 255)
  b% = Int(9 * t1 * t * t * t * 255)
  MAP i + 128 = RGB(r%, g%, b%)
Next i
' Fourth phase for 192-254
For i = 0 To 62
  t = i / 63.0
  MAP i + 192 = RGB(Int(t*255), Int((1-t)*255), Int(t*128+64))
Next i
MAP 255 = RGB(0, 0, 0)
MAP SET
FRAMEBUFFER CREATE
CLS RGB(0,0,0)
h% = 240
w% = 320
' Mandelbrot: R0=fb, R1=float ptr
' Just write iter & 255 as palette index, STRB
DIM INTEGER mb%(120)
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
  MOV R7, #255
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
  ' Write iter as palette index (0-255)
  MOV R2, R3
  POP {R1}
  POP {R0}
  PUSH {R0}
  MOV R7, R5
  MUL R7, R0, R7
  ADD R7, R7, R1
  ADD R7, R4, R7
  STRB R2, [R7]
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
