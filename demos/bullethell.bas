MODE 5
' Set palette via direct POKE (MAP n= has off-by-one bug)
rp% = &H2007e5b6
POKE SHORT rp%, 0
POKE SHORT rp%+2, &H7C00
POKE SHORT rp%+4, &H03E0
POKE SHORT rp%+6, &H7FE0
POKE SHORT rp%+8, &H001F
POKE SHORT rp%+10, &H7FFF
POKE SHORT rp%+12, &H7C1F
POKE SHORT rp%+14, &H03FF
MAP SET
CLS RGB(0,0,0)
FRAMEBUFFER CREATE
DIM INTEGER blt%(50)
ASM blt%()
  PUSH {R4, R5, R6, R7, LR}
  SUB SP, SP, #20
  STR R0, [SP]
  STR R2, [SP, #4]
  STR R3, [SP, #8]
  LDRB R4, [R1]
  LDRB R5, [R1, #1]
  ADD R6, R1, #2
  STR R6, [SP, #12]
  MOV R7, #0
ry:
  CMP R7, R5
  BGE rd
  LDR R0, [SP, #8]
  ADD R0, R0, R7
  CMP R0, #0
  BLT rn
  CMP R0, #230
  BGT rn
  PUSH {R1}
  MOV R1, #160
  LSL R1, R1, #1
  MUL R0, R0, R1
  POP {R1}
  LDR R1, [SP]
  ADD R0, R1, R0
  STR R0, [SP, #16]
  MOV R6, #0
rx:
  CMP R6, R4
  BGE rn
  LDR R0, [SP, #12]
  LDRB R0, [R0, R6]
  CMP R0, #0
  BEQ rs
  LDR R1, [SP, #4]
  ADD R1, R1, R6
  CMP R1, #0
  BLT rs
  CMP R1, #250
  BGT rs
  LDR R2, [SP, #16]
  STRB R0, [R2, R1]
rs:
  ADD R6, R6, #1
  B rx
rn:
  LDR R0, [SP, #12]
  ADD R0, R0, R4
  STR R0, [SP, #12]
  ADD R7, R7, #1
  B ry
rd:
  ADD SP, SP, #20
  POP {R4, R5, R6, R7, PC}
END ASM
DIM INTEGER clr%(10)
ASM clr%()
  PUSH {LR}
  MOV R1, #0
  MOV R2, #75
  LSL R2, R2, #10
cl:
  STRB R1, [R0, R2]
  SUBS R2, R2, #1
  BNE cl
  STRB R1, [R0]
  POP {PC}
END ASM
' 8x8 sprite
DIM INTEGER sdata%(8)
sa% = PEEK(VARADDR sdata%())
POKE BYTE sa%, 8 : POKE BYTE sa%+1, 8
For i=0 To 7:For j=0 To 7
  d=Sqr((i-3.5)^2+(j-3.5)^2)
  If d<1.5 Then c%=5
  ElseIf d<2.5 Then c%=3
  ElseIf d<3.5 Then c%=1
  Else c%=0
  End If
  POKE BYTE sa%+2+i*8+j, c%
Next j,i
' 4x4 bullet
DIM INTEGER bdata%(4)
ba% = PEEK(VARADDR bdata%())
POKE BYTE ba%, 4:POKE BYTE ba%+1, 4
For i=0 To 3:For j=0 To 3
  If Sqr((i-1.5)^2+(j-1.5)^2)<1.8 Then POKE BYTE ba%+2+i*4+j, 7 Else POKE BYTE ba%+2+i*4+j, 0
Next j,i
n% = 40
DIM x(n%-1),y(n%-1),dx(n%-1),dy(n%-1)
For i=0 To n%-1
  x(i)=Rnd*300+10:y(i)=Rnd*220+10
  dx(i)=Rnd*6-3:dy(i)=Rnd*6-3
  If Abs(dx(i))<0.5 Then dx(i)=1
  If Abs(dy(i))<0.5 Then dy(i)=1
Next
For frame=1 To 3000
  FRAMEBUFFER WRITE F
  fb%=MM.INFO(WRITEBUF)
  CALL clr%(), fb%
  For i=0 To n%-1
    x(i)=x(i)+dx(i):y(i)=y(i)+dy(i)
    If x(i)<4 Or x(i)>310 Then dx(i)=-dx(i)
    If y(i)<4 Or y(i)>230 Then dy(i)=-dy(i)
    If i<8 Then
      CALL blt%(), fb%, sa%, Int(x(i)), Int(y(i))
    Else
      CALL blt%(), fb%, ba%, Int(x(i)), Int(y(i))
    End If
  Next
  FRAMEBUFFER COPY F, N
Next
FRAMEBUFFER CLOSE
MODE 1
