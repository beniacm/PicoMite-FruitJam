MODE 5
MAP 0 = RGB(0,0,16)
MAP 1 = RGB(255,0,0)
MAP 4 = RGB(255,255,0)
MAP 5 = RGB(255,255,255)
MAP SET
CLS
DIM INTEGER ball(63)=(0,0,1,1,1,1,0,0, 0,1,4,4,4,4,1,0, 1,4,5,5,5,5,4,1, 1,4,5,5,5,5,4,1, 1,4,5,5,5,5,4,1, 1,4,5,5,5,5,4,1, 0,1,4,4,4,4,1,0, 0,0,1,1,1,1,0,0)
N%=64
NF%=500
SPRITE LOADARRAY 1, 8, 8, ball()
DIM FLOAT x(N%), y(N%), dx(N%), dy(N%)
FOR i=1 TO N%
 x(i)=4+RND*303 : y(i)=4+RND*223
 dx(i)=SGN(RND-0.5)*(0.5+RND*1.5)
 dy(i)=SGN(RND-0.5)*(0.5+RND*1.5)
NEXT i
FRAMEBUFFER CREATE
' -- COPY benchmark
t0=TIMER
FOR f=1 TO NF%
 FRAMEBUFFER WRITE F
 CLS
 FOR i=1 TO N%
  dx(i)=dx(i)*(1-2*(x(i)<1 OR x(i)>311))
  dy(i)=dy(i)*(1-2*(y(i)<1 OR y(i)>231))
  x(i)=x(i)+dx(i) : y(i)=y(i)+dy(i)
  SPRITE WRITE 1, INT(x(i)), INT(y(i))
 NEXT i
 FRAMEBUFFER COPY F, N, B
NEXT f
tcopy=TIMER-t0
' -- FLIP benchmark
t0=TIMER
FOR f=1 TO NF%
 FRAMEBUFFER WRITE F
 CLS
 FOR i=1 TO N%
  dx(i)=dx(i)*(1-2*(x(i)<1 OR x(i)>311))
  dy(i)=dy(i)*(1-2*(y(i)<1 OR y(i)>231))
  x(i)=x(i)+dx(i) : y(i)=y(i)+dy(i)
  SPRITE WRITE 1, INT(x(i)), INT(y(i))
 NEXT i
 FRAMEBUFFER FLIP
NEXT f
tflip=TIMER-t0
FRAMEBUFFER CLOSE
PRINT "COPY ";NF%;" frames ";tcopy;"ms ";INT(NF%*1000/tcopy*100)/100;" fps"
PRINT "FLIP ";NF%;" frames ";tflip;"ms ";INT(NF%*1000/tflip*100)/100;" fps"
END
