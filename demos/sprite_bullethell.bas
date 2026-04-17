MODE 5
MAP 0 = RGB(0,0,16)
MAP 1 = RGB(255,0,0)
MAP 4 = RGB(255,255,0)
MAP 5 = RGB(255,255,255)
MAP SET
CLS
DIM INTEGER ball(63)=(0,0,1,1,1,1,0,0, 0,1,4,4,4,4,1,0, 1,4,5,5,5,5,4,1, 1,4,5,5,5,5,4,1, 1,4,5,5,5,5,4,1, 1,4,5,5,5,5,4,1, 0,1,4,4,4,4,1,0, 0,0,1,1,1,1,0,0)
N%=64
SPRITE LOADARRAY 1, 8, 8, ball()
SPRITE COPY 1, 2, N%-1
DIM FLOAT x(N%), y(N%), dx(N%), dy(N%)
FOR i=1 TO N% : x(i)=4+RND*303 : y(i)=4+RND*223 : dx(i)=SGN(RND-0.5)*(0.5+RND*1.5) : dy(i)=SGN(RND-0.5)*(0.5+RND*1.5) : NEXT i
FOR i=1 TO N% : SPRITE SHOW i, INT(x(i)), INT(y(i)), 1 : NEXT i
FOR f=1 TO 2000 : FOR i=1 TO N% : dx(i)=dx(i)*(1-2*(x(i)<1 OR x(i)>311)) : dy(i)=dy(i)*(1-2*(y(i)<1 OR y(i)>231)) : x(i)=x(i)+dx(i) : y(i)=y(i)+dy(i) : SPRITE NEXT i, INT(x(i)), INT(y(i)) : NEXT i : SPRITE MOVE : NEXT f
END
