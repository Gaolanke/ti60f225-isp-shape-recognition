import numpy as np, math, sys
BAND_U=[(256,0),(251,46),(237,98),(217,143),(181,181),(143,217),(98,237),(46,251)]
def read_hex(fn):
    return [int(x,16) for x in open(fn).read().split()]
def load(prefix):
    ew=read_hex(prefix+'_edge.hex'); dw=read_hex(prefix+'_dir.hex')
    pts=[]
    for i in range(960*240):
        if (ew[i>>3]>>(i&7))&1:
            y,x=divmod(i,960); d=(dw[i>>3]>>((i&7)*4))&0xF
            pts.append((x,y,d>>1,d&1))
    return pts
pts=load(r'E:\FPGA\FPGA_MAX\sim\images\gen\circle_web')
print('pts',len(pts))
def hough(r,ry,pts):
    acc=np.zeros((31,256),int)
    for px,py,b,q in pts:
        ux,uy=BAND_U[b]; sy=-1 if q else 1
        dx=int(r*ux/256); dy=int(ry*sy*uy/256)
        for sg in (1,-1):
            ax,ay=px+960+sg*dx,py+sg*dy
            if 960<=ax<1920 and 0<=ay<240: acc[ay//8,ax//8]+=1
    idx=np.unravel_index(np.argmax(acc),acc.shape)
    return acc.max(),idx[1]*8,idx[0]*8
for r in [40,56,72,88,104]:
    print('r=',r,hough(r,r,pts))
