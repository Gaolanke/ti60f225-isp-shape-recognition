import numpy as np, math
H,V=1920,240; ROI=960
BAND_U=[(256,0),(251,46),(237,98),(217,143),(181,181),(143,217),(98,237),(46,251)]
TH=[0.203,0.414,0.668,1.0,1.496,2.414,5.031]
def band_of(gx,gy):
    ax,ay=abs(gx),abs(gy)
    if ax<1e-9: return 7
    r=ay/ax
    for i,t in enumerate(TH):
        if r<t: return i
    return 7
edge={}; dirs={}
def mark(px,py,cx,cy,rx,ry):
    gx=(px-cx)/(rx*rx); gy=(py-cy)/(ry*ry)
    b=band_of(gx,gy); q=1 if gx*gy<0 else 0
    edge[(px,py)]=(cx,cy,rx,ry); dirs[(px,py)]=(b,q)
def ellipse(cx,cy,rx,ry):
    for yy in range(cy-ry,cy+ry+1):
        dy=yy-cy; half=round(rx*math.sqrt(max(0,1-dy*dy/(ry*ry))))
        for j in range(2):
            mark(cx-half+j,yy,cx,cy,rx,ry); mark(cx+half-j,yy,cx,cy,rx,ry)
def rect(x0,y0,x1,y1):
    for j in range(2):
        for xx in range(x0,x1+1):
            edge[(xx,y0+j)]=None; dirs[(xx,y0+j)]=(7,0)
            edge[(xx,y1-j)]=None; dirs[(xx,y1-j)]=(7,0)
        for yy in range(y0,y1+1):
            edge[(x0+j,yy)]=None; dirs[(x0+j,yy)]=(0,0)
            edge[(x1-j,yy)]=None; dirs[(x1-j,yy)]=(0,0)
ellipse(420,150,160,80); rect(1050,30,1400,169)
print("edge pts",len(edge))
def scan(corrected):
    best=[]
    for asp,k in enumerate([1.3125,1.5625,0.625,0.5,1.0]):
        for ki in range(10):
            r=40+ki*16; ry=int({0:r+r//4+r//16,1:r+r//2+r//16,2:r//2+r//8,3:r//2,4:r}[asp])
            acc=np.zeros((31,256),dtype=int)
            for (px,py) in edge:
                b,q=dirs[(px,py)]
                ux,uy=BAND_U[b]; phi=math.atan2(uy,ux)
                if corrected:
                    theta=math.atan2(math.sin(phi), (r/ry)*math.cos(phi))
                    c,s=math.cos(theta),math.sin(theta)
                else:
                    c,s=ux/256.0,uy/256.0
                sy=-1 if q else 1
                dx=int(r*c); dy=int(ry*(sy*s))
                for sg in (1,-1):
                    ax,ay=px+sg*dx,py+sg*dy
                    if ROI<=ax<H and 0<=ay<V: acc[ay//8,ax//8]+=1
            idx=np.unravel_index(np.argmax(acc),acc.shape)
            best.append((acc.max(),idx[1]*8,idx[0]*8,r,ry,asp))
    best.sort(reverse=True)
    return best[:8]
for corr in (False,True):
    print("corrected=",corr)
    for row in scan(corr)[:6]: print("  peak=%d at (%d,%d) r=%d ry=%d asp=%d"%row)
