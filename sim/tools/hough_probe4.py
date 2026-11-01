import numpy as np, math
H,V=1920,240
BAND_U=[(256,0),(251,46),(237,98),(217,143),(181,181),(143,217),(98,237),(46,251)]
TH=[0.203,0.414,0.668,1.0,1.496,2.414,5.031]
def band_of(gx,gy):
    ax,ay=abs(gx),abs(gy); r=ay/ax if ax>1e-9 else 1e9
    for i,t in enumerate(TH):
        if r<t: return i
    return 7
pts=[];edgeset={}
def mark(px,py,cx,cy,rx,ry):
    gx=(px-cx)/(rx*rx); gy=(py-cy)/(ry*ry)
    pts.append((px,py,band_of(gx,gy),1 if gx*gy<0 else 0))
def ellipse(cx,cy,rx,ry):
    for yy in range(cy-ry,cy+ry+1):
        dy=yy-cy; half=round(rx*math.sqrt(max(0,1-dy*dy/(ry*ry))))
        for j in range(2):
            mark(cx-half+j,yy,cx,cy,rx,ry); mark(cx+half-j,yy,cx,cy,rx,ry)
ellipse(420,150,160,80); edgeset={(p[0],p[1]) for p in pts}
def arc_hits(cx,cy,rx,ry,tol=16):
    s=1/math.sqrt(2)
    samples=[(cx+rx,cy),(cx+rx*s,cy-ry*s),(cx,cy-ry),(cx-rx*s,cy-ry*s),
             (cx-rx,cy),(cx-rx*s,cy+ry*s),(cx,cy+ry),(cx+rx*s,cy+ry*s)]
    mask=0
    for i,(sx,sy) in enumerate(samples):
        hit=any(abs(x-sx)<=tol and abs(y-sy)<=tol for (x,y) in edgeset)
        if hit: mask|=1<<(7-i)
    return bin(mask).count('1'),mask
def scan(corrected):
    rows=[]
    for asp in range(5):
        for ki in range(10):
            r=40+ki*16; ry=int({0:r+r//4+r//16,1:r+r//2+r//16,2:r//2+r//8,3:r//2,4:r}[asp])
            acc=np.zeros((31,256),dtype=int)
            for px,py,b,q in pts:
                ux,uy=BAND_U[b]; phi=math.atan2(uy,ux)
                if corrected:
                    th=math.atan2(math.sin(phi),(r/ry)*math.cos(phi)); c,s=math.cos(th),math.sin(th)
                else: c,s=ux/256.0,uy/256.0
                sy=-1 if q else 1; dx=int(r*c); dy=int(ry*sy*s)
                for sg in (1,-1):
                    ax,ay=px+sg*dx,py+sg*dy
                    if 0<=ax<H and 0<=ay<V: acc[ay//8,ax//8]+=1
            idx=np.unravel_index(np.argmax(acc),acc.shape); cx,cy=idx[1]*8,idx[0]*8
            h,m=arc_hits(cx,cy,r,ry)
            rows.append((acc.max(),cx,cy,r,ry,asp,h,m))
    return rows
for corr in (False,True):
    rows=[x for x in scan(corr) if x[0]>=20]
    rows.sort(reverse=True)
    print("corrected",corr,"candidates>=20:",len(rows))
    for x in rows[:10]:
        print("  peak=%3d (%4d,%3d) r=%3d ry=%3d asp=%d arc=%d mask=%s"%(x[0],x[1],x[2],x[3],x[4],x[5],x[6],format(x[7],"08b")))
