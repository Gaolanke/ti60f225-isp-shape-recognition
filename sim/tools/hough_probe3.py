import numpy as np, math
H,V=1920,240; ROI=0
BAND_U=[(256,0),(251,46),(237,98),(217,143),(181,181),(143,217),(98,237),(46,251)]
TH=[0.203,0.414,0.668,1.0,1.496,2.414,5.031]
def band_of(gx,gy):
    ax,ay=abs(gx),abs(gy)
    if ax<1e-9: return 7
    r=ay/ax
    for i,t in enumerate(TH):
        if r<t: return i
    return 7
pts=[]
def mark(px,py,cx,cy,rx,ry):
    gx=(px-cx)/(rx*rx); gy=(py-cy)/(ry*ry)
    pts.append((px,py,band_of(gx,gy),1 if gx*gy<0 else 0))
def ellipse(cx,cy,rx,ry):
    for yy in range(cy-ry,cy+ry+1):
        dy=yy-cy; half=round(rx*math.sqrt(max(0,1-dy*dy/(ry*ry))))
        for j in range(2):
            mark(cx-half+j,yy,cx,cy,rx,ry); mark(cx+half-j,yy,cx,cy,rx,ry)
ellipse(420,150,160,80); print("pts",len(pts))
def scan(corrected):
    rows=[]
    for asp,k in enumerate([1.3125,1.5625,0.625,0.5,1.0]):
        for ki in range(10):
            r=40+ki*16; ry=int({0:r+r//4+r//16,1:r+r//2+r//16,2:r//2+r//8,3:r//2,4:r}[asp])
            acc=np.zeros((31,256),dtype=int)
            for px,py,b,q in pts:
                ux,uy=BAND_U[b]; phi=math.atan2(uy,ux)
                if corrected:
                    theta=math.atan2(math.sin(phi),(r/ry)*math.cos(phi))
                    c,s=math.cos(theta),math.sin(theta)
                else: c,s=ux/256.0,uy/256.0
                sy=-1 if q else 1
                dx=int(r*c); dy=int(ry*(sy*s))
                for sg in (1,-1):
                    ax,ay=px+sg*dx,py+sg*dy
                    if ROI<=ax<H and 0<=ay<V: acc[ay//8,ax//8]+=1
            idx=np.unravel_index(np.argmax(acc),acc.shape)
            rows.append((acc.max(),idx[1]*8,idx[0]*8,r,ry,asp))
    return rows
for corr in (False,True):
    rows=scan(corr)
    print("corrected",corr," top6:")
    for row in sorted(rows,reverse=True)[:6]: print("  peak=%d at (%d,%d) r=%d ry=%d asp=%d"%row)
    near=[x for x in rows if 360<=x[1]<=480]
    print("  ellipse-region best:", max(near) if near else None)
