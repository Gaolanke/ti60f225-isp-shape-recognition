import numpy as np, math
BAND_U=[(256,0),(251,46),(237,98),(217,143),(181,181),(143,217),(98,237),(46,251)]
TH=[0.203,0.414,0.668,1.0,1.496,2.414,5.031]
def band_of(gx,gy):
    ax,ay=abs(gx),abs(gy)
    if ax<1e-9: return 7
    r=ay/ax
    for i,t in enumerate(TH):
        if r<t: return i
    return 7
def vote_map(a,b,cx,cy,scan_r,scan_ry,corrected,n=1440):
    acc=np.zeros((31,256),dtype=int)
    for i in range(n):
        th=2*math.pi*i/n
        x=a*math.cos(th); y=b*math.sin(th)
        px_,py_=cx+x,cy+y
        gx=x/(a*a); gy=y/(b*b)
        bd=band_of(gx,gy); ux,uy=BAND_U[bd]; phi=math.atan2(uy,ux)
        if corrected:
            k=scan_r/scan_ry
            theta=math.atan2(math.sin(phi), k*math.cos(phi))
            c,s=math.cos(theta),math.sin(theta)
        else:
            c,s=ux/256.0,uy/256.0
        sy=-1 if (gx*gy)<0 else 1
        for sgn in (1,-1):
            px=px_+sgn*scan_r*c; py=py_+sgn*scan_ry*(sy*s)
            if 0<=px<1920 and 0<=py<240: acc[int(py)//8,int(px)//8]+=1
    at=acc[cy//8, cx//8]
    idx=np.unravel_index(np.argmax(acc),acc.shape)
    return acc.max(), at, idx[1]*8, idx[0]*8
print("ellipse rx160 ry80 center(420,150): score at center bin")
for sr,sry in [(152,76),(168,84),(160,80)]:
    for corr in (False,True):
        pk,at,x,y=vote_map(160,80,420,150,sr,sry,corr)
        print(f" scan r={sr} ry={sry} corr={corr}: centerbin={at} peak={pk}@({x},{y})")
print("circle r=80:")
for sr,sry in [(72,72),(88,88),(80,80)]:
    for corr in (False,True):
        pk,at,x,y=vote_map(80,80,420,150,sr,sry,corr)
        print(f" scan r={sr} ry={sry} corr={corr}: centerbin={at} peak={pk}@({x},{y})")
