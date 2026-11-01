import math
BAND=[(256,0),(251,46),(237,98),(217,143),(181,181),(143,217),(98,237),(46,251)]
ASP=[1.3125,1.5625,0.625,0.5,1.0]
names=['asp0 tall 1.3125','asp1 tall 1.5625','asp2 wide 0.625','asp3 wide 0.5','asp4 circle 1.0']
for ai,k in enumerate(ASP):
    print(f"// {names[ai]}  k=rx/ry={1/k:.4f}")
    for b,(ux,uy) in enumerate(BAND):
        phi=math.atan2(uy,ux)
        th=math.atan2(math.sin(phi),(1/k)*math.cos(phi))
        cx=round(256*math.cos(th)); sy=round(256*math.sin(th))
        print(f"  3'd{b}: begin par_x = 10'sd{cx}; par_y = 10'sd{sy}; end")
