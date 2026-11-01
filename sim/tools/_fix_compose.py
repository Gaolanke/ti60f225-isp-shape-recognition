p = r'E:\FPGA\FPGA_MAX\sim\tools\compose_sheet.py'
s = open(p, encoding='utf-8').read()
s = s.replace("""    if cnts:
        cv2.drawContours(out, [max(cnts, key=cv2.contourArea)], -1, 0, thickness=cv2.FILLED)
    return out""",
"""    if cnts:
        cv2.drawContours(out, [max(cnts, key=cv2.contourArea)], -1, 0, thickness=cv2.FILLED)
    ys, xs = np.nonzero(out < 128)
    return out[ys.min():ys.max() + 1, xs.min():xs.max() + 1]""")
s = s.replace("for name, shp, h, cx in [('circle', circle, 150, 140), ('rect', rect, 130, 420),",
              "for name, shp, h, cx in [('circle', circle, 160, 480), ('rect', rect, 130, 480),")
open(p, 'w', encoding='utf-8').write(s)
