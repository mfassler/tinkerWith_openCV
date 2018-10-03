
import numpy as np
import matplotlib.pyplot as plt


colors = plt.cm.jet(np.linspace(0,1,256))


print("uint8_t colormap[256][3] = {")

for i, oneColor in enumerate(colors):
    if i < 255:
        ch = ","
    else:
        ch = ""
    # OpenCV is in BGR order
    print("\t{%d, %d, %d}%s" % (255*oneColor[2], 255*oneColor[1], 255*oneColor[0], ch))

print("};")


