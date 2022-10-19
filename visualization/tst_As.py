import numpy as np
import matplotlib.pyplot as plt
from matplotlib.offsetbox import OffsetImage, AnnotationBbox
import matplotlib.cbook as cbook

np.random.seed(1234)

# load up Ada's image
vortexRF = plt.imread(("visualization/tractor.png"))
imagebox = OffsetImage(vortexRF, zoom=0.2)

# initiate plot
fig, ax = plt.subplots()

# place Ada in plot
ab = AnnotationBbox(imagebox, (0, 0), frameon=False)
cbar_ax = fig.add_axes([0.6, .42, 0.3, 0.3], zorder=-1)
cbar_ax.add_artist(ab)
cbar_ax.axis('off')

# add scatter plot
# ax.scatter(np.random.normal(np.tile(np.random.uniform(0, 1, 5), 1000), .1),
#            np.random.normal(np.tile(np.random.uniform(0, 1, 5), 1000), .1),
#            c=np.tile(['fuchsia', 'gold', 'coral', 'deepskyblue', 'chartreuse'], 1000),
#            s=3, alpha=0.2)

x = np.linspace(0,100,1000)
y = np.sin(x)
ax.plot(x,y)
# comment that Ada should be in the background
ax.set_title("we want the dots to be in front of Ada")

# make the background of the scatter plot fully transparent
ax.set_facecolor('none')

plt.show()