import matplotlib.pyplot as plt
import numpy as np
from sklearn.neighbors import NearestNeighbors
import networkx as nx
import time

def totuple(a):
    try:
        return tuple(totuple(i) for i in a)
    except TypeError:
        return a

x = np.linspace(0, 2 * np.pi, 1500, dtype=np.float64)
y = np.sin(x)

p_start = (x[0], y[0])
p_end = (x[-1], y[-1])

idx = np.random.permutation(x.size)
x = x[idx]
y = y[idx]
plt.plot(x, y)
    
points = np.c_[x, y]
if True:
    sw = time.time()
    clf = NearestNeighbors(2).fit(points)
    G = clf.kneighbors_graph()
    T = nx.from_scipy_sparse_matrix(G)
    paths = [list(nx.dfs_preorder_nodes(T, i)) for i in range(len(points))]
    mindist = np.inf
    minidx = 0
    for i in range(len(points)):
        p = paths[i]           # order of nodes
        ordered = points[p]    # ordered nodes
        # find cost of that order by the sum of euclidean distances between points (i) and (i+1)
        cost = (((ordered[:-1] - ordered[1:])**2).sum(1)).sum()
        if cost < mindist:
            mindist = cost
            minidx = i
    opt_order = paths[minidx]
    print("Dauer:", time.time()-sw)
    xx = x[opt_order]
    yy = y[opt_order]
    plt.plot(xx, yy)
    # plt.show()



from sklearn.neighbors import KDTree
import numpy as np
import networkx as nx

sw = time.time()
G = nx.Graph()  # A graph to hold the nearest neighbours

X = points  # Some list of points in 2D
print(X.shape)
tree = KDTree(X, leaf_size=2, metric='euclidean')  # Create a distance tree

# Now loop over your points and find the two nearest neighbours
# If the first and last points are also the start and end points of the line you can use X[1:-1]
for p in X:
    # print("p:", p, p.shape)
    dist, ind = tree.query(p.reshape(1, 2), k=3)
    # print ind

    # ind Indexes represent nodes on a graph
    # Two nearest points are at indexes 1 and 2. 
    # Use these to form edges on graph
    # p is the current point in the list
    # pp = tuple(map(tuple, p))
    # print(type(pp))
    G.add_node(totuple(p))
    n1, l1 = X[ind[0][1]], dist[0][1]  # The next nearest point
    n2, l2 = X[ind[0][2]], dist[0][2]  # The following nearest point  
    G.add_edge(totuple(p), totuple(n1))
    G.add_edge(totuple(p), totuple(n2))



path = nx.shortest_path(G, source=p_start, target=p_end)
path = np.asarray(path)
print(path.shape)
plt.plot(path[:,0], path[:,1])
print("Dauer:", time.time()-sw)
# plt.show()

import numpy as np
from scipy.signal import savgol_filter
from sklearn.decomposition import PCA
from scipy import interpolate


sw = time.time()
def XYclean(x,y): 
    
    xy = np.concatenate((x.reshape(-1,1), y.reshape(-1,1)), axis=1, dtype=np.float64)     
    
    # make PCA object
    pca = PCA(2)
    # fit on data
    pca.fit(xy)
    

    #transform into pca space   
    xypca = pca.transform(xy) 
    newx = xypca[:,0]
    newy = xypca[:,1]
    

    #sort
    indexSort = np.argsort(x)
    newx = newx[indexSort]
    newy = newy[indexSort]
    
    #add some more points (optional)
    f = interpolate.interp1d(newx, newy, kind='linear')        
    newX=np.linspace(np.min(newx), np.max(newx), 1500, dtype=np.float64)
    newY = f(newX)            

    #smooth with a filter (optional)
    window = 43
    newY = savgol_filter(newY, window, 2)

    #return back to old coordinates
    xyclean = pca.inverse_transform(np.concatenate((newX.reshape(-1,1), newY.reshape(-1,1)), axis=1, dtype=np.float64) )
    xc=xyclean[:,0]
    yc = xyclean[:,1]

    return xc, yc

xc, yc = XYclean(x, y)
plt.plot(path[:,0], path[:,1], "--")
print("Dauer:", time.time()-sw)
# plt.show()


##########################

def order_points(points, ind):
    points_new = [ points.pop(ind) ]  # initialize a new list of points with the known first point
    pcurr      = points_new[-1]       # initialize the current point (as the known point)
    while len(points)>0:
        d      = np.linalg.norm(np.array(points) - np.array(pcurr), axis=1)  # distances between pcurr and all other remaining points
        ind    = d.argmin()                   # index of the closest point
        points_new.append( points.pop(ind) )  # append the closest point to points_new
        pcurr  = points_new[-1]               # update the current point
    return points_new

# create sine curve:
x      = np.linspace(0, 2 * np.pi, 1500)
y      = np.sin(x)

# shuffle the order of the x and y coordinates:
idx    = np.random.permutation(x.size)
xs,ys  = x[idx], y[idx]   # shuffled points

# find the leftmost point:
ind    = xs.argmin()

sw = time.time()
# assemble the x and y coordinates into a list of (x,y) tuples:
points = [(xx,yy)  for xx,yy in zip(xs,ys)]

# order the points based on the known first point:
points_new = order_points(points, ind)
print("Dauer:", time.time()-sw)

# plot:
fig,ax = plt.subplots(1, 2, figsize=(10,4))
xn,yn  = np.array(points_new).T
ax[0].plot(xs, ys)  # original (shuffled) points
ax[1].plot(xn, yn)  # new (ordered) points
ax[0].set_title('Original')
ax[1].set_title('Ordered')
plt.tight_layout()
plt.show()