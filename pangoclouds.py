import numpy as np
import OpenGL.GL as gl
import pangolin
import math, random, time, sys
from geometric_points import Node, Point
from geometric_points import PointCloudCtx, CloudCluster 
import cube_detector

LIDAR_WIDTH = 640
LIDAR_HEIGHT = 16

pangolin.CreateWindowAndBind('Main', 640, 480)
gl.glEnable(gl.GL_DEPTH_TEST)

# Define Projection and initial ModelView matrix
scam = pangolin.OpenGlRenderState(
    pangolin.ProjectionMatrix(640, 480, 420, 420, 320, 240, 0.2, 100),
    pangolin.ModelViewLookAt(-2, 0, 1, 0, 0, 0, pangolin.AxisDirection.AxisZ))
handler = pangolin.Handler3D(scam)

# Create Interactive View in window
dcam = pangolin.CreateDisplay()
dcam.SetBounds(0.0, 1.0, 0.0, 1.0, -640.0/480.0)
dcam.SetHandler(handler)

# Load point cloud
points = Point.load_from(sys.argv[1])
nodes = np.array(
        [Node(Point(p), i) for i, p in enumerate(points)])
print('Nodes:', nodes.shape)

# Filter masks
cloud_ctx = PointCloudCtx.defaultFactory(LIDAR_WIDTH, LIDAR_HEIGHT, nodes)
invalid_points_mask = cloud_ctx.invalid_points_mask()
floor_mask = cloud_ctx.floor_mask()

start = time.time()
for node in nodes:
    node.attach_neighbors(nodes)

# Get rid of invalid nodes and points for easier processing
nodes = nodes[np.logical_and(invalid_points_mask, floor_mask)]
points = points[np.logical_and(invalid_points_mask, floor_mask)]

clusters = CloudCluster.cluster_nodes(nodes)
secs = time.time() - start
print('Nodes matching time: %.2f fps' % (1 / secs))

n_neighbors = sum([len(n.neighbors) for n in nodes]) 
print('Neighbors', n_neighbors)

COLORS = [
    (1, 0, 0),
    (0, 1, 0),
    (1, 1, 0),
    (0, 1, 1),
]

# cube_detector.generate_cluster_data(clusters, 'cubes.npy')
# exit()

is_cube = cube_detector.load_model()
for cluster in clusters:
    x = cluster.random_walk()
    p = is_cube(x)
    print(cluster.index, 'is cube:', p)
    if(p >= 0.95):
        cluster.geometry = 'CUBE'
    else:
        cluster.geometry = 'UNKNOWN'

while not pangolin.ShouldQuit():
    gl.glClear(gl.GL_COLOR_BUFFER_BIT | gl.GL_DEPTH_BUFFER_BIT)
    gl.glClearColor(0, 0, 0, 1)
    dcam.Activate(scam)

    gl.glPointSize(2)
    gl.glLineWidth(1)

    # Draw Cluster Cloud
    for cluster in clusters:
        if(cluster.geometry == 'CUBE'):
            gl.glColor3f(1, 0, 0)
        else:
            gl.glColor3f(0, 1, 0)
        pangolin.DrawPoints(
            [n.point.raw for n in cluster.nodes])


    ''' Draw edges
    gl.glColor3f(0, 0, 1)
    lines_from, lines_to = np.empty((n_neighbors, 3)), np.empty((n_neighbors, 3))
    i = 0
    for node in nodes:
        for neighbor in node.neighbors:
            lines_from[i] = node.point.raw
            lines_to[i] = neighbor.point.raw
            i += 1
    pangolin.DrawLines(lines_from[:i], lines_to[:i], 1)
    '''
    pangolin.FinishFrame()

