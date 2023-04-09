import numpy as np
import math, random, time
import sys

sys.setrecursionlimit(2000)

class Point:
    def load_from(filename):
        points = np.array([])
        with open(filename, 'rb') as f:
            points = np.load(f)
        return points


    def __init__(self, point):
        self.raw = point
        self.x = point[0]
        self.y = point[1]
        self.z = point[2]
        shadow = np.sqrt(self.x**2 + self.y**2)
        self.alpha = math.atan2(self.y, self.x)
        self.beta = math.atan2(self.z, shadow)

        if(self.x == float('inf') or self.x == float('-inf')):
            self.is_valid = False
        elif(self.y == float('inf') or self.y == float('-inf')):
            self.is_valid = False
        elif(self.z  == float('inf') or self.z == float('-inf')):
            self.is_valid = False
        else:
            self.is_valid = True


class Node:
    def __init__(self, point: Point, idx: int):
        self.point = point
        self.idx = idx
        # Needs ctx
        self.row, self.col = -1, -1
        self.ctx = None
        self.neighbors = []
        self.directions = {
            "1,0": None, "-1,0": None,
            "0,1": None, "0,-1": None,
        }


    def missing_directions(self):
        missing = []
        for d in self.directions:
            if(self.directions[d] is None):
                y, x = d.split(',')
                missing.append([int(y), int(x)])
        return missing


    def neighbor(self, direction):
        next_row = self.row + direction[0]
        next_col = self.col + direction[1]
        if(next_row >= self.ctx.LIDAR_HEIGHT):
            return -1
        elif(next_row < 0):
            return -1
        elif(next_col >= self.ctx.LIDAR_WIDTH):
            return -1
        elif(next_col < 0):
            return -1
        return next_row*self.ctx.LIDAR_WIDTH + next_col


    def attach_neighbors(self, nodes):
        if(self.ctx.is_floor(self.point) or not self.point.is_valid):
            return

        for dir_ in self.missing_directions():
            idx = self.neighbor(dir_)
            if(idx == -1):
                continue
            neighbor = nodes[idx]
            point = neighbor.point
            if(not point.is_valid or self.ctx.is_floor(point)):
                continue
            # str direction
            ds = str(dir_[0]) + ',' + str(dir_[1])
            # inv str direction
            inv = str(-dir_[0]) + ',' + str(-dir_[1])

            self.neighbors.append(neighbor)
            self.directions[ds] = neighbor

            neighbor.neighbors.append(self)
            neighbor.directions[inv] = self


class PointCloudCtx:
    # PointCloudCtx needs nodes to calculate floor
    # Nodes needs PointCloudCtx to calculate row and col
    # Circular dependency fixed with a factory
    def defaultFactory(LIDAR_WIDTH, LIDAR_HEIGHT, nodes):
        ctx = PointCloudCtx(LIDAR_WIDTH, LIDAR_HEIGHT, nodes)
        for node in nodes:
            node.ctx = ctx
            node.row = node.idx // LIDAR_WIDTH
            node.col = node.idx - node.row*LIDAR_WIDTH
        return ctx


    def __init__(self, LIDAR_WIDTH, LIDAR_HEIGHT, nodes):
        points_at_infinite_mask = np.array([n.point.is_valid for n in nodes])
        raw_points = np.array([n.point.raw for n in nodes])
        floor = np.amin(raw_points[points_at_infinite_mask][:,2])
        self.floor_thresh = floor + 0.02
        self.LIDAR_WIDTH = LIDAR_WIDTH
        self.LIDAR_HEIGHT = LIDAR_HEIGHT
        self.nodes = nodes


    def is_floor(self, point: Point):
        return point.z <= self.floor_thresh


    def floor_mask(self):
        return np.array([not self.is_floor(n.point) for n in self.nodes])


    def invalid_points_mask(self):
        return np.array([n.point.is_valid for n in self.nodes])


class CloudCluster:
    def dfs(node, funct):
        if not hasattr(node, 'visited'):
            node.visited = True
            funct(node)
            for neighbor in node.neighbors:
                CloudCluster.dfs(neighbor, funct)


    def not_visited_node(nodes):
        for node in nodes:
            if not hasattr(node, 'visited'):
                return node
        return None


    def cleanup_nodes(nodes):
        for n in nodes:
            del n.visited


    def cluster_nodes(nodes):
        clusters = []
        missing_visit = CloudCluster.not_visited_node(nodes)

        while(missing_visit is not None):
            nodes_in_cluster = []
            def tag_node(node):
                node.cluster = len(clusters)
                nodes_in_cluster.append(node)

            CloudCluster.dfs(missing_visit, tag_node)
            clusters.append(
                CloudCluster(missing_visit, np.array(nodes_in_cluster)))
            missing_visit = CloudCluster.not_visited_node(nodes)
        # Cleanup
        CloudCluster.cleanup_nodes(nodes)
        return clusters


    def __init__(self, init_node, nodes):
        self.init_node = init_node
        self.index = init_node.cluster
        self.nodes = nodes


    def non_redundant_nodes(self):
        return self.nodes
        grid = np.zeros((len(self.nodes), 4, 3))
        for i, node in enumerate(self.nodes):
            for j, n in enumerate(node.neighbors):
                grid[i][j] = n.point.raw - node.point.raw
                grid[i][j] /= np.linalg.norm(grid[i][j])
        grid = np.sum(grid, axis=1)
        grid = np.sum(grid, axis=1)
        redundant_mask = np.array([v > 0.2 for v in grid])
        return self.nodes[redundant_mask]


    def random_walk(self, l = 256):
        path = []
        for i in range(l):
            next_node = self.nodes[
                np.random.randint(0, len(self.nodes))]
            path.append(next_node.point.raw)
        path = np.array(path)
        path -= np.repeat([path[0]], len(path), axis=0)
        return path
        

