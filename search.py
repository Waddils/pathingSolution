import matplotlib.pyplot as plt
import numpy as np
import time
import matplotlib.animation as animation
import math

from utils import *
from grid import *

def gen_polygons(worldfilepath):
    polygons = []
    
    with open(worldfilepath, "r") as f:
        lines = f.readlines()
        lines = [line[:-1] for line in lines]
        
        for line in lines:
            polygon = []
            pts = line.split(';')
            
            for pt in pts:
                xy = pt.split(',')
                polygon.append(Point(int(xy[0]), int(xy[1])))
                
            polygons.append(polygon)
            
    return polygons


def pointsOnLineSeg(p1, p2):
    points = set()
    dx = abs(p2.x - p1.x)
    dy = abs(p2.y - p1.y)
    x = p1.x
    y = p1.y
    
    if (p1.x > p2.x):
        sx = -1
    else:
        sx = 1  
        
    if (p1.y > p2.y):
        sy = -1
    else:
        sy = 1

    # Finds the points along the line made by the polygons with some derivation(?) for accuracy
    if dx > dy:
        err = dx / 3.0
        
        while x != p2.x:
            points.add(Point(x, y))
            err -= dy
            
            if err < 0:
                y += sy
                err += dx
                
            x += sx
    else:
        err = dy / 3.0
        
        while y != p2.y:
            points.add(Point(x, y))
            err -= dx
            
            if err < 0:
                x += sx
                err += dy
                
            y += sy
    
    points.add(p2)
    return points


def pointsOnPolygonEdges(polygon):
    points = set()
    numPoints = len(polygon)
    
    for i in range(numPoints):
        p1 = polygon[i]
        p2 = polygon[(i + 1) % numPoints]
        points.update(pointsOnLineSeg(p1, p2))
        
    return points


def polygonDetection(polygons):
    points = set()
    
    for polygon in polygons:
        points.update(pointsOnPolygonEdges(polygon))
        
    return points


class Node:
    def __init__(self, state, parent = None, cost = 0, heuristic = 0):
        self.state = state
        self.parent = parent
        self.cost = cost
        self.heuristic = heuristic
        
        
def isCycle(node):
    while node.parent is not None:
        
        if node.parent.state == node.state:
            return True
        
        node = node.parent
        
    return False


def expand(node, enclosurePoints):
    x = node.state.x 
    y = node.state.y
    neighbors = []

    # 50x50 grid
    grid = 50  

    # Up / Right / Down / Left
    movements = [(0, 1), (1, 0), (0, -1), (-1, 0)]

    for dx, dy in movements:
        newX = x + dx 
        newY = y + dy
        newPoint = Point(newX, newY)
        
        if (0 <= newX < grid) and (0 <= newY < grid):
            if newPoint not in enclosurePoints:
                neighbors.append(newPoint)

    return neighbors


# Straight-line Distance Formula
def euclideanDistance(p1, p2):
    return math.sqrt((p1.x - p2.x) ** 2 + (p1.y - p2.y) ** 2)
        
      
 
###########################################
#                                         #
#          Breadth-First Search           #
#                                         #
###########################################
       
def BFS(source, dest, enclosurePoints, turfPoints):
    frontier = PriorityQueue()
    frontier.push(Node(source), 0)
    reached = {source: Node(source)}
    expandedNodes = 0
    path = []

    while not frontier.isEmpty():
        node = frontier.pop()
        expandedNodes += 1

        if dest == node.state:
            print("Nodes Expanded:", expandedNodes)
            print("Path Cost:", node.cost)

            path = []
            
            while node:
                path.append((node.state.x, node.state.y))
                node = node.parent
                
            path = path[::-1]
            
            return path

        for childState in expand(node, enclosurePoints):
            
            if childState in turfPoints:
                child = Node(state = childState, parent = node, cost = node.cost + 1.5)
            else:
                child = Node(state = childState, parent = node, cost = node.cost + 1)

            if child.state not in reached or child.cost < reached[child.state].cost:
                reached[child.state] = child
                frontier.push(child, child.cost)

    return None



###########################################
#                                         #
#           Depth-First Search            #
#                                         #
###########################################

def DFS(source, dest, enclosurePoints, turfPoints):
    frontier = Stack()
    visited = set()
    frontier.push(Node(source))
    expandedNodes = 0
    path = []

    while not frontier.isEmpty():
        node = frontier.pop()
        expandedNodes += 1

        if dest == node.state:
            print("Nodes Expanded:", expandedNodes)
            print("Path Cost:", node.cost)

            path = []
            
            while node:
                path.append((node.state.x, node.state.y))
                node = node.parent
                
            path = path[::-1]

            return path

        if node.state in visited:
            continue
        
        visited.add(node.state)

        for childState in expand(node, enclosurePoints):
            
            if childState in turfPoints:
                child = Node(state = childState, parent = node, cost = node.cost + 1.5)
            else:
                child = Node(state = childState, parent = node, cost = node.cost + 1)
                
            if not isCycle(child):
                frontier.push(child)
            
    return None



###########################################
#                                         #
#         Greedy Best-First Search        #
#                                         #
###########################################

def GBFS(source, dest, enclosurePoints, turfPoints):
    frontier = PriorityQueue()
    frontier.push(Node(state = source, heuristic = euclideanDistance(source, dest)), euclideanDistance(source, dest))
    reached = {source: Node(state = source, heuristic = euclideanDistance(source, dest))}
    expandedNodes = 0
    path = []

    while not frontier.isEmpty():
        node = frontier.pop()
        expandedNodes += 1

        if dest == node.state:
            print("Nodes Expanded:", expandedNodes)
            print("Path Cost:", node.cost)

            path = []
            
            while node:
                path.append((node.state.x, node.state.y))
                node = node.parent

            path = path[::-1]
            
            return path

        for child_state in expand(node, enclosurePoints):
            
            if child_state in turfPoints:
                child = Node(state = child_state, parent = node, cost = node.cost + 1.5, heuristic = euclideanDistance(child_state, dest))
            else:
                child = Node(state = child_state, parent = node, cost = node.cost + 1, heuristic = euclideanDistance(child_state, dest))

            if child.state not in reached or child.cost < reached[child.state].cost:
                reached[child.state] = child
                frontier.push(child, child.heuristic)

    return None



###########################################
#                                         #
#                A* Search                #
#                                         #
###########################################

def AStar(source, dest, enclosurePoints, turfPoints):
    frontier = PriorityQueue()
    frontier.push(Node(state = source, heuristic = euclideanDistance(source, dest)), euclideanDistance(source, dest))
    reached = {source: Node(state = source, parent = None, cost = 0, heuristic = euclideanDistance(source, dest))}
    expandedNodes = 0
    path = []

    while not frontier.isEmpty():
        node = frontier.pop()
        expandedNodes += 1

        if dest == node.state:
            print("Nodes Expanded:", expandedNodes)
            print("Path Cost:", node.cost)

            path = []
            
            while node:
                path.append((node.state.x, node.state.y))
                node = node.parent

            path = path[::-1]
            
            return path

        for child_state in expand(node, enclosurePoints):
            
            if child_state in turfPoints:
                newCost = node.cost + 1.5
            else:
                newCost = node.cost + 1
                
            heuristicCost = euclideanDistance(child_state, dest)
            total = newCost + heuristicCost

            if child_state not in reached or newCost < reached[child_state].cost:
                reached[child_state] = Node(state = child_state, parent = node, cost = newCost, heuristic = heuristicCost)
                frontier.push(reached[child_state], total)

    return None









if __name__ == "__main__":
    epolygons = gen_polygons('TestingGrid/world1_enclosures.txt')
    tpolygons = gen_polygons('TestingGrid/world1_turfs.txt')
    
    enclosurePoints = polygonDetection(epolygons)
    turfPoints = polygonDetection(tpolygons)
    
    source = Point(8,10)
    dest = Point(43,45)

    fig, ax = draw_board()
    draw_grids(ax)
    draw_source(ax, source.x, source.y)  # source point
    draw_dest(ax, dest.x, dest.y)  # destination point
    
    # Draw enclosure polygons
    for polygon in epolygons:
        for p in polygon:
            draw_point(ax, p.x, p.y)
    for polygon in epolygons:
        for i in range(0, len(polygon)):
            draw_line(ax, [polygon[i].x, polygon[(i+1)%len(polygon)].x], [polygon[i].y, polygon[(i+1)%len(polygon)].y])
    
    # Draw turf polygons
    for polygon in tpolygons:
        for p in polygon:
            draw_green_point(ax, p.x, p.y)
    for polygon in tpolygons:
        for i in range(0, len(polygon)):
            draw_green_line(ax, [polygon[i].x, polygon[(i+1)%len(polygon)].x], [polygon[i].y, polygon[(i+1)%len(polygon)].y])

    #### Here call your search to compute and collect res_path
    
    print("Breadth First Search:")
    search = BFS(source, dest, enclosurePoints, turfPoints)
    
    # print("Depth First Search:")
    # search = DFS(source, dest, enclosurePoints, turfPoints)
    
    # print("Greedy BFS:")
    # search = GBFS(source, dest, enclosurePoints, turfPoints)
    
    # print("A*:")
    # search = AStar(source, dest, enclosurePoints, turfPoints)    
    
    if search:
        for i in range(len(search) - 1):
            draw_result_line(ax, [search[i][0], search[i+1][0]], [search[i][1], search[i+1][1]])
            
        plt.show()
    else:
        print("No path found.")
        
        
    # Printing out the results
    print("Breadth First Search:")
    BFS(source, dest, enclosurePoints, turfPoints)
    print()
    
    print("Depth First Search:")
    DFS(source, dest, enclosurePoints, turfPoints)
    print()
    
    print("Greedy BFS:")
    GBFS(source, dest, enclosurePoints, turfPoints)
    print()
    
    print("A*:")
    AStar(source, dest, enclosurePoints, turfPoints)
    
