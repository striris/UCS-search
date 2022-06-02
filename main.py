# Title: <SSP5> CZ3005 AI Lab Assisgnment 1
# Authors: Ruan Donglin, Liu Xinran, Nan Wentai
# Date: 3/10/2022

import json
import math
from json.encoder import INFINITY
import heapq

g = open('G.json')
d = open('Dist.json')
c = open('Cost.json')
co = open('Coord.json')

graph = json.load(g)
dist = json.load(d)
cost = json.load(c)
coord = json.load(co)

# Q1: Uniform-Cost-Search Function without energy constraint
def ucs(src, goal, graph, dist):
    pq = []                             # Priority queue
    visited = {v: 0 for v in graph}     # Visited nodes
    pred = {v: None for v in graph}     # Predecessors
    heapq.heappush(pq, [0,  src, None]) # Push the source node [distance, node, predecessor] into pq
    while(len(pq) > 0):                 # While the priority queue is not empty
        p = heapq.heappop(pq)           # Pop out the node with minimum culmulative distance
        if p[1] == goal:                # if the removed the node is the goal, generate and print the path in the following lines
            pred[p[1]] = p[2]           
            path = []
            path.append(goal)
            i = goal
            while(pred[i]!= src):
                path.append(pred[i])
                i = pred[i]
            path.append(src)
            path.reverse()
            print('Task1: Uniform Cost Search without energy constraint\n')
            print('Shortest path: ', end='')
            printPath(path)
            print('\nShortest distance: '+str(p[0]))
            return
              
        if visited[p[1]]== 1:           # If the node has already been visited, pass
            pass
        else:                           # If the node hasn't been visited, update
            visited[p[1]] = 1           # Mark the node as visited
            pred[p[1]] = p[2]           # Record the predecessor
            for i in graph[p[1]]:       # Traverse through the child nodes
                heapq.heappush(pq, [p[0]+dist[p[1]+','+i], i, p[1]])   # Push the child nodes of the popped node into the priority queue

# Q2: Uniform-Cost-Search with energy constraint
def ucs_e(src, goal, budget, graph, dist, cost):
    pq = []    
    labels = {v: [] for v in graph}         # Record different expansions (distance, energyCost, predecessor) of every node {}
    labels[src].append([0, 0, src, None])   # Append the source node [distance, energyCost, node, predecessor] to labels{}
    heapq.heappush(pq, [0, 0, src, None])   # [distance, energyCost, node, predecessor]
    while(len(pq) > 0):                     # While the priority queue is not empty
        p = heapq.heappop(pq)               # Pop out the node with minimum culmulative distance
        if p[2] == goal:                    # if the removed the node is the goal, generate and print the path in the following lines
            path = []
            path.append(goal)
            end= p
            while end[3] != src:            # While the predecessor is not the source
                parent = end[3]
                for x in labels[parent]:    # Check the labels of parent node to find the right path
                    if x[1] == end[1] - cost[end[2]+','+parent]:
                        path.append(end[3])
                        end = x
                        break
            path.append(src)
            path.reverse()
            print('Task2: Uniform Cost Search with energy constraint\n')
            print('Shortest path: ', end='')
            printPath(path)
            print('\nShortest distance: '+str(p[0]))   
            print('Total energy cost: '+str(p[1]))
            return

        for i in graph[p[2]]:                   # if the removed the node is not the goal, update labels
            new_label= [p[0]+dist[p[2]+','+i], p[1]+cost[p[2]+','+i], i, p[2]] # Child node (calculate new distance and energy)
            if (new_label[1])<=budget:          # if the energy cost incurred is less than the budget
                flag= True                      # Set the initial value of flag to True
                for label in labels[i]:         # Traverse through different expansions of the child node
                    if label[0] <= new_label[0] and label[1] <= new_label[1]:       # If both the culmulative distance and energy cost are larger than that of the previous expansion 
                        flag = False            # Set the flag to False because this is a useless expansion
                        break
                if flag== True:                 # If the flag remains True
                    labels[i].append(new_label) # Record this new expansion in labels{}
                    heapq.heappush(pq, new_label)   # push the node into pq
        
# Q3: A* search
def aStar(src, goal, budget, graph, dist, cost):
    pq = []    
    labels = {v: [] for v in graph}  
    labels[src].append([heuristicDist(src,goal), 0, 0, src, None])
    pq.append([heuristicDist(src,goal), 0, 0, src, None])    # Append the source node [distance + heuristicDist, distance, energyCost, node]
    while(len(pq) > 0):    
        p = heapq.heappop(pq)   
        if p[3] == goal:    
            path = []
            path.append(goal)
            end= p
            while end[4] != src:
                parent = end[4]
                for x in labels[parent]:
                    if x[2] == end[2] - cost[end[3]+','+parent]:
                        path.append(end[4])
                        end = x
                        break
            path.append(src)
            path.reverse()
            print('Task3: A* Search with energy constraint\n')
            print('Shortest path: ', end='')
            printPath(path)
            print('\nShortest distance: '+str(p[1]))   
            print('Total energy cost: '+str(p[2]))
            return

        for i in graph[p[3]]: 
            hDist = heuristicDist(i,goal)           # Calculate the heristic distance using the function
            energyCost = p[2]+cost[p[3]+','+i]
            distance = p[1]+dist[p[3]+','+i]
            new_label= [distance+hDist, distance, energyCost, i, p[3]]
            if energyCost<=budget:    
                flag= True
                for label in labels[i]:
                    if label[0] <= new_label[0] and label[2] <= new_label[2]:
                        flag = False
                        break
                if flag== True:
                    labels[i].append(new_label)
                    heapq.heappush(pq, new_label)            

# Function to calculate heuristic Distance for A* search
def heuristicDist(i, goal):
    return math.sqrt((coord[i][0] - coord[goal][0])**2 + (coord[i][1] - coord[goal][1])**2)    

# Function to print path
def printPath(path):
    for i in path:
        print(i+'->', end='')


print('\n')
ucs('1', '50', graph, dist)
print('\n')
ucs_e('1', '50', 287932, graph, dist, cost)
print('\n')
aStar('1', '50', 287932, graph, dist, cost)
