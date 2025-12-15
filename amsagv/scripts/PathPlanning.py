#!/usr/bin/python3
# -*- coding: utf-8 -*-
from graph_gen import tagMap, tagDets
import math

class PathPlanning(object):
  def __init__(self):
    pass

  def findPath(self, startId, goalId):
    '''Find the shortest path
    
    Inputs:
      startId - Id of the start tag.
      goalId  - Id of the goal tag.
            
    Outputs:
      path - A list of ordered tags that lead from the start tag to the goal tag
             (including start and goal tag) or an empty list if path is not found.
    '''
    path = []
    
    #TODO Implement path planning algorithm here ...
    distances = [float('inf')] * (max(tagMap.keys())+1)
    heuristics = [float('inf')] * (max(tagMap.keys())+1)
    previous = {}
    visited = set()
    

    distances[startId] = 0 

    queue = []
    queue.append(startId)

    index=0

    while queue:
      # Get current node
      vertex = queue.pop()
      visited.add(vertex)

      if vertex == goalId:
        break
      # Check neighbours
      for i in range(2):
        current = tagMap[vertex][i*2] # Get 0 and 2ns place of tagMap - nodes

        # Check if node is existant
        if current != 0:
          #print(current)
          if current in visited:
            continue
          else:
            new_distance = distances[vertex] + tagMap[vertex][i*2+1] # Get 1st and 3rd places of tagMap - distances
            new_euclidan = new_distance + math.sqrt((tagDets[vertex][0] - tagDets[goalId][0])**2 + (tagDets[vertex][1] - tagDets[goalId][1])**2 )
            new_heuristic = new_distance+new_euclidan

          # Compare and update distance
          if new_heuristic < heuristics[current]:
            heuristics[current] = new_heuristic 
            previous[current] = vertex
            
          # Compare and update heursistic
          if new_distance < distances[current]:
            distances[current] = new_distance 
            previous[current] = vertex

          if current in queue:
            continue
          else:
            queue.append(current)
      
          queue.sort(key=lambda n: -heuristics[n])
          index+=1
      
    print(index)

    next = goalId
    
    # Calculate path 
    while next != startId:
      step = previous[next]
      path.append(step)
      next = step

    path.reverse()
    path.append(goalId)
    return path 

  def generateActions(self, path):
    '''Generate a list of actions for given path
    
    Inputs:
      path - A list of ordered tags that lead from the start tag to the goal tag
             (including start and goal tag) or an empty list if path is not found.

    Outputs:
      actions - A list of actions the AGV need to execute in order to reach the goal tag
                from the start tag or an empty list if no action is required/possible.
    '''
    actions = []

    #TODO Convert path to actions here ...
    #action = ('left', 20, 0.202)
    #actions.append(action)
    for i in range (len(path)-1):
      first = path[i]
      second = path[i+1]
      elem = tagMap[first]
      if elem[0] == second:
        value = ('left', elem[0], elem[1])
      elif elem[2] == second:
        value = ('right', elem[2], elem[3])
      elif len(elem) > 4 and elem[4] == second:
        value = ('pass', elem[4], elem[5])
      actions.append(value)
    return actions



if __name__ == '__main__':
  pp = PathPlanning()
  path = pp.findPath(1, 19)
  print(path)
  actions = pp.generateActions(path)
  print(actions)
