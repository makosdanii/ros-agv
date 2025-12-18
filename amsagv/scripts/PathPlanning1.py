#!/usr/bin/python3
# -*- coding: utf-8 -*-
from math import sqrt
from graph_gen import tagMap, tagDets

class PathPlanning(object):
  def __init__(self):
    pass

  @staticmethod
  def heuristic1(pos1, pos2, width=None, height=None):
    '''Heuristic 1
    
    Inputs:
      pos1 - A tuple with x and y coordinate of point 1 (in meters).
      pos2 - A tuple with x and y coordinate of point 2 (in meters).
      width - Map width (in meters).
      height - Map height (in meters).
    
    Outputs:
      h - Heuristic or None if not appropriate
    '''
    h = sqrt((pos1[0]-pos2[0])**2 + (pos1[1]-pos2[1])**2)
    return h    
  
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
    
    tagMap = self.tm.map['edges']
    tagDets = self.tm.map['vertices']

    open_list = [(startId, 0, None)]
    closed_list = []
    iteration = 0
    while len(open_list):
      iteration += 1
      # Difference between Dijkstra's as oepened nodes sorted on heuristical distance
      open_list = sorted(open_list, key=lambda arg: arg[2])
      best = open_list.pop(0)
      closed_list.append(best)
      if best[0] == goalId:
        break
      neighbours = tagMap[best[0]]
      for idx_n in range(0, len(neighbours)-1, 2):
        n1 = neighbours[idx_n]
        w1 = neighbours[idx_n+1]
        if n1 != 0 and not (n1 in [i[0] for i in closed_list]):
          # Dijkstra's way of computing next possible distance from opened node
          tentative = best[1]+w1
          # Compute heuristics as well
          h = PathPlanning.heuristic1(tagDets[n1], tagDets[goalId], self.tm.width, self.tm.height)
          for idx_o in range(len(open_list)):
            if open_list[idx_o][0] == n1:
              # check if the new route is shorter then the existing one to the neighbor
              if tentative < open_list[idx_o][1]:
                open_list[idx_o] = (n1, tentative, tentative + h, open_list[idx_o][3])
              break
          else:
            # Node haven't been traversed yet, therefore add computed distance, and the heuristics too
            open_list.append((n1, tentative, tentative + h, best[0]))
    else:
      print("No path found")

    # print(closed_list)
    def traceback(next_goal, path):
      path.append(next_goal[0])
      for elem in closed_list:
        if elem[0] == next_goal[3]:
          if elem[0] == startId:
            return path
          path = traceback(elem, path)
          break
      else:
        print("Traceback error")
      return path
    
    if len(closed_list) > 0:
      path = traceback(closed_list[-1], path)
      path.append(startId)
      path = list(reversed(path))
    
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

    return actions



if __name__ == '__main__':
  pp = PathPlanning()
  path = pp.findPath(111, 110)
  print(path)
  actions = pp.generateActions(path)
  print(actions)
