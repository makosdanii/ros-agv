#!/usr/bin/python3
# -*- coding: utf-8 -*-
from graph_gen import tagMap, tagDets

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
    open_list = [(startId, 0, None)]
    closed_list = []

    while len(open_list):
      open_list = sorted(open_list, key=lambda arg: arg[1])
      best = open_list.pop()
      closed_list.append(best)
      if best[0] == goalId:
        break
      neighbours = tagMap[best[0]]
      for idx in range(0, len(neighbours)-1, 2):
        n1 = neighbours[idx]
        w1 = neighbours[idx+1]
        if w1 != 0 and not (n1 in [i[0] for i in closed_list]):
          for n in open_list:
            if n[0] == n1:
              n[1] = min(n[1], best[1]+w1) 
              break
          else:
            open_list.append((n1, best[1]+w1, best[0]))
      
    print(closed_list)

    path = []
    def traceback(next_goal, path):
      path.append(next_goal[0])
      for elem in closed_list:
        if elem[0] == next_goal[2]:
          if elem[0] == startId:
            return path
          path = traceback(elem, path)
          break
      return path

    path = traceback(closed_list[-1], path)
    path.append(startId)
    print(path)
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
  path = pp.findPath(2, 12)
  print(path)
  actions = pp.generateActions(path)
  print(actions)
