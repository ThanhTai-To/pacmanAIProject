"""
In search.py, you will implement generic search algorithms which are called by
Pacman agents (in searchAgents.py).
"""

import util
import sys
from time import sleep

from game import Directions

n = Directions.NORTH
s = Directions.SOUTH
e = Directions.EAST
w = Directions.WEST


class SearchProblem:
    """
    This class outlines the structure of a search problem, but doesn't implement
    any of the methods (in object-oriented terminology: an abstract class).

    You do not need to change anything in this class, ever.
    """

    def getStartState(self):
        """
        Returns the start state for the search problem.
        """
        util.raiseNotDefined()

    def isGoalState(self, state):
        """
          state: Search state

        Returns True if and only if the state is a valid goal state.
        """
        util.raiseNotDefined()

    def getSuccessors(self, state):
        """
          state: Search state

        For a given state, this should return a list of triples, (successor,
        action, stepCost), where 'successor' is a successor to the current
        state, 'action' is the action required to get there, and 'stepCost' is
        the incremental cost of expanding to that successor.
        """
        util.raiseNotDefined()

    def getCostOfActions(self, actions):
        """
         actions: A list of actions to take

        This method returns the total cost of a particular sequence of actions.
        The sequence must be composed of legal moves.
        """
        util.raiseNotDefined()

# Source code https://github.com/Deus1223/Pac-Man/blob/master/search.py
def depthFirstSearch(problem):
    '''
    return a path to the goal
    '''
    # TODO 05
    from util import Stack

    # the component of frontier is (state, the path to the state)
    frontier = Stack()
    frontier.push(((problem.getStartState()), []))
    # print(f"frontier: {frontier}, problem.getStartState(): {problem.getStartState()}")
    # frontier: <util.Stack object at 0x7f373dfd0640>, problem.getStartState():
    # ((11, 6), <game.Grid object at 0x7f373dfd0790>)


    # store the visited state
    visited = [problem.getStartState()]


    while not frontier.isEmpty():

      print("\n\nFrontier before poping: ")
      for node in frontier.getNodesInStack():
          print(f"\t{node[0][0]}")

      # state[0] is coordinates and state[1] is game.Grid object
      (state, path) = frontier.pop()
      print(f"\n\tPOPED, its state: {state[0]}, its path: {path}")


      if(problem.isGoalState(state)):
        print(f"\n\n\n\n\n\nGoal, state={state} , path: {path}")
        break

      # successors contain node, game.Grid, path, cost
      successors = problem.getSuccessors(state)
      print(f"\n\tsuccessors: {successors}")
      for i in successors:
        #print(f"i: {i} in successors ^^^")
        if(i[0] not in visited):  # any state has been visited doesn't need to be visited again
          print(f"\n\t\ti[0]: {i[0]} not in visited:visited")
          visited.append(i[0])
          frontier.push((i[0], path + [i[1]]))
          print(f"\n\t\tPushed i[0]^^, path + [i[1]]: it path[direction: e.g: W, T, E,...]")

    return path

    util.raiseNotDefined()


def breadthFirstSearch(problem):
    '''
    return a path to the goal
    '''
    # TODO 06
    from util import Queue

    # the component of frontier is (state, the path to the state)
    frontier = Queue()
    frontier.enqueue(((problem.getStartState()), []))

    # store the visited state
    visited = []

    while(not frontier.isEmpty()):
        (state, path) = frontier.dequeue()
        if(problem.isGoalState(state)):
            print(f"path: {path}")
            return path

        successors = problem.getSuccessors(state)
        for i in successors:
            if(i[0] not in visited):  # any state has been visited doesn't need to be visited again
                visited.append(i[0])
                frontier.enqueue((i[0], path + [i[1]]))

    return path

    util.raiseNotDefined()


def uniformCostSearch(problem):
    '''
    return a path to the goal
    '''
    # TODO 07
    from util import PriorityQueue

    frontier = PriorityQueue()
    frontier.push((problem.getStartState(), [], 0), 0)

    print(problem.getStartState())
    visited = []

    while not frontier.isEmpty():

        #print(f"\n\nfrontier: {frontier}")

        curNode, path, curCost = frontier.pop()
        #print(f"\n\tcurCost: {curCost}, curNode: {curNode}, path: {path}")

        if curNode not in visited:
            visited.append(curNode)

            if problem.isGoalState(curNode):
                print(f"Path to goal: {path}")
                return path

            successors = problem.getSuccessors(curNode)
            #print(f"\n\t\t{successors}")

            for neighborNode, direction, cost in successors:
                #if neighbor not in visited:
                print(f"\n\nneighbor: {neighborNode}, direction: {direction}, cost: {cost}")
                frontier.update((neighborNode, path + [direction], curCost + cost), curCost + cost)





def nullHeuristic(state, problem=None):
    """
    A heuristic function estimates the cost from the current state to the nearest
    goal in the provided SearchProblem.  This heuristic is trivial.
    """
    return 0

# TODO 08 + 09
    '''
    students propose at least two heuristic functions for A*
    '''
def manhattanHeuristic(position, problem):
    # The Manhattan distance heuristic for a PositionSearchProblem
    pacPosition = position[0]
    xy2 = problem.getStartState()

    foods = xy2[1].asList()
    for food in foods:
        print(f"food: {food}, foods: {foods}, pacPosition: {pacPosition}")
        return abs(pacPosition[0] - food[0]) + abs(pacPosition[1] - food[1])


def euclideanHeuristic(position, problem):
    # The Euclidean distance heuristic for a PositionSearchProblem
    pacPosition = position[0]
    xy2 = problem.getStartState()

    foods = xy2[1].asList()
    for food in foods:
        return ((pacPosition[0] - food[0]) ** 2 + (pacPosition[1] - food[1]) ** 2) ** 0.5


def aStarSearch(problem, heuristic=nullHeuristic):
    '''
    return a path to the goal
    '''
    from util import PriorityQueue
    # TODO 10
    startingNode = problem.getStartState()
    if problem.isGoalState(startingNode):
        return []

    visitedNodes = []

    pQueue = util.PriorityQueue()
    #((coordinate/node , action to current node , cost to current node),priority)
    pQueue.push((startingNode, [], 0), 0)

    while not pQueue.isEmpty():

        currentNode, path, prevCost = pQueue.pop()
        print(f"\n\nCurrentNode: {currentNode[0]}, \n\tgoals: {currentNode[1].asList()}")

        if currentNode not in visitedNodes:
            visitedNodes.append(currentNode)

            if problem.isGoalState(currentNode):
                print(f"Path: {path}")
                return path

            for nextNode, direction, cost in problem.getSuccessors(currentNode):
                print(f"\n\tnextNode: {nextNode}")
                newAction = path + [direction]
                newCostToNode = prevCost + cost
                heuristicCost = newCostToNode + heuristic(nextNode,problem)
                print(f"\theuristicCost: {heuristicCost}, costToNode: {newCostToNode}")
                pQueue.push((nextNode, newAction, newCostToNode),heuristicCost)

    util.raiseNotDefined()


# Abbreviations
bfs = breadthFirstSearch
dfs = depthFirstSearch
astar = aStarSearch
ucs = uniformCostSearch
