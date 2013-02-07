# search.py
# ---------
# Licensing Information: Please do not distribute or publish solutions to this
# project. You are free to use and extend these projects for educational
# purposes. The Pacman AI projects were developed at UC Berkeley, primarily by
# John DeNero (denero@cs.berkeley.edu) and Dan Klein (klein@cs.berkeley.edu).
# Student side autograding was added by Brad Miller, Nick Hay, and Pieter 
# Abbeel in Spring 2013.
# For more info, see http://inst.eecs.berkeley.edu/~cs188/pacman/pacman.html

"""
In search.py, you will implement generic search algorithms which are called
by Pacman agents (in searchAgents.py).
"""

import util
import pdb

class SearchProblem:
    """
    This class outlines the structure of a search problem, but doesn't implement
    any of the methods (in object-oriented terminology: an abstract class).

    You do not need to change anything in this class, ever.
    """

    def getStartState(self):
        """
        Returns the start state for the search problem
        """
        util.raiseNotDefined()

    def isGoalState(self, state):
        """
          state: Search state

        Returns True if and only if the state is a valid goal state
        """
        util.raiseNotDefined()

    def getSuccessors(self, state):
        """
          state: Search state

        For a given state, this should return a list of triples,
        (successor, action, stepCost), where 'successor' is a
        successor to the current state, 'action' is the action
        required to get there, and 'stepCost' is the incremental
        cost of expanding to that successor
        """
        util.raiseNotDefined()

    def getCostOfActions(self, actions):
        """
         actions: A list of actions to take

        This method returns the total cost of a particular sequence of actions.  The sequence must
        be composed of legal moves
        """
        util.raiseNotDefined()


def tinyMazeSearch(problem):
    """
    Returns a sequence of moves that solves tinyMaze.  For any other
    maze, the sequence of moves will be incorrect, so only use this for tinyMaze
    """
    from game import Directions
    s = Directions.SOUTH
    w = Directions.WEST
    return  [s,s,w,s,w,w,s,w]

def depthFirstSearch(problem):
    """
    Search the deepest nodes in the search tree first

    Your search algorithm needs to return a list of actions that reaches
    the goal.  Make sure to implement a graph search algorithm

    To get started, you might want to try some of these simple commands to
    understand the search problem that is being passed in:

    print "Start:", problem.getStartState()
    print "Is the start a goal?", problem.isGoalState(problem.getStartState())
    print "Start's successors:", problem.getSuccessors(problem.getStartState())
    """
    "*** YOUR CODE HERE ***"
    
    from game import Directions
    
    fringe = util.Stack()
    closedSet = set()
    
    "fringe : element type is list of states, that is path"
    fringe.push([problem.getStartState()])
    
    while (1):
        if fringe.isEmpty():
            return util.raiseNotDefined()    
        
        # Popping strategy : Stack
        track = fringe.pop()
        node = track[-1]
        
        if problem.isGoalState(node):
            break
        
        if not node in closedSet:
            closedSet.add(node)
            for successor in problem.getSuccessors(node):
                childNode = successor[0]
                fringe.push(track+[childNode])
    
    # Now it's time to extract actions from the result solution.
    action = [];
    prev = track[0]
    
    for state in track[1:]:
        for successor in problem.getSuccessors(prev):
            if state == successor[0]: d = successor[1]
        action.append(d)
        prev = state
    return  action
    

def breadthFirstSearch(problem):
    """
    Search the shallowest nodes in the search tree first.
    """
    "*** YOUR CODE HERE ***"
    
    fringe = util.PriorityQueue()
    closedSet = set()
    
    "fringe : element type is list of a tuple of (path, depth)"
    fringe.push([problem.getStartState()],0)
    
    while (1):
        if fringe.isEmpty():
            return util.raiseNotDefined()
        
        # Popping strategy : PrioirtyQueue will choose one that has least depth
        track = fringe.pop()
        node = track[-1]
        
        if problem.isGoalState(node):
            break
        
        if not node in closedSet:
            closedSet.add(node)
            for successor in problem.getSuccessors(node):
                childNode = successor[0]
                fringe.push(track+[childNode], len(track))
    
    
    # Now it's time to extract actions from the result solution.
    action = [];
    prev = track[0]
    
    for state in track[1:]:
        for successor in problem.getSuccessors(prev):
            if state == successor[0]: d = successor[1]
        action.append(d)
        prev = state
    return  action
    

def uniformCostSearch(problem):
    "Search the node of least total cost first. "
    "*** YOUR CODE HERE ***"
    fringe = util.PriorityQueue()
    closedSet = set()
    
    "fringe : element type is list of a tuple of (track=state-list, cost)"
    fringe.push( [(problem.getStartState(), 0)], 0)
    
    while (1):
        if fringe.isEmpty():
            return util.raiseNotDefined()
        
        # Popping strategy : PrioirtyQueue will choose one that has least depth
        track = fringe.pop()
        node = track[-1][0]
        backwardcost = track[-1][1]
        
        if problem.isGoalState(node):
            break
        
        if not node in closedSet:
            closedSet.add(node)
            for successor in problem.getSuccessors(node):
                childNode = successor[0]
                cost = backwardcost + successor[2]
                fringe.push( track+[(childNode, cost)], cost)
    
    
    # Now it's time to extract actions from the result solution.
    action = [];
    prev = track[0]
    
    for state in track[1:]:
        for successor in problem.getSuccessors(prev[0]):
            if state[0] == successor[0]: d = successor[1]
        action.append(d)
        prev = state
    return  action
    

def nullHeuristic(state, problem=None):
    """
    A heuristic function estimates the cost from the current state to the nearest
    goal in the provided SearchProblem.  This heuristic is trivial.
    """
    return 0

def aStarSearch(problem, heuristic=nullHeuristic):
    "Search the node that has the lowest combined cost and heuristic first."
    "*** YOUR CODE HERE ***"
    
    fringe = util.PriorityQueue()
    closedSet = set()
    
    "fringe : element type is list of a tuple of (track=state-list, backwardcost)"
    fringe.push( [(problem.getStartState(), 0)], 0)
    
    while (1):
        if fringe.isEmpty():
            return util.raiseNotDefined()
        
        # Popping strategy : PrioirtyQueue will choose one that has least depth
        track = fringe.pop()
        node = track[-1][0]
        backwardcost = track[-1][1]
        
        if problem.isGoalState(node):
            break
        
        if not node in closedSet:
            closedSet.add(node)
            for successor in problem.getSuccessors(node):
                childNode = successor[0]
                h = backwardcost + successor[2]
                g = heuristic(childNode, problem)
                cost = h+g
                fringe.push( track+[(childNode, h)], cost)
    
    
    # Now it's time to extract actions from the result solution.
    action = [];
    prev = track[0]
    
    for state in track[1:]:
        for successor in problem.getSuccessors(prev[0]):
            if state[0] == successor[0]: d = successor[1]
        action.append(d)
        prev = state
    return  action
    

# Abbreviations
bfs = breadthFirstSearch
dfs = depthFirstSearch
astar = aStarSearch
ucs = uniformCostSearch
