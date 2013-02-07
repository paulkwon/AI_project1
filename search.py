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
    
    """
    fringe: 
     - type     : PriorityQueue
     - element  : ( (state, action, cost), parent state, priority value )
    """
        
    fringe = util.PriorityQueue()
    closedSet = set()
    record = dict()
    
    parentState = -1
    currentDepth = 0
    startState = problem.getStartState()
    fringe.push( ((startState,'none', 0), parentState, currentDepth), currentDepth)
    
    while (1):
        if fringe.isEmpty():
            return util.raiseNotDefined()
        
        # Popping strategy : PrioirtyQueue will choose one that has least depth
        successor, parentState, currentDepth = fringe.pop()
        currentState = successor[0]
        previousAction = successor[1]
        
        if problem.isGoalState(currentState):
            break
        
        if not currentState in closedSet:
            closedSet.add(currentState)
            record[currentState] = (previousAction, parentState)
            
            k = problem.getSuccessors(currentState)
            k.reverse()
            for successor in k: #problem.getSuccessors(currentState):
                fringe.push( (successor, currentState, currentDepth-1), currentDepth-1)
    
    # Now it's time to extract actions.
    actions = [previousAction]
    state = parentState
    while (not state == startState):
        actions.append(record[state][0])
        state = record[state][1]
    actions.reverse()
    return actions
    

def breadthFirstSearch(problem):
    """
    Search the shallowest nodes in the search tree first.
    """
    "*** YOUR CODE HERE ***"

    """
    fringe: 
     - type     : PriorityQueue
     - element  : ( (state, action, cost), parent state, priority value )
    """
        
    fringe = util.PriorityQueue()
    closedSet = set()
    record = dict()
    
    parentState = -1
    currentDepth = 0
    startState = problem.getStartState()
    fringe.push( ((startState,'none', 0), parentState, currentDepth), currentDepth)
    
    while (1):
        if fringe.isEmpty():
            return util.raiseNotDefined()
        
        # Popping strategy : PrioirtyQueue will choose one that has least depth
        successor, parentState, currentDepth = fringe.pop()
        currentState = successor[0]
        previousAction = successor[1]
        
        if problem.isGoalState(currentState):
            break
        
        if not currentState in closedSet:
            closedSet.add(currentState)
            record[currentState] = (previousAction, parentState)
            for successor in problem.getSuccessors(currentState):
                fringe.push( (successor, currentState, currentDepth+1), currentDepth+1)
    
    # Now it's time to extract actions.
    actions = [previousAction]
    state = parentState
    while (not state == startState):
        actions.append(record[state][0])
        state = record[state][1]
    actions.reverse()
    return actions
    

def uniformCostSearch(problem):
    "Search the node of least total cost first. "
    "*** YOUR CODE HERE ***"
    
    """
    fringe: 
     - type     : PriorityQueue
     - element  : ( (state, action, cost), parent state, priority value )
    """
        
    fringe = util.PriorityQueue()
    closedSet = set()
    record = dict()
    
    parentState = -1
    backwardCost = 0
    startState = problem.getStartState()
    fringe.push( ((startState,'none', 0), parentState, backwardCost), backwardCost)
    
    while (1):
        if fringe.isEmpty():
            return util.raiseNotDefined()
        
        # Popping strategy : PrioirtyQueue will choose one that has least depth
        successor, parentState, backwardCost = fringe.pop()
        currentState = successor[0]
        previousAction = successor[1]
        
        if problem.isGoalState(currentState):
            break
        
        if not currentState in closedSet:
            closedSet.add(currentState)
            record[currentState] = (previousAction, parentState)
            for successor in problem.getSuccessors(currentState):
                cost = backwardCost + successor[2]
                fringe.push( (successor, currentState, cost), cost)
    
    # Now it's time to extract actions.
    actions = [previousAction]
    state = parentState
    while (not state == startState):
        actions.append(record[state][0])
        state = record[state][1]
    actions.reverse()
    return actions

def nullHeuristic(state, problem=None):
    """
    A heuristic function estimates the cost from the current state to the nearest
    goal in the provided SearchProblem.  This heuristic is trivial.
    """
    return 0

def aStarSearch(problem, heuristic=nullHeuristic):
    "Search the node that has the lowest combined cost and heuristic first."
    "*** YOUR CODE HERE ***"
    
    """
    fringe: 
     - type     : PriorityQueue
     - element  : ( (state, action, cost), parent state, priority value )
    """
        
    fringe = util.PriorityQueue()
    closedSet = set()
    record = dict()
    
    parentState = -1
    backwardCost = 0
    startState = problem.getStartState()
    fringe.push( ((startState,'none', 0), parentState, backwardCost), backwardCost)
    
    while (1):
        if fringe.isEmpty():
            return util.raiseNotDefined()
        
        # Popping strategy : PrioirtyQueue will choose one that has least depth
        successor, parentState, backwardCost = fringe.pop()
        currentState = successor[0]
        previousAction = successor[1]
        
        if problem.isGoalState(currentState):
            break
                
        if not currentState in closedSet:
            # print "current", currentState
            closedSet.add(currentState)
            record[currentState] = (previousAction, parentState)
            for successor in problem.getSuccessors(currentState):
                h = backwardCost + successor[2]
                g = heuristic(successor[0], problem)
                cost = h+g
                # print "successor ", successor, " h+g ", cost 
                fringe.push( (successor, currentState, h), cost)
                
        # pdb.set_trace()
    
    # Now it's time to extract actions.
    actions = [previousAction]
    state = parentState
    while (not state == startState):
        actions.append(record[state][0])
        state = record[state][1]
    actions.reverse()
    return actions

# Abbreviations
bfs = breadthFirstSearch
dfs = depthFirstSearch
astar = aStarSearch
ucs = uniformCostSearch
