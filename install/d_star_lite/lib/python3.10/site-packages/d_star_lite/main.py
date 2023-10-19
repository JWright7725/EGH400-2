from cmath import inf
import heapq
from graph import Grid, Node
from d_star_lite import initDStarLite, computeShortestPath, moveAndRescan

def __init__():
    pass

if __name__ == "__main__":
    # Dimensions of the environment grid
    LENGTH = 5
    WIDTH = 5
    HEIGHT = 5
    # Start and end locations
    S_START = (0, 0, 0)
    S_GOAL = (4, 4, 4)

    # Define some viewing distance for UAVs
    VIEW_DISTANCE = 1

    # Create a grid object for the environment
    graph = Grid((LENGTH, WIDTH, HEIGHT))
    # Set the start and end locations of the path
    graph.setStart(S_START)
    graph.setGoal(S_GOAL)

    # Initialise the D* Lite algorithm
    graph, queue, k_m = initDStarLite(graph, S_START, S_GOAL)
    # Complete the initial search
    computeShortestPath(graph, queue, S_START, k_m)
    # Create a new variable to keep track of the current position
    s_current = S_START
    # Create a new array to keep track of the planned path
    path = [s_current]
    while s_current != S_GOAL:
        s_new, k_m = moveAndRescan(graph, queue, s_current, k_m, VIEW_DISTANCE)
        s_current = s_new
        path.append(s_current)
    print('\nThe Chosen Path Is:')
    print(path)