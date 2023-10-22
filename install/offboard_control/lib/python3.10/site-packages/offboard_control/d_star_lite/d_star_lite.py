import heapq
from cmath import inf

def hueristicFromS(location, s):
    # Return the maximum distance between points along any given axis
    return max([abs(d - s[ii]) for ii, d in enumerate(location)])

def topKey(queue):
    if queue:
        # Return the values of the top key (k_1, k_2) without the location data
        return queue[0][:2]
    else:
        # If there are no nodes in the queue, return (inf, inf)
        return(inf, inf)

def calculateKey(graph, location, s_current, k_m):
    return (min(graph.graph[location].g, graph.graph[location].rhs) + hueristicFromS(location, s_current) + k_m, min(graph.graph[location].g, graph.graph[location].rhs))

def updateVertex(graph, queue, location, s_current, k_m):
    if location != graph.goal:
        # Find the minimum rhs value from the successors to the current node
        min_rhs = min([graph.graph[succ].g + graph.graph[location].succ[succ] for succ in graph.graph[location].succ])
        graph.graph[location].rhs = min_rhs
    # Create a list of all queue items for the location
    location_in_queue = [item for item in queue if item[2:] == location]
    if location_in_queue != []:
        if len(location_in_queue) > 1:
            raise ValueError(f'More than one location: {location} in queue')
        queue.remove(location_in_queue[0])
    if graph.graph[location].g != graph.graph[location].rhs:
        heapq.heappush(queue, calculateKey(graph, location, s_current, k_m) + (location))

def computeShortestPath(graph, queue, s_start, k_m):
    while (topKey(queue) < calculateKey(graph, s_start, s_start, k_m)) or (graph.graph[s_start].rhs != graph.graph[s_start].g):
        k_old = topKey(queue) # (k_1, k_2)
        u = tuple(heapq.heappop(queue)[2:]) # location of the highest priority node
        if k_old < calculateKey(graph, u, s_start, k_m):
            heapq.heappush(queue, calculateKey(graph, u, s_start, k_m) + (u))
        elif graph.graph[u].g > graph.graph[u].rhs:
            graph.graph[u].g = graph.graph[u].rhs
            for pred in graph.graph[u].pred:
                updateVertex(graph, queue, pred, s_start, k_m)
        else:
            graph.graph[u].g = inf
            updateVertex(graph, queue, u, s_start, k_m)
            for pred in graph.graph[u].pred:
                updateVertex(graph, queue, pred, s_start, k_m)

def nextInShortestPath(graph, s_current):
    if graph.graph[s_current].g == inf:
            raise Exception('No Path Found')
    min_cost = inf
    # Search through each potential successor to the node for the optimnal path forward
    for succ in graph.graph[s_current].succ:
        child_cost = graph.graph[s_current].succ[succ] + graph.graph[succ].g
        # If the current successor has the lowest seen path cost, update the optimal successor
        if child_cost < min_cost:
            min_cost = child_cost
            s_next = succ
    # Return the optimal successor
    return s_next

# This process could scan all cells after each movement, however this is inefficient, as faraway cells will not
# effect the planned path of the UAV, therefore, to optimise the algorithm, only cells within a limited distance of the UAV are checked.
def scanForObstacles(graph, queue, s_current, k_m, view_distance, obstacle_location):
    # If the flight environment is known to be static, the UAV should not check for new obstacles
    # as an optimisation method. This can be achieved by setting the view distance as either 0 or -1.
    if view_distance < 1:
        pass
    # Preload the states to be checked with the successors of the current position
    else:
        cells_to_update = {succ: graph.cells[succ] for succ in graph.graph[s_current].succ}
        range_viewed = 1
        # Successively add further successors from the current cell, up to the n-th order
        while range_viewed < view_distance:
            new_cells = {succ: graph.cells[succ] for cell in cells_to_update for succ in graph.graph[cell].succ}
            range_viewed += 1
            cells_to_update = new_cells
        for cell in cells_to_update:

            # If their are multiple UAVs in the mission
            if obstacle_location:
                # If the updated cell does contain an obstacle
                if (cell[0] >= obstacle_location[0] - 1 and cell[0] <= obstacle_location[0] + 1) and \
                   (cell[1] >= obstacle_location[1] - 1 and cell[1] <= obstacle_location[1] + 1) and \
                   (cell[2] >= obstacle_location[2] - 1 and cell[2] <= obstacle_location[2] + 1):
                    # Update the cell state
                    graph.cells[cell] = 1
                    for succ in graph.graph[cell].succ:
                        # Update the cost of moving between the obstacle cell and its successors, and vice versa
                        graph.graph[cell].succ[succ] = inf
                        graph.graph[succ].succ[cell] = inf
                        # Update the queue of potential successors with this new cost
                        updateVertex(graph, queue, cell, s_current, k_m)
                else:
                    # Update the cell state
                    graph.cells[cell] = 0
                    for succ in graph.graph[cell].succ:
                        # Update the cost of moving between the obstacle cell and its successors, and vice versa
                        graph.graph[cell].succ[succ] = 1
                        graph.graph[succ].succ[cell] = 1
                        # Update the queue of potential successors with this new cost
                        updateVertex(graph, queue, cell, s_current, k_m)               


def moveAndRescan(graph, queue, s_current, k_m, view_distance, obstacle_location):
    # Determine the optimal successor
    s_next = nextInShortestPath(graph, s_current)

    # If the cell chosen as the successor is known to contain a newer discovered obstacle
    if graph.cells[s_next] == 1:
        # Reset the next cell to the current cell (Wait)
        s_next = s_current

    # Check the surrounding cells for obstacles
    scanForObstacles(graph, queue, s_current, k_m, view_distance, obstacle_location)
    # Update the value of k_m
    k_m += hueristicFromS(s_current, s_next)
    # Compute the new shortest path from the current cell
    computeShortestPath(graph, queue, s_current, k_m)

    return s_next, k_m

def initDStarLite(graph, s_start, s_goal):
    queue = [] # contains keys and location data: (k_1, k_2, location)
    heapq.heapify(queue) # Turn the queue into a heap
    k_m = 0 # 
    graph.graph[s_goal].rhs = 0
    heapq.heappush(queue, calculateKey(graph, s_goal, s_start, k_m) + (s_goal))

    return graph, queue, k_m