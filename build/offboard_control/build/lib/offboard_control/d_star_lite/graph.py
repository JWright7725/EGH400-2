from cmath import inf

class GridNode:
    def __init__(self, location):
        # location of the node within the grid
        self.location = location
        # dictionary of the nodes predecessors
        # key = location of the neighbour
        # value = travel cost to the neighbour
        self.pred = {}
        # dictionary of the nodes successors
        # key = location of the neighbour
        # value = travel cost to the neighbour
        self.succ = {}
        # g approximation
        self.g = inf
        # rhs value
        self.rhs = inf

    # Define how Node objects should be displayed when printed
    def __str__(self):
        return f'Node: {self.location} g: {self.g} rhs: {self.rhs}'
    
    def __repr__(self):
        return self.__str__()

class Grid:
    def __init__(self, dimensions):
        # Create a dictionary of all nodes within the graph
        self.graph = {}

        # Store the dimensions of the grid in a tuple (x, y, z)
        self.dimensions = dimensions

        # Create an element for each cell in the grid denoting its current status:
        # 0: Empty (Non-Obstacle)
        # 1: Filled (Obstacle)
        self.cells = {(ii, jj, kk): 0 for ii in range(dimensions[0]) for jj in range(dimensions[1]) for kk in range(dimensions[2])}
        # print(self.cells)
        self.generateGraph()

    # Define how Grid objects should be displayed when printed
    def __str__(self):
        msg = 'Graph:'
        for i in self.graph:
            msg += '\n  node: ' + str(i) + ' g: ' + \
                str(self.graph[i].g) + ' rhs: ' + str(self.graph[i].rhs) + \
                ' neighbors: ' + str(self.graph[i].pred)
        return msg
    
    def __repr__(self):
        return self.__str__()
    

    def setStart(self, location):
        # If the desired start location exists within the graph
        if self.graph[location]:
            self.start = location
        else:
            raise ValueError('Start Location not valid')
    
    def setGoal(self, location):
        # If the desired goal location exists within the graph
        if self.graph[location]:
            self.goal = location
        else:
            raise ValueError('Goal Location not valid')


    # UPDATE THIS FUNCTION TO ACCEPT KNOWN OBSTACLES AT THE BEGINNING OF THE SEARCH
    def generateGraph(self):
        for z in range(self.dimensions[2]):
            for y in range(self.dimensions[1]):
                for x in range(self.dimensions[0]):
                    # Create a node object at each cell
                    node = GridNode((x, y, z))

                    # Populate the neighbours of the node object
                    for zz in range(z-1, z+2):
                        for yy in range(y-1, y+2):
                            for xx in range(x-1, x+2):
                                # Do not add a cell as its own neighbour
                                if (xx, yy, zz) != (x, y, z):
                                    # Check if the neighbouring cell is within the bounds of the graph
                                    if xx >= 0 and xx < self.dimensions[0] and yy >= 0 and yy < self.dimensions[1] and zz >= 0 and zz < self.dimensions[2]:
                                        # Add the distance to the neighbour as the value to the dictionary entry for the neighbour
                                        node.pred[(xx, yy, zz)] = 1
                                        node.succ[(xx, yy, zz)] = 1
                    # Add the node to the graph
                    self.graph[(x, y, z)] = node




def addNodeToGraph(graph, location, neighbours):
    node = Node(location)
    for neighbour in neighbours:
        node.pred[neighbour] = 1
        node.succ[neighbour] = 1
    graph[location] = node
    return graph
