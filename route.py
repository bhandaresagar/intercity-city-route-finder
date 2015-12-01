__author__ = 'sagar, chitesh'
'''
1. Analysis of the results-

We have used Adjency matrix to create the graph and priority queue for astar.

On running the program for set of source and destination.

BFS --
Regardless of the routing option, BFS - finds the path (similar for all routing options) to the destination in a way data is structured, i.e. alphabetically in case of cities first, and for junctions, the way they appear in road-segment.txt file.
If at a level the destination is found, it will exit and give the path. Path obtained as preference through cities as that's the way data is structured.

DFS --
Regardless of the routing option, DFS also finds the same path (similar for all routing options) to the destination such that the alphabetically connecting city is chosen and expanded till it finds the path. It goes through a large number of segments to reach the destination.

A*  --

Routing option : Time --
It finds the least time taken to reach the destination. There's no data maintained for visited nodes, as we find lesser timed path to an already explored node. Works in all cases tested.

Routing option : Distance --
It finds the least distance taken to reach the destination. There's also no data maintained for visited nodes, as we find shorted path (in terms of distance) to an already explored node. Works well in all cases tested

Routing option : Segment --
It finds lesser number of segments taken to reach the destination. Here, we maintain data for visited nodes, as it sometimes tries to find through all possible closest nodes (one hop of segment). It always gave lesser number of segements compared to other time and distance, routing option.
The route given sometimes has higher distance and time compared to routes with same number of segments, as visited nodes are not explored again, there sometimes turns out to be better path to reach the destination.

2. Which search algorithm for which routing option?
To find the route with least number of segments, BFS algorithm works on par with astar with "segement" routing option. Computation time is relatively more in BFS, but relatively lesser space compared to astar to find the route.

To find the route with least time and distance, Astar algorithm works the best. But in rare cases when routes happen to be best alphabeticaly placed, then BFS or DFS might give good results. But it is rare.
For some cases, even BFS gave decent routes to the destination, but not the best as received from astar

3.  Algorithm is fastest in terms of the amount of computation time required by your program, and by how much, according to your experiments?

To populate the data into the matrix, it takes 1.43 seconds.

Astar for routing option as segment has the best results, in computational time.
For,
Source: Columbus,_Ohio
Destination: Chicago,_Illinois

Segments:
astar with segment computed in 0.16 seconds and bfs found route with same number of segments in 0.38 seconds, Route with better distance and time were found through astar (segment).
Number of segments found by both - 19.

Time:
astar with time computed in 0.33 seconds and is the best available result.
Best time: 6.15 hrs and distance 301 miles

Distance:
astar with distance computed in 0.33 seconds and is the best available result.
Best distance: 296 miles and time 6.43 hrs

DFS has found a route with number of Segments as 684, distance 12038 miles and time of 244.95 hrs in 0.55 seconds.

4. Heuristic function

While parsing the input file and populating the data, we have computed the following..

totalDistance  - Distance of all valid road segments,

totalSegments - total number of valid road segments,

totalTime - sum of individual time taken to travel at maximum speed limit on the road segment with the distance for the road segement

averageSpeed - totalDistance/totalTime

averageSegmentDistance -  totalSegment/totalDistance (This shows segments per mile)

Base heuristic - "As crow flies" distance/greatest-circle distances between two points on a sphere from their longitudes and latitudes using "haversine formula"

As junctions do not have a latitude and longitude (not provided!), we try to find the next level of connected city to the junction or junction to junction to city and take the lat long of the city to calculate the distance to goal node.

For astar,

routing option:

distance -- It uses base heuristic as heurisitc cost, as both are in miles

segments -- It uses (base heuristic cost * averageSegmentDistance [given as segments per miles]), as it will give a heuristic value for number of segements for the base heurisitc distance and gives an approximate

time -- It uses (base heuristic cost/ average speed [given in miles per hour]), as it will give a heurisitc value of time taken to travel the base heuristic cost distance  in miles

The heuristic used gives good results on teh test cases tested and compared with values of Google Maps!

To improve the heuristic, instead of taking the average for segement distance and speed, we could use the values of the on-going route real time, like distance of the routes taken, speed of those routes, segements n more..
As there could be few outliers in the data and could deviate the value of average values.

The heuristic used gives good results.

'''


import sys, math, copy, time, Queue as queue
from collections import Counter

class FringNode:
    def __init__(self,paramCost,nodeIndex,pathToFringe):
        self.paramCost = paramCost
        self.nodeIndex = nodeIndex
        self.pathToFringe = pathToFringe

    def __lt__(self, other):
        return self.paramCost < other.paramCost

# Create Graph; Nodes -> City, Junctions and Edges -> Road-segment
class GraphNode:
    def __init__(self, name, isJunction, latitude, longitude):
        self.isJunction = isJunction
        if isJunction:
            self.name = name
        else:
            self.name = name
            self.latitude = float(latitude)
            self.longitude = float(longitude)

# Create Edge; Road-segment
class GraphEdge:
    def __init__(self, distance, speedLimit, highwayName):
        self.distance = int(distance)
        self.speedLimit = int(speedLimit)
        self.highwayName = highwayName

class Route:
    def __init__(self):
        self.GraphAdjMatrix = []
        self.cityGpsDict = {}
        self.roadSegmentFile = []
        self.JunctionList = []
        self.GraphNodeList = []
        self.totalSpeed = 0
        self.totalDistance = 0
        self.totalSegment = 0
        self.totalTime = 0.0
        self.averageSpeed = 0.0
        self.averageSegmentDistance = 0
        self.startNodeIndex = 0
        self.goalNodeIndex = 0
        self.routingOption = 0
        self.routingAlgorithm = ''
        self.finalPath = ''
        self.goalNodeGPSCordinates = ()

    def fetchData(self):
        # read city-gps.txt
        cityGpsfileName = "city-gps.txt"
        cityGpsFile = open(cityGpsfileName, 'r').read().split('\n')
        for i in range(0, len(cityGpsFile) - 1):
            line = cityGpsFile[i].split(' ')
            self.cityGpsDict[line[0]] = i + 1
            # Instatiate Object of class GraphNode
            GraphNodeObj = GraphNode(name=line[0], isJunction=False, latitude=line[1], longitude=line[2])
            self.GraphNodeList.append(GraphNodeObj)
        roadSegmentFileName = "road-segments.txt"
        self.roadSegmentFile = open(roadSegmentFileName, 'r').read().split('\n')
        index = len(self.cityGpsDict) + 2
        for i in range(0, len(self.roadSegmentFile) - 1):
            line = self.roadSegmentFile[i].split(' ')
            if line[0].startswith('Jct') and line[0] not in self.JunctionList:
                self.cityGpsDict[line[0]] = index
                GraphNodeObj = GraphNode(name=line[0], isJunction=True, latitude=0, longitude=0)
                self.JunctionList.append(GraphNodeObj)
                index += + 1
            if line[1].startswith('Jct') and line[1] not in self.JunctionList:
                self.cityGpsDict[line[1]] = index
                GraphNodeObj = GraphNode(name=line[1], isJunction=True, latitude=0, longitude=0)
                self.JunctionList.append(GraphNodeObj)
                index += + 1

    def populateData(self):

        #create Adjency Matrix for Graph
        '''
        |X          City1-Node_obj  City2-Node_obj   City3-Node_obj . . . . Junct1-Node_Obj   Junct2-Node_Obj ...
        |City1_obj    None           RS_C1-C2_Obj         RS_C1-C3_Obj  . . . RS_C1-J1_Obj      None
        |City2_obj  RS_C1-C2_Obj        None              RS_C2-C3_Obj  ....    None            RS_C2-J2_Obj
        |City3_obj
        |.
        |.
        |Junct1-Node_Obj None        RS_J1-C2_Obj                               None            RS_J1-J2_Obj
        |Junct2-Node_Obj
        '''

        self.GraphNodeList = self.GraphNodeList + self.JunctionList
        emptyList = [None for i in range(len(self.GraphNodeList))]
        for node in self.GraphNodeList:
            GraphAdjMatrix_row = copy.copy(emptyList)
            GraphAdjMatrix_row.insert(0, node)
            self.GraphAdjMatrix.append(GraphAdjMatrix_row)
        self.GraphNodeList.insert(0, None)
        self.GraphAdjMatrix.insert(0, self.GraphNodeList)
        for i in range(0, len(self.roadSegmentFile) - 1):
            line = self.roadSegmentFile[i].split(' ')
            try:
                rowIndex = self.cityGpsDict[line[0]]
                columnIndex = self.cityGpsDict[line[1]]
                if int(line[2]) == 0 or int(line[3]) == 0:
                    continue
                self.totalSpeed += int(line[3])
                self.totalDistance += int(line[2])
                time = round(int(line[2]) / float(int(line[3])),2)
                self.totalTime += time
                self.totalSegment += 1
                GraphEdgeObj = GraphEdge(line[2], line[3], line[4])
                self.GraphAdjMatrix[rowIndex][columnIndex] = self.GraphAdjMatrix[columnIndex][rowIndex] = GraphEdgeObj
            except:
                continue

        self.averageSpeed = round(self.totalDistance/self.totalTime,2)
        self.averageSegmentDistance = round(self.totalSegment / float(self.totalDistance), 2)

    def fetchArgs(self):
        args = sys.argv
        startNode = args[1]
        print "Start Node:",startNode
        goalNode = args[2]
        print "Goal Node:",goalNode
        routingOption = args[3]
        print "Routing Option:",routingOption
        if routingOption == "segment":
            self.routingOption = 1  # segment
        elif routingOption == "distance":
            self.routingOption = 2  # distance
        else:
            self.routingOption = 3  # time
        self.routingAlgorithm = args[4]
        print "Routing Algorithm:",self.routingAlgorithm
        self.startNodeIndex = self.cityGpsDict[startNode]
        self.goalNodeIndex = self.cityGpsDict[goalNode]
        #startFetchTime = t1.time()

    def solve(self):
        if self.routingAlgorithm == "bfs":
            self.bfs()
        elif self.routingAlgorithm == "dfs":
            self.dfs()
        elif self.routingAlgorithm == "astar":
            self.astar()
        elif self.routingAlgorithm == "uniformcost":
            self.uniform_cost_search()

    def calculateDistance(self,gpsCordinates1, gpsCordinates2):
        #conversion from degrees to radians
        lat1_radians = gpsCordinates1[0] * (math.pi / 180)
        long1_radians = gpsCordinates1[1] * (math.pi / 180)

        lat2_radians = gpsCordinates2[0] * (math.pi / 180)
        long2_radians = gpsCordinates2[1] * (math.pi / 180)

        radius_of_earth = 3959 #in miles; source: Google

        # Haversine Formula; source: https://en.wikipedia.org/wiki/Haversine_formula#The_haversine_formula
        haversine = pow(math.sin((lat2_radians - lat1_radians) / 2), 2) + math.cos(lat1_radians) * math.cos(lat2_radians) * pow(
            math.sin((long2_radians - long1_radians) / 2), 2)
        distance = 2 * radius_of_earth * math.asin(math.sqrt(haversine))
        return round(distance,2)

    def dfs(self):
        fringe = []
        visitedNodes = []
        pathForFringe = []
        fringe.append(self.startNodeIndex)
        pathForFringe.append(
            ([self.GraphAdjMatrix[self.startNodeIndex][0].name], 0, 0))  # First Param -> Entire Path; Second Param -> Distance; Third Param -> Time
        # Search from adjencyMatrix, all outgoing nodes
        while self.goalNodeIndex not in fringe:
            if fringe[0] not in visitedNodes:
                # print GraphAdjMatrix[fringe[0]][0].name
                visitedNodes.append(fringe[0])
                childrenList = self.GraphAdjMatrix[fringe[0]]
                childrenListIndex = []
                #childrenListParam = []
                for nodeIndex in range(1, len(childrenList)):
                    if childrenList[nodeIndex] is not None and nodeIndex is not fringe[0]:
                        childrenListIndex.append(nodeIndex)
                fringe.pop(0)
                parentPath = pathForFringe[0][0]
                parentPathDistance = pathForFringe[0][1]
                parentPathTime = round(pathForFringe[0][2], 2)
                pathForFringe.pop(0)
                for childIndex in range(0, len(childrenListIndex)):
                    pathForFringe.insert(childIndex,
                                         (parentPath + [self.GraphAdjMatrix[childrenListIndex[childIndex]][0].name],
                                          parentPathDistance +
                                          int(childrenList[childrenListIndex[childIndex]].distance), parentPathTime +
                                          round(childrenList[childrenListIndex[childIndex]].distance / float(
                                              childrenList[childrenListIndex[childIndex]].speedLimit), 2)))
                fringe = childrenListIndex + fringe
            else:
                fringe.pop(0)
                pathForFringe.pop(0)
        self.finalPath = pathForFringe[fringe.index(self.goalNodeIndex)]
        # DFS; that's a wrap!!

    def bfs(self):
        fringe = []
        visitedNodes = []
        pathForFringe = []
        fringe.append(self.startNodeIndex)
        pathForFringe.append(
            ([self.GraphAdjMatrix[self.startNodeIndex][0].name], 0, 0))  # First Param -> Entire Path; Second Param -> Distance; Third Param -> Time
        # Search from adjencyMatrix, all outgoing nodes
        while self.goalNodeIndex not in fringe:
            if fringe[0] not in visitedNodes:
                # print GraphAdjMatrix[fringe[0]][0].name
                visitedNodes.append(fringe[0])
                childrenList = self.GraphAdjMatrix[fringe[0]]
                childrenListIndex = []
                for nodeIndex in range(1, len(childrenList)):
                    if childrenList[nodeIndex] is not None and nodeIndex is not fringe[0]:
                        childrenListIndex.append(nodeIndex)
                fringe.pop(0)
                parentPath = pathForFringe[0][0]
                parentPathDistance = pathForFringe[0][1]
                parentPathTime = round(pathForFringe[0][2], 2)
                pathForFringe.pop(0)
                for childIndex in range(0, len(childrenListIndex)):
                    pathForFringe.append(
                        (parentPath + [self.GraphAdjMatrix[childrenListIndex[childIndex]][0].name],
                         parentPathDistance +
                         int(childrenList[childrenListIndex[childIndex]].distance), parentPathTime +
                         round(childrenList[childrenListIndex[childIndex]].distance / float(
                             childrenList[childrenListIndex[childIndex]].speedLimit), 2)))
                fringe = fringe + childrenListIndex
            else:
                fringe.pop(0)
                pathForFringe.pop(0)
        self.finalPath = pathForFringe[fringe.index(self.goalNodeIndex)]
        # BFS; that's a wrap!!

    def uniform_cost_search(self):
        fringe = []
        visitedNodes = []
        pathForFringe = []
        fringe.append(self.startNodeIndex)
        pathForFringe.append(
            ([self.GraphAdjMatrix[self.startNodeIndex][0].name], 0, 0))  # First Param -> Entire Path; Second Param -> Distance; Third Param -> Time
        # Search from adjencyMatrix, all outgoing nodes
        while self.goalNodeIndex != fringe[0]:
            if fringe[0] not in visitedNodes:
                # print GraphAdjMatrix[fringe[0]][0].name
                visitedNodes.append(fringe[0])
                childrenList = self.GraphAdjMatrix[fringe[0]]
                childrenListIndex = []
                childrenListParam = []
                for nodeIndex in range(1, len(childrenList)):
                    if childrenList[nodeIndex] is not None and nodeIndex is not fringe[0]:
                        childrenListIndex.append(nodeIndex)
                        if self.routingOption == 1:
                            childrenListParam.append(1)
                        elif self.routingOption == 2:
                            childrenListParam.append(childrenList[nodeIndex].distance)
                        elif self.routingOption == 3:
                            distance = childrenList[nodeIndex].distance
                            speedLimit = childrenList[nodeIndex].speedLimit
                            time = round(distance / float(speedLimit), 2)
                            childrenListParam.append(time)
                fringe.pop(0)
                sortedChildrenList = sorted(range(len(childrenListParam)), key=lambda k: childrenListParam[k])
                childrenListIndex = [childrenListIndex[i] for i in sortedChildrenList]
                parentPath = pathForFringe[0][0]
                parentPathDistance = pathForFringe[0][1]
                parentPathTime = round(pathForFringe[0][2], 2)
                pathForFringe.pop(0)
                for childIndex in range(0, len(childrenListIndex)):
                    pathForFringe.append(
                        (parentPath + [self.GraphAdjMatrix[childrenListIndex[childIndex]][0].name],
                         parentPathDistance +
                         int(childrenList[childrenListIndex[childIndex]].distance), parentPathTime +
                         round(childrenList[childrenListIndex[childIndex]].distance / float(
                             childrenList[childrenListIndex[childIndex]].speedLimit), 2)))
                fringe = fringe + childrenListIndex
            else:
                fringe.pop(0)
                pathForFringe.pop(0)
        self.finalPath = pathForFringe[fringe.index(self.goalNodeIndex)]

    def getChildren(self,parentNode, visitedNodes, visitedChildrenNodes, childrenListIndex, childrenListParam, childrenListAttr,
                    isJunction, junctionCost):
        childrenList = self.GraphAdjMatrix[parentNode]
        for childIndex in range(1, len(childrenList)):
            # check for none-Non road segments
            if childrenList[childIndex] is not None and visitedNodes[childIndex] == 0 and visitedChildrenNodes[childIndex] == 0:
                #visitedChildrenNodes.append(childIndex)
                visitedChildrenNodes[childIndex] = 1
                name = self.GraphAdjMatrix[childIndex][0].name
                # check if junction
                if self.GraphAdjMatrix[childIndex][0].name.startswith('Jct'):
                    distance = self.GraphAdjMatrix[parentNode][childIndex].distance
                    speedLimit = self.GraphAdjMatrix[parentNode][childIndex].speedLimit
                    time = round(distance / float(speedLimit), 2)
                    if isJunction:
                        junctionCost[childIndex] = [junctionCost[parentNode][0] + [name],
                                                    junctionCost[parentNode][1] + distance,
                                                    junctionCost[parentNode][2] + time]
                    else:
                        junctionCost[childIndex] = [[name], distance, time]
                    self.getChildren(childIndex, visitedNodes, visitedChildrenNodes, childrenListIndex, childrenListParam,
                                childrenListAttr, True, junctionCost)
                else:
                    childrenListIndex.append(childIndex)
                    visitingNodeGPSCordinates = (
                        self.GraphAdjMatrix[childIndex][0].latitude, self.GraphAdjMatrix[childIndex][0].longitude)
                    heuristicCost = self.calculateDistance(self.goalNodeGPSCordinates, visitingNodeGPSCordinates)
                    distance = childrenList[childIndex].distance
                    speedLimit = childrenList[childIndex].speedLimit
                    time = round(distance / float(speedLimit), 2)
                    segmentCount = 1
                    if isJunction:
                        distance += junctionCost[parentNode][1]
                        time += junctionCost[parentNode][2]
                        segmentCount += len(junctionCost[parentNode][0])
                    if self.routingOption == 1:
                        childrenListParam.append(segmentCount + round(heuristicCost * self.averageSegmentDistance, 2))
                    elif self.routingOption == 2:
                        childrenListParam.append(distance + heuristicCost)
                    elif self.routingOption == 3:
                        childrenListParam.append(time + round(heuristicCost / float(self.averageSpeed), 2))
                    if isJunction:
                        childrenListAttr.append((junctionCost[parentNode][0] + [name], distance, time))
                    else:
                        childrenListAttr.append(([name], distance, time))

    def astar(self):
        self.goalNodeGPSCordinates = (self.GraphAdjMatrix[self.goalNodeIndex][0].latitude, self.GraphAdjMatrix[self.goalNodeIndex][0].longitude)
        fringePQ = queue.PriorityQueue()
        #visitedNodes = Counter()
        visitedNodes = Counter()
        pathForFringe = ([self.GraphAdjMatrix[self.startNodeIndex][0].name],  # First Param -> Entire Path;
             0,  # Second Param -> Distance;
             0,  # Third Param -> Time;
             0)  # (cost + heuristic) for Last Node in path
        startNodeGPSCordinates =  (self.GraphAdjMatrix[self.startNodeIndex][0].latitude, self.GraphAdjMatrix[self.startNodeIndex][0].longitude)
        fringeParam = 0 + self.calculateDistance(self.goalNodeGPSCordinates,startNodeGPSCordinates)
        fringePQ.put(FringNode(fringeParam,self.startNodeIndex,pathForFringe))
        fringeTop = fringePQ.get()
        while fringeTop.nodeIndex != self.goalNodeIndex:
            if visitedNodes[fringeTop.nodeIndex] == 0:
                if self.routingOption == 1:
                    visitedNodes[fringeTop.nodeIndex] = 1
                parentNode = fringeTop.nodeIndex
                childrenListIndex = []
                childrenListParam = []
                childrenListAttr = []
                visitedChildrenNodes = Counter()
                junctionCost = {}
                # get all children from parentIndex
                self.getChildren(parentNode, visitedNodes, visitedChildrenNodes, childrenListIndex, childrenListParam,
                            childrenListAttr, False, junctionCost)
                parentPath = fringeTop.pathToFringe[0]
                parentPathDistance = fringeTop.pathToFringe[1]
                parentPathTime = fringeTop.pathToFringe[2]
                for childIndex in range(0, len(childrenListIndex)):
                    if self.routingOption == 1:
                        paramCost = len(parentPath) + childrenListParam[childIndex]
                    elif self.routingOption == 2:
                        paramCost = parentPathDistance + childrenListParam[childIndex]
                    elif self.routingOption == 3:
                        paramCost = parentPathTime + childrenListParam[childIndex]
                    paramCost = round(paramCost, 2)
                    pathForFringeNode = (parentPath + childrenListAttr[childIndex][0],
                                          parentPathDistance +
                                          childrenListAttr[childIndex][1], parentPathTime +
                                          childrenListAttr[childIndex][2])
                    fringePQ.put(FringNode(paramCost,childrenListIndex[childIndex],pathForFringeNode))
                fringeTop = fringePQ.get()
            else:
                fringeTop = fringePQ.get()
        self.finalPath = fringeTop.pathToFringe
        # A*; that's a wrap!!

    def printFinalPath(self):
        print "Number of Segments", len(self.finalPath[0])
        print self.finalPath[1], self.finalPath[2], ' '.join(self.finalPath[0])


def main():
    route = Route()

    route.fetchData()
    start = time.time()
    route.populateData()
    end = time.time() - start
    if end > 60:
        print "Populated data in",round((end / 60),2),"minutes"
    else:
        print "Populated data in",round(end,2),"seconds"
    route.fetchArgs()
    start = time.time()
    route.solve()
    end = time.time() - start
    if end > 60:
        print "Route generated in",round((end / 60),2),"minutes"
    else:
        print "Route generated in",round(end,2),"seconds"
    route.printFinalPath()

if __name__ == '__main__':
    main()