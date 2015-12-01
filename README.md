# intercity-city-route-finder
Finds  good driving directions between pairs of cities given by the user

The dataset is set of major highway segments of the United States(and parts of southern Canada and northern Mexico), including highway names, distances, and speed limits. 
you can visualize this as a graph with nodes as towns and highway segments as edges. The data also has cities and towns with corresponding latitude-longitude positions. 
The program runs on commandline like this:
python route.py [start-city] [end-city] [routing-option] [routing-algorithm]
where:
 start-city and end-city are the cities we need a route between.
 routing-option is one of:
 segments finds a route with the fewest number of turns (i.e. edges of the graph)
 distance finds a route with the shortest total distance
 time finds the fastest route, for a car that always travels at the speed limit
 routing-algorithm is one of:
 bfs uses breadth-first search
 dfs uses depth-first search
 astar uses A* search, with a suitable heuristic function
