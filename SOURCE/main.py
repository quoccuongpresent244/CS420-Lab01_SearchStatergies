from helper import *
from algo import *

graph, goal = read_graph()
name = ["Uniform-cost Search", "Iterative Deepening Search", "Greedy Breadth First Search", "A-star Search"]
func = [ucs, ids, gbfs, astar]
for i in range(len(func)):
    if i == 1:
        escape_time = 0
        path, explored_nodes = func[i](graph,0,goal,100)
        for j in range(len(explored_nodes)):
            escape_time += len(explored_nodes[j])
    else: 
        path, explored_nodes, escape_time = func[i](graph, 0, goal)
    output(path, explored_nodes, escape_time, name[i])




