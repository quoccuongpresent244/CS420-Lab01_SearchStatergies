from helper import getPath, get_manhattan_heuristic
import queue

############ Uniformed Cost Search ############
def ucs(graph, initial, goal):
    path = [0] * len(graph)
    frontier = queue.PriorityQueue()
    frontier_list = []

    explored_nodes = []
    frontier.put((0, initial))
    frontier_list.append(initial)

    while len(frontier_list) > 0:
        current_path_cost, current_path = frontier.get()

        frontier_list.remove(current_path)
        explored_nodes.append(current_path)

        if (current_path == goal):
            return getPath(path, initial, goal), explored_nodes, len(explored_nodes)

        for neighbour in graph[current_path]:
            if neighbour not in explored_nodes and neighbour not in frontier_list:
                frontier.put((current_path_cost+1, neighbour))
                frontier_list.append(neighbour)
                path[neighbour] = current_path
            elif neighbour in frontier_list:
                for node in frontier.queue:
                    if node[1] == neighbour and node[0] > current_path_cost + 1:
                        frontier.queue[frontier.queue.index(node)] = (
                            current_path_cost+1, neighbour)

    return None, None

############ Greedy Breadth First Search ############
def gbfs(graph, initial, goal):
    path = [0]*len(graph)
    frontier = queue.PriorityQueue()
    frontier_list = []

    explored_nodes = []
    frontier.put((get_manhattan_heuristic(initial, goal), initial))
    frontier_list.append(initial)

    while len(frontier_list) > 0:
        _, current_node = frontier.get()
        frontier_list.remove(current_node)

        if (current_node == goal):
            return getPath(path, initial, goal), explored_nodes, len(explored_nodes)

        explored_nodes.append(current_node)

        for neighbour in graph[current_node]:
            heuristic = get_manhattan_heuristic(neighbour, goal)
            if neighbour not in explored_nodes and neighbour not in frontier_list:
                frontier.put((heuristic, neighbour))
                frontier_list.append(neighbour)
                path[neighbour] = current_node
            elif neighbour in frontier_list:
                for node in frontier.queue:
                    if node[1] == neighbour and node[0] > heuristic:
                        frontier.queue[frontier.queue.index(
                            node)] = (heuristic, neighbour)

    return None, None

############ A* search ############
def astar(graph, initial, goal):
    path = [0] * len(graph)
    frontier = queue.PriorityQueue()
    frontier_list = []

    explored_nodes = []
    frontier.put((get_manhattan_heuristic(initial, goal), initial))
    frontier_list.append(initial)

    while frontier.empty:
        current_path_cost, current_node = frontier.get()
        path_cost = current_path_cost - \
            get_manhattan_heuristic(current_node, goal) + 1

        frontier_list.remove(current_node)
        explored_nodes.append(current_node)

        if current_node == goal:
            return getPath(path, initial, goal), explored_nodes, len(explored_nodes)

        for neighbour in graph[current_node]:
            path_cost_heuristic = path_cost + \
                get_manhattan_heuristic(neighbour, goal)

            if neighbour not in explored_nodes and neighbour not in frontier_list:
                frontier.put((path_cost_heuristic, neighbour))
                frontier_list.append(neighbour)
                path[neighbour] = current_node
            elif neighbour in frontier_list:
                for node in frontier.queue:
                    if node[1] == neighbour and node[0] > path_cost_heuristic:
                        frontier.queue[frontier.queue.index(node)] = (
                            path_cost_heuristic, neighbour)

    return None, None

############ Iterative Deepening Search 
def dfs(graph, currentNode, goal, maxDepth, curList, explored_nodes):
    curList.append(currentNode)
    explored_nodes.append(currentNode)
    
    if currentNode== goal: 
         return True
    if maxDepth<=0:
        return False
    for neighbour in graph[currentNode]:
        if neighbour not in explored_nodes:
            if neighbour != goal:
                if dfs(graph, neighbour, goal, maxDepth-1, curList, explored_nodes):
                    return True
                else:
                    curList.pop()
            else:
                return True
    return False

def ids(graph, currentNode, goal, maxDepth):
    explored_nodes_depth = []
    for i in range(maxDepth):
        curList = []
        explored_nodes = []
        
        if dfs(graph, currentNode, goal, i, curList, explored_nodes):
            explored_nodes_depth.append(explored_nodes)
            return curList, explored_nodes_depth
        else:
            explored_nodes_depth.append(explored_nodes)