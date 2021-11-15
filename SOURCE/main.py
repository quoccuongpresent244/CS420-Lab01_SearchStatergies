import queue


def getPath(path, begin, end):
    result = [end]
    current = end
    while current != begin:
        current = path[current]
        result.insert(0, current)
    return result


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


def get_manhattan_heuristic(node, goal):
    i, j = divmod(int(node), 8)
    i_goal, j_goal = divmod(int(goal), 8)
    i_delta = abs(i - i_goal)
    j_delta = abs(j - j_goal)

    manhattan_dist = i_delta + j_delta
    return manhattan_dist


def GBFS(graph, initial, goal):
    path = [0]*len(graph)
    frontier = queue.PriorityQueue()
    frontier_list = []

    explored_nodes = []
    frontier.put((get_manhattan_heuristic(initial, goal), initial))
    frontier_list.append(initial)

    while frontier.empty:
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


def Astar(graph, initial, goal):
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


def read_graph():
    with open('../INPUT/input.txt') as f:
        lines = f.readlines()

    size = int(lines.pop(0))
    goal = int(lines.pop(-1))
    graph = [0]*(size*size)

    for i in range(size*size):
        data = lines[i].split(' ')
        data = [int(x) for x in data]
        graph[i] = data

    f.close()
    return graph, goal


def output(path_returned, explored_nodes, escape_time):
    f = open('../OUTPUT/output.txt', 'w')
    f.write('Escape time: ' + str(escape_time) + ' mins\n')
    f.write('Explored nodes: ' + str(explored_nodes) + '\n')
    f.write('Path returned: ' + str(path_returned) + '\n')
    f.close()


def DFS(graph, currentNode, goal, maxDepth, curList, explored_nodes):
    curList.append(currentNode)
    explored_nodes.append(currentNode)
    
    if currentNode== goal: 
         return True
    if maxDepth<=0:
        return False
    for neighbour in graph[currentNode]:
        if neighbour not in explored_nodes:
            if DFS(graph, neighbour, goal, maxDepth-1, curList, explored_nodes):
                return True
            else:
                curList.pop()
    return False

def IDS(currentNode, goal, graph, maxDepth):
    explored_nodes_depth = []
    for i in range(maxDepth):
        curList = []
        explored_nodes = []
        
        if DFS(graph, currentNode, goal, i, curList, explored_nodes):
            explored_nodes_depth.append(explored_nodes)
            return curList, explored_nodes_depth
        else:
            explored_nodes_depth.append(explored_nodes)

graph, goal = read_graph()
path, explored_nodes_depth = IDS(0, goal ,graph ,50)
print(explored_nodes_depth)



'''graph, goal = read_graph()
path_returned, explored_nodes, escape_time = ucs(graph, 0, goal)
output(path_returned, explored_nodes, escape_time)'''




