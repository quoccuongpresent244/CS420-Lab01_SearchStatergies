import queue

def getPath(path, begin ,end):
    result = [end]
    current = end
    while current != begin:
        current = path[current]
        result.insert(0, current)
    return result

def ucs(graph, initial, goal):
    path = [0]* len(graph)
    frontier = queue.PriorityQueue()
    frontier_list = []

    expanded_state = [] 
    frontier.put((0, initial))
    frontier_list.append(initial)

    while len(frontier_list) > 0 :
        current_path_cost, current_path = frontier.get()

        frontier_list.remove(current_path)
        expanded_state.append(current_path)

        if (current_path == goal): 
            return getPath(path, initial, goal), expanded_state, len(expanded_state)



        for neighbour in graph[current_path]:
            if neighbour not in expanded_state and neighbour not in frontier_list: 
                frontier.put((current_path_cost+1, neighbour))
                frontier_list.append(neighbour)
                path[neighbour] = current_path
            elif neighbour in frontier_list:
                for node in frontier.queue:
                    if node[1] == neighbour and node[0] > current_path_cost + 1:
                        frontier.queue[frontier.queue.index(node)] = (current_path_cost+1, neighbour)

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

    expanded = []
    frontier.put((get_manhattan_heuristic(initial, goal), initial))
    frontier_list.append(initial)

    while frontier.empty: 
        current = frontier.get()

        frontier_list.remove(current[1])

        if (current[1] == goal):
            return getPath(path, initial, goal), expanded, len(expanded)

        expanded.append(current[1])
        for neighbour in graph[current[1]]: 
            heuristic = get_manhattan_heuristic(neighbour, goal)
            if neighbour not in expanded and neighbour not in frontier_list:
                frontier.put((heuristic, neighbour))
                frontier_list.append(neighbour)
                path[neighbour] = current[1]
            elif neighbour in frontier_list:
                for node in frontier.queue: 
                    if node[1] == neighbour and node[0] > heuristic:
                        frontier.queue[frontier.queue.index(node)] = (heuristic, neighbour)

    return None, None

def Astar(graph, initial, goal): 
    path = [0] * len(graph)
    frontier = queue.PriorityQueue()
    frontier_list = []

    expanded_state = []
    frontier.put((0, initial))
    frontier_list.append(initial)

    while frontier.empty:
        current = frontier.get() 
        frontier_list.remove(current[1])

        if current[1] == goal: 
            return getPath(path, initial, goal), expanded_state, len(expanded_state)

        expanded_state.append(current[1])

        for neighbour in graph[current[1]]:
            path_cost = current[0] - get_manhattan_heuristic(current[1], goal) + 1
            path_cost_heuristic = path_cost + get_manhattan_heuristic(neighbour, goal)

            if neighbour not in expanded_state and neighbour not in frontier_list: 
                frontier.put((path_cost_heuristic, neighbour))
                frontier_list.append(neighbour)
                path[neighbour] = current[1]
            elif neighbour in frontier_list: 
                for node in frontier.queue: 
                    if node[1] == neighbour and node[0] > path_cost_heuristic: 
                        frontier.queue[frontier.queue.index(node)] = (path_cost_heuristic, neighbour)

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
        graph[i]= data

    return graph

graph = read_graph()
print(ucs(graph, 0, 61))