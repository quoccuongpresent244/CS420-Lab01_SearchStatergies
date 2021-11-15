import queue

def getPath(graph, begin ,end):
    return None

def ucs(graph, initial, goal):
    path = [0]* len(graph)
    frontier = queue.PriorityQueue()
    frontier_list = []

    expanded_state = [] 
    frontier.put((0, initial))
    frontier_list.append(initial)

    while len(frontier) > 0 :
        current = frontier.get()

        frontier_list.remove(current[1])

        if (current[1] == goal): 
            return getPath(path, initial, goal), expanded_state

        expanded_state.append(current[1])

        for neighbour in graph[current[1]]:
            if neighbour not in expanded_state and neighbour not in frontier_list: 
                frontier.put((current[0]+1, neighbour))
                frontier_list.append(neighbour)
                path[neighbour] = current[1]
            elif neighbour in frontier_list:
                for node in frontier.queue:
                    if node[1] == neighbour and node[0] > current[0] + 1:
                        frontier.queue[frontier.queue.index(node)] = (current[0]+1, neighbour)

    return None, None

def GBFS(graph, initial, goal):
    path = [0]*len(graph)
    frontier = queue.PriorityQueue()
    frontier_list = []

    expanded = []
    frontier.put((get_manhattan_heuristic(initial, goal), initial))
    frontier_list.append(initial)

    while len(frontier_list) > 0: 
        current = frontier.get()

        frontier_list.remove(current[1])

        if (current[1] == goal):
            return getPath(path, initial, goal), expanded

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



def get_manhattan_heuristic(node, goal):    
    i, j = divmod(int(node), 8)    
    i_goal, j_goal = divmod(int(goal), 8)    
    i_delta = abs(i - i_goal)    
    j_delta = abs(j - j_goal)    
    
    manhattan_dist = i_delta + j_delta    
    return manhattan_dist  