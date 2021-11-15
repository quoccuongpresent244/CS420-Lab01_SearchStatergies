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
                frontier_list(neighbour)
                path[neighbour] = current[1]
            elif neighbour in frontier_list:
                for node in frontier.queue:
                    if node[1] == neighbour and node[0] > current[0] + 1:
                        frontier.queue[frontier.queue.index(node)] = (current[0]+1, neighbour)

    return None, None

