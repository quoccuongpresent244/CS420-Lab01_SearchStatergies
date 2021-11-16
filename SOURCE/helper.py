def getPath(path, start, goal):
    result = [goal]
    current = goal
    while current != start:
        current = path[current]
        result.insert(0, current)
    return result

def get_manhattan_heuristic(node, goal):
    i, j = divmod(node, 8)
    i_goal, j_goal = divmod(goal, 8)
    i_delta = abs(i - i_goal)
    j_delta = abs(j - j_goal)

    manhattan_dist = i_delta + j_delta
    return manhattan_dist

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

def output(path_returned, explored_nodes, escape_time, list):
    f = open('../OUTPUT/'+list+".txt", 'w')
    f.write("################## " + list + " ##################\n") 
    f.write('Escape time: ' + str(escape_time) + ' mins\n')
    f.write('Explored nodes: ' + str(explored_nodes) + '\n')
    f.write('Path returned: ' + str(path_returned) + '\n\n')
    f.close()