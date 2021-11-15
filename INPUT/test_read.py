def read_graph():
    with open('input.txt') as f:
        lines = f.readlines()

    size = int(lines.pop(0)) 
    goal = int(lines.pop(-1))
    graph = [0]*(size*size)

    for i in range(size*size):
        data = lines[i].split(' ')
        data = [int(x) for x in data]
        graph[i]= data

    return graph
