# Initialization
import cPickle, copy


#grid = 	  [[0, 1, 0, 0, 0, 0],
#           [0, 1, 0, 0, 0, 0],
#           [0, 1, 0, 0, 0, 0],
#           [0, 1, 0, 0, 0, 0],
#           [0, 0, 0, 0, 1, 0]]

basic_grid = cPickle.load(open('/home/ahmedeltaweel/Desktop/laptop/save.p' , 'r'))
grid = copy.deepcopy(basic_grid)

init = [29, 29]
goal = [len(grid) - 5, 5]
cost = 1

rows_no = len(grid)
cols_no = len(grid[0])


delta = [ [-1, 0 ], # go up          -   previous row, same column
            [ 0, -1], # go left        -   same row, previous column
            [ 1, 0 ], # go down        -   next row, same column
            [ 0, 1 ]] # go right       -   same row, next column

moves_no = len(delta)

# heuristic
# heuristic grid Initialization
heuristics = [[1000 for row in range(cols_no)] for col in range(rows_no )]

def get_heuristics():
    heuristics[goal[0]][goal[1]] = 0
    x = goal[0]
    y = goal[1]
    open = [(x, y)]
    while open:
        next = open.pop(0)
        x = next[0]
        y = next[1]
        h_cost = heuristics[x][y]
        for i in range(moves_no):
            x2 = x + delta[i][0]
            y2 = y + delta[i][1]
            if x2 >= 0 and x2 < len(grid) and y2 >=0 and y2 < len(grid[0]):
                if heuristics[x2][y2] == 1000:
                    heuristics[x2][y2] = h_cost + 1
                    open.append((x2, y2))

get_heuristics()

delta_name = ['^', '<', 'v', '>']   # policy line # policy line # policy line # policy line

# closed grid Initialization
rows_no = len(grid)
cols_no = len(grid[0])

closed = [ [0 for row in range(cols_no)] for col in range(rows_no )]
closed[init[0]][init[1]] = 1

action = [[1000 for row in range(cols_no)] for col in range(rows_no)] # policy line # policy line

def search():
    # First Step
    x = init[0]
    y = init[1]
    g = 0

    path = [(x, y)]
    h = heuristics[x][y]     # heuristic line # heuristic line
    f = g + h               # heuristic line # heuristic line

    open = [[f, g, (x, y), path]]   # heuristic line # heuristic line

    found = False  # flag that is set when search is complet
    resign = False # flag set if we can't find elements to expand

    # Looping
    while not found and not resign:
        if len(open) == 0:                      # failing
            resign = True
            return 'fail'

        else:
            open.sort()
            open.reverse()
            next = open.pop()

            g = next[1]
            x = next[2][0]
            y = next[2][1]
            path = next[3]

            if x == goal[0] and y == goal[1]:   # Goal found
                found = True
                return path

            else:
                # Add new cells to open list
                for i in range(moves_no):
                    x2 = x + delta[i][0]
                    y2 = y + delta[i][1]
                    # check conditions (not obstacle, not closed)
                    if x2 >= 0 and x2 < len(grid) and y2 >=0 and y2 < len(grid[0]):
                        if closed[x2][y2] == 0 and grid[x2][y2] == 0:

                            g2 = g + cost
                            path2 = path + [(x2, y2)]
                            h2 = heuristics[x2][y2]      # heuristic line
                            f2 = g2 + h2                # heuristic line

                            open.append( [f2, g2, (x2, y2), path2])

                            closed[x2][y2] = 1

                            action[x2][y2] = i      # policy line # policy line # policy line

def policy():
    policy = [[' ' for row in range(len(grid[0]))] for col in range(len(grid))]
    x = goal[0]
    y = goal[1]
    policy[x][y] = '*'

    while x != init[0] or y != init[1]:
        x2 = x - delta[action[x][y]][0]
        y2 = y - delta[action[x][y]][1]
        policy[x2][y2] = delta_name[action[x][y]]
        x = x2
        y = y2

    print( 'Path direction Grid: ')
    for row in policy:
        print (row)

path = search()
print( 'path:' , len(path))
for row in path:
    print (row)

#todo add the funxtion to reduce the nubmer of points

#if path != 'fail': policy()
