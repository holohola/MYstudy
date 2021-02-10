import time
import numpy as np
import sys
import copy
sys.path.insert(0, "aipython")
from aipython.searchGeneric import *
from aipython.searchProblem import *
from aipython.cspProblem import *
from aipython.cspSearch import *

goal = [[1, 2, 3, 4],
        [5, 6, 7, 8],
        [9, 10, 11, 12],
        [13, 14, 15, 0]]


class GameFifteenProblem(Search_problem):

    def __init__(self, start, goal):
        self.start_ = start
        self.goal_ = goal
        self.g_ = np.copy(self.goal_)

    def start_node(self):
        """Returns the start node"""
        return self.start_

    def is_goal(self, node):
        """Returns True if the node is the goal, otherwise False"""
        try:
            for i in range(len(self.goal_)):
                for j in range(len(self.goal_[i])):
                    if (self.goal_[i][j] != node[i][j]):
                        return False
        except:
            return False

        return True

    def neighbors(self, node):
        """Returns a list of the arcs for the neighbors of node, for example:
        return [Arc(node, to_neighbor_node1, cost1), Arc(node, to_neighbor_node2, cost2)]"""
        ret = []

        # find the empty tile
        loc = np.argwhere(np.copy(node) == 0)
        x, y = loc[0][0], loc[0][1]

        if x > 0:
            nb = copy.deepcopy(node)
            nb[x][y] = nb[x - 1][y]
            nb[x - 1][y] = 0
            ret.append(Arc(copy.deepcopy(node), nb, 1))

        if x < 3:
            nb = copy.deepcopy(node)
            nb[x][y] = nb[x + 1][y]
            nb[x + 1][y] = 0
            ret.append(Arc(copy.deepcopy(node), nb, 1))

        if y > 0:
            nb = copy.deepcopy(node)
            nb[x][y] = nb[x][y - 1]
            nb[x][y - 1] = 0
            ret.append(Arc(copy.deepcopy(node), nb, 1))

        if y < 3:
            nb = copy.deepcopy(node)
            nb[x][y] = nb[x][y + 1]
            nb[x][y + 1] = 0
            ret.append(Arc(copy.deepcopy(node), nb, 1))

        return ret

    def heuristic(self, node):
        """Returns the heuristic value of the node
        based on the Manhattan distance"""
        n = np.copy(node)

        dist = 0

        for i in range(1, 16):
            loc = np.argwhere(self.g_ == i)
            gx, gy = loc[0][0], loc[0][1]

            loc = np.argwhere(n == i)
            nx, ny = loc[0][0], loc[0][1]

            dist += abs(gx - nx) + abs(gy - ny)

        return dist

start = [[1, 2, 3, 4],
         [9, 5, 6, 7],
         [10, 11, 8, 0],
         [13, 14, 15, 12]]

puzzle = GameFifteenProblem(start, goal)
searcher = AStarSearcher(puzzle)
solution = searcher.search()
print('Cost: ',  solution.cost)


class GameFifteenProblemEuclidean(GameFifteenProblem):
    def __init__(self, start, goal):
        (super().__init__(start, goal))

    def heuristic(self, node):
        """Returns the heuristic value of the node
        based on the Euclidean distance"""
        g = np.copy(self.goal_)
        n = np.copy(node)

        dist = 0

        for i in range(1, 16):
            loc = np.argwhere(g == i)
            gx, gy = loc[0][0], loc[0][1]

            loc = np.argwhere(n == i)
            nx, ny = loc[0][0], loc[0][1]

            dist += np.sqrt((gx - nx) ** 2 + (gy - ny) ** 2)

        return dist


class GameFifteenProblemInversions(GameFifteenProblem):
    def __init__(self, start, goal):
        (super().__init__(start, goal))

    def heuristic(self, node):
        """Returns the heuristic value of the node
        based on the sum of the inversion number of a permutation"""
        n = np.copy(node).flatten()

        dist = 0

        for j in range(0, 16):
            if n[j] == 0:
                continue
            for k in range(j + 1, 16):
                if n[k] != 0 and n[j] > n[k]:
                    dist += 1

        return dist

def test_h(puzzle, title):
    searcher = AStarSearcher(puzzle)
    solution = searcher.search()
    print("[{}] h-value: {}".format(title, puzzle.heuristic(puzzle.start_node())))
    print("[{}] cost: {}".format(title, solution.cost))

# optimal path cost: 14
start14 = [[1, 2, 8, 3],
           [5, 6, 7, 4],
           [9, 15, 14, 11],
           [13, 10, 12, 0]]

test_h(GameFifteenProblem(start14, goal), "start14 Manhattan")
test_h(GameFifteenProblemEuclidean(start14, goal), "start14 Euclidean")
test_h(GameFifteenProblemInversions(start14, goal), "start14 Inversions")

# optimal path cost: 17
start17 = [[1, 3, 6, 4],
           [5, 2, 8, 14],
           [9, 15, 7, 0],
           [13, 10, 12, 11]]

test_h(GameFifteenProblem(start17, goal), "start17 Manhattan")
test_h(GameFifteenProblemEuclidean(start17, goal), "start17 Euclidean")
test_h(GameFifteenProblemInversions(start17, goal), "start17 Inversions")

# optimal path cost: 23
start23 = [[1, 3, 6, 4],
           [5, 8, 15, 14],
           [9, 2, 7, 0],
           [13, 10, 12, 11]]

test_h(GameFifteenProblem(start23, goal), "start23 Manhattan")
test_h(GameFifteenProblemEuclidean(start23, goal), "start23 Euclidean")
test_h(GameFifteenProblemInversions(start23, goal), "start23 Inversions")

print('task1 done')