import time
import numpy as np
from aipython.searchGeneric import *
from aipython.searchProblem import *
from aipython.cspProblem import *

import time
import numpy as np
import sys
import copy
sys.path.insert(0, "aipython")
from aipython.searchGeneric import *
from aipython.searchProblem import *
from aipython.cspProblem import *
from aipython.cspSearch import *
'''
Put your class and function here
'''


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


class BreadthFirstSearcher(Searcher):

    def __init__(self, problem):
        super().__init__(problem)

    """ Initializes the forontier """

    def initialize_frontier(self):
        self.frontier_ = []

    """ Returns True if there are no more nodes to expand """

    def empty_frontier(self):
        return len(self.frontier_) == 0

    """ Adds the path to the forontier """

    def add_to_frontier(self, path):
        self.frontier_.append(path)

    """returns (next) path from the problem's start node
        to a goal node. """

    def search(self):
        """returns (next) path from the problem's start node
        to a goal node.
        Returns None if no path exists.
        """
        visited = set()
        start_t = time.time()

        while not self.empty_frontier():
            if len(self.frontier_) >= 1000000:
                print("Mem")
                return None

            if time.time() - start_t > 300:
                print("Time")
                return None

            path = self.frontier_.pop(0)

            self.num_expanded += 1

            sign = tuple(tuple(i) for i in path.end())
            if sign in visited:
                continue
            visited.add(sign)

            if self.problem.is_goal(path.end()):  # solution found
                print("Expanded {}, Remained: {}".format(self.num_expanded, len(self.frontier_)))
                self.solution = path  # store the solution found
                return path
            else:
                neighs = self.problem.neighbors(path.end())
                for arc in reversed(list(neighs)):
                    np = Path(path, arc)
                    sign = tuple(tuple(i) for i in np.end())
                    if sign not in visited:
                        self.add_to_frontier(np)

        print("No more solution, explored".format(self.num_expaned))
        return None


def test_s(searcher, title):
    print("-" * 60 + "\n" + title)

    solution = searcher.search()
    if solution is not None:
        print("Cost: {}".format(solution.cost))


class IterativeDeepeningSearcher(Searcher):
    def __init__(self, problem):
        super().__init__(problem)

        self.visited_ = None
        self.start_t_ = None
        self.path_ = None

    """ Initializes the forontier """

    def initialize_frontier(self):
        self.frontier_ = []

    """ Returns True if there are no more nodes to expand """

    def empty_frontier(self):
        return len(self.frontier_) == 0

    """ Adds the path to the forontier """

    def add_to_frontier(self, path):
        self.frontier_.append(path)

    def search(self):
        self.start_t_ = time.time()
        self.path_ = self.frontier_.pop(0)

        for limit in range(1, 1000000):
            self.visited_ = set()

            ret = self.dfs_(0, limit)

            if ret is not None:
                print("Expanded {}, Remained: {}".format(self.num_expanded, len(self.frontier_)))
                self.solution = ret
                return ret

            if time.time() - self.start_t_ > 300:
                print("Time")
                return None

        # exceed 1000000 memory limit
        print("Mem")
        return None

    def dfs_(self, depth, limit):
        if self.problem.is_goal(self.path_.end()):
            print("Found solution")
            return self.path_

        if depth >= limit:
            return None

        sign = (tuple(tuple(i) for i in self.path_.end()), depth)
        if sign in self.visited_:
            return None
        self.visited_.add(sign)

        self.num_expanded += 1

        neighs = self.problem.neighbors(self.path_.end())
        for arc in list(neighs):
            tmp = self.path_

            # add new node
            self.path_ = Path(self.path_, arc)
            ret = self.dfs_(depth + 1, limit)

            # find solution
            if ret is not None:
                return ret

            if time.time() - self.start_t_ > 300:
                return None

            # backtracking to previous path
            self.path_ = tmp

        return None


def test_s(searcher, title):
    print("-" * 60 + "\n" + title)

    solution = searcher.search()
    if solution is not None:
        print("Cost: {}".format(solution.cost))
        print(solution)


class IterativeDeepeningAStarSearcher(Searcher):
    def __init__(self, problem):
        super().__init__(problem)

        self.visited_ = None
        self.start_t_ = None
        self.path_ = None
        self.node_generate_ = None

    """ Initializes the forontier """

    def initialize_frontier(self):
        self.frontier_ = []

    """ Returns True if there are no more nodes to expand """

    def empty_frontier(self):
        return len(self.frontier_) == 0

    """ Adds the path to the forontier """

    def add_to_frontier(self, path):
        self.frontier_.append(path)

    def search(self):
        self.start_t_ = time.time()
        self.path_ = self.frontier_.pop(0)
        self.node_generate_ = 0

        for limit in range(1, 100, 2):
            self.visited_ = set()
            ret = self.dfs_(limit)

            if ret is not None:
                print("Expanded {}, Generated: {}".format(self.num_expanded, self.node_generate_))
                self.solution = ret
                return ret

            if time.time() - self.start_t_ > 300:
                print("Expanded: {}, Status: {}".format(self.num_expanded, "Time"))
                return None

        # exceed 1000000 memory limit
        print("Mem")
        return None

    def dfs_(self, limit):
        if self.problem.is_goal(self.path_.end()):
            print("Found solution")
            return self.path_

        self.num_expanded += 1

        neighs = self.problem.neighbors(self.path_.end())
        q = FrontierPQ()
        for arc in list(neighs):
            path = Path(self.path_, arc)
            value = path.cost + self.problem.heuristic(path.end())

            if value <= limit:
                q.add(path, value)
                self.node_generate_ += 1

        while not q.empty():
            tmp = self.path_
            self.path_ = q.pop()

            ret = self.dfs_(limit)

            # find solution
            if ret is not None:
                return ret

            if time.time() - self.start_t_ > 300:
                return None

            # backtracking to previous path
            self.path_ = tmp

        return None


def test_s(searcher, title):
    print("-" * 60 + "\n" + title)

    solution = searcher.search()
    if solution is not None:
        print("Cost: {}".format(solution.cost))
        print(solution)

class UniformCostSearcher(Searcher):
    """ Initializes the forontier """
    def initialize_frontier(self):
        self.frontier = FrontierPQ()

    """ Returns True if there are no more nodes to expand """
    def empty_frontier(self):
        return self.frontier.empty()

    """ Adds the path to the forontier """
    def add_to_frontier(self, path):
        self.frontier.add(path, path.cost)
        if len(self.frontier) > 1000000:
            print("Mem")
            raise


def test_s(searcher, title):
    print("-" * 60 + "\n" + title)
    solution = searcher.search()
    if solution is not None:
        print("[{}] cost: {}".format(title, solution.cost))


def test_as(searcher, title):
    print("-" * 60 + "\n" + title)
    solution = searcher.search()
    if solution is not None:
        print("[{}] cost: {}".format(title, solution.cost))


def grid_to_csp(grid):
    domains = list(range(9))

    constraints = [

    ]

    def get_next(g):
        loc = np.argwhere(np.copy(g) == 0)
        if len(loc) > 0:
            return loc[0][0], loc[0][1]
        else:
            return -1, -1

    def validate(g, i, j, assignment):
        row_validation = all([assignment != g[i][c] for c in range(0, 9)])
        col_validation = all([assignment != g[r][j] for r in range(0, 9)])

        if not all([row_validation, col_validation]):
            return False

        ulx = 3 * (i // 3)
        uly = 3 * (j // 3)
        block_validation = all([assignment != g[r][c] for r in range(ulx, ulx + 3) for c in range(uly, uly + 3)])

        return block_validation

    def dfs(g, i, j):
        i, j = get_next(g)

        if i == -1:
            return True

        for assignment in range(1, 10):
            if not validate(g, i, j, assignment):
                continue

            g[i][j] = assignment

            if dfs(g, i, j):
                return True

            g[i][j] = 0

        return False

    def solve(g):
        return dfs(g, *get_next(g))

    if solve(grid):
        print(np.array(grid))
    else:
        print("Cannot find a solution")
    return CSP(domains, constraints)




if __name__ == "__main__":
    pass