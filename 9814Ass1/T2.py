import time
import numpy as np
import sys
import copy
sys.path.insert(0, "aipython")
from aipython.searchGeneric import *
from aipython.searchProblem import *
from aipython.cspProblem import *
from aipython.cspSearch import *
from T1 import *

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


GFP = GameFifteenProblem
start1 = [[1, 2, 3, 4],
          [5, 6, 7, 8],
          [9, 10, 11, 12],
          [13, 14, 0, 15]]

# optimal path cost: 10
start10 = [[2, 3, 7, 4],
           [1, 6, 11, 8],
           [5, 10, 0, 12],
           [9, 13, 14, 15]]

test_s(BreadthFirstSearcher(GFP(start1, goal)), "start1 BFS")
test_s(BreadthFirstSearcher(GFP(start10, goal)), "start10 BFS")


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


GFP = GameFifteenProblem
start1 = [[1, 2, 3, 4],
          [5, 6, 7, 8],
          [9, 10, 11, 12],
          [13, 14, 0, 15]]

# optimal path cost: 10
start10 = [[2, 3, 7, 4],
           [1, 6, 11, 8],
           [5, 10, 0, 12],
           [9, 13, 14, 15]]

# optimal path cost: 24
start24 = [[2, 7, 11, 4],
           [6, 3, 12, 0],
           [1, 5, 15, 8],
           [9, 10, 13, 14]]

test_s(IterativeDeepeningSearcher(GFP(start1, goal)), "start1 IDS")
test_s(IterativeDeepeningSearcher(GFP(start10, goal)), "start10 IDS")
test_s(IterativeDeepeningSearcher(GFP(start24, goal)), "start24 IDS")


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


GFP = GameFifteenProblem
start1 = [[1, 2, 3, 4],
          [5, 6, 7, 8],
          [9, 10, 11, 12],
          [13, 14, 0, 15]]

# optimal path cost: 10
start10 = [[2, 3, 7, 4],
           [1, 6, 11, 8],
           [5, 10, 0, 12],
           [9, 13, 14, 15]]

# optimal path cost: 24
start24 = [[2, 7, 11, 4],
           [6, 3, 12, 0],
           [1, 5, 15, 8],
           [9, 10, 13, 14]]

print(test_s(IterativeDeepeningAStarSearcher(GFP(start1, goal)), "start1 IDSA*"))
test_s(IterativeDeepeningAStarSearcher(GFP(start10, goal)), "start10 IDSA*")
test_s(IterativeDeepeningAStarSearcher(GFP(start24, goal)), "start24 IDSA*")

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


# optimal path cost: 10
start10 = [[2, 3, 7, 4],
           [1, 6, 11, 8],
           [5, 10, 0, 12],
           [9, 13, 14, 15]]

# optimal path cost: 24
start24 = [[2, 7, 11, 4],
           [6, 3, 12, 0],
           [1, 5, 15, 8],
           [9, 10, 13, 14]]

# optimal path cost: 30
start30 = [[2, 7, 11, 4],
           [6, 3, 12, 0],
           [1, 5, 15, 14],
           [9, 10, 8, 13]]

# optimal path cost: 36
start36 = [[7, 11, 12, 4],
           [2, 3, 14, 1],
           [6, 5, 13, 8],
           [9, 10, 15, 0]]

# optimal path cost: 41
start41 = [[7, 11, 12, 4],
           [2, 3, 8, 14],
           [10, 0, 5, 1],
           [6, 9, 13, 15]]


def test_s(searcher, title):
    print("-" * 60 + "\n" + title)
    solution = searcher.search()
    if solution is not None:
        print("[{}] cost: {}".format(title, solution.cost))


GFP = GameFifteenProblem

# your code
try:
    test_s(BreadthFirstSearcher(GFP(start10, goal)), "start10 BFS")
    test_s(BreadthFirstSearcher(GFP(start24, goal)), "start24 BFS")
    test_s(BreadthFirstSearcher(GFP(start30, goal)), "start30 BFS")
    test_s(BreadthFirstSearcher(GFP(start36, goal)), "start36 BFS")
    test_s(BreadthFirstSearcher(GFP(start41, goal)), "start41 BFS")
except:
    print("Continue to next algorithm")

try:
    test_s(IterativeDeepeningSearcher(GFP(start10, goal)), "start10 IDS")
    test_s(IterativeDeepeningSearcher(GFP(start24, goal)), "start24 IDS")
    test_s(IterativeDeepeningSearcher(GFP(start30, goal)), "start30 IDS")
    test_s(IterativeDeepeningSearcher(GFP(start36, goal)), "start36 IDS")
    test_s(IterativeDeepeningSearcher(GFP(start41, goal)), "start41 IDS")
except:
    print("Continue to next algorithm")

try:
    test_s(IterativeDeepeningAStarSearcher(GFP(start10, goal)), "start10 IDA*")
    test_s(IterativeDeepeningAStarSearcher(GFP(start24, goal)), "start24 IDA*")
    test_s(IterativeDeepeningAStarSearcher(GFP(start30, goal)), "start30 IDA*")
    test_s(IterativeDeepeningAStarSearcher(GFP(start36, goal)), "start36 IDA*")
    test_s(IterativeDeepeningAStarSearcher(GFP(start41, goal)), "start41 IDA*")
except:
    print("Continue to next algorithm")

try:
    test_s(UniformCostSearcher(GFP(start10, goal)), "start10 UCS")
    test_s(UniformCostSearcher(GFP(start24, goal)), "start24 UCS")
    test_s(UniformCostSearcher(GFP(start30, goal)), "start30 UCS")
    test_s(UniformCostSearcher(GFP(start36, goal)), "start36 UCS")
    test_s(UniformCostSearcher(GFP(start41, goal)), "start41 UCS")
except:
    print("Continue to next algorithm")

try:
    test_s(AStarSearcher(GFP(start10, goal)), "start10 A*")
    test_s(AStarSearcher(GFP(start24, goal)), "start24 A*")
    test_s(AStarSearcher(GFP(start30, goal)), "start30 A*")
    test_s(AStarSearcher(GFP(start36, goal)), "start36 A*")
    test_s(AStarSearcher(GFP(start41, goal)), "start41 A*")
except:
    print("Done")

print('Task2 Done')