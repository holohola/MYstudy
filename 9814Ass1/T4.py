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
from T2 import *
from T3 import *
grid = [[9, 5, 0, 8, 2, 7, 3, 0, 0],
        [0, 8, 0, 1, 4, 0, 0, 5, 0],
        [0, 1, 0, 5, 9, 0, 0, 0, 0],
        [8, 3, 0, 0, 0, 0, 0, 7, 5],
        [1, 6, 9, 7, 5, 2, 4, 3, 0],
        [0, 7, 0, 0, 8, 0, 0, 6, 0],
        [0, 9, 1, 0, 6, 0, 8, 4, 0],
        [7, 0, 8, 0, 3, 1, 0, 0, 6],
        [6, 2, 0, 4, 7, 8, 0, 9, 0]]


# define constraint function(s)

# function that returns a csp
def grid_to_csp(grid):
    domains = list(range(9))

    constraints = [

    ]

    return CSP(domains, constraints)


# csp
csp = grid_to_csp(grid)
print(csp)


# your code
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


print('Task4 Done')