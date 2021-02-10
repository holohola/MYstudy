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

start37_1 = [[7, 11, 12, 4],
             [2, 3, 14, 1],
             [6, 5, 13, 0],
             [9, 10, 15, 8]]

start37_2 = [[7, 11, 12, 4],
             [2, 3, 8, 14],
             [6, 0, 1, 15],
             [9, 5, 10, 13]]


# your code

def test_as(searcher, title):
    print("-" * 60 + "\n" + title)
    solution = searcher.search()
    if solution is not None:
        print("[{}] cost: {}".format(title, solution.cost))


test_as(IterativeDeepeningAStarSearcher(GFP(start37_1, goal)), "start37_1 IDA*")
test_as(IterativeDeepeningAStarSearcher(GFP(start37_2, goal)), "start37_2 IDA*")
print('Task 3 Done')