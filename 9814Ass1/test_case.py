'''
    GOOD LUCK ~
'''

goal = [[1, 2, 3, 4], 
        [5, 6, 7, 8], 
        [9, 10, 11, 12], 
        [13, 14, 15, 0]]

grid = [[4, 5, 0, 8, 0, 6, 3, 0, 0],
        [0, 8, 0, 1, 4, 0, 0, 5, 0],
        [0, 1, 0, 0, 2, 0, 0, 0, 0],
        [8, 3, 0, 0, 0, 0, 0, 7, 5],
        [1, 0, 9, 7, 0, 2, 4, 0, 0],
        [0, 7, 0, 0, 8, 0, 0, 6, 0],
        [0, 9, 1, 0, 6, 0, 8, 4, 0],
        [7, 0, 8, 0, 3, 1, 0, 0, 6],
        [6, 2, 0, 4, 0, 8, 0, 9, 0]]

search_case = {
    'start11': [[2, 3, 7, 4], [1, 6, 11, 8], [5, 10, 14, 12], [9, 13, 0, 15]], 
    'start11_1': [[2, 3, 7, 4], [1, 6, 11, 8], [5, 10, 12, 0], [9, 13, 14, 15]], 
    'start11_2': [[2, 3, 7, 4], [1, 6, 11, 8], [5, 0, 10, 12], [9, 13, 14, 15]], 
    'start12': [[2, 3, 7, 4], [1, 6, 11, 8], [5, 10, 14, 12], [9, 13, 15, 0]], 
    'start12_1': [[2, 3, 7, 4], [1, 6, 11, 8], [5, 10, 14, 12], [9, 0, 13, 15]], 
    'start12_2': [[2, 3, 7, 4], [1, 6, 11, 8], [5, 10, 14, 12], [9, 0, 13, 15]], 
    'start13': [[2, 3, 7, 4], [1, 6, 11, 8], [5, 10, 14, 0], [9, 13, 15, 12]], 
    'start13_1': [[2, 3, 7, 4], [1, 6, 11, 8], [5, 0, 14, 12], [9, 10, 13, 15]], 
    'start13_2': [[2, 3, 7, 4], [0, 6, 11, 8], [1, 5, 10, 12], [9, 13, 14, 15]], 
    'start14': [[2, 3, 7, 4], [1, 6, 11, 0], [5, 10, 14, 8], [9, 13, 15, 12]], 
    'start14_1': [[2, 3, 7, 4], [1, 6, 11, 8], [5, 10, 0, 14], [9, 13, 15, 12]], 
    'start14_2': [[2, 3, 7, 4], [1, 6, 11, 8], [0, 5, 14, 12], [9, 10, 13, 15]], 
    'start15': [[2, 3, 7, 0], [1, 6, 11, 4], [5, 10, 14, 8], [9, 13, 15, 12]], 
    'start15_1': [[2, 3, 7, 4], [1, 6, 0, 11], [5, 10, 14, 8], [9, 13, 15, 12]], 
    'start15_2': [[2, 3, 7, 4], [6, 11, 0, 8], [1, 5, 10, 12], [9, 13, 14, 15]], 
    'start16': [[2, 3, 0, 7], [1, 6, 11, 4], [5, 10, 14, 8], [9, 13, 15, 12]], 
    'start16_1': [[2, 3, 7, 4], [1, 6, 14, 11], [5, 10, 0, 8], [9, 13, 15, 12]], 
    'start16_2': [[2, 3, 7, 4], [1, 6, 11, 8], [10, 14, 0, 12], [5, 9, 13, 15]],
    'start17': [[1, 3, 6, 4], [5, 2, 8, 14], [9, 15, 7, 0], [13, 10, 12, 11]], 
    'start17_1': [[1, 3, 6, 4], [5, 2, 8, 14], [9, 15, 7, 0], [13, 10, 12, 11]], 
    'start17_2': [[1, 3, 6, 4], [5, 2, 8, 14], [9, 15, 7, 0], [13, 10, 12, 11]], 
    'start18': [[1, 3, 6, 4], [5, 2, 8, 14], [9, 15, 7, 11], [13, 10, 12, 0]], 
    'start18_1': [[1, 3, 6, 4], [5, 2, 8, 14], [9, 15, 0, 7], [13, 10, 12, 11]], 
    'start18_2': [[1, 3, 6, 4], [5, 2, 8, 14], [9, 15, 0, 7], [13, 10, 12, 11]], 
    'start19': [[1, 3, 6, 4], [5, 2, 8, 14], [9, 15, 7, 11], [13, 10, 0, 12]], 
    'start19_1': [[1, 3, 6, 4], [5, 2, 8, 14], [9, 15, 12, 7], [13, 10, 0, 11]], 
    'start19_2': [[1, 3, 6, 4], [5, 2, 0, 14], [9, 15, 8, 7], [13, 10, 12, 11]], 
    'start20': [[1, 3, 6, 4], [5, 2, 8, 14], [9, 15, 0, 11], [13, 10, 7, 12]], 
    'start20_1': [[1, 3, 6, 4], [5, 2, 8, 14], [9, 15, 7, 11], [13, 0, 10, 12]], 
    'start20_2': [[1, 3, 6, 4], [5, 2, 8, 14], [0, 9, 15, 7], [13, 10, 12, 11]], 
    'start21': [[1, 3, 6, 4], [5, 2, 0, 14], [9, 15, 8, 11], [13, 10, 7, 12]], 
    'start21_1': [[1, 3, 6, 4], [5, 2, 8, 14], [9, 15, 11, 0], [13, 10, 7, 12]], 
    'start21_2': [[1, 3, 6, 4], [5, 2, 8, 14], [9, 15, 12, 7], [0, 13, 10, 11]],
    'start22': [[1, 3, 6, 4], [5, 2, 8, 14], [0, 15, 12, 7], [9, 13, 10, 11]], 
    'start22_1': [[1, 3, 6, 4], [5, 2, 8, 14], [0, 15, 12, 7], [9, 13, 10, 11]], 
    'start22_2': [[1, 3, 6, 4], [5, 2, 8, 14], [0, 9, 12, 7], [13, 15, 10, 11]], 
    'start23': [[1, 3, 6, 4], [0, 2, 8, 14], [5, 15, 12, 7], [9, 13, 10, 11]], 
    'start23_1': [[1, 3, 6, 4], [5, 2, 8, 14], [15, 0, 12, 7], [9, 13, 10, 11]], 
    'start23_2': [[1, 3, 6, 4], [5, 2, 8, 14], [15, 0, 12, 7], [9, 13, 10, 11]], 
    'start24': [[0, 3, 6, 4], [1, 2, 8, 14], [5, 15, 12, 7], [9, 13, 10, 11]], 
    'start24_1': [[1, 3, 6, 4], [2, 0, 8, 14], [5, 15, 12, 7], [9, 13, 10, 11]], 
    'start24_2': [[1, 3, 6, 4], [5, 2, 8, 14], [15, 12, 0, 7], [9, 13, 10, 11]], 
    'start25': [[3, 0, 6, 4], [1, 2, 8, 14], [5, 15, 12, 7], [9, 13, 10, 11]], 
    'start25_1': [[1, 3, 6, 4], [2, 15, 8, 14], [5, 0, 12, 7], [9, 13, 10, 11]], 
    'start25_2': [[1, 3, 6, 4], [5, 2, 0, 14], [15, 12, 8, 7], [9, 13, 10, 11]],
    'start26': [[1, 3, 6, 4], [5, 0, 2, 14], [15, 12, 8, 7], [9, 13, 10, 11]], 
    'start26_1': [[1, 3, 6, 4], [5, 2, 8, 14], [15, 12, 10, 7], [9, 13, 11, 0]], 
    'start26_2': [[1, 3, 6, 4], [5, 0, 2, 14], [15, 12, 8, 7], [9, 13, 10, 11]], 
    'start27': [[1, 3, 6, 4], [5, 12, 2, 14], [15, 0, 8, 7], [9, 13, 10, 11]], 
    'start27_1': [[1, 0, 6, 4], [5, 3, 2, 14], [15, 12, 8, 7], [9, 13, 10, 11]], 
    'start27_2': [[1, 3, 6, 4], [0, 5, 2, 14], [15, 12, 8, 7], [9, 13, 10, 11]], 
    'start28': [[1, 3, 6, 4], [5, 12, 2, 14], [15, 13, 8, 7], [9, 0, 10, 11]], 
    'start28_1': [[1, 3, 6, 4], [5, 12, 2, 14], [15, 8, 0, 7], [9, 13, 10, 11]], 
    'start28_2': [[0, 3, 6, 4], [1, 5, 2, 14], [15, 12, 8, 7], [9, 13, 10, 11]],
    'start29': [[3, 0, 6, 4], [1, 5, 2, 14], [15, 12, 8, 7], [9, 13, 10, 11]], 
    'start29_1': [[1, 3, 6, 4], [15, 5, 2, 14], [9, 12, 8, 7], [0, 13, 10, 11]], 
    'start29_2': [[3, 0, 6, 4], [1, 5, 2, 14], [15, 12, 8, 7], [9, 13, 10, 11]], 
    'start30': [[3, 5, 6, 4], [1, 0, 2, 14], [15, 12, 8, 7], [9, 13, 10, 11]], 
    'start30_1': [[3, 6, 0, 4], [1, 5, 2, 14], [15, 12, 8, 7], [9, 13, 10, 11]], 
    'start30_2': [[3, 6, 0, 4], [1, 5, 2, 14], [15, 12, 8, 7], [9, 13, 10, 11]], 
    'start31': [[3, 5, 6, 4], [1, 12, 2, 14], [15, 0, 8, 7], [9, 13, 10, 11]], 
    'start31_1': [[3, 5, 6, 4], [1, 2, 0, 14], [15, 12, 8, 7], [9, 13, 10, 11]], 
    'start31_2': [[1, 3, 6, 4], [15, 5, 2, 14], [9, 12, 8, 7], [13, 10, 0, 11]], 
    'start32': [[3, 5, 6, 4], [1, 12, 2, 14], [15, 13, 8, 7], [9, 0, 10, 11]], 
    'start32_1': [[3, 5, 6, 4], [1, 12, 2, 14], [15, 8, 0, 7], [9, 13, 10, 11]], 
    'start32_2': [[3, 6, 4, 14], [1, 5, 2, 0], [15, 12, 8, 7], [9, 13, 10, 11]],
    'start33': [[3, 6, 4, 14], [1, 5, 2, 7], [15, 12, 8, 0], [9, 13, 10, 11]], 
    'start33_1': [[3, 6, 4, 14], [1, 5, 0, 2], [15, 12, 8, 7], [9, 13, 10, 11]], 
    'start33_2': [[3, 6, 4, 14], [1, 5, 0, 2], [15, 12, 8, 7], [9, 13, 10, 11]], 
    'start34': [[3, 6, 4, 14], [1, 5, 2, 7], [15, 12, 8, 11], [9, 13, 10, 0]], 
    'start34_1': [[3, 6, 4, 14], [1, 5, 2, 7], [15, 12, 0, 8], [9, 13, 10, 11]], 
    'start34_2': [[3, 6, 4, 14], [1, 5, 8, 2], [15, 13, 12, 7], [9, 0, 10, 11]], 
    'start35': [[3, 6, 4, 14], [1, 5, 2, 7], [15, 12, 8, 11], [9, 13, 0, 10]], 
    'start35_1': [[3, 6, 4, 14], [1, 5, 2, 7], [15, 12, 10, 8], [9, 13, 0, 11]], 
    'start35_2': [[3, 6, 4, 14], [0, 1, 5, 2], [15, 12, 8, 7], [9, 13, 10, 11]], 
    'start36': [[3, 6, 4, 14], [1, 5, 2, 7], [15, 12, 0, 11], [9, 13, 8, 10]], 
    'start36_1': [[3, 6, 4, 14], [1, 5, 2, 7], [15, 12, 8, 11], [9, 0, 13, 10]], 
    'start36_2': [[3, 6, 4, 14], [1, 5, 8, 0], [15, 12, 7, 2], [9, 13, 10, 11]],
    'start37': [[3, 6, 4, 14], [1, 5, 0, 8], [15, 12, 7, 2], [9, 13, 10, 11]], 
    'start37_1': [[3, 6, 4, 14], [1, 5, 8, 2], [15, 12, 7, 11], [9, 13, 0, 10]], 
    'start37_2': [[3, 6, 4, 14], [1, 5, 0, 8], [15, 12, 7, 2], [9, 13, 10, 11]], 
    'start38': [[3, 6, 4, 14], [1, 5, 7, 8], [15, 12, 0, 2], [9, 13, 10, 11]], 
    'start38_1': [[3, 6, 0, 14], [1, 5, 4, 8], [15, 12, 7, 2], [9, 13, 10, 11]], 
    'start38_2': [[3, 6, 4, 14], [1, 5, 7, 8], [15, 13, 12, 2], [9, 0, 10, 11]], 
    'start39': [[3, 6, 4, 14], [1, 5, 7, 8], [15, 12, 10, 2], [9, 13, 0, 11]], 
    'start39_1': [[3, 6, 14, 0], [1, 5, 4, 8], [15, 12, 7, 2], [9, 13, 10, 11]], 
    'start39_2': [[3, 6, 4, 14], [0, 1, 5, 8], [15, 12, 7, 2], [9, 13, 10, 11]], 
    'start40': [[3, 6, 4, 14], [1, 5, 7, 8], [15, 12, 10, 2], [9, 13, 11, 0]], 
    'start40_1': [[3, 6, 4, 14], [1, 5, 7, 8], [15, 12, 10, 2], [9, 0, 13, 11]]}