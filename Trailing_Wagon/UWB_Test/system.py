from math import sqrt
import numpy as np
from scipy.optimize import fsolve

class system:

    def __init__(self, a_pos, b_pos, c_pos):
        ''' Initialize sensor positions for system 
            Each sensor holds an x and y coordinate e.g.  a = (x,y)
        '''
        self.a = a_pos
        self.b = b_pos
        self.c = c_pos
    
    def trilaterate(self, dist_a, dist_b, dist_c):
        ''' Trilaterate user through least squares '''

        def func(z, data):
            ''' Defines system of equations'''
            x,y,w = z

            ax = data[0]
            ay = data[1]

            bx = data[2]
            by = data[3]

            cx = data[4]
            cy = data[5]

            F1 = dist_a**2 - x**2 + 2*ax*x - ax**2 - y**2 + 2*y*ay - ay**2
            F2 = dist_b**2 - x**2 + 2*bx*x - bx**2 - y**2 + 2*y*by - by**2
            F3 = dist_c**2 - x**2 + 2*cx*x - cx**2 - y**2 + 2*y*cy - cy**2
            return [F1, F2, F3]

        # Solve system of equations and cut off extraneous information
        solution_guess = [1,1,1]
        xy = fsolve(func, solution_guess, [a[0], a[1], b[0], b[1], c[0], c[1]])
        coordinate = (int(xy[0]), int(xy[1]))
        return coordinate