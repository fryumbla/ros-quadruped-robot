
from sympy import *
from time import time
from mpmath import radians
import tf


def cal_IK(x,y):
    class Position:
        def __init__(self, EE_pos):
            self.x = EE_pos[0]
            self.y = EE_pos[1]
    
    def TF_Matrix(alpha, a, d, q):
        TF = Matrix([[           cos(q),           -sin(q),           0,             a],
                     [sin(q)*cos(alpha), cos(q)*cos(alpha), -sin(alpha), -sin(alpha)*d],
                     [sin(q)*sin(alpha), cos(q)*sin(alpha), cos(alpha), cos(alpha)*d],
                     [                0,                 0,          0,             1]])
        return TF
    

    q2 = acos((x*x + y*y + L1*L1 + L2*L2)/(2*L1*L2))
    r = atan((L2*sin(q2))/(L1+L2*cos(q2)))
    q1 = atan(x/y)


