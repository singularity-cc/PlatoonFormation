import matplotlib.pyplot as plt
import matplotlib.animation as animation
import numpy as np
import pandas as pd
from gurobipy import *
import math
import time
# import imageio
import random
from matplotlib.ticker import MaxNLocator


"""Initialize an optimization model"""

m = Model("mip1")

"""Initialize variables"""
x = m.addVar(lb=0, ub=1, vtype=GRB.BINARY, name="x")
y = m.addVar(lb=-1, ub=1, vtype=GRB.CONTINUOUS, name="y")
z = m.addVar(lb=0, ub=1, vtype=GRB.BINARY, name="y")

w = m.addVar(lb=0, ub=1, vtype=GRB.BINARY, name="w")

"""Add Constraints"""

# m.addConstr(quicksum(quicksum(y[w,r,0] for r in range(R)) for w in range(W)) == 0, name="target_cell_0step")

m.addConstr(w == x * z, name="w=x*y")


"""Add Objective Function"""
m.update()
m.setObjective(w * y, GRB.MINIMIZE)

m.optimize()
# m.printQuality()
# print(m.display())
