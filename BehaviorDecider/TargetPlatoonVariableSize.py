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
T = 10  # time tick
tau = 1 # safe time headway
N = 5  # number of CAVs
Sd = 30  # desired platoon spacing
P = 20  # prediction horizon
W = 30  # number of longitudinal cells
R = 3  # number of lanes
l = 4  # CAV length
v_free = 80 * 1.6 * 1000 / 3600  # traffic flow free velocity
v_c = 50 * 1.6 * 1000 / 3600  # minimum traffic flow velocity for leading CAV
L = max(N * Sd, v_free * T)  # Width of the cell
d_jam = 320 / 1000  # veh/m
d_up = d_jam / 1.5
d_ctc = 90 / 1000  # veh/m
f_max = 6000 / 3600  # veh/s
s_slope = 10.5
M = 30000

# initial CAV demo
# r represents lane id, x represents longitudinal position, w represents the cell index
r_CAV = [0, 0, 1, 2, 2]
x_CAV = [0, 25, 50, 100, 200]
v_CAV = [15, 15, 15, 15, 15]
w_CAV = x_CAV[:]
for i in range(N):
    w_CAV[i] = x_CAV[i] / L
print(w_CAV)

# optimization parameters
Q1 = 1
Q2 = 1
Q3 = 1
Q4 = 1
Q5 = 10000000000000000

# important hyperparameters
# need some function to calculate vmin and vmax for each CAV according to their surrounding environments
# for demo example, just pick same values
vmin = [0, 0, 0, 0, 0]
vmax = [30, 30, 30, 30, 30]
dv = [1, 1, 1, 1, 1] # neighboring CAV speed difference threshold
dx = [Sd, Sd, Sd, Sd, Sd] # neighboring CAV speed difference threshold



# Initialize variables

y = m.addVars(W+2, R, P+1, lb=0, ub=1, vtype=GRB.BINARY, name="y")
d = m.addVars(W+2, R, P+1, lb=0, ub=d_jam, vtype=GRB.CONTINUOUS,
              name="d")  # d <= d_jam is already in the equation here
f = m.addVars(W+2, R, P+1, lb=0, ub=f_max, vtype=GRB.CONTINUOUS, name="f")
D = m.addVars(W+2, R, P+1, lb=0, ub=f_max, vtype=GRB.CONTINUOUS, name="D")
S = m.addVars(W+2, R, P+1, lb=0, ub=f_max, vtype=GRB.CONTINUOUS, name="S")
S_auxi = m.addVars(W+2, R, P+l, lb=0,  vtype=GRB.CONTINUOUS, name="S_auxi")
D_auxi = m.addVars(W+2, R, P+1, lb=0, vtype=GRB.CONTINUOUS, name="D_auxi")
x_f = m.addVars(N, lb=0, vtype=GRB.CONTINUOUS, name="x_f")
v_f = m.addVars(N, lb=0, vtype=GRB.CONTINUOUS, name="v_f")
z = m.addVars(N, lb=0, ub=1, vtype=GRB.BINARY, name="z")
z_sum = m.addVar(lb=0, ub=N, vtype=GRB.INTEGER, name="z_sum")
lane_change = m.addVars(W+2, R, P+1, N, lb=-R, ub=R,
                          vtype=GRB.CONTINUOUS, name="lane_change")
lane_change_square = m.addVars(W+2, R, P+1, N, lb=-R*R, ub=R*R,
                          vtype=GRB.CONTINUOUS, name="lane_change_square")
d_distance_to_cr = m.addVars(W+2, R, P+1, lb = -d_jam, ub = d_jam, vtype=GRB.CONTINUOUS, name = "d_to_cr")


"""Add Constraints"""

# dummy constraints
# z_sum constraint
m.addConstr(z_sum == quicksum(z[i] for i in range(N)), name="z_sum")
# m.addConstr(z_sum == N, name="z_sum")

for p in range(1, P+1):
    for w in range(W+2):
        for r in range(R):
            for i in range(N):
                m.addConstr(lane_change[w, r, p, i] == r_CAV[i] - r * y[w, r, p], name="lane_change") 
                m.addConstr(lane_change_square[w, r, p, i] == lane_change[w, r, p, i] * lane_change[w, r, p, i], name="lane_change_square") 
            m.addConstr(d_distance_to_cr[w, r, p] == d[w, r, p] - d_ctc, name="d_distance_to_cr")
            
            
# Initial Conditions
# Note: w = 0 means input flow cell, w = W + 1 means output flow cell; w = 1, 2, ..., W - 1, W means real flow cells in our model
for w in range(W+2):
    for r in range(R):
        # density
        # initialize the environment density map
        m.addConstr(d[w, r, 0] == random.uniform(0, d_up), name="density")
        # supply
        if w <= W:
            m.addConstr(S_auxi[w, r, 0] == s_slope *
                        (d_jam - d[w, r, 0]), name="S_auxi")
        else:
            m.addConstr(S_auxi[w, r, 0] == f_max, name="S_auxi")
        m.addConstr(S[w, r, 0] == min_(S_auxi[w, r, 0], f_max), name="S")
        # demand
        if w == 0:
            m.addConstr(D_auxi[w, r, 0] == v_free *
                        random.uniform(0, d_up), name="D_auxi")
        else:
            m.addConstr(D_auxi[w, r, 0] == v_free * d[w, r, 0], name="D_auxi")
        m.addConstr(D[w, r, 0] == min_(D_auxi[w, r, 0], f_max), name="D")
        # flow
        if w == 0:
            m.addConstr(f[w, r, 0] == S[w, r, 0], name="f")
        else:
            m.addConstr(f[w, r, 0] == min_(D[w-1, r, 0], S[w, r, 0]), name="f")

        # constraints for target cell
        for i in range(N):
            m.addConstr(w * y[w, r, 0] >= z[i] * w_CAV[i] + p*T * vmin[i] /
                        L + M * (y[w, r, 0] - 1), name="cell_min_reachable")
            m.addConstr(w * y[w, r, 0] <= z[i] * w_CAV[i], name="cell_max_reachable")




# intermediate constraints at future time steps
for p in range(1, P+1):
    for w in range(W+2):
        for r in range(R):
            # density, ctm model
            if w <= W:
                m.addConstr(d[w, r, p] == d[w, r, p-1] + T/L * (f[w, r, p-1] -
                            f[w+1, r, p-1] + y[w, r, p-1] * z_sum / L), name="density")
            else:
                m.addConstr(d[w, r, p] == 0, name="density")
            # supply
            if w <= W:
                m.addConstr(S_auxi[w, r, p] == s_slope *
                            (d_jam - d[w, r, p]), name="S_auxi")
            else:
                m.addConstr(S_auxi[w, r, p] == f_max, name="S_auxi")
            m.addConstr(S[w, r, p] == min_(S_auxi[w, r, p], f_max), name="S")
            # demand
            if w == 0:
                m.addConstr(D_auxi[w, r, p] == v_free *
                            random.uniform(0, d_up), name="D_auxi")
            else:
                m.addConstr(D_auxi[w, r, p] == v_free *
                            d[w, r, p], name="D_auxi")
            m.addConstr(D[w, r, p] == min_(D_auxi[w, r, p], f_max), name="D")
            # flow
            if w == 0:
                m.addConstr(f[w, r, p] == S[w, r, p], name="f")
            else:
                m.addConstr(f[w, r, p] == min_(
                    D[w - 1, r, p], S[w, r, p]), name="f")

            # constraints for target cell
            for i in range(N):
                m.addConstr(w * y[w, r, p] >= z[i] * w_CAV[i] + p*T * vmin[i] /
                            L + M * (y[w, r, p] - 1), name="cell_min_reachable")
                m.addConstr(w * y[w, r, p] <= z[i] * w_CAV[i] + p *
                            T * vmax[i] / L, name="cell_max_reachable")

# m.addConstr(quicksum(quicksum(y[w,r,0] for r in range(R)) for w in range(W)) == 0, name="target_cell_0step")
# constraint for only one target cell
m.addConstr(quicksum(quicksum(quicksum(y[w, r, p] for p in range(P))
                     for r in range(R)) for w in range(1, W+1)) == 1, "one_target_cell")

# constraints for micro-level 

for i in range(1, N):
    # adjacent xf, vf relations
    # note that i = 0 corresponds to the last CAV, which is different from the paper equation
    m.addConstr(v_f[i - 1] >= v_f[i] + z[i-1] * dv[i-1], name="dv")
    m.addConstr(x_f[i] == x_f[i - 1] + z[i] * (dx[i - 1] + v_f[i - 1] * tau), name="dx")
    

for i in range(N):
    #speed constraint
    m.addConstr(v_f[i] >= vmin[i], "min_v")
    m.addConstr(v_f[i] <= vmax[i], "max_v")
    
    for p in range(0, P + 1):
        for w in range(1, W + 1):
            for r in range(R):
                m.addConstr(x_f[i] <= w * y[w, r, p] * L + (1 - y[w, r, p]) * M + (1 - z[i]) * M, name="xf_in_cell_upper")
                m.addConstr(x_f[i] >= (w - 1) * y[w, r, p] * L + (y[w, r, p] - 1) * M + (z[i] - 1) * M, name="xf_in_cell_lower")
                m.addConstr(x_f[i] <= x_CAV[i] + vmax[i] * p * T * y[w, r, p] + (1 - y[w, r, p]) * M , name="xf_max_reachable")
                m.addConstr(x_f[i] >= x_CAV[i] + vmin[i] * p * T * y[w, r, p] + (y[w, r, p] - 1) * M , name="xf_min_reachable")
                
                
                


"""Add Objective Function"""

sum1 = Q1 * 1000 * quicksum(quicksum(quicksum(quicksum(z[i] * lane_change_square[w, r, p, i]
                                                       for i in range(N)) for r in range(R)) for w in range(W+2)) for p in range(P+1))
sum2 = Q2 * quicksum(quicksum(quicksum((p * y[w, r, p]) * (p * y[w, r, p])
                                       for r in range(R)) for w in range(W+2)) for p in range(P+1))
sum3 = Q3 * quicksum(quicksum(quicksum(d_distance_to_cr[w, r, p] * d_distance_to_cr[w, r, p]
                                       for r in range(R)) for w in range(W+2)) for p in range(P+1))
sum4 = Q4 * quicksum(((v_f[i] - v_CAV[i]) * (v_f[i] - v_CAV[i]) + (x_f[i] - x_CAV[i]) * (x_f[i] - x_CAV[i])) for i in range(N))

sum5 = Q5 * (N - z_sum) * (N - z_sum)
                           
m.update()
m.setObjective(sum1 + sum2 + sum3 + sum4 + sum5, GRB.MINIMIZE)

m.optimize()
# m.printQuality()
# print(m.display())

def GetNumCAVsJoinPlatoon():
    return z_sum.x

def GetTargetCellIndex():
    for w in range(1, W+1):
        for r in range(R):
            for p in range(P):
                if abs(y[w, r, p].x - 1) <= 0.01:
                    return w, r, p

def GetCAVTargetState(cav_index):
    return x_f[cav_index].x, v_f[cav_index].x

def GetCAVsTargetStates():
    target_states = []
    for i in range(N):
        target_states.append(GetCAVTargetState(i))
    return target_states

def DisplayCAVInitStates():
    print()
    print("**********Display CAV Initial States****************")
    for i in range(N):
        print(f"CAV {i} init x: {x_CAV[i]}, v: {v_CAV[i]}, r: {r_CAV[i]}, w: {w_CAV[i]}")
    
def DisplayOptimizationResults():
    print()
    print("**********Display Optimization Results****************")
    print(f"Time to compute optimization: {m.Runtime} seconds")
    print(f"Number of CAVs intend to join platoon is {GetNumCAVsJoinPlatoon()}" )
    print(f"Target Cell index [w, r, p]:  {GetTargetCellIndex()}" )

    cavs_target_states = GetCAVsTargetStates()
    for i in range(N):
        print(f"CAV {i} target states [xf, vf]: {cavs_target_states[i]}")

def DisplayOptimizationOutput():
    print()
    print("**********Display Optimization Outputs****************")
    if m.status == GRB.Status.OPTIMAL:
        for i in range(m.SolCount):
            m.Params.SolutionNumber = i
            m.write(f"09202022CTM-{i}.sol")
    else:
        m.computeIIS()
        m.write("07062023.ilp")
        
def DisplayOptimizationSettings():
    print()
    print("**********Display Optimization Settings****************")
    print(f"Q1 = {Q1}")
    print(f"Q2 = {Q2}")
    print(f"Q3 = {Q3}")
    print(f"Q4 = {Q4}")
    print(f"Q5 = {Q5}")
    for i in range(N):
        print(f"CAV {i} vmin: {vmin[i]}, vmax: {vmax[i]}, dv: {dv[i]}, dx: {dx[i]}")
    

DisplayCAVInitStates()
DisplayOptimizationSettings()
DisplayOptimizationResults()




