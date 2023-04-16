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
N = 9  # number of CAVs
Sd = 30  # desired platoon spacing
P = 20  # prediction horizon
W = 30  # number of longitudinal cells
R = 3  # number of lanes
l = 4  # CAV length
v_free = 80 * 1.6 * 1000 / 3600  # traffic flow free velocity
v_c = 50 * 1.6 * 1000 / 3600  # minimum traffic flow velocity for leading CAV
L = max(N * Sd, v_free * T)  # Width of the cell

Q1 = 10
Q2 = 10
Q3 = 10
d_jam = 320 / 1000  # veh/m
d_up = d_jam / 1.5
d_ctc = 90 / 1000  # veh/m
f_max = 6000 / 3600  # veh/s
r_CAV = [0, 0, 0, 1, 1, 1, 1, 2, 2, 2]
w_init_1 = 2
w_init_0 = 1

M = 300

y = m.addVars(W+2, R, P+1, lb=0, ub=1, vtype=GRB.BINARY, name="y")
d = m.addVars(W+2, R, P+1, lb=0, ub=d_jam, vtype=GRB.CONTINUOUS, name="d")
f = m.addVars(W+2, R, P+1, lb=0, ub=f_max, vtype=GRB.CONTINUOUS, name="f")
D = m.addVars(W+2, R, P+1, lb=0, ub=f_max, vtype=GRB.CONTINUOUS, name="D")
S = m.addVars(W+2, R, P+1, lb=0, ub=f_max, vtype=GRB.CONTINUOUS, name="S")
S_auxi = m.addVars(W+2, R, P+l, lb=0,  vtype=GRB.CONTINUOUS, name="S_auxi")
D_auxi = m.addVars(W+2, R, P+1, lb=0, vtype=GRB.CONTINUOUS, name="D_auxi")
x_f = m.addVars(N, lb=0, vtype=GRB.CONTINUOUS, name="x_f")
v_f = m.addVars(N, lb=0, vtype=GRB.CONTINUOUS, name="v_f")
x_s = m.addVars(N, lb=0, vtype=GRB.CONTINUOUS, name="x_s")
v_s = m.addVars(N, lb=0, vtype=GRB.CONTINUOUS, name="v_s")


"""Add Constraints"""
# Initial Conditions

for w in range(W+2):
    for r in range(R):
        # density
        m.addConstr(d[w, r, 0] == random.uniform(0, d_up), name="density")
        # supply
        if w <= W:
            m.addConstr(S_auxi[w, r, 0] == 10.5 *
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
        m.addConstr(w*y[w, r, 0] >= w_init_1 + M *
                    (y[w, r, 0]-1), name="cell_init_1")
        m.addConstr(w*y[w, r, 0] <= w_init_0, name="cell_init_0")

for p in range(1, P+1):
    for w in range(W+2):
        for r in range(R):
            # density
            if w <= W:
                m.addConstr(d[w, r, p] == d[w, r, p-1] + T/L * (f[w, r, p-1] -
                            f[w+1, r, p-1] + y[w, r, p-1] * N / L), name="density")
            else:
                m.addConstr(d[w, r, p] == 0, name="density")
            # supply
            if w <= W:
                m.addConstr(S_auxi[w, r, p] == 10.5 *
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
            m.addConstr(w*y[w, r, p] >= w_init_1 + p*T * v_c /
                        L + M*(y[w, r, p]-1), name="cell_init_1")
            m.addConstr(w*y[w, r, p] <= w_init_0 + p *
                        T * v_free/L, name="cell_init_0")

# m.addConstr(quicksum(quicksum(y[w,r,0] for r in range(R)) for w in range(W)) == 0, name="target_cell_0step")
m.addConstr(quicksum(quicksum(quicksum(y[w, r, p] for p in range(P))
                     for r in range(R)) for w in range(1, W+1)) == 1, "one_target_cell")

"""Add Objective Function"""

sum1 = Q1 * 1000 * quicksum(quicksum(quicksum(quicksum((r_CAV[i] - r * y[w, r, p]) * (r_CAV[i] - r * y[w, r, p])
                                                       for i in range(N)) for r in range(R)) for w in range(W+2)) for p in range(P+1))
sum2 = Q2 * quicksum(quicksum(quicksum((p * y[w, r, p]) * (p * y[w, r, p])
                                       for r in range(R)) for w in range(W+2)) for p in range(P+1))
sum3 = Q3 * quicksum(quicksum(quicksum((d[w, r, p] - d_ctc) * (d[w, r, p] - d_ctc)
                                       for r in range(R)) for w in range(W+2)) for p in range(P+1))
m.update()
m.setObjective(sum1+sum2+sum3, GRB.MINIMIZE)

m.optimize()
# m.printQuality()
# print(m.display())


def DrawFigure():
    H = np.zeros((R, W+2, P+1))
    filenames = []
    target_cell_time = 0

    for p in range(P+1):
        for w in range(W+2):
            for r in range(R):
                if abs(y[w, r, p].x - 1) < 0.01:
                    target_cell_time = (w, r, p)
                H[r, w, p] = d[w, r, p].x * L
    print(target_cell_time)
    w, r, p = target_cell_time
    # print(w - (w_init_1 + p*T * v_c/L))
    # print(w - (w_init_0 + p * T * v_free / L))
    fig, ax = plt.subplots()
    i = 0
    im = ax.imshow(H[:, :, i], animated=True)
    CAV_cells = f"Init CAV cells: from w = {w_init_0} - {w_init_1}"
    CAV_lanes = f"Init CAV lanes: 3 CAVs on lane 0; 4 CAVs on lane 1; 3 CAVs on lane 2"
    ax.text(0, 6, CAV_cells)
    ax.text(0, 7, CAV_lanes)
    ax.text(0, 8, f"target cell: w={w}, r={r}, p={p}")
    ax.text(0, 9, f"computation time: {m.Runtime} sec")

    # for (i, j), label in np.ndenumerate(H[:,:,i]):
    #     ax.text(i, j, label, ha='center', va='center')

    # plt.matshow(H)
    # for (x, y), value in np.ndenumerate(H[:,:,i]):
    #     plt.text(x, y, f"{value:.2f}", va="center", ha="center")
    # for (i, j), z in np.ndenumerate(data):
    #     ax.text(j, i, '{:0.1f}'.format(z), ha='center', va='center',
    #             bbox=dict(boxstyle='round', facecolor='white', edgecolor='0.3'))
    def updatefig(*args):
        global i
        if (i < P):
            i += 1
        else:
            i = 0
        im.set_array(H[:, :, i])
        return im,

    ani = animation.FuncAnimation(fig, updatefig, blit=True)
    plt.show()


if m.status == GRB.Status.OPTIMAL:
    for i in range(m.SolCount):
        m.Params.SolutionNumber = i
        m.write(f"09202022CTM-{i}.sol")
    DrawFigure()

else:
    m.computeIIS()
    m.write("09202022CTM.ilp")


# CTM dynamic and constraints
# for p in range(1, P+1):
#     for r in range(R):
#         # demand of the starting cell
#         m.addConstr(D[0,r,p] == min(v_free * random.uniform(0, d_jam), f_max), name="demand_init")
#         # supply/flow of the end cell
#         m.addConstr(f[W,r,p] == S[W-1,r,p], name="supply_end")
#         for w in range(1,W):
#             m.addConstr(D_auxi[w, r, p] == v_free * d[w - 1, r, p], name="demand_auxi")
#             m.addConstr(D[w, r, p] == min_(D_auxi[w, r, p], f_max), name="demand") # D[w,r,p] represnts the flow from D[w-1,r,p] tpo D[w,r,p]
#
#         for w in range(W):
#             m.addConstr(d[w,r,p] == d[w,r,p-1] + T/L * (f[w,r,p-1] - f[w+1,r,p-1]) + y[w,r,p] * N / L, name="dynamics")
#             m.addConstr(f[w,r,p] == min_(D[w,r,p], S[w,r,p]), name="flow")
#
#
#             m.addConstr(S_auxi[w,r,p] == 10.5 * (d_jam - d[w,r,p]), name="suppy_auxi")
#             m.addConstr(S[w,r,p] == min_(S_auxi[w,r,p], f_max), name="supply")
#
#             # m.addConstr(w * y[w,r,p] - w_init_1 >= M * (y[w,r,p] - 1), name="initial_1")
#             # m.addConstr(w * y[w,r,p] - w_init_0 <= p * T * v_free / L, name="initial_0")
#
# m.addConstr(quicksum(quicksum(y[w,r,0] for r in range(R)) for w in range(W)) == 0, name="target_cell_0step")
# m.addConstr(quicksum(quicksum(quicksum(y[w,r,p] for p in range(1, P+1))
#                      for r in range(R)) for w in range(W))== 0, "one_target_cell")
