import gurobipy as gp
from gurobipy import GRB
import random
import math
import matplotlib.pyplot as plt
import sys
import numpy as np
from typing import Dict, Any

def generate_data() -> Dict[str, Any]:
    print("--- Generating Data (Multi-Objective) ---")
    num_customers = 10
    num_personnel = 2
    customers = list(range(1, num_customers + 1))
    candidates = [100, 101, 102]
    nodes = customers + candidates
    
    setup_costs = {100: 0, 101: 0, 102: 0}
    main_depot_dists = {100: 150, 101: 250, 102: 200}
    
    coords = {}
    main_depot_coords = (50, 150)
    coords[100] = (20, 80)
    coords[101] = (50, 80)
    coords[102] = (80, 20)
    
    random.seed(42)
    for i in customers:
        coords[i] = (random.randint(0, 100), random.randint(0, 100))
        
    dist_matrix = {}
    for i in nodes:
        for j in nodes:
            d = math.sqrt((coords[i][0]-coords[j][0])**2 + (coords[i][1]-coords[j][1])**2)
            dist_matrix[i, j] = d

    return {
        "customers": customers,
        "candidates": candidates,
        "personnel": list(range(num_personnel)),
        "nodes": nodes,
        "setup_costs": setup_costs,
        "main_depot_dists": main_depot_dists,
        "main_depot_coords": main_depot_coords,
        "coords": coords,
        "dist_matrix": dist_matrix,
        "big_m": 1000
    }

def solve_weighted_sum(alpha, data):
    N = data["customers"]
    K = data["candidates"]
    P = data["personnel"]
    V = data["nodes"]
    d_ij = data["dist_matrix"]
    setup_costs = data["setup_costs"]
    main_depot_dists = data["main_depot_dists"]
    BigM = data["big_m"]
    
    m = gp.Model(f"MultiObj_Alpha_{alpha}")
    m.setParam('OutputFlag', 0) # Silence output
    m.setParam('TimeLimit', 300) # 5-minute time limit
    
    y = m.addVars(K, vtype=GRB.BINARY, name="y")
    
    valid_arcs = []
    for p in P:
        for i in N:
            for j in N:
                if i != j: valid_arcs.append((i, j, p))
        for k in K:
            for j in N:
                valid_arcs.append((k, j, p))
                valid_arcs.append((j, k, p))

    x = m.addVars(valid_arcs, vtype=GRB.BINARY, name="x")
    u = m.addVars(N, P, vtype=GRB.CONTINUOUS, lb=0, ub=len(N), name="u")
    
    # Workload variables
    W_max = m.addVar(vtype=GRB.CONTINUOUS, name="W_max")
    W_min = m.addVar(vtype=GRB.CONTINUOUS, name="W_min")
    
    # Route Lengths
    route_lens = {}
    for p in P:
        route_lens[p] = gp.quicksum(d_ij[i, j] * x[i, j, p] for (i, j, p_val) in valid_arcs if p_val == p)
    
    # Constraints
    m.addConstr(gp.quicksum(y[k] for k in K) == 1)
    
    for i in N:
        m.addConstr(gp.quicksum(x[j, i, p] for p in P for j in V if (j, i, p) in valid_arcs) == 1)
        
    for p in P:
        for i in N:
            inflow = gp.quicksum(x[j, i, p] for j in V if (j, i, p) in valid_arcs)
            outflow = gp.quicksum(x[i, j, p] for j in V if (i, j, p) in valid_arcs)
            m.addConstr(inflow == outflow)
            
    for p in P:
        for k in K:
            outflow_k = gp.quicksum(x[k, j, p] for j in N)
            m.addConstr(outflow_k <= y[k])
            inflow_k = gp.quicksum(x[j, k, p] for j in N)
            m.addConstr(outflow_k == inflow_k)
            
    # MTZ
    for p in P:
        for i in N:
            for j in N:
                if i != j:
                    m.addConstr(u[i, p] - u[j, p] + BigM * x[i, j, p] <= BigM - 1)
                    
    # Workload Balance Constraints
    # Only consider active personnel? Or all?
    # If a personnel is unused, route len is 0. W_min will be 0.
    # To make it fair, we usually want to balance active routes.
    # But for simplicity, let's assume we want to balance ALL (forcing usage if possible).
    # Or, let's force usage of all personnel to make balance meaningful.
    for p in P:
        m.addConstr(gp.quicksum(x[k, j, p] for k in K for j in N) == 1, name=f"Force_Use_{p}")
        
    for p in P:
        m.addConstr(route_lens[p] <= W_max)
        m.addConstr(route_lens[p] >= W_min)
        
    # Objectives
    # Z1: Total Cost (Setup + Truck + Routing)
    cost_setup = gp.quicksum(setup_costs[k] * y[k] for k in K)
    cost_truck = gp.quicksum(2 * main_depot_dists[k] * y[k] for k in K)
    cost_routing = gp.quicksum(route_lens[p] for p in P)
    Z1 = cost_setup + cost_truck + cost_routing
    
    # Z2: Balance
    Z2 = W_max - W_min
    
    # Global Objective
    # Normalize? Z1 is ~500, Z2 is ~10-100.
    # Without normalization, Z1 dominates.
    # Let's just use raw weighted sum as requested.
    m.setObjective(alpha * Z1 + (1 - alpha) * Z2, GRB.MINIMIZE)
    
    m.optimize()
    
    if m.SolCount > 0:
        return Z1.getValue(), Z2.getValue(), m.ObjVal, m, x, y, route_lens
    else:
        return None, None, None, None, None, None, None

def solve_mobile_vrp():
    original_stdout = sys.stdout
    with open("output_projectv5.txt", "w") as f:
        sys.stdout = f
        data = generate_data()
        
        alphas = [0.0, 0.2, 0.4, 0.6, 0.8, 1.0]
        results = []
        
        print(f"{'Alpha':<10} | {'Z1 (Cost)':<15} | {'Z2 (Balance)':<15} | {'ObjVal':<15}")
        print("-" * 65)
        
        for alpha in alphas:
            z1, z2, obj, model, x, y, route_lens = solve_weighted_sum(alpha, data)
            if z1 is not None:
                print(f"{alpha:<10.1f} | {z1:<15.5f} | {z2:<15.5f} | {obj:<15.5f}")
                results.append((z1, z2))
                
                # Detailed Output for this Alpha
                print(f"\n--- Details for Alpha = {alpha} ---")
                K = data["candidates"]
                P = data["personnel"]
                N = data["customers"]
                V = data["nodes"]
                d_ij = data["dist_matrix"]
                
                for k in K:
                    if y[k].X > 0.5:
                        print(f"Selected Depot: {k}")
                
                for p in P:
                    print(f"Personnel {p}:")
                    start_node = None
                    for k in K:
                        if y[k].X > 0.5: start_node = k
                    
                    used = False
                    # Check if any arc leaves start_node for p
                    for j in N:
                        # Need to reconstruct valid_arcs check or just try/except
                        if (start_node, j, p) in x and x[start_node, j, p].X > 0.5:
                            used = True
                            break
                    
                    if not used:
                        print("  Unused")
                        continue
                        
                    route = [start_node]
                    curr = start_node
                    dist = 0
                    while True:
                        next_node = None
                        for j in V:
                            if (curr, j, p) in x and x[curr, j, p].X > 0.5:
                                next_node = j
                                dist += d_ij[curr, j]
                                break
                        if next_node is None: break
                        route.append(next_node)
                        curr = next_node
                        if curr == start_node: break
                    
                    print(f"  Path: {' -> '.join(map(str, route))}")
                    print(f"  Distance: {dist:.5f}")
                print("-" * 30 + "\n")

            else:
                print(f"{alpha:<10.1f} | {'Infeasible':<15} | {'-':<15} | {'-':<15}")
                
    sys.stdout = original_stdout
    print("Output saved to output_projectv5.txt")
    
    # Plot Pareto
    if results:
        z1_vals = [r[0] for r in results]
        z2_vals = [r[1] for r in results]
        
        plt.figure(figsize=(10, 8))
        plt.grid(True, linestyle='--', alpha=0.7)
        
        plt.plot(z1_vals, z2_vals, 'o-', markersize=10, linewidth=2, color='purple', label='Pareto Frontier')
        
        plt.xlabel('Z1: Total Cost (Minimize)', fontsize=12)
        plt.ylabel('Z2: Workload Imbalance (Minimize)', fontsize=12)
        plt.title('Multi-Objective Optimization: Cost vs Balance', fontsize=14)
        
        # Annotate points
        for i, alpha in enumerate(alphas):
            if i < len(results):
                plt.annotate(f"α={alpha}", (z1_vals[i], z2_vals[i]), 
                             xytext=(10, 10), textcoords='offset points',
                             bbox=dict(boxstyle="round,pad=0.3", fc="white", ec="gray", alpha=0.8))
                             
        plt.legend()
        plt.tight_layout()
        plt.savefig("plot_projectv5.png")

if __name__ == "__main__":
    solve_mobile_vrp()
