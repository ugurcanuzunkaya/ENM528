import gurobipy as gp
from gurobipy import GRB
import random
import math
import matplotlib.pyplot as plt
import sys
from typing import Dict, Any, Tuple

def generate_data() -> Dict[str, Any]:
    print("--- Generating Data (Heterogeneous Fleet) ---")
    
    num_customers = 10
    # Personnel 0: Cargo Bike, Personnel 1: E-Scooter
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
    demands = {}
    for i in customers:
        coords[i] = (random.randint(0, 100), random.randint(0, 100))
        demands[i] = random.randint(5, 15) # kg
        
    dist_matrix = {}
    for i in nodes:
        for j in nodes:
            d = math.sqrt((coords[i][0]-coords[j][0])**2 + (coords[i][1]-coords[j][1])**2)
            dist_matrix[i, j] = d

    # Heterogeneous Fleet Parameters
    # Personnel 0: Cargo Bike (High Cap, High Cost)
    # Personnel 1: E-Scooter (Low Cap, Low Cost)
    personnel_props = {
        0: {"capacity": 100, "cost_per_km": 1.5, "name": "Cargo Bike"},
        1: {"capacity": 40, "cost_per_km": 1.0, "name": "E-Scooter"}
    }

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
        "demands": demands,
        "personnel_props": personnel_props,
        "big_m": 1000
    }

def solve_mobile_vrp():
    original_stdout = sys.stdout
    with open("output_projectv4.txt", "w") as f:
        sys.stdout = f
        try:
            _solve_internal()
        finally:
            sys.stdout = original_stdout
    print("Output saved to output_projectv4.txt")

def _solve_internal():
    data = generate_data()
    
    N = data["customers"]
    K = data["candidates"]
    P = data["personnel"]
    V = data["nodes"]
    d_ij = data["dist_matrix"]
    setup_costs = data["setup_costs"]
    main_depot_dists = data["main_depot_dists"]
    demands = data["demands"]
    props = data["personnel_props"]
    BigM = data["big_m"]
    
    print("--- Building Model (Heterogeneous Fleet) ---")
    m = gp.Model("Mobile_VRP_V4")
    m.setParam('TimeLimit', 300)  # 5-minute time limit
    
    # Variables
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
    
    # Objective: Setup + Truck + Weighted Distance
    cost_setup = gp.quicksum(setup_costs[k] * y[k] for k in K)
    cost_truck = gp.quicksum(2 * main_depot_dists[k] * y[k] for k in K)
    
    cost_routing = 0
    for (i, j, p) in valid_arcs:
        cost_routing += d_ij[i, j] * props[p]["cost_per_km"] * x[i, j, p]
        
    m.setObjective(cost_setup + cost_truck + cost_routing, GRB.MINIMIZE)
    
    # Constraints
    
    # 1. Single Depot
    m.addConstr(gp.quicksum(y[k] for k in K) == 1, name="Single_Depot")
    
    # 2. Visit Each Customer Once
    for i in N:
        m.addConstr(gp.quicksum(x[j, i, p] for p in P for j in V if (j, i, p) in valid_arcs) == 1, name=f"Visit_{i}")
        
    # 3. Flow Balance
    for p in P:
        for i in N:
            inflow = gp.quicksum(x[j, i, p] for j in V if (j, i, p) in valid_arcs)
            outflow = gp.quicksum(x[i, j, p] for j in V if (i, j, p) in valid_arcs)
            m.addConstr(inflow == outflow, name=f"Flow_{i}_{p}")
            
    # 4. Depot Logic
    for p in P:
        for k in K:
            outflow_k = gp.quicksum(x[k, j, p] for j in N)
            m.addConstr(outflow_k <= y[k], name=f"Depot_Use_{k}_{p}")
            inflow_k = gp.quicksum(x[j, k, p] for j in N)
            m.addConstr(outflow_k == inflow_k, name=f"Depot_Bal_{k}_{p}")
            
        # Force start (optional, but good for LRP)
        # m.addConstr(gp.quicksum(x[k, j, p] for k in K for j in N) <= 1) 
        
    # 5. Capacity Constraints (Heterogeneous)
    for p in P:
        # Load = Sum of demands of customers visited by p
        # Visited if inflow (or outflow) is 1
        load = gp.quicksum(demands[i] * gp.quicksum(x[j, i, p] for j in V if (j, i, p) in valid_arcs) for i in N)
        m.addConstr(load <= props[p]["capacity"], name=f"Cap_{p}")
        
    # 6. MTZ
    for p in P:
        for i in N:
            for j in N:
                if i != j:
                    m.addConstr(u[i, p] - u[j, p] + BigM * x[i, j, p] <= BigM - 1, name=f"MTZ_{i}_{j}_{p}")

    print("--- Solving ---")
    m.optimize()
    
    if m.SolCount > 0:
        print(f"\nBest Solution Found (Obj: {m.ObjVal:.5f})")
        for k in K:
            if y[k].X > 0.5:
                print(f"Selected Depot: {k}")
                
        for p in P:
            print(f"\nPersonnel {p} ({props[p]['name']}):")
            # Find route
            start_node = None
            for k in K:
                if y[k].X > 0.5: start_node = k
            
            # Check if used
            used = False
            for j in N:
                if x[start_node, j, p].X > 0.5:
                    used = True
                    break
            
            if not used:
                print("  Unused")
                continue
                
            route = [start_node]
            curr = start_node
            dist = 0
            load = 0
            while True:
                next_node = None
                for j in V:
                    if (curr, j, p) in valid_arcs and x[curr, j, p].X > 0.5:
                        next_node = j
                        dist += d_ij[curr, j]
                        if j in N: load += demands[j]
                        break
                if next_node is None: break
                route.append(next_node)
                curr = next_node
                if curr == start_node: break
            
            print(f"  Path: {' -> '.join(map(str, route))}")
            print(f"  Distance: {dist:.5f}")
            print(f"  Load: {load}/{props[p]['capacity']}")
            
        plot_solution(data, m, x, y, "plot_projectv4.png")

def plot_solution(data, model, x, y, save_path):
    coords = data["coords"]
    N = data["customers"]
    K = data["candidates"]
    P = data["personnel"]
    props = data["personnel_props"]
    
    plt.figure(figsize=(12, 10))
    plt.grid(True, linestyle='--', alpha=0.7)
    
    # Plot Main Depot
    plt.scatter(data["main_depot_coords"][0], data["main_depot_coords"][1], c='black', marker='^', s=200, label='Main Depot', zorder=5)
    plt.text(data["main_depot_coords"][0], data["main_depot_coords"][1]+3, "Main", ha='center', fontsize=10, fontweight='bold')
    
    # Plot Customers
    for i in N:
        plt.scatter(coords[i][0], coords[i][1], c='blue', s=100, zorder=4, edgecolors='white')
        plt.text(coords[i][0], coords[i][1]+3, f"C{i}", ha='center', fontsize=9, fontweight='bold', 
                 bbox=dict(facecolor='white', alpha=0.7, edgecolor='none', pad=1))
        
    # Plot Depots
    selected_k = None
    for k in K:
        if y[k].X > 0.5:
            selected_k = k
            plt.scatter(coords[k][0], coords[k][1], c='red', marker='s', s=150, label=f'Active Depot {k}', zorder=5, edgecolors='black')
        else:
            plt.scatter(coords[k][0], coords[k][1], c='gray', marker='s', s=100, alpha=0.5, label='Potential Depot' if k==K[0] else "")
            
    # Plot Truck Route
    if selected_k:
        plt.plot([data["main_depot_coords"][0], coords[selected_k][0]],
                 [data["main_depot_coords"][1], coords[selected_k][1]], 'k--', linewidth=2, label='Truck Route', alpha=0.6)
                 
    colors = ['green', 'orange', 'purple', 'cyan']
    for p in P:
        # Plot routes
        for (i, j, p_idx), var in x.items():
            if p_idx == p and var.X > 0.5:
                # Line
                plt.plot([coords[i][0], coords[j][0]], [coords[i][1], coords[j][1]], 
                         c=colors[p % len(colors)], lw=2, zorder=3, 
                         label=f"{props[p]['name']}" if (i==selected_k or j==selected_k) else "")
                
                # Arrow for direction
                mid_x = (coords[i][0] + coords[j][0]) / 2
                mid_y = (coords[i][1] + coords[j][1]) / 2
                # Simple arrow annotation
                plt.annotate('', xy=(mid_x, mid_y), xytext=(coords[i][0], coords[i][1]),
                             arrowprops=dict(arrowstyle='->', color=colors[p % len(colors)], lw=2), zorder=3)
                         
    plt.title("Heterogeneous Fleet VRP Solution", fontsize=14)
    plt.xlabel("X Coordinate")
    plt.ylabel("Y Coordinate")
    
    # Fix legend duplicates
    handles, labels = plt.gca().get_legend_handles_labels()
    by_label = dict(zip(labels, handles))
    plt.legend(by_label.values(), by_label.keys(), loc='upper right', bbox_to_anchor=(1.15, 1))
    
    plt.tight_layout()
    plt.savefig(save_path)

if __name__ == "__main__":
    solve_mobile_vrp()
