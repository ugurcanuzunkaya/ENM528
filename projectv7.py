import gurobipy as gp
from gurobipy import GRB
import random
import math
import matplotlib.pyplot as plt
import sys
from typing import Dict, Any

def generate_data() -> Dict[str, Any]:
    print("--- Generating Data (VRPTW) ---")
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
    time_windows = {}
    service_times = {}
    
    # Generate feasible time windows based on a simple heuristic
    # Assume a TSP-like path 1->2->...->10 to set windows
    current_time = 0
    # Main depot to first customer
    current_time += 50 # Travel
    
    # Generate feasible time windows based on a simple heuristic
    # Create a dummy tour 1->2->...->10 to ensure at least one feasible path exists
    # Then widen windows slightly
    
    current_time = 0
    # Assume starting from a depot (say 100)
    current_time += 50 # Travel from depot
    
    # Shuffle customers to make it less trivial, but keep a known sequence
    seq = list(customers)
    random.shuffle(seq)
    
    for i in customers:
        coords[i] = (random.randint(0, 100), random.randint(0, 100))

    for i in seq:
        # Arrival at i
        # Set window around current_time
        # We make the window large enough to be hit
        start = max(0, current_time - 30)
        end = current_time + 60 # 60 min window
        time_windows[i] = (int(start), int(end))
        service_times[i] = 10
        
        # Move to next "dummy" location
        # We assume a large travel time to ensure feasibility regardless of distance
        # Max distance in 100x100 grid is ~142. 
        # So we add 150 to be safe + 10 service
        current_time += 10 + 150
        
    # Depots have wide windows
    for k in candidates:
        time_windows[k] = (0, 2000)
        service_times[k] = 0
        
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
        "time_windows": time_windows,
        "service_times": service_times,
        "big_m": 10000
    }

def solve_mobile_vrp():
    original_stdout = sys.stdout
    with open("output_projectv7.txt", "w") as f:
        sys.stdout = f
        try:
            _solve_internal()
        finally:
            sys.stdout = original_stdout
    print("Output saved to output_projectv7.txt")

def _solve_internal():
    data = generate_data()
    
    N = data["customers"]
    K = data["candidates"]
    P = data["personnel"]
    V = data["nodes"]
    d_ij = data["dist_matrix"]
    TW = data["time_windows"]
    ST = data["service_times"]
    BigM = data["big_m"]
    
    print("--- Building Model (VRPTW) ---")
    m = gp.Model("Mobile_VRP_V7")
    m.setParam('TimeLimit', 300) # 5-minute time limit
    
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
    
    # Arrival Time Variables
    arr_time = m.addVars(V, P, vtype=GRB.CONTINUOUS, lb=0, ub=5000, name="arr_time")
    
    # Objective: Minimize Total Distance
    cost_setup = gp.quicksum(data["setup_costs"][k] * y[k] for k in K)
    cost_truck = gp.quicksum(2 * data["main_depot_dists"][k] * y[k] for k in K)
    cost_routing = gp.quicksum(d_ij[i, j] * x[i, j, p] for (i, j, p) in valid_arcs)
    
    m.setObjective(cost_setup + cost_truck + cost_routing, GRB.MINIMIZE)
    
    # Standard Constraints
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
            
    # --- VRPTW Constraints ---
    
    # 1. Time Propagation
    # Arr[j] >= Arr[i] + Service[i] + Travel[i,j] - M(1-x)
    for (i, j, p) in valid_arcs:
        travel_time = d_ij[i, j] # Assume speed = 1 unit/min for simplicity
        
        if i in K:
            # Leaving depot: Start time is 0
            # arr_time[j] >= 0 + ST[i] + travel - M(1-x)
            m.addConstr(arr_time[j, p] >= 0 + ST[i] + travel_time - BigM * (1 - x[i, j, p]), 
                        name=f"TimeProp_DepotStart_{i}_{j}_{p}")
        else:
            # Normal propagation (Customer -> Customer or Customer -> Depot)
            m.addConstr(arr_time[j, p] >= arr_time[i, p] + ST[i] + travel_time - BigM * (1 - x[i, j, p]), 
                        name=f"TimeProp_{i}_{j}_{p}")
        
    # 2. Time Windows
    for p in P:
        for i in V:
            # Only enforce if visited? 
            # If not visited, variable is free but bounded.
            # But if visited, must be within window.
            # Since we don't have a "visited" variable for depots easily accessible (except y),
            # and customers are always visited.
            
            # For customers:
            if i in N:
                m.addConstr(arr_time[i, p] >= TW[i][0], name=f"TW_Start_{i}_{p}")
                m.addConstr(arr_time[i, p] <= TW[i][1], name=f"TW_End_{i}_{p}")
                
            # For depots:
            # Start time at depot is usually 0 or flexible.
            # Return time must be within window.
            if i in K:
                m.addConstr(arr_time[i, p] >= TW[i][0])
                m.addConstr(arr_time[i, p] <= TW[i][1])

    # Note: MTZ is not strictly needed if Time Windows + Time Propagation are present and windows are tight enough,
    # but to be safe against cycles with 0 cost/time (unlikely here), we can keep MTZ or rely on time.
    # Time propagation prevents cycles if time increases.
    # Since ServiceTime > 0 or TravelTime > 0, cycles are impossible.
    
    print("--- Solving ---")
    m.optimize()
    
    if m.SolCount > 0:
        print(f"\nBest Solution Found (Obj: {m.ObjVal:.5f})")
        
        for p in P:
            print(f"\nPersonnel {p}:")
            start_node = None
            for k in K:
                if y[k].X > 0.5: start_node = k
            
            used = False
            for j in N:
                if x[start_node, j, p].X > 0.5: used = True; break
            
            if not used: print("  Unused"); continue
            
            route = [start_node]
            curr = start_node
            dist = 0
            while True:
                next_node = None
                for j in V:
                    if (curr, j, p) in valid_arcs and x[curr, j, p].X > 0.5:
                        next_node = j
                        dist += d_ij[curr, j]
                        break
                if next_node is None: break
                route.append(next_node)
                curr = next_node
                if curr == start_node: break
            
            print(f"  Path: {' -> '.join(map(str, route))}")
            print(f"  Distance: {dist:.5f}")
            
            # Print Arrival Times
            print("  Arrival Times:")
            for node in route:
                print(f"    Node {node}: {arr_time[node, p].X:.2f} (Window: {TW[node]})")
            
        plot_solution(data, m, x, y, "plot_projectv7.png")

def plot_solution(data, model, x, y, save_path):
    coords = data["coords"]
    N = data["customers"]
    K = data["candidates"]
    P = data["personnel"]
    
    plt.figure(figsize=(12, 10))
    plt.grid(True, linestyle='--', alpha=0.7)
    
    # Plot Main Depot
    plt.scatter(data["main_depot_coords"][0], data["main_depot_coords"][1], c='black', marker='^', s=200, label='Main Depot', zorder=5)
    plt.text(data["main_depot_coords"][0], data["main_depot_coords"][1]+3, "Main", ha='center', fontsize=10, fontweight='bold')
    
    # Plot Customers
    for i in N:
        plt.scatter(coords[i][0], coords[i][1], c='blue', s=100, zorder=4, edgecolors='white')
        # Show Time Window in label
        tw = data['time_windows'][i]
        label_text = f"C{i}\n[{tw[0]}-{tw[1]}]"
        plt.text(coords[i][0], coords[i][1]+4, label_text, ha='center', fontsize=8, fontweight='bold', 
                 bbox=dict(facecolor='white', alpha=0.7, edgecolor='none', pad=1))
        
    # Plot Depots
    selected_k = None
    for k in K:
        if y[k].X > 0.5:
            selected_k = k
            plt.scatter(coords[k][0], coords[k][1], c='red', marker='s', s=150, label=f'Active Depot {k}', zorder=5, edgecolors='black')
        else:
            plt.scatter(coords[k][0], coords[k][1], c='gray', marker='s', s=100, alpha=0.5, label='Potential Depot' if k==K[0] else "")
            
    if selected_k:
        plt.plot([data["main_depot_coords"][0], coords[selected_k][0]],
                 [data["main_depot_coords"][1], coords[selected_k][1]], 'k--', linewidth=2, label='Truck Route', alpha=0.6)
                 
    colors = ['green', 'orange', 'purple', 'cyan']
    for p in P:
        for (i, j, p_idx), var in x.items():
            if p_idx == p and var.X > 0.5:
                # Line
                plt.plot([coords[i][0], coords[j][0]], [coords[i][1], coords[j][1]], 
                         c=colors[p % len(colors)], lw=2, zorder=3, label=f"Personnel {p}" if (i==selected_k or j==selected_k) else "")
                
                # Arrow
                mid_x = (coords[i][0] + coords[j][0]) / 2
                mid_y = (coords[i][1] + coords[j][1]) / 2
                plt.annotate('', xy=(mid_x, mid_y), xytext=(coords[i][0], coords[i][1]),
                             arrowprops=dict(arrowstyle='->', color=colors[p % len(colors)], lw=2), zorder=3)
                         
    plt.title("VRPTW Solution (Time Windows)", fontsize=14)
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
