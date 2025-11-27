import gurobipy as gp
from gurobipy import GRB
import random
import math
import matplotlib.pyplot as plt
import sys
from typing import Dict, Any, Tuple

def generate_data() -> Dict[str, Any]:
    """
    Generates data for the Mobile VRP problem (Open VRP variant).
    """
    print("--- Generating Data ---")
    
    num_customers = 10
    num_personnel = 2
    
    # Sets
    # Customers: 1 to 10
    customers = list(range(1, num_customers + 1))
    
    # Candidates: 100, 101, 102
    candidates = [100, 101, 102]
    
    # Sink Node (Virtual end point for Open VRP)
    sink = 999
    
    # All nodes
    nodes = customers + candidates + [sink]
    
    # Parameters
    # Setup costs for candidates
    setup_costs = {100: 20, 101: 10, 102: 30}
    
    # Distances from Main Depot to Candidates
    main_depot_dists = {100: 150, 101: 250, 102: 200}
    
    # Coordinates
    coords: Dict[int, Tuple[int, int]] = {}
    
    # Main Depot Coordinate
    main_depot_coords = (50, 150)
    
    # Fixed locations for candidates
    coords[100] = (20, 80)
    coords[101] = (50, 80)
    coords[102] = (80, 20)
    
    # Random locations for customers
    random.seed(42)
    for i in customers:
        coords[i] = (random.randint(0, 100), random.randint(0, 100))
        
    # Sink coordinate (just for plotting logic, though we won't plot it)
    coords[sink] = (0, 0) 
        
    # Calculate Euclidean Distance Matrix
    dist_matrix = {}
    for i in nodes:
        for j in nodes:
            if i == sink or j == sink:
                dist_matrix[i, j] = 0.0
            else:
                d = math.sqrt((coords[i][0]-coords[j][0])**2 + (coords[i][1]-coords[j][1])**2)
                dist_matrix[i, j] = d

    return {
        "customers": customers,
        "candidates": candidates,
        "sink": sink,
        "personnel": list(range(num_personnel)),
        "nodes": nodes,
        "setup_costs": setup_costs,
        "main_depot_dists": main_depot_dists,
        "main_depot_coords": main_depot_coords,
        "coords": coords,
        "dist_matrix": dist_matrix,
        "max_route_length": 600,
        "big_m": len(customers) + 10
    }

def solve_mobile_vrp():
    # Redirect stdout to file
    original_stdout = sys.stdout
    with open("output_projectv2.txt", "w") as f:
        sys.stdout = f
        try:
            _solve_mobile_vrp_internal()
        finally:
            sys.stdout = original_stdout
    print("Output saved to output_projectv2.txt")

def _solve_mobile_vrp_internal():
    # 1. Get Data
    data = generate_data()
    
    N = data["customers"]
    K = data["candidates"]
    Sink = data["sink"]
    P = data["personnel"]
    V = data["nodes"]
    d_ij = data["dist_matrix"]
    setup_costs = data["setup_costs"]
    main_depot_dists = data["main_depot_dists"]
    L_p = data["max_route_length"]
    BigM = data["big_m"]
    
    # 2. Model Formulation
    print("--- Building Model (Open VRP) ---")
    m = gp.Model("Mobile_VRP_Optimization_V2")
    
    # --- Decision Variables ---
    
    # y[k]: 1 if candidate mobile station 'k' is selected
    y = m.addVars(K, vtype=GRB.BINARY, name="y")
    
    # x[i,j,p]: 1 if personnel 'p' goes from i to j
    valid_arcs = []
    for p in P:
        # Customer to Customer
        for i in N:
            for j in N:
                if i != j:
                    valid_arcs.append((i, j, p))
        
        # Candidate to Customer (Start of route)
        for k in K:
            for j in N:
                valid_arcs.append((k, j, p))
                
        # Customer to Sink (End of route - Open VRP)
        for i in N:
            valid_arcs.append((i, Sink, p))

    x = m.addVars(valid_arcs, vtype=GRB.BINARY, name="x")
    
    # u[i,p]: Auxiliary variable for Subtour Elimination (MTZ)
    u = m.addVars(N, P, vtype=GRB.CONTINUOUS, lb=0, ub=len(N), name="u")
    
    # Z_max: Variable for Minimax
    Z_max = m.addVar(vtype=GRB.CONTINUOUS, name="Z_max")

    # --- Objective Function ---
    # Min Z = Setup Cost + Truck Round Trip + Max Route Length (Minimax)
    cost_setup = gp.quicksum(setup_costs[k] * y[k] for k in K)
    
    # Truck goes Main -> Candidate -> Main (Round Trip = 2 * distance)
    cost_main_dist = gp.quicksum(2 * main_depot_dists[k] * y[k] for k in K)
    
    m.setObjective(cost_setup + cost_main_dist + Z_max, GRB.MINIMIZE)

    # --- Constraints ---

    # 1. Single Depot Selection
    m.addConstr(gp.quicksum(y[k] for k in K) == 1, name="Single_Depot_Select")

    # 2. Customer Assignment
    for i in N:
        m.addConstr(gp.quicksum(x[j, i, p] for p in P for j in V if (j, i, p) in valid_arcs) == 1, 
                    name=f"Visit_Customer_{i}")

    # 3. Flow Conservation at Customers
    # Inflow = Outflow (Flow enters from somewhere, and leaves to somewhere - potentially Sink)
    for p in P:
        for i in N:
            inflow = gp.quicksum(x[j, i, p] for j in V if (j, i, p) in valid_arcs)
            outflow = gp.quicksum(x[i, j, p] for j in V if (i, j, p) in valid_arcs)
            m.addConstr(inflow == outflow, name=f"Flow_Balance_{i}_{p}")

    # 4. Depot Logic (Start)
    for p in P:
        # Vehicle can only leave a candidate k if k is selected
        for k in K:
            outflow_k = gp.quicksum(x[k, j, p] for j in N)
            m.addConstr(outflow_k <= y[k], name=f"Depot_Usage_{k}_{p}")

        # Force Start: Must start exactly once from one of the candidates
        m.addConstr(gp.quicksum(x[k, j, p] for k in K for j in N) == 1, name=f"Force_Start_{p}")

    # 5. Sink Logic (End)
    for p in P:
        # Force End: Must go to Sink exactly once (Open VRP)
        m.addConstr(gp.quicksum(x[i, Sink, p] for i in N) == 1, name=f"Force_End_{p}")

    # 6. Max Route Length & Minimax Constraint
    for p in P:
        # Filter arcs for THIS personnel p
        my_arcs = [(i, j) for (i, j, p_val) in valid_arcs if p_val == p]
        # Note: d_ij to Sink is 0, so it doesn't add to cost
        route_len = gp.quicksum(d_ij[i, j] * x[i, j, p] for (i, j) in my_arcs)
        
        m.addConstr(route_len <= L_p, name=f"Max_Len_{p}")
        m.addConstr(Z_max >= route_len, name=f"Minimax_Def_{p}")

    # 7. Subtour Elimination (MTZ)
    for p in P:
        for i in N:
            for j in N:
                if i != j:
                    m.addConstr(
                        u[i, p] - u[j, p] + BigM * x[i, j, p] <= BigM - 1,
                        name=f"MTZ_{i}_{j}_{p}"
                    )

    # 3. Solve
    print("--- Solving Model ---")
    m.optimize()

    # 4. Output & Visualization
    if m.status == GRB.OPTIMAL:
        print("\nOptimal Solution Found!")
        print(f"Total Objective Cost: {m.ObjVal:.2f}")
        
        selected_k = None
        for k in K:
            if y[k].X > 0.5:
                selected_k = k
                print(f"Selected Candidate Mobile Depot: {k}")
                print(f"  Setup Cost: {setup_costs[k]}")
                print(f"  Truck Round Trip Cost: {2 * main_depot_dists[k]}")
                break
        
        print("\nRoutes (Open VRP):")
        for p in P:
            print(f"Personnel {p}:")
            # Find start
            start_node = selected_k
            
            route = [start_node]
            curr = start_node
            dist_p = 0
            
            while True:
                next_node = None
                # Look for next node
                for j in V:
                    if (curr, j, p) in valid_arcs and x[curr, j, p].X > 0.5:
                        next_node = j
                        dist_p += d_ij[curr, j]
                        break
                
                if next_node is None or next_node == Sink:
                    break
                
                route.append(next_node)
                curr = next_node
            
            print(f"  Path: {' -> '.join(map(str, route))}")
            print(f"  Distance: {dist_p:.2f}")

        # Visualization
        plot_solution(data, m, x, y, "plot_projectv2.png")

    else:
        print("No optimal solution found.")

def plot_solution(data, model, x, y, save_path=None):
    coords = data["coords"]
    main_depot_coords = data["main_depot_coords"]
    N = data["customers"]
    K = data["candidates"]
    P = data["personnel"]
    Sink = data["sink"]
    
    plt.figure(figsize=(10, 10))
    
    # Plot Main Depot
    plt.scatter(main_depot_coords[0], main_depot_coords[1], c='black', marker='^', s=250, label='Main Depot', zorder=4)
    plt.text(main_depot_coords[0]+2, main_depot_coords[1], "Main Depot", fontsize=12, fontweight='bold')
    
    # Plot Customers
    for i in N:
        plt.scatter(coords[i][0], coords[i][1], c='blue', s=100, zorder=2)
        plt.text(coords[i][0]+1, coords[i][1]+1, f"C{i}", fontsize=12)
        
    # Plot Candidates
    selected_k = None
    for k in K:
        is_selected = y[k].X > 0.5
        if is_selected:
            selected_k = k
            
        color = 'red' if is_selected else 'gray'
        size = 200 if is_selected else 100
        label = f"Depot {k}" if is_selected else f"Candidate {k}"
        plt.scatter(coords[k][0], coords[k][1], c=color, marker='s', s=size, label=label, zorder=3)
        plt.text(coords[k][0]+1, coords[k][1]+1, f"D{k}", fontsize=10)

    # Plot Truck Route (Main Depot <-> Selected Candidate)
    if selected_k is not None:
        plt.plot([main_depot_coords[0], coords[selected_k][0]], 
                 [main_depot_coords[1], coords[selected_k][1]], 
                 c='black', lw=3, linestyle='--', zorder=1, label='Truck Route')

    # Plot Routes
    colors = ['green', 'orange', 'purple']
    for p in P:
        # Collect arcs
        for (i, j, p_idx), var in x.items():
            if p_idx == p and var.X > 0.5:
                # Don't plot arcs to Sink
                if j == Sink:
                    continue
                plt.plot([coords[i][0], coords[j][0]], [coords[i][1], coords[j][1]], 
                         c=colors[p % len(colors)], lw=2, zorder=1)

    plt.title("Optimal Mobile VRP Routes (Open VRP)")
    plt.legend()
    plt.grid(True)
    if save_path:
        plt.savefig(save_path)
        print(f"Plot saved to {save_path}")
    # plt.show()

if __name__ == "__main__":
    solve_mobile_vrp()
