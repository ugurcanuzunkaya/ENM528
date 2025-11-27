import gurobipy as gp
from gurobipy import GRB
import random
import math
import matplotlib.pyplot as plt

def solve_mobile_vrp():
    # =========================================================================
    # 1. DATA GENERATION (Sample Instance based on your notes)
    # =========================================================================
    print("--- Generating Data ---")
    
    # Sets sizes
    num_customers = 10
    num_candidates = 3
    num_personnel = 2
    
    # Sets
    N = list(range(1, num_customers + 1))       # Customers: [1, 2, ..., 10]
    A = list(range(num_candidates))             # Candidate Depots: [0, 1, 2]
    P = list(range(num_personnel))              # Personnel: [0, 1]
    O_prime = 0                                 # Main Depot Node Index (O')
    V = [O_prime] + N                           # All routing nodes: [0, 1, ..., 10]
    
    # Parameters
    # d_oa: Cost/Distance from Center to Candidate 'a' (Values from image)
    d_oa = {0: 20, 1: 10, 2: 30} 
    
    # Coordinates (Randomly generated for demo purposes)
    coords = {}
    coords[O_prime] = (50, 50) # The location of O' (The active mobile hub)
    
    # Generate random customer locations
    random.seed(42) # Fixed seed for reproducibility
    for i in N:
        coords[i] = (random.randint(0, 100), random.randint(0, 100))
        
    # Calculate Euclidean Distance Matrix (d_ij)
    d_ij = {}
    for i in V:
        for j in V:
            dist = math.sqrt((coords[i][0]-coords[j][0])**2 + (coords[i][1]-coords[j][1])**2)
            d_ij[i, j] = dist

    # Max Route Length (Lp) - Set high enough to be feasible for this random data
    L_p = 300 
    
    # Big M for MTZ
    BigM = len(N)

    # =========================================================================
    # 2. MODEL FORMULATION
    # =========================================================================
    print("--- Building Model ---")
    
    # Initialize Model
    m = gp.Model("Mobile_VRP_Optimization")
    
    # --- Decision Variables ---
    
    # y[a]: 1 if candidate mobile station 'a' is selected
    y = m.addVars(A, vtype=GRB.BINARY, name="y")
    
    # x[i,j,p]: 1 if personnel 'p' goes from i to j
    x = m.addVars(V, V, P, vtype=GRB.BINARY, name="x")
    
    # u[i,p]: Auxiliary variable for Subtour Elimination (MTZ)
    u = m.addVars(N, P, vtype=GRB.CONTINUOUS, lb=0, name="u")

    # --- Objective Function ---
    # Min Z = Sum(d_oa * y_a) + Sum(Sum(Sum(d_ij * x_ij^p)))
    
    cost_setup = gp.quicksum(d_oa[a] * y[a] for a in A)
    cost_routing = gp.quicksum(d_ij[i, j] * x[i, j, p] 
                               for p in P for i in V for j in V)
    
    m.setObjective(cost_setup + cost_routing, GRB.MINIMIZE)

    # --- Constraints ---

    # 1. Single Depot Selection (Tek Aday Nokta Seçimi)
    # Sum(y_a) = 1
    m.addConstr(gp.quicksum(y[a] for a in A) == 1, name="Single_Depot_Select")

    # 2. Customer Assignment (Müşteri Atama)
    # Each customer must be visited exactly once.
    # Sum(p) Sum(j in N U O') x[j,i,p] = 1 for all i in N
    for i in N:
        m.addConstr(gp.quicksum(x[j, i, p] for p in P for j in V) == 1, 
                    name=f"Visit_Customer_{i}")

    # 3. Flow Conservation (Akış Dengesi)
    # Inflow = Outflow for every customer and personnel
    # Sum(j) x[i,j,p] = Sum(j) x[j,i,p]
    for p in P:
        for i in N:
            m.addConstr(
                gp.quicksum(x[i, j, p] for j in V) == gp.quicksum(x[j, i, p] for j in V),
                name=f"Flow_Balance_{i}_{p}"
            )

    # 4. Route Start/End at O' (Validation)
    # If a vehicle is used, it must start and end at O' (Node 0)
    # This is implicitly handled by Flow Conservation + MTZ, 
    # but we ensure flow balance at O' specifically:
    for p in P:
        m.addConstr(
            gp.quicksum(x[O_prime, j, p] for j in N) == gp.quicksum(x[j, O_prime, p] for j in N),
            name=f"Depot_Balance_{p}"
        )

    # 5. Max Route Length (Maksimum Rota Uzunluğu)
    # Sum(d_ij * x_ij^p) <= Lp
    for p in P:
        route_len = gp.quicksum(d_ij[i, j] * x[i, j, p] for i in V for j in V)
        m.addConstr(route_len <= L_p, name=f"Max_Len_{p}")

    # 6. Subtour Elimination (MTZ Constraints)
    # u_i - u_j + |N|*x_ij <= |N| - 1
    # Note: Only defined for i, j in N (Customers), not O'
    for p in P:
        for i in N:
            for j in N:
                if i != j:
                    m.addConstr(
                        u[i, p] - u[j, p] + BigM * x[i, j, p] <= BigM - 1,
                        name=f"MTZ_{i}_{j}_{p}"
                    )

    # =========================================================================
    # 3. SOLVE AND OUTPUT
    # =========================================================================
    print("--- Solving Model ---")
    m.optimize()

    if m.status == GRB.OPTIMAL:
        print("\nOptimal Solution Found!")
        print(f"Total Objective Cost: {m.ObjVal:.2f}")
        
        # Print Selected Candidate
        for a in A:
            if y[a].X > 0.5:
                print(f"Selected Candidate Mobile Depot: A{a} (Cost: {d_oa[a]})")
        
        # Print Routes
        print("\nRoutes:")
        active_arcs = []
        for p in P:
            print(f"Personnel {p}:")
            route_str = f"{O_prime}"
            curr = O_prime
            dist_p = 0
            
            # Simple loop to reconstruct path for printing
            while True:
                next_node = None
                for j in V:
                    if x[curr, j, p].X > 0.5:
                        next_node = j
                        active_arcs.append((curr, j))
                        dist_p += d_ij[curr, j]
                        break
                
                if next_node is None:
                    break
                    
                route_str += f" -> {next_node}"
                curr = next_node
                if curr == O_prime:
                    break
            
            if route_str != f"{O_prime}":
                print(f"  Path: {route_str}")
                print(f"  Distance: {dist_p:.2f}")
            else:
                print("  (Unused)")

        # =====================================================================
        # 4. VISUALIZATION
        # =====================================================================
        plt.figure(figsize=(10, 8))
        
        # Plot Customers
        for i in N:
            plt.scatter(coords[i][0], coords[i][1], c='blue', s=100, zorder=2)
            plt.text(coords[i][0]+1, coords[i][1]+1, f"C{i}", fontsize=12)
            
        # Plot Depot (O')
        plt.scatter(coords[O_prime][0], coords[O_prime][1], c='red', marker='s', s=200, label='Depot (O\')', zorder=3)
        
        # Plot Routes
        colors = ['green', 'orange', 'purple']
        for p in P:
            p_arcs = [(i, j) for i in V for j in V if x[i, j, p].X > 0.5]
            for (i, j) in p_arcs:
                plt.plot([coords[i][0], coords[j][0]], [coords[i][1], coords[j][1]], 
                         c=colors[p % len(colors)], lw=2, zorder=1, label=f'Personnel {p}' if (i==O_prime) else "")

        plt.title("Optimal VRP Routes")
        plt.legend()
        plt.grid(True)
        plt.show()

    else:
        print("No optimal solution found.")
        # If infeasible, compute IIS to debug
        # m.computeIIS()
        # m.write("model.ilp")

if __name__ == "__main__":
    solve_mobile_vrp()
