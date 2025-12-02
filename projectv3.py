import gurobipy as gp
from gurobipy import GRB
import random
import math
import matplotlib.pyplot as plt
import sys
from typing import Dict, Any, Tuple


def generate_data() -> Dict[str, Any]:
    """
    Generates data for the Mobile VRP problem (Two-Echelon Open VRP).
    """
    print("--- Generating Data ---")

    num_customers = 10
    num_personnel_per_depot = 2

    # Sets
    customers = list(range(1, num_customers + 1))
    candidates = [100, 101, 102]
    main_depot = 0  # Main Depot ID for Truck
    sink = 999  # Virtual Sink for Personnel

    # Parameters
    setup_costs = {100: 0, 101: 0, 102: 0}

    # Coordinates
    coords: Dict[int, Tuple[int, int]] = {}
    coords[main_depot] = (50, 150)
    coords[100] = (20, 80)
    coords[101] = (50, 80)
    coords[102] = (80, 20)

    random.seed(42)
    for i in customers:
        coords[i] = (random.randint(0, 100), random.randint(0, 100))

    coords[sink] = (0, 0)  # Dummy

    # Distance Matrix (General)
    # We need distances for:
    # 1. Truck: Main <-> Candidates, Candidate <-> Candidate
    # 2. Personnel: Candidate -> Customer, Customer -> Customer, Customer -> Sink

    dist_matrix = {}
    all_nodes = customers + candidates + [main_depot, sink]

    for i in all_nodes:
        for j in all_nodes:
            if i == sink or j == sink:
                dist_matrix[i, j] = 0.0
            else:
                d = math.sqrt(
                    (coords[i][0] - coords[j][0]) ** 2
                    + (coords[i][1] - coords[j][1]) ** 2
                )
                dist_matrix[i, j] = d

    return {
        "customers": customers,
        "candidates": candidates,
        "main_depot": main_depot,
        "sink": sink,
        "num_personnel": num_personnel_per_depot,
        "setup_costs": setup_costs,
        "coords": coords,
        "dist_matrix": dist_matrix,
        "max_route_length": 600,
        "big_m": len(customers) + 10,
    }


def solve_mobile_vrp():
    # Redirect stdout to file
    original_stdout = sys.stdout
    with open("output_projectv3.txt", "w") as f:
        sys.stdout = f
        try:
            _solve_mobile_vrp_internal()
        finally:
            sys.stdout = original_stdout
    print("Output saved to output_projectv3.txt")


def _solve_mobile_vrp_internal():
    data = generate_data()

    N = data["customers"]
    K = data["candidates"]
    O = data["main_depot"]
    Sink = data["sink"]
    NumP = data["num_personnel"]
    d_ij = data["dist_matrix"]
    setup_costs = data["setup_costs"]
    L_p = data["max_route_length"]
    BigM = data["big_m"]

    # Personnel set: List of tuples (k, p_idx)
    # Personnel are specific to a candidate depot
    Personnel = [(k, p) for k in K for p in range(NumP)]

    print("--- Building Model (Two-Echelon Open VRP) ---")
    m = gp.Model("Mobile_VRP_Optimization_V3")

    # --- Decision Variables ---

    # y[k]: 1 if candidate mobile station 'k' is selected (visited by truck)
    y = m.addVars(K, vtype=GRB.BINARY, name="y")

    # z[u,v]: Truck route arcs (Main Depot + Candidates)
    # Truck nodes: T_nodes = {O} U K
    T_nodes = [O] + K
    z = m.addVars(T_nodes, T_nodes, vtype=GRB.BINARY, name="z")

    # u_truck[k]: MTZ variables for truck
    u_truck = m.addVars(K, vtype=GRB.CONTINUOUS, lb=1, ub=len(K), name="u_truck")

    # x[i,j,k,p]: Personnel route arcs
    # i, j in N U {k} U {Sink}
    # But personnel (k, p) only starts at k.
    # So valid arcs for personnel (k, p):
    #   k -> j (j in N)
    #   i -> j (i, j in N)
    #   i -> Sink (i in N)

    x = {}
    for k in K:
        for p in range(NumP):
            # Start arcs
            for j in N:
                x[k, j, k, p] = m.addVar(vtype=GRB.BINARY, name=f"x_{k}_{j}_{k}_{p}")
            # Internal arcs
            for i in N:
                for j in N:
                    if i != j:
                        x[i, j, k, p] = m.addVar(
                            vtype=GRB.BINARY, name=f"x_{i}_{j}_{k}_{p}"
                        )
            # End arcs
            for i in N:
                x[i, Sink, k, p] = m.addVar(
                    vtype=GRB.BINARY, name=f"x_{i}_Sink_{k}_{p}"
                )

    # u_pers[i]: MTZ variables for personnel (to prevent subtours among customers)
    # We can share u variables across personnel or have them per personnel.
    # Standard MTZ is usually per node.
    u_pers = m.addVars(N, vtype=GRB.CONTINUOUS, lb=1, ub=len(N), name="u_pers")

    # Z_max: Minimax variable
    Z_max = m.addVar(vtype=GRB.CONTINUOUS, name="Z_max")

    # --- Objective Function ---

    # 1. Setup Costs
    cost_setup = gp.quicksum(setup_costs[k] * y[k] for k in K)

    # 2. Truck Route Cost
    cost_truck = gp.quicksum(
        d_ij[u, v] * z[u, v] for u in T_nodes for v in T_nodes if u != v
    )

    # 3. Minimax Personnel Route
    m.setObjective(cost_setup + cost_truck + Z_max, GRB.MINIMIZE)

    # --- Constraints ---

    # --- TRUCK ROUTING ---

    # 1. Flow Balance for Truck
    # For Main Depot O: Outflow = 1, Inflow = 1 (Truck must leave and return)
    m.addConstr(gp.quicksum(z[O, k] for k in K) == 1, name="Truck_Start")
    m.addConstr(gp.quicksum(z[k, O] for k in K) == 1, name="Truck_End")

    # For Candidates k:
    # If y[k]=1, Inflow=1, Outflow=1. If y[k]=0, Inflow=0, Outflow=0.
    for k in K:
        m.addConstr(
            gp.quicksum(z[u, k] for u in T_nodes if u != k) == y[k],
            name=f"Truck_In_{k}",
        )
        m.addConstr(
            gp.quicksum(z[k, v] for v in T_nodes if v != k) == y[k],
            name=f"Truck_Out_{k}",
        )

    # 2. Truck Subtour Elimination (MTZ)
    # u_i - u_j + |K| * z_ij <= |K| - 1  for i, j in K
    for i in K:
        for j in K:
            if i != j:
                m.addConstr(
                    u_truck[i] - u_truck[j] + len(K) * z[i, j] <= len(K) - 1,
                    name=f"Truck_MTZ_{i}_{j}",
                )

    # --- PERSONNEL ROUTING ---

    # 3. Customer Assignment
    # Each customer visited exactly once by ONE personnel from ONE depot
    for i in N:
        m.addConstr(
            gp.quicksum(
                x[u, i, k, p]
                for k in K
                for p in range(NumP)
                for u in ([k] + N)
                if (u, i, k, p) in x
            )
            == 1,
            name=f"Visit_Cust_{i}",
        )

    # 4. Personnel Flow Balance
    for k in K:
        for p in range(NumP):
            # Start at Depot k: Outflow <= y[k] (Can only use if depot selected)
            # Also Outflow <= 1 (Single route per personnel)
            start_flow = gp.quicksum(x[k, j, k, p] for j in N)
            m.addConstr(start_flow <= y[k], name=f"Pers_Start_Valid_{k}_{p}")

            # Flow Conservation at Customers
            for j in N:
                inflow = gp.quicksum(
                    x[u, j, k, p] for u in ([k] + N) if (u, j, k, p) in x
                )
                outflow = gp.quicksum(
                    x[j, v, k, p] for v in (N + [Sink]) if (j, v, k, p) in x
                )
                m.addConstr(inflow == outflow, name=f"Pers_Flow_{j}_{k}_{p}")

            # End at Sink: Inflow = Start Flow (If started, must end)
            end_flow = gp.quicksum(x[i, Sink, k, p] for i in N)
            m.addConstr(end_flow == start_flow, name=f"Pers_End_{k}_{p}")

    # 5. Personnel Subtour Elimination (MTZ)
    # u_i - u_j + BigM * x_ij <= BigM - 1
    # This needs to hold for ANY active arc (i, j) used by ANY personnel
    # We can aggregate x variables for the constraint or add for each p
    # Aggregating is cleaner: sum(x_ij_kp)
    for i in N:
        for j in N:
            if i != j:
                sum_x = gp.quicksum(x[i, j, k, p] for k in K for p in range(NumP))
                m.addConstr(
                    u_pers[i] - u_pers[j] + BigM * sum_x <= BigM - 1,
                    name=f"Pers_MTZ_{i}_{j}",
                )

    # 6. Max Route Length & Minimax
    for k in K:
        for p in range(NumP):
            # Calculate length
            # Arcs: (k, j), (i, j)
            # (i, Sink) has 0 cost
            route_len = 0
            for j in N:
                route_len += d_ij[k, j] * x[k, j, k, p]
            for i in N:
                for j in N:
                    if i != j:
                        route_len += d_ij[i, j] * x[i, j, k, p]

            m.addConstr(route_len <= L_p, name=f"Max_Len_{k}_{p}")
            m.addConstr(Z_max >= route_len, name=f"Minimax_{k}_{p}")

    # 3. Solve
    print("--- Solving Model ---")
    m.optimize()

    # 4. Output & Visualization
    if m.status == GRB.OPTIMAL:
        print("\nOptimal Solution Found!")
        print(f"Total Objective Cost: {m.ObjVal:.5f}")

        # Truck Route
        print("\nTruck Route:")
        curr = O
        truck_path = [O]
        truck_dist = 0
        while True:
            next_node = None
            for v in T_nodes:
                if v != curr and z[curr, v].X > 0.5:
                    next_node = v
                    break

            if next_node is None:
                break

            truck_path.append(next_node)
            truck_dist += d_ij[curr, next_node]
            curr = next_node
            if curr == O:
                break
        print(f"  Path: {' -> '.join(map(str, truck_path))}")
        print(f"  Distance: {truck_dist:.5f}")

        # Personnel Routes
        print("\nPersonnel Routes:")
        for k in K:
            if y[k].X > 0.5:
                print(f"Depot {k} (Selected):")
                for p in range(NumP):
                    # Reconstruct path
                    # Start from k
                    path = [k]
                    curr = k
                    dist = 0
                    started = False

                    # Check start
                    for j in N:
                        if x[k, j, k, p].X > 0.5:
                            curr = j
                            path.append(j)
                            dist += d_ij[k, j]
                            started = True
                            break

                    if not started:
                        print(f"  Personnel {p}: Unused")
                        continue

                    while True:
                        next_node = None
                        # Check internal
                        for j in N:
                            if curr != j and x[curr, j, k, p].X > 0.5:
                                next_node = j
                                break

                        if next_node:
                            path.append(next_node)
                            dist += d_ij[curr, next_node]
                            curr = next_node
                        else:
                            # Check sink
                            if x[curr, Sink, k, p].X > 0.5:
                                # End
                                break
                            else:
                                break

                    print(
                        f"  Personnel {p}: {' -> '.join(map(str, path))} (Dist: {dist:.5f})"
                    )

        plot_solution(data, m, x, z, y, "plot_projectv3.png")

    else:
        print("No optimal solution found.")


def plot_solution(data, model, x, z, y, save_path=None):
    coords = data["coords"]
    N = data["customers"]
    K = data["candidates"]
    O = data["main_depot"]
    Sink = data["sink"]
    NumP = data["num_personnel"]

    plt.figure(figsize=(10, 10))

    # Plot Main Depot
    plt.scatter(
        coords[O][0],
        coords[O][1],
        c="black",
        marker="^",
        s=250,
        label="Main Depot",
        zorder=5,
    )

    # Plot Customers
    for i in N:
        plt.scatter(coords[i][0], coords[i][1], c="blue", s=100, zorder=3)
        plt.text(coords[i][0] + 1, coords[i][1] + 1, f"C{i}", fontsize=12)

    # Plot Candidates
    for k in K:
        is_selected = y[k].X > 0.5
        color = "red" if is_selected else "gray"
        size = 200 if is_selected else 100
        label = f"Depot {k}" if is_selected else f"Candidate {k}"
        plt.scatter(
            coords[k][0],
            coords[k][1],
            c=color,
            marker="s",
            s=size,
            label=label,
            zorder=4,
        )
        plt.text(coords[k][0] + 1, coords[k][1] + 1, f"D{k}", fontsize=10)

    # Plot Truck Route
    # z[u,v]
    T_nodes = [O] + K
    for u in T_nodes:
        for v in T_nodes:
            if u != v and z[u, v].X > 0.5:
                plt.plot(
                    [coords[u][0], coords[v][0]],
                    [coords[u][1], coords[v][1]],
                    c="black",
                    lw=3,
                    linestyle="--",
                    zorder=2,
                    label="Truck Route" if u == O else "",
                )

    # Plot Personnel Routes
    colors = ["green", "orange", "purple", "cyan", "magenta"]
    color_idx = 0

    for k in K:
        if y[k].X > 0.5:
            for p in range(NumP):
                # Collect arcs
                arcs = []
                # Start
                for j in N:
                    if x[k, j, k, p].X > 0.5:
                        arcs.append((k, j))
                # Internal
                for i in N:
                    for j in N:
                        if i != j and x[i, j, k, p].X > 0.5:
                            arcs.append((i, j))

                if not arcs:
                    continue

                c = colors[color_idx % len(colors)]
                color_idx += 1

                for u, v in arcs:
                    plt.plot(
                        [coords[u][0], coords[v][0]],
                        [coords[u][1], coords[v][1]],
                        c=c,
                        lw=2,
                        zorder=1,
                    )

    plt.title("Optimal Mobile VRP Routes (Truck TSP + Open VRP)")
    # Deduplicate legend
    handles, labels = plt.gca().get_legend_handles_labels()
    by_label = dict(zip(labels, handles))
    plt.legend(by_label.values(), by_label.keys())

    plt.grid(True)
    if save_path:
        plt.savefig(save_path)
        print(f"Plot saved to {save_path}")
    # plt.show()


if __name__ == "__main__":
    solve_mobile_vrp()
