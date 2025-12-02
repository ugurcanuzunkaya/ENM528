import gurobipy as gp
from gurobipy import GRB
import random
import math
import matplotlib.pyplot as plt
import sys
from typing import Dict, Any


def generate_data() -> Dict[str, Any]:
    print("--- Generating Data (Green VRP) ---")
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
    demands = {}
    for i in customers:
        coords[i] = (random.randint(0, 100), random.randint(0, 100))
        demands[i] = random.randint(5, 15)

    dist_matrix = {}
    for i in nodes:
        for j in nodes:
            d = math.sqrt(
                (coords[i][0] - coords[j][0]) ** 2 + (coords[i][1] - coords[j][1]) ** 2
            )
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
        "demands": demands,
        "big_m": 1000,
        "battery_capacity": 5000.0,
        "base_rate": 0.01,  # Energy per km empty
        "load_factor": 0.0001,  # Additional energy per km per kg
    }


def solve_mobile_vrp():
    original_stdout = sys.stdout
    with open("output_projectv6.txt", "w") as f:
        sys.stdout = f
        try:
            _solve_internal()
        finally:
            sys.stdout = original_stdout
    print("Output saved to output_projectv6.txt")


def _solve_internal():
    data = generate_data()

    N = data["customers"]
    K = data["candidates"]
    P = data["personnel"]
    V = data["nodes"]
    d_ij = data["dist_matrix"]
    demands = data["demands"]
    BigM = data["big_m"]
    BatCap = data["battery_capacity"]
    BaseRate = data["base_rate"]
    LoadFactor = data["load_factor"]

    print("--- Building Model (Green VRP) ---")
    m = gp.Model("Mobile_VRP_V6")
    m.setParam("TimeLimit", 300)  # 5-minute time limit

    # Variables
    y = m.addVars(K, vtype=GRB.BINARY, name="y")

    valid_arcs = []
    for p in P:
        for i in N:
            for j in N:
                if i != j:
                    valid_arcs.append((i, j, p))
        for k in K:
            for j in N:
                valid_arcs.append((k, j, p))
                valid_arcs.append((j, k, p))

    x = m.addVars(valid_arcs, vtype=GRB.BINARY, name="x")
    u = m.addVars(N, P, vtype=GRB.CONTINUOUS, lb=0, ub=len(N), name="u")

    # Green VRP Variables
    # Load at node i for personnel p
    load_var = m.addVars(V, P, vtype=GRB.CONTINUOUS, lb=0, ub=200, name="load")
    # SoC at node i for personnel p
    soc_var = m.addVars(V, P, vtype=GRB.CONTINUOUS, lb=0, ub=BatCap, name="soc")

    # Objective: Minimize Total Distance (Standard)
    cost_setup = gp.quicksum(data["setup_costs"][k] * y[k] for k in K)
    cost_truck = gp.quicksum(2 * data["main_depot_dists"][k] * y[k] for k in K)
    cost_routing = gp.quicksum(d_ij[i, j] * x[i, j, p] for (i, j, p) in valid_arcs)

    m.setObjective(cost_setup + cost_truck + cost_routing, GRB.MINIMIZE)

    # Standard Constraints
    m.addConstr(gp.quicksum(y[k] for k in K) == 1)
    for i in N:
        m.addConstr(
            gp.quicksum(x[j, i, p] for p in P for j in V if (j, i, p) in valid_arcs)
            == 1
        )
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
    for p in P:
        for i in N:
            for j in N:
                if i != j:
                    m.addConstr(u[i, p] - u[j, p] + BigM * x[i, j, p] <= BigM - 1)

    # --- Green VRP Constraints ---

    for p in P:
        # 1. Load Conservation
        for i, j, p_val in valid_arcs:
            if p_val != p:
                continue

            demand_i = demands[i] if i in N else 0

            # If j is a depot, we don't need to propagate load into it (it's the end of route)
            # We just ensure the load before returning is valid (handled by lb=0 on load_var[i])
            if j in K:
                continue

            # If i is a depot, load_var[i] is the starting load
            # If i is a customer, load_var[i] is the load arriving at i

            m.addConstr(
                load_var[j, p] <= load_var[i, p] - demand_i + BigM * (1 - x[i, j, p]),
                name=f"LoadProp_{i}_{j}_{p}",
            )
            m.addConstr(
                load_var[j, p] >= load_var[i, p] - demand_i - BigM * (1 - x[i, j, p]),
                name=f"LoadProp2_{i}_{j}_{p}",
            )

        # 2. Battery Logic (SoC)
        for i, j, p_val in valid_arcs:
            if p_val != p:
                continue

            demand_i = demands[i] if i in N else 0
            dist = d_ij[i, j]

            # Energy consumption depends on load
            # If i is depot, load is load_var[i] (start load)
            # If i is customer, load is load_var[i] (arrival load)
            # Load on arc (i,j) is load_var[i] - demand_i

            energy_consumption = dist * BaseRate + dist * LoadFactor * (
                load_var[i, p] - demand_i
            )

            if i in K:
                # Leaving depot: Start with full battery
                m.addConstr(
                    soc_var[j, p]
                    <= BatCap - energy_consumption + BigM * (1 - x[i, j, p]),
                    name=f"SoCProp_Start_{i}_{j}_{p}",
                )
            else:
                # Normal propagation
                m.addConstr(
                    soc_var[j, p]
                    <= soc_var[i, p] - energy_consumption + BigM * (1 - x[i, j, p]),
                    name=f"SoCProp_{i}_{j}_{p}",
                )

        # Initial SoC at Depot (Redundant if we use BatCap in propagation, but harmless)
        # We can remove the loop "Initial SoC at Depot" as it's handled above.

    print("--- Solving ---")
    m.optimize()

    if m.SolCount > 0:
        print(f"\nBest Solution Found (Obj: {m.ObjVal:.5f})")

        for p in P:
            print(f"\nPersonnel {p}:")
            start_node = None
            for k in K:
                if y[k].X > 0.5:
                    start_node = k

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
            while True:
                next_node = None
                for j in V:
                    if (curr, j, p) in valid_arcs and x[curr, j, p].X > 0.5:
                        next_node = j
                        dist += d_ij[curr, j]
                        break
                if next_node is None:
                    break
                route.append(next_node)
                curr = next_node
                if curr == start_node:
                    break

            print(f"  Path: {' -> '.join(map(str, route))}")
            print(f"  Distance: {dist:.5f}")
            # Print SoC
            print(f"  Start SoC: {BatCap:.2f}")
            print(f"  End SoC: {soc_var[start_node, p].X:.2f}")

        plot_solution(data, m, x, y, "plot_projectv6.png")


def plot_solution(data, model, x, y, save_path):
    coords = data["coords"]
    N = data["customers"]
    K = data["candidates"]
    P = data["personnel"]

    plt.figure(figsize=(12, 10))
    plt.grid(True, linestyle="--", alpha=0.7)

    # Plot Main Depot
    plt.scatter(
        data["main_depot_coords"][0],
        data["main_depot_coords"][1],
        c="black",
        marker="^",
        s=200,
        label="Main Depot",
        zorder=5,
    )
    plt.text(
        data["main_depot_coords"][0],
        data["main_depot_coords"][1] + 3,
        "Main",
        ha="center",
        fontsize=10,
        fontweight="bold",
    )

    # Plot Customers
    for i in N:
        plt.scatter(
            coords[i][0], coords[i][1], c="blue", s=100, zorder=4, edgecolors="white"
        )
        plt.text(
            coords[i][0],
            coords[i][1] + 3,
            f"C{i}",
            ha="center",
            fontsize=9,
            fontweight="bold",
            bbox=dict(facecolor="white", alpha=0.7, edgecolor="none", pad=1),
        )

    # Plot Depots
    selected_k = None
    for k in K:
        if y[k].X > 0.5:
            selected_k = k
            plt.scatter(
                coords[k][0],
                coords[k][1],
                c="red",
                marker="s",
                s=150,
                label=f"Active Depot {k}",
                zorder=5,
                edgecolors="black",
            )
        else:
            plt.scatter(
                coords[k][0],
                coords[k][1],
                c="gray",
                marker="s",
                s=100,
                alpha=0.5,
                label="Potential Depot" if k == K[0] else "",
            )

    if selected_k:
        plt.plot(
            [data["main_depot_coords"][0], coords[selected_k][0]],
            [data["main_depot_coords"][1], coords[selected_k][1]],
            "k--",
            linewidth=2,
            label="Truck Route",
            alpha=0.6,
        )

    colors = ["green", "orange", "purple", "cyan"]
    for p in P:
        for (i, j, p_idx), var in x.items():
            if p_idx == p and var.X > 0.5:
                # Line
                plt.plot(
                    [coords[i][0], coords[j][0]],
                    [coords[i][1], coords[j][1]],
                    c=colors[p % len(colors)],
                    lw=2,
                    zorder=3,
                    label=f"Personnel {p}"
                    if (i == selected_k or j == selected_k)
                    else "",
                )

                # Arrow
                mid_x = (coords[i][0] + coords[j][0]) / 2
                mid_y = (coords[i][1] + coords[j][1]) / 2
                plt.annotate(
                    "",
                    xy=(mid_x, mid_y),
                    xytext=(coords[i][0], coords[i][1]),
                    arrowprops=dict(
                        arrowstyle="->", color=colors[p % len(colors)], lw=2
                    ),
                    zorder=3,
                )

    plt.title("Green VRP Solution (Load Dependent Energy)", fontsize=14)
    plt.xlabel("X Coordinate")
    plt.ylabel("Y Coordinate")

    # Fix legend duplicates
    handles, labels = plt.gca().get_legend_handles_labels()
    by_label = dict(zip(labels, handles))
    plt.legend(
        by_label.values(), by_label.keys(), loc="upper right", bbox_to_anchor=(1.15, 1)
    )

    plt.tight_layout()
    plt.savefig(save_path)


if __name__ == "__main__":
    solve_mobile_vrp()
