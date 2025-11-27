# Mobile VRP Optimization Project

This project implements three variations of the Mobile Vehicle Routing Problem (Mobile VRP) using Gurobi Optimizer. The problem involves a main depot, candidate mobile depots (truck stops), and customers. A truck transports goods/personnel to candidate depots, and personnel distribute them to customers.

## Scenarios

### Scenario 1: Location-Routing Problem (LRP) with Minimax Objective
**File:** `project.py`
*   **Description**: A truck travels from the Main Depot to a **single** selected Candidate Depot and returns. Personnel start at the selected depot, visit customers, and **return** to the depot (Closed Loop).
*   **Objective**: Minimize (Setup Cost + Truck Round Trip Cost + Max Personnel Route Length).
*   **Key Constraints**:
    *   Single depot selection.
    *   Closed loop personnel routes (TSP-like).
    *   Minimax objective for personnel workload balancing.

### Scenario 2: Open Location-Routing Problem (Open LRP)
**File:** `projectv2.py`
*   **Description**: Similar to Scenario 1, but personnel **do not return** to the depot after visiting the last customer (Open Loop). This simulates scenarios where personnel might be picked up elsewhere or finish their shift at the last customer.
*   **Objective**: Minimize (Setup Cost + Truck Round Trip Cost + Max Personnel Route Length).
*   **Key Constraints**:
    *   Single depot selection.
    *   Open loop personnel routes (Path-like).
    *   Minimax objective.

### Scenario 3: Two-Echelon Open Location-Routing Problem (2E-Open LRP)
**File:** `projectv3.py`
*   **Description**: The truck performs a **TSP tour** visiting **one or more** selected Candidate Depots. At each selected depot, personnel are deployed to visit customers and **do not return** (Open Loop).
*   **Objective**: Minimize (Setup Costs + Truck Route Distance + Max Personnel Route Length).
*   **Key Constraints**:
    *   Multiple depot selection allowed.
    *   Truck routing modeled as TSP/VRP.
    *   Open loop personnel routes.
    *   Two-echelon synchronization.

## Results Summary

| Scenario | Total Cost | Selected Depot(s) | Truck Cost | Notes |
| :--- | :--- | :--- | :--- | :--- |
| **Scenario 1** | **584.18** | 100 | 300.00 | High cost due to personnel return trips and single depot constraint. |
| **Scenario 2** | **497.78** | 100 | 300.00 | **~15% improvement** over Scenario 1 by eliminating return trips. |
| **Scenario 3** | **303.13** | 100, 101 | 176.16 | **~48% improvement** over Scenario 1. The truck efficiently connects multiple depots, reducing the "last mile" distance for personnel. |

## Output Files
*   **Text Output**: Detailed solver logs and route descriptions are saved in `output_project.txt`, `output_projectv2.txt`, and `output_projectv3.txt`.
*   **Visualizations**: Route plots are saved as `plot_project.png`, `plot_projectv2.png`, and `plot_projectv3.png`.

## Requirements
*   Python 3.x
*   `gurobipy` (Gurobi Optimizer)
*   `matplotlib`

