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

### Scenario 4: Heterogeneous Fleet VRP
**File:** `projectv4.py`
*   **Description**: Introduces different types of personnel/vehicles (e.g., Cargo Bike vs. E-Scooter) with different capacities and costs.
*   **Objective**: Minimize Total Cost (Setup + Truck + Weighted Distance).
*   **Key Features**:
    *   **Cargo Bike**: High Capacity (100), High Cost (1.5/km).
    *   **E-Scooter**: Low Capacity (40), Low Cost (1.0/km).
    *   Capacity constraints included.

### Scenario 5: Multi-Objective Optimization
**File:** `projectv5.py`
*   **Description**: Balances **Operational Cost** vs. **Workload Balance** (Social Fairness) using the Weighted Sum Method.
*   **Objective**: Minimize $\alpha \cdot Z_1 + (1 - \alpha) \cdot Z_2$.
*   **Key Features**:
    *   Pareto Frontier analysis.
    *   $Z_1$: Total Cost.
    *   $Z_2$: Workload Imbalance ($W_{max} - W_{min}$).

### Scenario 6: Green VRP (Load-Dependent Energy)
**File:** `projectv6.py`
*   **Description**: Models Electric Vehicle (EV) physics where energy consumption depends on distance and carried load.
*   **Key Features**:
    *   Battery State of Charge (SoC) tracking.
    *   Energy consumption formula: $E = Dist \times (Base + Factor \times Load)$.
    *   Ensures vehicles return/finish with non-negative battery.

### Scenario 7: VRPTW (Time Windows)
**File:** `projectv7.py`
*   **Description**: Adds temporal constraints where customers must be visited within specific time windows.
*   **Key Features**:
    *   Time propagation logic.
    *   Service times at customers.
    *   Hard time window constraints.

## Results Summary

| Scenario | Total Cost | Selected Depot(s) | Truck Cost | Notes |
| :--- | :--- | :--- | :--- | :--- |
| **Scenario 1** | **564.18** | 100 | 300.00 | High cost due to personnel return trips and single depot constraint. |
| **Scenario 2** | **477.78** | 100 | 300.00 | **~15% improvement** over Scenario 1 by eliminating return trips. |
| **Scenario 3** | **273.13** | 100, 101 | 176.16 | **~52% improvement** over Scenario 1. The truck efficiently connects multiple depots. |
| **Scenario 4** | **820.71** | 100 | 300.00 | Heterogeneous Fleet (Total Weighted Cost). Includes capacity constraints. |
| **Scenario 5** | *Pareto* | 100 | 300.00 | Trade-off between Cost (685-1141) and Balance (0-342). |
| **Scenario 6** | **640.93** | 100 | 300.00 | Green VRP. Energy consumption limits route length based on load. |
| **Scenario 7** | **801.65** | 100 | 300.00 | VRPTW. Feasible routes found within strict time windows. |

## Output Files
*   **Text Output**: Detailed solver logs and route descriptions are saved in `output_project*.txt`.
*   **Visualizations**: Route plots are saved as `plot_project*.png`. The latest versions include **grids, directional arrows, and clear labels** for better readability.

## Requirements
*   Python 3.x
*   `gurobipy` (Gurobi Optimizer)
*   `matplotlib`

