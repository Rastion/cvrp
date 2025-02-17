import os
import sys
import math
import random
from qubots.base_problem import BaseProblem

def read_elem(filename):

    # Resolve relative path with respect to this module’s directory.
    if not os.path.isabs(filename):
        base_dir = os.path.dirname(os.path.abspath(__file__))
        filename = os.path.join(base_dir, filename)

    with open(filename) as f:
        return [str(elem) for elem in f.read().split()]

def compute_dist(xi, xj, yi, yj):
    exact_dist = math.sqrt((xi - xj)**2 + (yi - yj)**2)
    return int(math.floor(exact_dist + 0.5))

def compute_distance_matrix(customers_x, customers_y):
    nb_customers = len(customers_x)
    distance_matrix = [[None for _ in range(nb_customers)] for _ in range(nb_customers)]
    for i in range(nb_customers):
        distance_matrix[i][i] = 0
        for j in range(nb_customers):
            dist = compute_dist(customers_x[i], customers_x[j], customers_y[i], customers_y[j])
            distance_matrix[i][j] = dist
            distance_matrix[j][i] = dist
    return distance_matrix

def compute_distance_depots(depot_x, depot_y, customers_x, customers_y):
    nb_customers = len(customers_x)
    distance_depots = [None] * nb_customers
    for i in range(nb_customers):
        distance_depots[i] = compute_dist(depot_x, customers_x[i], depot_y, customers_y[i])
    return distance_depots

def read_input_cvrp(filename):
    """
    Reads a CVRP instance file in Augerat Set A format.
    Expected keywords:
      - DIMENSION: <value>
      - CAPACITY: <value>
      - EDGE_WEIGHT_TYPE: EUC_2D
      - NODE_COORD_SECTION: followed by one line per node (id, x, y)
      - DEMAND_SECTION: followed by one line per node (id, demand) (depot demand should be 0)
      - DEPOT_SECTION: depot id followed by -1
    Customer indices in the file are given as 2..n (with depot as node 1). Internally,
    customers are 0-indexed.
    """
    file_it = iter(read_elem(filename))
    nb_nodes = 0
    truck_capacity = 0
    while True:
        token = next(file_it)
        if token == "DIMENSION":
            next(file_it)  # Skip ":"
            nb_nodes = int(next(file_it))
            nb_customers = nb_nodes - 1
        elif token == "CAPACITY":
            next(file_it)  # Skip ":"
            truck_capacity = int(next(file_it))
        elif token == "EDGE_WEIGHT_TYPE":
            next(file_it)  # Skip ":"
            token = next(file_it)
            if token != "EUC_2D":
                print("Edge Weight Type " + token + " is not supported (only EUC_2D)")
                sys.exit(1)
        elif token == "NODE_COORD_SECTION":
            break

    customers_x = [None] * nb_customers
    customers_y = [None] * nb_customers
    depot_x = depot_y = 0
    for n in range(nb_nodes):
        node_id = int(next(file_it))
        if node_id != n + 1:
            print("Unexpected index in NODE_COORD_SECTION")
            sys.exit(1)
        if node_id == 1:
            depot_x = int(next(file_it))
            depot_y = int(next(file_it))
        else:
            # Customers: indices 2..n become 0-indexed.
            customers_x[node_id - 2] = int(next(file_it))
            customers_y[node_id - 2] = int(next(file_it))
    distance_matrix = compute_distance_matrix(customers_x, customers_y)
    distance_depots = compute_distance_depots(depot_x, depot_y, customers_x, customers_y)

    token = next(file_it)
    if token != "DEMAND_SECTION":
        print("Expected token DEMAND_SECTION")
        sys.exit(1)
    demands = [None] * nb_customers
    for n in range(nb_nodes):
        node_id = int(next(file_it))
        if node_id != n + 1:
            print("Unexpected index in DEMAND_SECTION")
            sys.exit(1)
        if node_id == 1:
            if int(next(file_it)) != 0:
                print("Depot demand should be 0")
                sys.exit(1)
        else:
            demands[node_id - 2] = int(next(file_it))
    token = next(file_it)
    if token != "DEPOT_SECTION":
        print("Expected token DEPOT_SECTION")
        sys.exit(1)
    depot_id = int(next(file_it))
    if depot_id != 1:
        print("Depot id should be 1")
        sys.exit(1)
    end_token = int(next(file_it))
    if end_token != -1:
        print("Expected -1 after depot id")
        sys.exit(1)
    return nb_customers, truck_capacity, distance_matrix, distance_depots, demands

def get_nb_trucks(filename):
    # Extract number of trucks from filename pattern: A-nXX-kNBTRUCKS.vrp.
    begin = filename.rfind("-k")
    if begin != -1:
        begin += 2
        end = filename.find(".", begin)
        return int(filename[begin:end])
    print("Error: nb_trucks could not be read from the file name. Provide it via command line.")
    sys.exit(1)

class CVRPProblem(BaseProblem):
    """
    Capacitated Vehicle Routing Problem (CVRP)

    A fleet of vehicles with uniform capacity must service all customers with known demand.
    Vehicles start and end their routes at a common depot. Each customer is visited exactly once.

    Instance Data:
      - nb_customers: number of customers (excluding the depot)
      - truck_capacity: capacity per vehicle
      - dist_matrix: 2D distance matrix among customers (0-indexed)
      - dist_depot: distance from the depot to each customer (0-indexed)
      - demands: demand for each customer (0-indexed)

    Candidate Solution:
      A dictionary with key "customersSequences" mapping to a list of lists (one per vehicle).
      Each inner list is a permutation of customer indices (0-indexed) representing the route of that truck.
      Together, these lists form a partition of {0, ..., nb_customers - 1}.

    Objective:
      For each truck route:
         - Compute the route distance as:
             depot to first customer + sum of distances between consecutive customers +
             distance from last customer back to depot.
         - Ensure that the total demand on the route does not exceed truck_capacity.
      Also count the number of trucks used (i.e. routes with at least one customer).
      The overall objective is lexicographically minimizing:
         (number of trucks used, total distance traveled).
      (For instance, overall = trucks_used * M + total_distance, with M large.)
    """
    def __init__(self, instance_file=None, nb_customers=None, truck_capacity=None,
                 dist_matrix=None, dist_depot=None, demands=None, nb_trucks=None):
        if instance_file is not None:
            self._load_instance(instance_file)
        else:
            if None in (nb_customers, truck_capacity, dist_matrix, dist_depot, demands, nb_trucks):
                raise ValueError("Either instance_file or all instance parameters must be provided.")
            self.nb_customers = nb_customers
            self.truck_capacity = truck_capacity
            self.dist_matrix = dist_matrix
            self.dist_depot = dist_depot
            self.demands = demands
            self.nb_trucks = nb_trucks

    def _load_instance(self, filename):
        (self.nb_customers, self.truck_capacity, self.dist_matrix,
         self.dist_depot, self.demands) = read_input_cvrp(filename)
        self.nb_trucks = get_nb_trucks(filename)

    def evaluate_solution(self, solution) -> float:
        PENALTY = 1e9
        if not isinstance(solution, dict) or "customersSequences" not in solution:
            return PENALTY
        routes = solution["customersSequences"]
        if not isinstance(routes, list) or len(routes) != self.nb_trucks:
            return PENALTY
        # Check that each customer is visited exactly once.
        visited = [False] * self.nb_customers
        for route in routes:
            if not isinstance(route, list):
                return PENALTY
            for cust in route:
                if not isinstance(cust, int) or cust < 0 or cust >= self.nb_customers:
                    return PENALTY
                if visited[cust]:
                    return PENALTY
                visited[cust] = True
        if not all(visited):
            return PENALTY

        total_distance = 0
        trucks_used = 0
        for route in routes:
            if not route:
                continue
            trucks_used += 1
            route_demand = sum(self.demands[cust] for cust in route)
            if route_demand > self.truck_capacity:
                return PENALTY
            route_distance = self.dist_depot[route[0]]
            for i in range(1, len(route)):
                route_distance += self.dist_matrix[route[i-1]][route[i]]
            route_distance += self.dist_depot[route[-1]]
            total_distance += route_distance

        M = 10000
        overall = trucks_used * M + total_distance
        return overall

    def random_solution(self):
        customers = list(range(self.nb_customers))
        random.shuffle(customers)
        routes = [[] for _ in range(self.nb_trucks)]
        for i, cust in enumerate(customers):
            routes[i % self.nb_trucks].append(cust)
        for route in routes:
            random.shuffle(route)
        return {"customersSequences": routes}

