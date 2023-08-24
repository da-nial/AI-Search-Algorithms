from copy import deepcopy
from timeit import timeit

from GUI import visualize
from helper import read_input, manhattan_distance, is_in_goal, get_valid_movements, get_path_movements, perform_move, \
    get_duplicate, pprint_path
from node import Node


def get_closest_plate(point, plates_locs):
    """
    finds the closest plate to the given point

    :param point:
    :param plates_locs:
    :return:
    """
    closets_plate_loc = None
    closest_plate_distance = 1000000

    for plate_loc in plates_locs:
        distance = manhattan_distance(point, plate_loc)
        if distance < closest_plate_distance:
            closets_plate_loc = plate_loc
            closest_plate_distance = distance

    return closets_plate_loc, closest_plate_distance


def heuristic_1(point, plates_locs):
    """
    Calculate a heuristic value based on the distances from 'point' to a list of plate locations.

    The function calculates the total distance from the input 'point' to each plate in 'plates_locs'. It iterates
    through the plates, selecting the closest one in each step, and accumulates the distances as the heuristic value.

    :param point: Starting point coordinates (x, y).
    :param plates_locs: List of plate locations [(x1, y1), (x2, y2), ...].
    :return: Calculated heuristic value.
    """
    h = 0

    plates_locs_cpy = deepcopy(plates_locs)
    while plates_locs_cpy:
        closest_plate, closest_plate_distance = get_closest_plate(point, plates_locs_cpy)

        plates_locs_cpy.remove(closest_plate)
        h += closest_plate_distance

        point = closest_plate

    return h


def get_heuristic_matrix(objects, plates_locs, heuristic_func):
    """
    Computes a matrix of heuristic values by applying a given heuristic function to each object's position and a list of plate locations.

    :param objects: 2D list representing object layout.
    :param plates_locs: List of plate locations [(x1, y1), ...].
    :param heuristic_func: Function for object-plate heuristic calculation.
    :return: Matrix of heuristic values for each object and plate.
    """
    num_rows, num_cols = len(objects), len(objects[0])
    heuristic_matrix = [[heuristic_func((i, j), plates_locs) for j in range(num_cols)] for i in range(num_rows)]

    return heuristic_matrix


def update_frontier(frontier, explored, new_node):
    """
    Update frontier with new node if criteria met.

    :param frontier: A list containing nodes that are candidates for exploration.
    :type frontier: list

    :param explored: A set or list containing nodes that have already been explored.
    :type explored: set or list

    :param new_node: The new node that needs to be considered for inclusion in the frontier.
    :type new_node: Node class or a similar structure

    :return: None
    """
    if new_node not in explored:
        duplicate_node = get_duplicate(frontier, new_node)
        if duplicate_node is not None:
            if new_node.cost_f < duplicate_node.cost_f:
                frontier.remove(duplicate_node)
                frontier.append(new_node)
        else:
            frontier.append(new_node)

    frontier.sort(key=lambda x: x.cost_f)


def expand(node, costs, heuristic_matrix, frontier, explored, bookkeeping):
    """
    Expand a node by generating child nodes through valid movements.

    :param node: The node to be expanded.
    :type node: Node class or similar structure

    :param costs: Matrix of movement costs corresponding to each cell in the layout.
    :type costs: list of lists

    :param heuristic_matrix: Matrix of heuristic values for object-plate distances.
    :type heuristic_matrix: list of lists

    :param frontier: List of nodes to be considered for future exploration.
    :type frontier: list

    :param explored: List of nodes that have already been explored.
    :type explored: list

    :param bookkeeping: Dictionary for tracking various metrics.
    :type bookkeeping: dict

    :return: None
    """
    objects, robot_loc, depth, g, f = node.objects, node.robot_loc, node.depth, node.cost_g, node.cost_f

    valid_movements = get_valid_movements(objects, robot_loc)
    for movement in valid_movements:
        new_objects, new_robot_loc = perform_move(objects, robot_loc, movement)

        new_robot_x, new_robot_y = new_robot_loc
        new_g = g + costs[new_robot_x][new_robot_y]
        new_f = new_g + heuristic_matrix[new_robot_x][new_robot_y]

        child_node = Node(new_objects, new_robot_loc, depth + 1, movement, node, new_g, new_f)
        bookkeeping["nodes_created"] += 1
        update_frontier(frontier, explored, child_node)

    explored.append(node)


def get_path(node):
    """
    Retrieve the path from a node back to the starting point.

    :param node: The end node for which to extract the path.
    :return: A list representing the path from the starting point to the given node.
    """
    path = []

    while node != "":
        path.append(node)
        node = node.parent

    path.reverse()
    return path


def a_star(costs, heuristic_matrix, root_node, max_depth, bookkeeping):
    """
    Implementation of A* Alogorithm

    :param costs: the cost matrix of all cells
    :param heuristic_matrix: the calculated heuristic matrix
    :param root_node: the node which the search algorithm starts with
    :param max_depth: maximum allowed depth
    :param bookkeeping: a dictionary to store some info about algorithm while it is running
     such as number of nodes created and number of nodes expanded
    :type bookkeeping: dict

    :return:
    """
    frontier, explored = [root_node], []

    while frontier:
        expanding_node = frontier.pop(0)
        bookkeeping["nodes_expanded"] += 1

        if is_in_goal(expanding_node.objects):
            return expanding_node

        expand(expanding_node, costs, heuristic_matrix, frontier, explored, bookkeeping)

        if expanding_node.depth >= max_depth:
            break

    return None


def main(test_case_path, max_depth):
    max_depth = int(max_depth)
    costs, objects, robot_loc, plates_locs = read_input(test_case_path)
    bookkeeping = {
        "nodes_created": 0,
        "nodes_expanded": 0
    }
    robot_loc_x, robot_loc_y = robot_loc

    heuristic_matrix = get_heuristic_matrix(objects, plates_locs, heuristic_1)
    root_cost_g = 0
    root_cost_f = heuristic_matrix[robot_loc_x][robot_loc_y]

    root_node = Node(objects, robot_loc, 0, "", "", root_cost_g, root_cost_f)

    final_node = a_star(costs, heuristic_matrix, root_node, max_depth, bookkeeping)

    if final_node is not None:
        found_path = get_path(final_node)
        pprint_path(found_path)
        visualize(found_path)
        print(f"Path Movements: {get_path_movements(found_path)}")
        print("Depth Reached: ", len(found_path))
    else:
        print("can't pass the butter")

    print(f"Nodes Created: {bookkeeping['nodes_created']}, Nodes Expanded: {bookkeeping['nodes_expanded']}")


if __name__ == "__main__":
    setup = '''from __main__ import main'''
    statement = '''main(*sys.argv[1:])'''
    time = timeit(setup=setup, stmt=statement, number=1)
    print(f"Execution Time: {time}")
