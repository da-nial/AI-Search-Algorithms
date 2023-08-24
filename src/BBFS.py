from copy import deepcopy
from timeit import timeit

from GUI import visualize
from helper import read_input, get_valid_movements, get_valid_backward_movements, get_goal_states_nodes, perform_move, \
    perform_backward_move, get_path_movements, pprint_path, get_reverse_movement
from node import Node


def get_intersection(forward_explored, backward_explored):
    """
    Iterates through two given lists, and checks to see if there is two nodes with
    similar objects and robot location. if it found such two nodes, it returns them, otherwise returns None

    :param forward_explored:
    :type forward_explored: list
    :param backward_explored:
    :type backward_explored: list
    :return: a tuple consisting two nodes that CHAIN the forward and backward path
    :rtype: tuple
    """
    for forward_node in forward_explored:
        for backward_node in backward_explored:
            if forward_node == backward_node:
                return forward_node, backward_node.parent

    return None


def _bfs(cur_node, frontier, bookkeeping, direction="forward"):
    """
    Implementation of BFS (breadth first search) algorithm

    :param cur_node:
    :param frontier:
    :param direction:
    :return:
    """
    objects, robot_loc, depth = cur_node.objects, cur_node.robot_loc, cur_node.depth

    children = []
    if direction == "forward":
        valid_movements = get_valid_movements(objects, robot_loc)
        for movement in valid_movements:
            new_objects, new_robot_loc = perform_move(objects, robot_loc, movement)

            child_node = Node(new_objects, new_robot_loc, depth + 1, movement, cur_node)
            children.append(child_node)

    elif direction == "backward":
        valid_movements = get_valid_backward_movements(objects, robot_loc)
        for movement in valid_movements:
            new_objects_ls, new_robot_loc = perform_backward_move(objects, robot_loc, movement)

            for new_objects in new_objects_ls:
                child_node = Node(new_objects, new_robot_loc, depth + 1, movement, cur_node)
                children.append(child_node)

    children_not_in_frontier = [child for child in children if child not in frontier]
    bookkeeping["nodes_created"] += len(children_not_in_frontier)
    frontier += children_not_in_frontier
    return frontier


def normalize_path(intersection):
    """
    Concatenates lists of forward path and reverse of backward path, in order to output the
    final result path.

    :param intersection: a tuple consisting two nodes that CHAIN the forward and backward path
    :type intersection: tuple
    :return: concatenated list of forward path and backward path reversed
    :rtype: list
    """
    forward_node, backward_node = intersection

    path1, path2 = [], []
    forward_node_copy = deepcopy(forward_node)
    backward_node_copy = deepcopy(backward_node)

    while forward_node_copy.depth > 0:
        path1.insert(0, forward_node_copy)
        forward_node_copy = deepcopy(forward_node_copy.parent)

    while type(backward_node_copy) != str and backward_node_copy.depth >= 0:
        backward_node_copy.movement = get_reverse_movement(backward_node_copy.movement)
        path2.append(backward_node_copy)
        backward_node_copy = deepcopy(backward_node_copy.parent)

    concatenated_path = path1 + path2
    return concatenated_path


def bbfs(forward_frontier, backward_frontier, bookkeeping):
    """
    Implementation of BBFS (bidirectional Breadth first search) algorithm.

    :param forward_frontier: the initial list of frontier in forward direction
    :type forward_frontier: list
    :param backward_frontier: the initial list of frontier in backward direction
    :type backward_frontier: list
    :param bookkeeping: a dictionary to store some info about algorithm while it is running
     such as number of nodes created and number of nodes expanded
    :type bookkeeping: dict
    :return: if the goal is reachable path to the goal (as a list of nodes), otherwise an empty list
    :rtype: list
    """
    forward_explored = []
    backward_explored = []

    while forward_frontier or backward_frontier:
        if forward_frontier:
            expanding_node = forward_frontier.pop(0)
            forward_explored.append(expanding_node)
            bookkeeping["nodes_expanded"] += 1

            _bfs(expanding_node, forward_frontier, bookkeeping, direction='forward')

        if backward_frontier:
            expanding_node = backward_frontier.pop(0)
            backward_explored.append(expanding_node)
            bookkeeping["nodes_expanded"] += 1

            _bfs(expanding_node, backward_frontier, bookkeeping, direction='backward')

        intersection = get_intersection(forward_explored, backward_explored)
        if intersection is not None:
            path = normalize_path(intersection)
            return path

    return []


def main(test_case_path):
    _, objects, robot_loc, __ = read_input(test_case_path)
    bookkeeping = {
        "nodes_created": 0,
        "nodes_expanded": 0
    }

    starting_node = Node(objects, robot_loc, 0, "", "")
    forward_frontier = [starting_node]
    backward_frontier = get_goal_states_nodes(objects)

    found_path = bbfs(forward_frontier, backward_frontier, bookkeeping)

    if found_path:
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
