from timeit import timeit

from GUI import visualize
from helper import read_input, is_in_goal, get_valid_movements, perform_move, get_path_movements, pprint_path
from node import Node


def generate_children(node):
    """
    Generates children for the given node, based on the valid movements that robot can take
    from its current location

    :param node:
    :type node: Node
    :return: a list of given nodes children.
    """
    children = []

    objects, robot_loc, depth = node.objects, node.robot_loc, node.depth
    valid_movements = get_valid_movements(objects, robot_loc)
    for movement in valid_movements:
        new_objects, new_robot_loc = perform_move(objects, robot_loc, movement)
        new_depth = depth + 1

        child_node = Node(new_objects, new_robot_loc, new_depth, movement, node)

        children = [child_node] + children

    return children


def _dls(cur_node, limit, bookkeeping):
    """
    Implementation of dls (depth limited search)

    Checks if the current node satisfies the goal condition, if it does, returns it,
    otherwise generates its children, and recursively calls _dls on them.
    if the algorithm reaches its specified depth limit, then it returns an empty list.

    :param cur_node: current node to be checked and expanded
    :type cur_node: Node
    :param limit: maximum depth limit
    :type limit: int
    :return: if the goal is reachable, path to the goal (as a list of nodes), otherwise an empty list
    :rtype: list
    """
    bookkeeping["nodes_expanded"] += 1
    if is_in_goal(cur_node.objects):
        return [cur_node]

    if limit <= 0:
        return []

    children = generate_children(cur_node)
    bookkeeping["nodes_created"] += len(children)

    for child in children:
        nodes = _dls(child, limit - 1, bookkeeping)
        if len(nodes) > 0:
            return nodes + [child]

    return []


def ids(starting_node, max_depth, bookkeeping):
    """
    Implementation of IDS (iterative deepening search) algorithm.

    This function repeatedly calls _dls algorithm, incrementing its depth limit by one each time.
    if at any step a path was found by _dls, then it returns the path,
     otherwise if the limit is reached, returns an empty list.

    :param starting_node: root node, which algorithm starts it search with
    :type starting_node: Node
    :param max_depth: maximum depth limit
    :type max_depth: int
    :param bookkeeping: a dictionary to store some info about algorithm while it is running
     such as number of nodes created and number of nodes expanded
    :type bookkeeping: dict
    :return: if the goal is reachable with the given limit,
     path to the goal (as a list of nodes), otherwise an empty list
    """
    for depth in range(max_depth):
        nodes = _dls(starting_node, depth, bookkeeping)
        if len(nodes):
            return nodes[1:]

    return []


def main(test_case_path, max_depth):
    max_depth = int(max_depth)
    _, objects, robot_loc, __ = read_input(test_case_path)
    bookkeeping = {
        "nodes_created": 0,
        "nodes_expanded": 0
    }

    root_node = Node(objects, robot_loc, 0, "", "")
    found_path = ids(root_node, max_depth, bookkeeping)
    found_path.reverse()

    if len(found_path) > 0:
        pprint_path(found_path)
        # visualize(found_path)
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
