from copy import deepcopy

from node import Node


def read_input(file_path):
    """

    :param file_path:
    :return:
    """
    with open(file_path, 'r') as file:
        num_rows, num_cols = list(map(int, file.readline().rstrip().split("\t")))

        costs = [[1 for _ in range(num_cols)] for _ in range(num_rows)]
        objects = [["" for _ in range(num_cols)] for _ in range(num_rows)]
        robot_loc = (0, 0)
        plates_locs = []

        for i in range(num_rows):
            row = list(file.readline().rstrip().split("\t"))
            for j in range(num_cols):
                costs[i][j], objects[i][j], is_robot_in_it, is_plate_in_it = parse_cell(row[j])

                if is_robot_in_it:
                    robot_loc = (i, j)
                if is_plate_in_it:
                    plates_locs.append((i, j))

    return costs, objects, robot_loc, plates_locs


def parse_cell(cell_str):
    i = 0
    for i in range(len(cell_str)):
        if not cell_str[i].isdigit():
            break

    cell_cost = int(cell_str[:i]) if i != 0 else int(cell_str) if cell_str.isdigit() else None
    cell_objects = cell_str[i:] if i != 0 else "" if cell_str.isdigit() else cell_str
    is_robot_in_it = "r" in cell_objects
    is_plate_in_it = "p" in cell_objects

    if is_robot_in_it:
        cell_objects_list = list(cell_objects)
        cell_objects_list.remove("r")
        cell_objects = "".join(cell_objects_list) if cell_objects_list else ""

    return cell_cost, cell_objects, is_robot_in_it, is_plate_in_it


def is_in_goal(objects):
    """
    checks to see if the given objects satisfy the goal condition or not
    the goal condition is that there will not be any cell with butter ALONE.
    :param objects: objects of the field
    :type objects: list
    :return: True if the given objects satisfy goal condition, False otherwise
    :rtype: bool
    """
    for row in objects:
        if any(cell == "b" for cell in row):
            return False

    return True


def get_reverse_movement(movement):
    if movement == "u":
        return "d"
    elif movement == "r":
        return "l"
    elif movement == "d":
        return "u"
    elif movement == "l":
        return "r"


def calc_new_loc(loc, movement):
    # returns new location of robot, based on the type of the movement. IT DOES NOT CHANGE objects list
    x, y = loc
    if movement == "u":
        return (x - 1, y)
    elif movement == "r":
        return (x, y + 1)
    elif movement == "d":
        return (x + 1, y)
    elif movement == "l":
        return (x, y - 1)


def calc_backward_new_loc(loc, movement):
    # returns new location of robot, based on the type of the backward movement. IT DOES NOT CHANGE objects list
    x, y = loc
    if movement == "u":
        return (x + 1, y)
    elif movement == "r":
        return (x, y - 1)
    elif movement == "d":
        return (x - 1, y)
    elif movement == "l":
        return (x, y + 1)


def is_loc_in_boundaries(objects, loc):
    """
    checks to see if the given location is in the boundaries of the field
    :param objects: objects of the field
    :type objects: list
    :param loc: location to be checked
    :return: True if loc is in boundaries, False otherwise.
    :rtype: bool
    """
    num_rows, num_cols = len(objects), len(objects[0])
    x, y = loc

    return (0 <= x < num_rows) and (0 <= y < num_cols)


def is_loc_obstacle(objects, loc):
    """
    checks to see if object in the the given location is an obstacle or not
    An obstacle is a cell which has bp or x in it.
    :param objects: objects of the field
    :type objects: list
    :param loc: location to be checked
    :type loc: tuple
    :return: True if object in loc is an obstacle, False otherwise
    :rtype: bool
    """
    x, y = loc
    return objects[x][y] == "bp" or objects[x][y] == "x"


def one_layer_check(objects, loc, movement):
    """
    checks to see if the new location (result of movement) is not out of boundaries and
     object in the the given location is not an obstacle.
    :param objects: objects of the field
    :type objects: list
    :param loc: location to be checked
    :type loc: tuple
    :param movement: the move that is to be made
    :type movement: str (from the values "u", "r", "d", "l")
    :return: True if the new location in boundaries and doesn't have any obstacle, False otherwise
    :rtype: bool
    """
    new_loc = calc_new_loc(loc, movement)

    if not is_loc_in_boundaries(objects, new_loc):
        return False

    if is_loc_obstacle(objects, new_loc):
        return False

    return True


def is_move_valid(objects, robot_loc, movement):
    """
    checks to see if the given movement from the robot location is valid or not.
    :param objects: objects of the field
    :type objects: list
    :param robot_loc: robot location
    :type robot_loc: tuple
    :param movement: the move that is to be made
    :type movement: str (from the values "u", "r", "d", "l")
    :return: True if the move is allowed, False otherwise
    :rtype: bool
    """
    if not one_layer_check(objects, robot_loc, movement):
        return False

    robot_new_loc = calc_new_loc(robot_loc, movement)
    robot_new_x, robot_new_y = robot_new_loc

    if objects[robot_new_x][robot_new_y] == "b":
        butter_loc = robot_new_x, robot_new_y

        if not one_layer_check(objects, butter_loc, movement):
            return False

        butter_new_loc = calc_new_loc(butter_loc, movement)
        butter_new_x, butter_new_y = butter_new_loc
        if objects[butter_new_x][butter_new_y] == "b":
            return False

    return True


def is_backward_move_valid(objects, robot_loc, movement):
    """
    checks to see it the given backward movement from robot location is valid or not.
    :param objects: objects of the field
    :type objects: list
    :param robot_loc: robot location
    :type robot_loc: tuple
    :param movement: the move that is to be made
    :type movement: str (from the values "u", "r", "d", "l")
    :return: True if the backward move is allowed, False otherwise
    :rtype: bool
    """
    if not one_layer_check(objects, robot_loc, movement):
        return False

    robot_new_loc = calc_new_loc(robot_loc, movement)
    robot_new_x, robot_new_y = robot_new_loc
    if objects[robot_new_x][robot_new_y] == "b":
        return False

    return True


def get_valid_movements(objects, robot_loc):
    """
    :param objects: objects of the field
    :type objects: list
    :param robot_loc: robot location
    :type robot_loc: tuple
    :return: the set of all valid movements from robot location
    :rtype: list
    """
    movements = ["u", "r", "d", "l"]
    valid_movements = []
    for movement in movements:
        if is_move_valid(objects, robot_loc, movement):
            valid_movements.append(movement)

    return valid_movements


def get_valid_backward_movements(objects, robot_loc):
    """
    :param objects: objects of the field
    :type objects: list
    :param robot_loc: robot location
    :type robot_loc: tuple
    :return: the set of all valid backward movements from robot location
    :rtype: list
    """
    movements = ["u", "r", "d", "l"]
    valid_movements = []
    for movement in movements:
        if is_backward_move_valid(objects, robot_loc, movement):
            valid_movements.append(movement)

    return valid_movements


def perform_move(objects, robot_loc, movement):
    """
    Updates objects list (environment) according to the given movement, and
     returns the new_objects (new environment) and new robot location.
    :param objects: objects of the field
    :type objects: list
    :param robot_loc: robot location
    :type robot_loc: tuple
    :param movement: the move that is to be made
    :type movement: str (from the values "u", "r", "d", "l")
    :return: tuple of new_objects and new robot location
    :rtype: tuple
    """
    robot_new_loc = calc_new_loc(robot_loc, movement)
    robot_new_x, robot_new_y = robot_new_loc

    new_objects = deepcopy(objects)
    if objects[robot_new_x][robot_new_y] == "b":
        butter_new_loc = calc_new_loc(robot_new_loc, movement)
        butter_new_x, butter_new_y = butter_new_loc

        new_objects[butter_new_x][butter_new_y] = "b" + new_objects[butter_new_x][butter_new_y]
        new_objects[robot_new_x][robot_new_y] = ""

    return new_objects, robot_new_loc


def perform_backward_move(objects, robot_loc, movement):
    """
    Updates objects list (environment) according to the given BACKWARD movement, and
     returns the new_objects (new environment) and new robot location.
    :param objects: objects of the field
    :type objects: list
    :param robot_loc: robot location
    :type robot_loc: tuple
    :param movement: the move that is to be made
    :type movement: str (from the values "u", "r", "d", "l")
    :return: tuple of new_objects and new robot location
    :rtype: tuple
    """
    robot_x, robot_y = robot_loc
    robot_new_loc = calc_new_loc(robot_loc, movement)

    opposite_neighbor_loc = calc_backward_new_loc(robot_loc, movement)
    opposite_neighbor_x, opposite_neighbor_y = opposite_neighbor_loc

    new_objects_ls = []
    new_objects_1 = deepcopy(objects)
    new_objects_ls.append(new_objects_1)

    if is_loc_in_boundaries(objects, opposite_neighbor_loc):
        if "b" in objects[opposite_neighbor_x][opposite_neighbor_y]:
            new_objects_2 = deepcopy(objects)

            opposite_neighbor_object = objects[opposite_neighbor_x][opposite_neighbor_y]
            opposite_neighbor_object = list(opposite_neighbor_object)
            opposite_neighbor_object.remove("b")
            opposite_neighbor_object = "".join(opposite_neighbor_object)

            new_objects_2[robot_x][robot_y] = "b" + new_objects_1[robot_x][robot_y]
            new_objects_2[opposite_neighbor_x][opposite_neighbor_y] = opposite_neighbor_object
            new_objects_ls.append(new_objects_2)
    else:
        pass

    return new_objects_ls, robot_new_loc


def get_goal_states(objects):
    """
    calculates ALL possible goal states (objects and
    :param objects:
    :return:
    """
    num_rows, num_cols = len(objects), len(objects[0])

    goal_objects = deepcopy(objects)
    bp_loc = []

    for i in range(num_rows):
        for j in range(num_cols):
            if goal_objects[i][j] == "p":
                goal_objects[i][j] = "bp"
                bp_loc.append((i, j))

            elif goal_objects[i][j] == "b":
                goal_objects[i][j] = ""

    goal_robot_locs = []

    for loc in bp_loc:
        movements = ["u", "r", "d", "l"]
        for movement in movements:
            if not one_layer_check(objects, loc, movement):
                continue

            neighbor_loc = calc_new_loc(loc, movement)
            if not one_layer_check(objects, neighbor_loc, movement):
                continue

            goal_robot_locs.append(neighbor_loc)

    return goal_objects, goal_robot_locs


def get_goal_states_nodes(objects):
    """
    converts goal states to node objects, and returns them
    :param objects:
    :return:
    """
    goal_objects, goal_robot_locs = get_goal_states(objects)
    return [Node(goal_objects, goal_robot_locs[i], 0, "", "") for i in range(len(goal_robot_locs))]


def get_path_movements(path):
    """
    Extracts movements taken in the given path, and returns them as a list
    :param path: given path, which is a list of nodes
    :type path: list
    :return: list of movements taken in path
    """
    return [node.movement for node in path]


def pprint_path(path):
    """
    Neatly prints the given path, by calling the .to_cli function on each node of the path
    :param path: given path, which is a list of nodes
    :type path: list
    :return: None
    """
    for node in path:
        print("#" * 120)
        print(f"Movement: {node.movement}")
        node.to_cli()


def manhattan_distance(point_1, point_2):
    """
    Calculates and returns manhattan distance between point_1 and point_2
    :param point_1:
    :type point_1: tuple
    :param point_2:
    :type point_2: tuple
    :return:
    :rtype: int
    """
    point_1_x, point_1_y = point_1
    point_2_x, point_2_y = point_2

    return abs(point_1_x - point_2_x) + abs(point_1_y - point_2_y)


def get_closest_plate(point, plates_locs):
    closets_plate_loc = None
    closest_plate_distance = 100000

    for plate_loc in plates_locs:
        distance = manhattan_distance(point, plate_loc)
        if distance < closest_plate_distance:
            closets_plate_loc = plate_loc
            closest_plate_distance = distance

    return closets_plate_loc, closest_plate_distance


def get_duplicate(nodes_list, target_node):
    """
    checks to see if equivalent of target_node is found in given nodes_list
    Equality between two node is defined by __eq__ function, where it compares
     objects list and robot location of two nodes.
    :param nodes_list:
    :type nodes_list: list
    :param target_node:
    :type target_node: Node
    :return: True if equivalent of target_node is found in nodes_list, False otherwise
    """
    for node in nodes_list:
        if node == target_node:
            return node

    return None


def write_output(file_path, path_movement, depth):
    with open(file_path, 'w') as file:
        file.write(" ".join(path_movement))
        file.write('\nDepth: ' + str(depth))
