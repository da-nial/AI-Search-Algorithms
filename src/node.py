from copy import deepcopy

import TableIt


class Node:
    def __init__(self, objects, robot_loc, depth, movement, parent, cost_g=None, cost_f=None):
        """
        Basic Node class, representing each item of our search.

        :param objects: A 2d list of lists, containing a string for each cell of the field,
         where the string has all the objects in it
        :type objects: list
        :param robot_loc: robot location
        :type robot_loc: tuple
        :param depth: depth of this node in the search tree
        :type depth: int
        :param movement: the move that is to be made
        :type movement: str (from the values "u", "r", "d", "l")        :param parent:
        :param cost_g: g cost
        :type cost_g: int
        :param cost_f: f cost
        :type cost_f: int
        """
        self.objects = objects
        self.robot_loc = robot_loc
        self.depth = depth
        self.movement = movement
        self.parent = parent
        self.cost_g = cost_g
        self.cost_f = cost_f

        self.img = {
            "b": "butter",
            "bp": "butter",
            "p": "plate2",
            "": None,
            "x": "obstacle1"
        }

    def __eq__(self, other):
        """
        To implement 'in' operator
        We name two nodes equal, if the both have the same objects list and robot location
        """
        if isinstance(other, Node):
            return self.objects == other.objects and self.robot_loc == other.robot_loc

    def to_cli(self):
        """
        Neatly prints objects list and robot location of node to console
        :return: None
        """
        objects_cpy = deepcopy(self.objects)

        robot_x, robot_y = self.robot_loc
        objects_cpy[robot_x][robot_y] = "r" + objects_cpy[robot_x][robot_y]

        TableIt.printTable(objects_cpy)

    def to_gui(self, field):
        """
        places objects list and robot location of node in the given field GUI object
        :param field: GUI object to place the cells on
        :type field: game2dboard.Board
        :return: None
        """
        objects, robot_loc = self.objects, self.robot_loc

        num_rows, num_cols = len(objects), len(objects[0])
        robot_x, robot_y = robot_loc

        for i in range(num_rows):
            for j in range(num_cols):
                field[i][j] = self.img[objects[i][j]]

        field[robot_x][robot_y] = "robot"
