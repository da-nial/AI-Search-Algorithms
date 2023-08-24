from tkinter import messagebox

from game2dboard import Board

BLOCK_SIZE = 120

field = None
path = None
cur_node_index = 0


def step():
    global field
    global path
    global cur_node_index

    path[cur_node_index].to_gui(field)
    cur_node_index += 1

    if cur_node_index >= len(path):
        messagebox.askokcancel("Alo Butter", "All butters are in their place!\nExit?")
        field.close()


def visualize(_path):
    global field
    global path

    path = _path
    num_rows, num_cols = len(path[0].objects), len(path[0].objects[0])
    field = Board(num_rows, num_cols)

    field.cell_size = BLOCK_SIZE
    field.title = "Alo Butter!"
    field.cursor = None  # Hide the cursor
    field.margin = 20
    field.grid_color = "AntiqueWhite"
    field.margin_color = "AntiqueWhite"
    field.cell_color = "Peru"
    field.on_timer = step
    field.start_timer(700)  # 300 ms
    field.show()
