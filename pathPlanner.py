class Node:
    """Class representing a single node in the grid. 
        Because we cant add the class in another file, i will add it here. 
        and because we cant import libraries, i had to write this code.
        I used a class because it is easier to manage the nodes and their properties."""

    def __init__(self, parent=None, position=None):
        self.parent = parent  # Parent node (for path reconstruction)
        self.position = position  # (x, y) coordinates
        self.past = 0  # Cost from start node to current node
        self.future = 0  # Estimated cost (heuristic) to goal node
        self.total = 0  # Total cost: total = past + future

    def __lt__(self, other):
        """Less than operator for comparing nodes by total cost."""
        return self.total < other.total

    def __eq__(self, other):
        """Check if two nodes have the same position."""
        return self.position == other.position

    def __hash__(self):
        """Hash function to allow storing nodes in sets."""
        return hash(self.position)


def calculate_future_value(pos2, pos1):
    """
    Calculate the Euclidean distance between two positions.
    Formula: h = √((x2 - x1)^2 + (y2 - y1)^2)
    """
    return ((pos2[0] - pos1[0]) ** 2 + (pos2[1] - pos1[1]) ** 2) ** 0.5


def get_neighbors(position, grid):
    """
    Get all valid neighbors of a given position (up, down, left, right).
    We dont do diagonal here...it is against our coursework constraints.

    Parameters:
    - position: Current position (x, y).
    - grid: 2D grid representing the map.

    Returns:
    - List of valid neighbor positions.
    """
    # neighbors is a list of tuples representing valid neighbor positions and will be returned.
    neighbors = []
    max_x, max_y = len(grid), len(grid[0])

    for x, y in [(0, -1), (0, 1), (-1, 0), (1, 0)]:  # Up, down, left, right
        new_position = (position[0] + x, position[1] + y)

        # Check if the neighbor is within grid bounds there are cells for path
        if 0 <= new_position[0] < max_x and 0 <= new_position[1] < max_y:
            # Only consider cells exclude obstacles where cells are 1 and obstacles are 0 as mentioned in the program.
            if grid[new_position[0]][new_position[1]] == 1: 
                neighbors.append(new_position)

    return neighbors


def reconstruct_path(curr_node):
    """
    Reconstruct the path from the end node to the start node by following parent pointers (I know there is no pointers in python, this is a work around). 
    follows the path from the goal back to the start by following each node’s parent.
    inserts each position at the front of the list and reverses.
    reteun the path in the correct order.

    Parameters:
    - curr_node: The end node.

    Returns:
    - List of tuples representing the path from start to goal.
    """
    path = []
    while curr_node:
        # I added Add position to the front, it has to be in front because A* works by finding the goal first, so the path needs to be reconstructed backwards. (end to start)
        # To get the path in the right order (start → goal), reverse the path after tracing it.
        # easiest way to do that is to insert each position at the front of the list.
        path.insert(0, curr_node.position)
        curr_node = curr_node.parent
    return path


def get_lowest_total_node(unvisited_list):
    """    Find the node with the lowest total in the node that are not visited list (unvisited_list).

    Parameters:
    - unvisited_list: List of nodes to explore.

    Returns:
    - Node with the lowest total.
    """
    lowest_index = 0
    for i in range(1, len(unvisited_list)):
        if unvisited_list[i].total < unvisited_list[lowest_index].total:
            lowest_index = i
    return unvisited_list[lowest_index]


def is_better_path(new_node, unvisited_list):
    """
    Check the current path to this neighbor is better than any existing path.

    Parameters:
    - new_node: The new neighbor node being evaluated.
    - unvisited_list: The list of nodes currently in the unvisited nodes list (unvisited_list).

    Returns:
    - True if the new node has a better path, False otherwise.
    """
    for node in unvisited_list:
        # If a node with the same position is found, check its cost
        if new_node == node:
            # Return False if the new node has a higher or equal past cost
            if new_node.past >= node.past:
                return False
    return True


def process_neighbors(curr_node, end_node, unvisited_list, best_past_dict, visited_set, grid):
    """
    Process all valid neighbors of the current node.

    Parameters:
    - curr_node: The node currently being processed.
    - end_node: The target goal node.
    - unvisited_list: List of nodes to explore.
    - best_past_dict: Dictionary to track the lowest past cost to a position.
    - visited_set: Set of already visited nodes.
    - grid: 2D grid representing the map.
    """
    for neighbor_pos in get_neighbors(curr_node.position, grid):
        if neighbor_pos in visited_set:
            continue

        # Create a new neighbor node
        neighbor_node = Node(curr_node, neighbor_pos)
        neighbor_node.past = curr_node.past + 1
        neighbor_node.future = calculate_future_value(neighbor_pos, end_node.position)
        neighbor_node.total = neighbor_node.past + neighbor_node.future

        # Check if this path is better than any existing one
        if neighbor_pos not in best_past_dict or neighbor_node.past < best_past_dict[neighbor_pos]:
            unvisited_list.append(neighbor_node)
            best_past_dict[neighbor_pos] = neighbor_node.past

# i tried to make it cleaner by remooving code from here and calling methods instead but I cant simplify it more than this.
def search(grid, start, end, display_message):
    """
    Perform A* search to find the shortest path from start to end.

    Parameters:
    - grid: 2D grid with 1s (cells) and 0s (obstacles).
    - start: Tuple (x, y) representing the start position.
    - end: Tuple (x, y) representing the goal position.
    - display_message: Function to display messages.

    Returns:
    - List of tuples representing the shortest path or an empty list if no path is found.
    """

    # Initialize start and end nodes
    start_node = Node(None, start)
    end_node = Node(None, end)
    start_node.future = calculate_future_value(start_node.position, end_node.position)
    start_node.total = start_node.past + start_node.future

    # not visited nodes list (nodes to be explored) and visited set (explored nodes)
    unvisited_list = [start_node]
    visited_set = set()
    best_past_dict = {start_node.position: start_node.past}

    # Main A* search loop
    while unvisited_list:
        # Get the node with the lowest total cost
        curr_node = get_lowest_total_node(unvisited_list)
        unvisited_list.remove(curr_node)
        best_past_dict.pop(curr_node.position, None)
        visited_set.add(curr_node.position)

        # Check if the goal is reached
        if curr_node.position == end_node.position:
            display_message("End location reached, path found.")
            path = reconstruct_path(curr_node)
            display_message("Path: " + str(path))
            return path

        # Process all valid neighbors
        process_neighbors(curr_node, end_node, unvisited_list, best_past_dict, visited_set, grid)

    # If no path is found, return an empty list
    display_message("No path found.")
    return []


# The main path planning function. Additional functions, classes, 
# variables, libraries, etc. can be added to the file, but this
# function must always be defined with these arguments and must 
# return an array ('list') of coordinates (col,row).
#DO NOT EDIT THIS FUNCTION DECLARATION
def do_a_star(grid, start, end, display_message):
    """
    Main function to run the A* algorithm.

    Parameters:
    - grid: 2D list where 1 represents cells and 0 represents obstacles.
    - start: Tuple (x, y) representing the start position.
    - end: Tuple (x, y) representing the goal position.
    - display_message: Function to display messages for debugging or status updates.

    Returns:
    - List of tuples representing the shortest path from start to end.
    """

    path = search(grid, start, end, display_message)
    
    # Send the path points back to the gui to be displayed
    #FUNCTION MUST ALWAYS RETURN A LIST OF (col,row) COORDINATES
    return path
#end of file
