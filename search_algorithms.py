from typing import TypeVar, Protocol, List, Dict, Tuple, Iterator, Optional
from pprint import pprint
import collections
import heapq
import math

Position = TypeVar('Position')
T = TypeVar('T')


# we need graph to find the neighbors of each position


class Graph(Protocol):
    def neighbors(self, id: Position) -> List[Position]:
        pass


class SimpleGraph:
    def __init__(self):
        self.edges: Dict[Position, List[Position]] = {}

    def neighbors(self, id: Position) -> List[Position]:
        return self.edges[id]


class Queue:
    def __init__(self):
        self.elements = collections.deque()

    def empty(self):
        return not self.elements

    def put(self, x):
        self.elements.append(x)

    def get(self):
        return self.elements.popleft()


GridLocation = Tuple[int, int]


class SquareGrid:
    def __init__(self, width: int, height: int):
        self.width = width
        self.height = height
        self.walls: List[GridLocation] = []  # barriers in the map

    def in_bounds(self, id: GridLocation):
        (x, y) = id
        return 0 <= x < self.width and 0 <= y < self.height

    def passable(self, id: GridLocation):
        return id not in self.walls  # can't pass through the walls

    def neighbors(self, id: GridLocation) -> Iterator[GridLocation]:
        (x, y) = id
        neighbors = [(x + 1, y), (x - 1, y), (x, y - 1), (x, y + 1)]  # Right left top bottom
        if (x + y) % 2 == 0:
            neighbors.reverse()
        results = filter(self.in_bounds, neighbors)  # returns inbound neighbors of a position
        results = filter(self.passable, results)  # returns passable positions from inbound positions
        return results


def breadth_first_search(graph: SquareGrid, start: Position, goal: Position):
    frontier = Queue()
    frontier.put(start)
    came_from: Dict[Position, Optional[Position]] = dict()
    came_from[start] = None
    while not frontier.empty():
        current: Position = frontier.get()
        if current == goal:
            break
        for next in graph.neighbors(current):
            if next not in came_from:
                frontier.put(next)
                came_from[next] = current
    return came_from


def draw_tile(graph, id, style):
    r = " . "
    if 'number' in style and id in style['number']: r = " %-2d" % style['number'][id]
    if 'point_to' in style and style['point_to'].get(id, None) is not None:
        (x1, y1) = id
        (x2, y2) = style['point_to'][id]
        if x2 == x1 + 1: r = " > "
        if x2 == x1 - 1: r = " < "
        if y2 == y1 + 1: r = " v "
        if y2 == y1 - 1: r = " ^ "
    if 'path' in style and id in style['path']:   r = " @ "
    if 'start' in style and id == style['start']: r = " S "
    if 'goal' in style and id == style['goal']:   r = " E "
    if id in graph.walls: r = "###"
    return r


def draw_grid(graph, **style):
    print("___" * graph.width)
    for y in range(graph.height):
        for x in range(graph.width):
            print("%s" % draw_tile(graph, (x, y), style), end="")
        print()
    print("~~~" * graph.width)


# data from main article
DIAGRAM1_WALLS = [(1, 7), (1, 8), (2, 7), (2, 8), (3, 7), (3, 8), (30, 40), (30, 41), (30, 42), (30, 43), (30, 44),
                  (30, 45),
                  (30, 46), (30, 47), (30, 48), (30, 49), (20, 47), (20, 46), (20, 45), (20, 44), (20, 43), (20, 42),
                  (20, 41), (20, 40), (20, 39), (20, 38), (20, 37), (20, 36), (20, 35), (30, 39), (30, 38), (30, 37),
                  (30, 36),
                  (30, 35), (30, 34), (30, 33), (30, 32), (30, 31), (30, 30), (30, 29), (30, 28), (30, 27), (30, 26),
                  (30, 25), (30, 24),
                  (30, 23), (30, 22), (30, 21), (30, 20), (20, 1), (20, 0),
                  (20, 1),
                  (20, 2),
                  (20, 3),
                  (20, 4),
                  (20, 5),
                  (20, 6),
                  (20, 7),
                  (20, 8),
                  (20, 9),
                  (20, 10),
                  (20, 11),
                  (20, 12),
                  (20, 13),
                  (20, 14),
                  (20, 15),
                  (20, 16),
                  (20, 17),
                  (20, 18),
                  (20, 19),
                  (20, 20),
                  (20, 21),
                  (20, 22),
                  (20, 23),
                  (20, 24),
                  (20, 25),
                  (20, 26),
                  (20, 27),
                  (20, 28),
                  (20, 29),
                  (20, 30),
                  (20, 31),
                  (20, 32),
                  (20, 33),
                  (20, 34),
                  (20, 35),
                  (20, 36),
                  (20, 37),
                  (20, 38),
                  (20, 39),
                  (0, 15),
                  (1, 15),
                  (2, 15),
                  (3, 15),
                  (4, 15),
                  (5, 15),
                  (6, 15),
                  (7, 15),
                  (8, 15),
                  (9, 15),
                  (10, 15),
                  (11, 15),
                  (12, 15),
                  (13, 15),
                  (14, 15),
                  (1, 30),
                  (2, 30),
                  (3, 30),
                  (4, 30),
                  (5, 30),
                  (6, 30),
                  (7, 30),
                  (8, 30),
                  (9, 30),
                  (10, 30),
                  (11, 30),
                  (12, 30),
                  (13, 30),
                  (14, 30),
                  (15, 30),
                  (16, 30),
                  (17, 30),
                  (18, 30),
                  (19, 30)]


class WeightedGraph(Graph):
    def cost(self, from_id: Position, to_id: Position) -> float:
        pass


class WeightedGrid(SquareGrid):
    def __init__(self, width: int, height: int):
        super().__init__(width, height)
        self.weights: Dict[GridLocation, float] = {}

    def cost(self, from_node: GridLocation, to_node: GridLocation) -> float:
        return self.weights.get(to_node, 1)


diagram4 = WeightedGrid(50, 50)
diagram4.walls = [(1, 7), (1, 8), (2, 7), (2, 8), (3, 7), (3, 8), (30, 40), (30, 41), (30, 42), (30, 43), (30, 44),
                  (30, 45),
                  (30, 46), (30, 47), (30, 48), (30, 49), (20, 47), (20, 46), (20, 45), (20, 44), (20, 43), (20, 42),
                  (20, 41), (20, 40), (20, 39), (20, 38), (20, 37), (20, 36), (20, 35), (30, 39), (30, 38), (30, 37),
                  (30, 36),
                  (30, 35), (30, 34), (30, 33), (30, 32), (30, 31), (30, 30), (30, 29), (30, 28), (30, 27), (30, 26),
                  (30, 25), (30, 24),
                  (30, 23), (30, 22), (30, 21), (30, 20), (20, 1), (20, 0),
                  (20, 1),
                  (20, 2),
                  (20, 3),
                  (20, 4),
                  (20, 5),
                  (20, 6),
                  (20, 7),
                  (20, 8),
                  (20, 9),
                  (20, 10),
                  (20, 11),
                  (20, 12),
                  (20, 13),
                  (20, 14),
                  (20, 15),
                  (20, 16),
                  (20, 17),
                  (20, 18),
                  (20, 19),
                  (20, 20),
                  (20, 21),
                  (20, 22),
                  (20, 23),
                  (20, 24),
                  (20, 25),
                  (20, 26),
                  (20, 27),
                  (20, 28),
                  (20, 29),
                  (20, 30),
                  (20, 31),
                  (20, 32),
                  (20, 33),
                  (20, 34),
                  (20, 35),
                  (20, 36),
                  (20, 37),
                  (20, 38),
                  (20, 39),
                  (0, 15),
                  (1, 15),
                  (2, 15),
                  (3, 15),
                  (4, 15),
                  (5, 15),
                  (6, 15),
                  (7, 15),
                  (8, 15),
                  (9, 15),
                  (10, 15),
                  (11, 15),
                  (12, 15),
                  (13, 15),
                  (14, 15),
                  (1, 30),
                  (2, 30),
                  (3, 30),
                  (4, 30),
                  (5, 30),
                  (6, 30),
                  (7, 30),
                  (8, 30),
                  (9, 30),
                  (10, 30),
                  (11, 30),
                  (12, 30),
                  (13, 30),
                  (14, 30),
                  (15, 30),
                  (16, 30),
                  (17, 30),
                  (18, 30),
                  (19, 30)]
# diagram4.weights = {loc: 5 for loc in [(3, 4), (3, 5), (4, 1), (4, 2),
#                                        (4, 3), (4, 4), (4, 5), (4, 6),
#                                        (4, 7), (4, 8), (5, 1), (5, 2),
#                                        (5, 3), (5, 4), (5, 5), (5, 6),
#                                        (5, 7), (5, 8), (6, 2), (6, 3),
#                                        (6, 4), (6, 5), (6, 6), (6, 7),
#                                        (7, 3), (7, 4), (7, 5)]}


class PriorityQueue:
    def __init__(self):
        self.elements: List[Tuple[float, T]] = []

    def empty(self):
        return not self.elements

    def put(self, item, priority):
        return heapq.heappush(self.elements, (priority, item))

    def get(self):
        return heapq.heappop(self.elements)[1]


def dijkstra_search(graph: WeightedGrid, start, goal):
    frontier = PriorityQueue()
    frontier.put(start, 0)
    came_from: Dict[Position, Optional[Position]] = dict()
    cost_so_far: Dict[Position, float] = dict()
    came_from[start] = None
    cost_so_far[start] = 0

    while not frontier.empty():
        current: Position = frontier.get()
        if current == goal:
            break

        for next in graph.neighbors(current):
            new_cost = cost_so_far[current] + graph.cost(current, next)
            if next not in cost_so_far or new_cost < cost_so_far[next]:
                cost_so_far[next] = new_cost
                priority = new_cost
                frontier.put(next, priority)
                came_from[next] = current

    return came_from, cost_so_far


def reconstruct_path(came_from: Dict[Position, Position], start: Position, goal: Position):
    current = goal
    path = []
    while current != start:
        path.append(current)
        current = came_from[current]
    path.append(start)
    path.reverse()
    return path


def heuristic(a: GridLocation, b: GridLocation):
    D = 1
    D2 = 1
    (x1, y1) = a
    (x2, y2) = b
    dx = abs(x1 - x2)
    dy = abs(y1 - y2)
    # manhatten distance
    return dx + dy

    # euclidian distance
    #return math.sqrt(dx**2 + dy**2)

    # diagonal distance
    #return (dx + dy) + (D2 - 2 * D) * min(dx, dy)


def a_star_search_algorithm(graph: WeightedGrid, start, goal):
    frontier = PriorityQueue()
    frontier.put(start, 0)
    came_from: Dict[Position, Optional[Position]] = dict()
    cost_so_far: Dict[Position, float] = dict()
    came_from[start] = None
    cost_so_far[start] = 0

    while not frontier.empty():
        current: Position = frontier.get()
        if current == goal:
            break

        for next in graph.neighbors(current):
            new_cost = cost_so_far[current] + graph.cost(current, next)
            if next not in cost_so_far or new_cost < cost_so_far[next]:
                cost_so_far[next] = new_cost
                priority = new_cost + heuristic(next, goal)
                frontier.put(next, priority)
                came_from[next] = current

    return came_from, cost_so_far


# implementation
# breadth first search
def implement_breadth_first_search():
    g = SquareGrid(50, 50)
    g.walls = DIAGRAM1_WALLS
    start = (1, 4)
    goal = (49, 49)
    parents = breadth_first_search(g, start, goal)
    draw_grid(g, point_to=parents, start=start, goal=goal)
    draw_grid(g, path=reconstruct_path(parents, start=start, goal=goal))
    print(parents)


def implement_dijkstra_algorithm():
    start = (1, 4)
    goal = (49, 49)
    came_from, cost_till_now = dijkstra_search(diagram4, start, goal)
    draw_grid(diagram4, point_to=came_from, start=start, goal=goal)
    draw_grid(diagram4, path=reconstruct_path(came_from, start=start, goal=goal))
    pprint(cost_till_now)


def implement_astar_algorithm():
    start, goal = (1, 4), (49, 49)
    came_from, cost_so_far = a_star_search_algorithm(diagram4, start, goal)
    draw_grid(diagram4, point_to=came_from, start=start, goal=goal)
    draw_grid(diagram4, path=reconstruct_path(came_from, start=start, goal=goal))
    pprint(cost_so_far)