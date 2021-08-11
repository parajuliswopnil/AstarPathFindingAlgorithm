from typing import TypeVar, Protocol, List, Dict, Tuple, Iterator, Optional
from pprint import pprint
import collections
import heapq

Position = TypeVar('Position')
T = TypeVar('T')

# we need graph to find the neighbors of each position


class Graph(Protocol):
    def neighbors(self, id: Position) -> List[Position]:
        pass


class SimpleGraph:
    def __init__(self):
        self.edges: Dict[Position, List[Position]] ={}

    def neighbors(self, id: Position) -> List[Position]:
        return self.edges[id]


# implementation of a simple graph using our simple graph class
graph = SimpleGraph()
graph.edges = {
    'A': ['B'],
    'B': ['C'],
    'C': ['B', 'D', 'F'],
    'D': ['C', 'E'],
    'E': ['F'],
    'F': [],
}
pprint(graph.neighbors('C'))


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
        return 0<= x < self.width and 0 <= y < self.height

    def passable(self, id: GridLocation):
        return id not in self.walls  # can't pass through the walls

    def neighbors(self, id: GridLocation) -> Iterator[GridLocation]:
        (x, y) = id
        neighbors = [(x+1, y), (x-1, y), (x, y-1), (x, y+1)]  # Right left top bottom
        if (x + y) % 2 == 0:
            neighbors.reverse()
        results = filter(self.in_bounds, neighbors)  # returns inbound neighbors of a position
        results = filter(self.passable, results)  # returns passable positions from inbound positions
        return results


def from_id_width(id, width):
    return (id % width, id // width)


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
DIAGRAM1_WALLS = [from_id_width(id, width=30) for id in
                  [21, 22, 51, 52, 81, 82, 93, 94, 111, 112, 123, 124, 133, 134, 141, 142, 153, 154, 163, 164, 171, 172,
                   173, 174, 175, 183, 184, 193, 194, 201, 202, 203, 204, 205, 213, 214, 223, 224, 243, 244, 253, 254,
                   273, 274, 283, 284, 303, 304, 313, 314, 333, 334, 343, 344, 373, 374, 403, 404, 433, 434]]


class WeightedGraph(Graph):
    def cost(self, from_id: Position, to_id: Position) -> float:
        pass


class WeightedGrid(SquareGrid):
    def __init__(self, width: int, height: int):
        super().__init__(width, height)
        self.weights: Dict[GridLocation, float] = {}

    def cost(self, from_node: GridLocation, to_node: GridLocation) -> float:
        return self.weights.get(to_node, 1)


diagram4 = WeightedGrid(10, 10)
diagram4.walls = [(1, 7), (1, 8), (2, 7), (2, 8), (3, 7), (3, 8)]
diagram4.weights = {loc: 5 for loc in [(3, 4), (3, 5), (4, 1), (4, 2),
                                       (4, 3), (4, 4), (4, 5), (4, 6),
                                       (4, 7), (4, 8), (5, 1), (5, 2),
                                       (5, 3), (5, 4), (5, 5), (5, 6),
                                       (5, 7), (5, 8), (6, 2), (6, 3),
                                       (6, 4), (6, 5), (6, 6), (6, 7),
                                       (7, 3), (7, 4), (7, 5)]}


class PriorityQueue:
    def __init__(self):
        self.elements: List[Tuple[float, T]] = []

    def empty(self):
        return not self.elements

    def put(self, item, priority):
        return heapq.heappush(self.elements, (priority, item))

    def get(self):
        return heapq.heappop(self.elements)[1]


def dijkstra_search(graph: WeightedGraph, start, goal):
    frontier = PriorityQueue()
    frontier.put(start, 0)
    came_from: Dict[Position, Optional[Position]] = {}
    cost_so_far: Dict[Position, float] = {}
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
    (x1, y1) = a
    (x2, y2) = b
    return abs(x1-x2) + abs(y1 - y2)


def a_star_search_algorithm(graph: WeightedGraph, start, goal):
    frontier = PriorityQueue()
    frontier.put(start, 0)
    came_from: Dict[Position, Optional[Position]] = {}
    cost_so_far: Dict[Position, float] = {}
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