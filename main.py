from astarsearch import *
g = SquareGrid(30,15)
g.walls = DIAGRAM1_WALLS
draw_grid(g)

start, goal = (1, 4), (8, 3)
came_from, cost_so_far = a_star_search_algorithm(diagram4, start, goal)
draw_grid(diagram4, point_to=came_from, start=start, goal=goal)
print()
draw_grid(diagram4, path=reconstruct_path(came_from, start=start, goal=goal))
draw_grid(diagram4, number=cost_so_far, start=start, goal=goal)
