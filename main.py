from search_algorithms import *


def algorithm_selector():
    algorithms = {'1': 'breadth_first_search', '2': 'dijkstra_search', '3': 'astar_search'}
    print('Welcome to path finder algorithm visualizer.')
    print('Please select your algorithm')
    print('1: breadth first search \n2: dijkstra search \n3: A* search')
    desired_algo = input('Enter 1, 2 or 3: ')
    if desired_algo not in algorithms.keys():
        print('Not a valid selection. Only select the numbers given above. Select correctly and rerun.')

    if desired_algo == '1':
        implement_breadth_first_search()

    if desired_algo == '2':
        implement_dijkstra_algorithm()

    if desired_algo == '3':
        implement_astar_algorithm()


if __name__ == '__main__':
    algorithm_selector()
