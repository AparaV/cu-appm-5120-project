from aco.colonization import ant_colony
import math

test_nodes = {0: (12, 4), 1: (3, 9), 2: (3, 8), 3: (14, 11), 4: (8, 11),
	5: (15, 6), 6: (6, 15), 7: (15, 9), 8: (12, 10), 9: (10, 7)}

def distance(start, end):
	x_distance = abs(start[0] - end[0])
	y_distance = abs(start[1] - end[1])
	
	return math.sqrt(pow(x_distance, 2) + pow(y_distance, 2))

colony = ant_colony(test_nodes, distance, start=1)

path, dist = colony.run()
print(path)
print(dist)