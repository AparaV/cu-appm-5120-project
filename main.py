import os
import math
from aco.colonization import ant_colony


data_path = "data"

def read_file(fname):

	found_cities = False
	graph = dict()

	with open(fname) as f:
		for line in f:
			if found_cities:
				line = line.split()
				graph[line[0]] = (float(line[1]), float(line[2]))

			if line == 'NODE_COORD_SECTION\n':
				found_cities = True

	return graph

def distance(start, end):
	x_distance = abs(start[0] - end[0])
	y_distance = abs(start[1] - end[1])
		
	return math.sqrt(pow(x_distance, 2) + pow(y_distance, 2))

if __name__ == "__main__":

	fname = os.path.join(data_path, "djibouti.tsp")
	nodes = read_file(fname)

	colony = ant_colony(nodes, distance, start='1', iterations=200)

	path, dist = colony.run()
	print(path)
	print(dist)