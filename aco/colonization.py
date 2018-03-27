'''
Implementation of ACO
Author: Aparajithan Venkateswaran

Based on https://github.com/pjmattingly/ant-colony-optimization
'''

from aco.ants import ant

class ant_colony:

	def __init__(self, nodes, distance, start=None, ant_count=50, alpha=.5, beta=1.2, evap_coeff=.40, pher_const=1000.0, iterations=100):
		'''
		Initializes the ant colony
		
		nodes:      A dict() mapping node ids to coordinates
		distance:   Function that calculates distance between two nodes			
		start:      The starting node. If unset, takes the first node in sorted list of nodes
		ant_count:  Number of ants in the colony
		alpha:      Parameter that determines edge selection
		beta:       Parameter that determines edge selection
		evap_coeff: Pheromone evaporation coefficient. Parameter that determines edge selection
		pher_const: Pheromone constant. Parameter that determines how much pheromone is deposited by ants
		iterations: Number of iterations. Defaults to 100
		
		id_to_key:          Maps each node to integer
		nodes:              Maps coordinates of each node to integer
		adj_matrix:         The adjacency matrix for the graph
		pheromone_map:      Matrix containing final pheromone values
		temp_pher_map:      Temporary matrix at each iteration to hold current pheromone values
		ants:               List of ant objects (see ants.py)
		first_pass:         Boolean flag to keep track of first iteration (see ants.py)
		shortest_distance:  Shortest distance seen from an ant traversal
		shortest_path_seen: Shortest path seen from a traversal (see also shortest_distance)
		'''

		# Error checking
		if type(nodes) is not dict:
			raise TypeError("nodes must be a dictionary")
		if len(nodes) < 1:
			raise ValueError("There must be at least one node")
		if not callable(distance):
			raise TypeError("distance must be a function")
		if type(ant_count) is not int:
			raise TypeError("ant_count must be int")
		if ant_count < 1:
			raise ValueError("ant_count must be >= 1")
		if (type(alpha) is not int) and type(alpha) is not float:
			raise TypeError("alpha must be int or float")
		if alpha < 0:
			raise ValueError("alpha must be >= 0")
		if (type(beta) is not int) and type(beta) is not float:
			raise TypeError("beta must be int or float")
		if beta < 1:
			raise ValueError("beta must be >= 1")
		if (type(evap_coeff) is not int) and type(evap_coeff) is not float:
			raise TypeError("evap_coeff must be int or float")
		if (type(pher_const) is not int) and type(pher_const) is not float:
			raise TypeError("pher_const must be int or float")
		if (type(iterations) is not int):
			raise TypeError("iterations must be int")
		if iterations < 0:
			raise ValueError("iterations must be >= 0")
		
		self.id_to_key, self.nodes = self._init_nodes(nodes)
		self.adj_matrix = self._init_matrix(len(nodes))
		self.pheromone_map = self._init_matrix(len(nodes))
		self.temp_pher_map = self._init_matrix(len(nodes))
		
		self.distance = distance
		
		if start is None:
			self.start = 0
		else:
			self.start = None
			for key, value in self.id_to_key.items():
				if value == start:
					self.start = key
			
			if self.start is None:
				raise KeyError("Key: " + str(start) + " not found in the nodes dict passed.")
		
		self.ant_count = ant_count
		self.alpha = float(alpha)
		self.beta = float(beta)
		self.pheromone_evaporation_coefficient = float(evap_coeff)
		self.pheromone_constant = float(pher_const)
		self.iterations = iterations
		
		self.first_pass = True
		self.ants = self._init_ants(self.start)
		self.shortest_distance = None
		self.shortest_path_seen = None
		
	def _get_distance(self, start, end):
		'''
		Finds distance between start and end
		Uses adj_matrix to cache values
		'''
		if not self.adj_matrix[start][end]:
			distance = self.distance(self.nodes[start], self.nodes[end])			
			self.adj_matrix[start][end] = float(distance)
			return distance
		return self.adj_matrix[start][end]
		
	def _init_nodes(self, nodes):
		'''
		Creates mapping between the dictionary passed in and integers
		id_to_key:    Maps integers to node name
		id_to_values: Maps integers to coordinates of node
		'''
		id_to_key = dict()
		id_to_values = dict()
		
		i = 0
		for key in sorted(nodes.keys()):
			id_to_key[i] = key
			id_to_values[i] = nodes[key]
			i += 1
			
		return id_to_key, id_to_values
		
	def _init_matrix(self, size, value=0.0):
		'''
		Initializes a size x size matrix with value
		'''
		ret = []
		for row in range(size):
			ret.append([float(value) for x in range(size)])
		return ret
	
	def _init_ants(self, start):
		'''
		During first iteration, create a colony of ants
		On subsequent iterations, reset them
		'''

		if self.first_pass:
			return [ant(start, list(self.nodes), self.pheromone_map, self._get_distance,
				self.alpha, self.beta, first_iter=True) for x in range(self.ant_count)]

		for x in self.ants:
			x.__init__(start, list(self.nodes), self.pheromone_map, self._get_distance, self.alpha, self.beta)
	
	def _update_pheromone_map(self):
		'''
		Update pheromone_map by decaying values
		Add pheromone_values from all ants from temp_pher_map
		'''

		for start in range(len(self.pheromone_map)):
			for end in range(len(self.pheromone_map)):
				# Decay the pheromone value along this edge
				self.pheromone_map[start][end] = (1-self.pheromone_evaporation_coefficient)*self.pheromone_map[start][end]
				# Update pheromone value with contributions from ants
				self.pheromone_map[start][end] += self.temp_pher_map[start][end]
	
	def _populate_temp_pher_map(self, ant):
		'''
		Populate temp_pher_map after each ant's traversal
		'''

		route = ant.get_route()
		for i in range(len(route)-1):
			current_pheromone_value = float(self.temp_pher_map[route[i]][route[i+1]])
			new_pheromone_value = self.pheromone_constant/ant.get_distance_traveled()
			self.temp_pher_map[route[i]][route[i+1]] = current_pheromone_value + new_pheromone_value
			self.temp_pher_map[route[i+1]][route[i]] = current_pheromone_value + new_pheromone_value
		
	def run(self):
		'''
		Runs the ACO algorithm
		'''
		
		for i in range(self.iterations):

			# Start the multi-threaded ants
			for ant in self.ants:
				ant.start()
			
			# Wait for all ants to finish traversing the graph
			# Based on http://stackoverflow.com/a/11968818/5343977
			for ant in self.ants:
				ant.join()
			
			# Update shortest path and temp_pher_map
			for ant in self.ants:	
				self._populate_temp_pher_map(ant)
				
				# Update shortes path
				if not self.shortest_distance:
					self.shortest_distance = ant.get_distance_traveled()
				if not self.shortest_path_seen:
					self.shortest_path_seen = ant.get_route()
				if ant.get_distance_traveled() < self.shortest_distance:
					self.shortest_distance = ant.get_distance_traveled()
					self.shortest_path_seen = ant.get_route()
			
			# Update pheromone_map
			self._update_pheromone_map()
			
			if self.first_pass:
				self.first_pass = False
			
			# Reset ants and temporary pheromone map
			self._init_ants(self.start)
			self.temp_pher_map = self._init_matrix(len(self.nodes), value=0)
		
		# Backtrack shortest path
		path = []
		for id in self.shortest_path_seen:
			path.append(self.id_to_key[id])
		
		return path, self.shortest_distance