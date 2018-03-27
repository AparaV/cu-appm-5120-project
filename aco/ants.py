'''
Implementation of ants in the ACO
Author: Aparajithan Venkateswaran

Based on https://github.com/pjmattingly/ant-colony-optimization
'''

import math
import random
import struct

from threading import Thread

class ant(Thread):
	"""docstring for ants"""

	def __init__(self, init_loc, raw_map, pheromone_map, distance, alpha, beta, first_iter=False):
		'''
		Initializes a new ant to traverse the graph
		init_loc:     The starting point for ant
		raw_map:      The raw map of the graph with coordinates of each nodes
		phermone_map: The phermone values between each pair of nodes
		distance:     A function to calculate the distance between two nodes
		alpha:        Alpha parameter used in the edge selection step
		beta:         Beta paramter used in the edge selection step
		first_iter:   If first iteration, then select next city to traverse randomly
		'''

		Thread.__init__(self)

		self.init_loc = init_loc
		self.raw_map = raw_map
		self.pheromone_map = pheromone_map
		self.distance = distance
		self.alpha = alpha
		self.beta = beta
		self.first_iter = first_iter

		self.route = []
		self.total_distance = 0.0
		self.current_loc = init_loc
		self.update_route(init_loc, last=False)
		self.tour_complete = False

	def run(self):
		'''
		Until the ant has visited all nodes:
			pick_path() to find a next node to traverse to
			traverse() to:
				update_route() (to show latest traversal)
				update_distance_traveled() (after traversal)
		return the ants route and its distance, for use in ant_colony:
			do pheromone updates
			check for new possible optimal solution with this ants latest tour
		'''

		while self.raw_map:
			next = self.pick_node()
			self.traverse(self.current_loc, next)
		# Complete the cycle
		self.traverse(next, self.init_loc, last=True)
			
		self.tour_complete = True
	
	def pick_node(self):
		'''
		Edge selection algorithm of ACO
		Calculate the probability of each possible edge from the current location
		Randomly choose next edge, based on the probability
		'''

		# In the first iteration, randomly choose next node
		if self.first_iter:
			return random.choice(self.raw_map)
		
		attractiveness = dict()
		sum_total = 0.0

		# For each possible edge, find its probability
		# Sum all probability amounts for calculating probability of each route in the next step
		for next_loc in self.raw_map:
			pheromone_amount = float(self.pheromone_map[self.current_loc][next_loc])
			distance = float(self.distance(self.current_loc, next_loc))
			
			attractiveness[next_loc] = pow(pheromone_amount, self.alpha) * pow(1/distance, self.beta)
			sum_total += attractiveness[next_loc]
		
		# Fix rounding errors
		# Based on http://stackoverflow.com/a/10426033/5343977
		if sum_total == 0.0:

			def next_up(x):
				if math.isnan(x) or (math.isinf(x) and x > 0):
					return x

				if x == 0.0:
					x = 0.0

				n = struct.unpack('<q', struct.pack('<d', x))[0]
				
				if n >= 0:
					n += 1
				else:
					n -= 1
				return struct.unpack('<d', struct.pack('<q', n))[0]
				
			for key in attractiveness:
				attractiveness[key] = next_up(attractiveness[key])
			sum_total = next_up(sum_total)
		
		# Probabilistic edge selection
		# Based on http://stackoverflow.com/a/3679747/5343977
		toss = random.random()
				
		cummulative = 0
		for next_loc in attractiveness:
			weight = (attractiveness[next_loc] / sum_total)
			if toss <= weight + cummulative:
				return next_loc
			cummulative += weight
	
	def traverse(self, start, end, last=False):
		'''
		update_route() to show new traversal
		update_distance_traveled() to record new distance traveled
		self.location update to new location
		'''

		self.update_route(end, last)
		self.update_distance_traveled(start, end)
		self.current_loc = end
	
	def update_route(self, new, last):
		'''
		Add new node to self.route
		Remove new node from self.raw_map
		'''
		self.route.append(new)
		if not last:
			self.raw_map.remove(new)
		self.current_loc = new
		
	def update_distance_traveled(self, start, end):
		'''
		Update the distance traveled by ant
		'''
		self.total_distance += float(self.distance(start, end))

	def get_route(self):
		if self.tour_complete:
			return self.route
		return None
		
	def get_distance_traveled(self):
		if self.tour_complete:
			return self.total_distance
		return None
