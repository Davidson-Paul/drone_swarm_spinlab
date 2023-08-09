#!/usr/bin/env python3

#Script for generating Adjecency Matrix

import random
import numpy as np
import time

def loop():
	# #undirected 5, leader changing
	# while True:
	# 	mat = np.array([[0,1,1,1,1,0],[1,0,1,1,1,0],[1,1,0,1,1,0],[1,1,1,0,1,0],[1,1,1,1,0,0],[0,0,0,0,0,0]])
	# 	leader_node = random.randint(0,4)
	# 	print(leader_node)
	# 	mat[leader_node,5]=1

	#partial random directed graph
	while True:
		mat = np.zeros([6,6])
		#leader node (gets info from VL)
		leader_node = random.randint(0,4)
		print(leader_node)
		mat[leader_node,5]=1

		prev_node = leader_node
		for i in range(1,5):
			next_node = (prev_node+1)%5
			mat[next_node,prev_node]=1
			while True:
				neighbour = random.randint(0,4)
				if(neighbour!=next_node and neighbour!=prev_node):
					mat[next_node,neighbour] = 1
					break
			prev_node = next_node

		while True:
			neighbour1 = random.randint(0,4)
			neighbour2 = random.randint(0,4)
			if(neighbour1!=leader_node and neighbour1!=neighbour2 and neighbour2!=leader_node):
				mat[leader_node,neighbour1] = 1
				mat[leader_node,neighbour2] = 1
				break

		print(mat)
		np.savetxt('adjacency_matrix.txt', mat, fmt='%d')
		time.sleep(5)

if __name__ == '__main__':
	loop()