
# Example of Localization where the world is 5 blocks.

# with maximum confusion, initailly, the probability that we are in any given
# block is 0.2. Total Probability / # of Grids.


# uniform distribution of belief

# p = [0.2, 0.2, 0.2, 0.2, 0.2] where n = 5

n = 5
p = []
world = ['green', 'red', 'red', 'green', 'green']

Z = 'red'


pHit = 0.6
pMiss = 0.2

for i in range(n):
	p.append(1/n)



# Returns normalized probability distribution after a measurement
# Turns a prior into a posteriror

# Called A Measurement Update

def sense(p,Z):
	q=[]
	for i in range(len(p)):
		hit =(Z == world[i])
		q.append(p[i]*(hit*pHit + (1-hit) * pMiss) )
	s = sum(q)
	for i in range(len(p)):
		q[i] = q[i]/s
	return q



pSum = sum(p)







