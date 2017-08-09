

p=[0.2, 0.2, 0.2, 0.2, 0.2]
world=['green', 'red', 'red', 'green', 'green']
measurements = ['red', 'green']
pHit = 0.6
pMiss = 0.2

def sense(p, Z):
    q=[]
    for i in range(len(p)):
        hit = (Z == world[i])
        q.append(p[i] * (hit * pHit + (1-hit) * pMiss))
    s = sum(q)
    for i in range(len(q)):
        q[i] = q[i] / s
    return q


for i in range (len(measurements)):
	p = sense(p, measurements[i])


print p


# With exact motion, we shift each value in the distribution by the amount that it moved
pExact = 0.8
pOvershoot = 0.1
pUndershoot = 0.1



def move(p,U):
	q=[]
	for i in range(len(p)):
		s = p[(i-U)% len(p)] * pExact
		s = s + p[(i-U-1)% len(p)] * pOvershoot
		s = s + p[(i-U)% len(p)] * pUndershoot
		q.append(s)
	return q


# Lets move 1 step, 1000 times to try and see what the limit Distribution is.

for k in range(1000):
	p = move(p,1)

print(p)


# The limit should be maximimum confusion 
