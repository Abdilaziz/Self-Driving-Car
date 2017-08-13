from math import *
import random




landmarks = [[20.0, 20.0],
			 [80.0, 80.0],
			 [20.0, 80.0],
			 [80.0, 20.0]]

world_size = 100.0 # world is 100 by 100


class robot:
	def __init__(self):
		self.x = random.random() * world_size
		self.y = random.random() * world_size
		self.orientation = random.random() * 2.0 * pi
		self.forward_noise = 0.0
		self.turn_noise = 0.0
		self.sense_noise = 0.0


	def set(self, new_x, new_y, new_orientation):
		if  new_x < 0 or new_x >= world_size:
			raise(ValueError, 'X coordinate out of bound')
		if new_y < 0 or new_y >= world_size:
			raise(ValueError, 'Y coordinate out of bound')
		if new_orientation < 0 or new_orientation >= 2*pi:
			raise(ValueError, 'Orientation must be in [0..2pi]')
		self.x = float(new_x)
		self.y = float(new_y)
		self.orientation = float(new_orientation)

	def set_noise(self, new_f_noise, new_t_noise, new_s_noise):
		# makes it possible to change the noise parameters
		# this is often useful in partical filters
		self.forward_noise = float(new_f_noise);
		self.turn_noise = float(new_t_noise);
		self.sense_noise = float(new_s_noise);

	def sense(self):
		Z = []
		for i in range(len(landmarks)):
			dist = sqrt( (self.x - landmarks[i][0]) ** 2 + (self.y - landmarks[i][1])**2 )
			dist += random.gauss(0.0, self.sense_noise)
			Z.append(dist)
		return Z

	def move(self, turn, forward):
		if forward < 0:
			raise(ValueError, 'Robot cant move backwards')     

		# turn, and add randomness to the turning command
		orientation = self.orientation + float(turn) + random.gauss(0.0, self.turn_noise)
		orientation %= 2 * pi
        
        # move, and add randomness to the motion command
		dist = float(forward) + random.gauss(0.0, self.forward_noise)
		x = self.x + (cos(orientation) * dist)
		y = self.y + (sin(orientation) * dist)
		x %= world_size    # cyclic truncate
		y %= world_size

        # set particle
		res = robot()
		res.set(x, y, orientation)
		res.set_noise(self.forward_noise, self.turn_noise, self.sense_noise)
		return res

	def Gaussian(self, mu, sigma, x):
		# calculates the probability of x for 1-dim Gaussian with mean mu and var. sigma
		return exp(- ((mu - x) ** 2) / (sigma ** 2) / 2.0) / sqrt(2.0 * pi * (sigma ** 2))

	def measurement_prob(self, measurement):
        
		# calculates how likely a measurement should be
		prob = 1.0;
		for i in range(len(landmarks)):
			dist = sqrt((self.x - landmarks[i][0]) ** 2 + (self.y - landmarks[i][1]) ** 2)
			prob *= self.Gaussian(dist, self.sense_noise, measurement[i])
		return prob

	def __repr__(self):
		return '[x=%.6s y=%.6s orient=%.6s]' % (str(self.x), str(self.y), str(self.orientation))



# myrobot = robot()


# # starts at 30.0, 50.0, heading north(=pi/2)
# # turns clockwise by pi/2, moves 15 meters
# # senses
# # turns clockwise by pi/2, moves 10 meters
# # senses
# myrobot.set_noise(5.0, 0.1, 5.0)

# myrobot.set(30.0,50.0, pi/2)


# myrobot = myrobot.move(-pi/2, 15.0)
# print(myrobot.sense())
# myrobot = myrobot.move(-pi/2, 10.0)
# print(myrobot.sense())


def eval(r, p):
	sum = 0.0
	for i in range(len(p)):
		dx = (p[i].x - r.x + (world_size/2.0)) %world_size - (world_size/2.0) # to handle circular world where 0.00001 and 99.999999 are condisdered widely different
		dy = (p[i].y - r.y + (world_size/2.0)) % world_size - (world_size/2.0)
		err = sqrt(dx * dx + dy * dy)
		sum += err
	return sum / float(len(p))



#here is my actual robot
myrobot = robot()





# -------------------------------

# created 1000 particles with each of their own x, y, and orientation


N = 1000
T= 10
p = []
for i in range(N):
	x = robot()
	x.set_noise(0.05, 0.05, 5.0)
	p.append(x)

print(len(p))

for t in range(T):

	myrobot = myrobot.move(0.1, 5.0)
	Z = myrobot.sense()


	# now lets simulate movement by turning 0.1 and move 5

	for i in range(N):
		p[i] = p[i].move(0.1, 5.0)


	# now we can calculate an importance weight so we can find what particles are
	# have measurements that resemble our actual robot.

	weights = []
	for i in range(N):
		weights.append(p[i].measurement_prob(Z))

	# now the weights consists of probabilities that the particle is
	# the actual robot


	# the final step is to sample particles from p with a probability that is
	# proportional to its corresponding weight

	# particles with a large weight should be drawn more frequently than ones
	# with a small weight value



	# Normalize the weights
	# norm_w =[]
	# total = sum(weights)
	# for i in range(N):
	# 	norm_w.append( weights[i]/total)


	# Now we need to resample our particles p, based on the normalized weights.
	# we can resample using an algorithm called the Resampling Wheel

	# RESAMPLING WHEEL
	# random uniform particle index, so random variable from 1 to N, (or 0 to N-1)

	# a beta value that is intially 0, but it beta a value from 0 to 2 times the max weight 


	index = int(random.random() * N)
	beta = 0.0
	mw = max(weights)
	resampled_p = []
	for i in range(N):
		beta += random.random() * 2.0 * mw
		while beta > weights[index]:
			beta -= weights[index]
			index = (index + 1) % N
		resampled_p.append(p[index])
	p = resampled_p
	
	# print(resampled_p) # all particles that have positions close to the acutal postion
	# # however orientation is random due to the fact that it isnt considered when resampliing the particles



	print(eval(myrobot, p))







# -------------------------------



