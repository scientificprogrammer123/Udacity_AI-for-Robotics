#2 program a car yourself
# There is a self driving car Carla, 
# you can program it at the end of self driving car nanodegree.

#3 quiz state space
# class 1 = histogram filter, or monte carlo filter, 
#           discrete state space, multimodal distribution, 
#           exponential efficienty, approximate
# class 2 = kalman filter
#           continuous state space, unimodal, 
#           quadratic efficiency, covariance matrix is quadratic, approximate
# class 3 = particle filters
#           continuous state space, multimodal distributions, approximate, 
#           efficiency not sure, but no good for >4 dims
# advantage of particle filter is that it is easy to program.

#7 particle filters
# Here is a floor plan of environment, where robot is located, and it as to perform
# global localization, which is it has no clue where it is and it has to find out 
# where it is just based on sensor measurements.
# The robot has range sensors, blue stripes, those use sonar sensors, which means
# sound, to range the distance of nearer obstacles, and it has to use these range 
# sensrs to determine a good posterior distribution as to where it is. What the 
# robot doesn't know is that it is starting in the middle of the corridor. In fact
# it is completely uncertain as to where it is.Now the particle filter represents
# this using particles, each of these red dots of which there several thousands here
# is a discreet guess where the robot might be. it is structured as an x coordinate
# y coordinate and a heading direction, and those 3 values together comprise a 
# single guess. But a single guess is not a filter, it is the set of several 
# thousand of such guesses that together comprise an approximate representation for
# the posterior of the robot. 
# In the beginning, the particles are uniformly spread, but the particle filter make
# them survive, in proportion to how consistnt 1 of these particles is with the 
# sensor measurement. Very quickly the robot has figured out it's in the corridor, 
# but 2 clouds survive, based on symmetry of the corridor. As the robot enters one
# of the offices, the symmetry is broken and the correct set of particles survive.
# The essense of particle filter is to have these particles guess where the robot
# might be moving, but to have them survive using effectively survival of the fittest, 
# so that particles that are more consistent with the measurements are more likely 
# to survive and as a result places of high probability will collect more particles,
# and therefore be more representative of the robot's posterior belief.
# Those particles together, those thousands of particles, are now clustered in a 
# single location. Those comprise the approximate belief of the robot as it localizes
# itself.

#10 moving robot
# # Make a robot called myrobot that starts at
# coordinates 30, 50 heading north (pi/2).
# Have your robot turn clockwise by pi/2, move
# 15 m, and sense. Then have it turn clockwise
# by pi/2 again, move 10 m, and sense again.
#
# Your program should print out the result of
# your two sense measurements.
#
# Don't modify the code below. Please enter
# your code at the bottom.
from math import *
import random
landmarks  = [[20.0, 20.0], [80.0, 80.0], [20.0, 80.0], [80.0, 20.0]] #square
world_size = 100.0
class robot:
    def __init__(self): #initialize object
        self.x = random.random() * world_size
        self.y = random.random() * world_size
        self.orientation = random.random() * 2.0 * pi
        self.forward_noise = 0.0; #added gaussian noise 
        self.turn_noise    = 0.0;
        self.sense_noise   = 0.0;    
    def set(self, new_x, new_y, new_orientation): #set position and orientation
        if new_x < 0 or new_x >= world_size:
            raise ValueError('X coordinate out of bound')
        if new_y < 0 or new_y >= world_size:
            raise ValueError('Y coordinate out of bound')
        if new_orientation < 0 or new_orientation >= 2 * pi:
            raise ValueError('Orientation must be in [0..2pi]')
        self.x = float(new_x)
        self.y = float(new_y)
        self.orientation = float(new_orientation)    
    def set_noise(self, new_f_noise, new_t_noise, new_s_noise): #set noise based on values in __init__
        # makes it possible to change the noise parameters
        # this is often useful in particle filters
        self.forward_noise = float(new_f_noise);
        self.turn_noise    = float(new_t_noise);
        self.sense_noise   = float(new_s_noise);   
    def sense(self): #sense the distances from where location to landmarks
        Z = []
        for i in range(len(landmarks)):
            dist = sqrt((self.x - landmarks[i][0]) ** 2 + (self.y - landmarks[i][1]) ** 2)
            dist += random.gauss(0.0, self.sense_noise)
            Z.append(dist)
        return Z    
    def move(self, turn, forward): #move the robot based on direction
        if forward < 0:
            raise ValueError('Robot cant move backwards')                 
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
    # end of class robot:
def eval(r, p): #some sort of error function
    sum = 0.0;
    for i in range(len(p)): # calculate mean error
        dx = (p[i].x - r.x + (world_size/2.0)) % world_size - (world_size/2.0)
        dy = (p[i].y - r.y + (world_size/2.0)) % world_size - (world_size/2.0)
        err = sqrt(dx * dx + dy * dy)
        sum += err
    return sum / float(len(p))
####   DON'T MODIFY ANYTHING ABOVE HERE! ENTER CODE BELOW ####
myrobot = robot() #declare an instance of the object
myrobot.set_noise(5.0, 0.1, 5.0) #set noise for a particular move, #11
myrobot.set(30.0, 50.0, pi/2) #pointing up, I think
myrobot = myrobot.move(-pi/2, 15.0) #should be at (45,50), pointing at 0
print(myrobot.sense()) #these distances measurementscomprise of the measurement wor
myrobot = myrobot.move(-pi/2, 10.0) #should be at (45,40), pointing down
print(myrobot.sense()) #these distances measurementscomprise of the measurement world
# the world wraps around

#13 creating a particle
# The particle filter you are going to program contains a set of 1000 random
# guesses as to where the robot might be. Now I'm not going to draw 1000 dots here, 
# but let me explain how each of these dots look like. Each of these dots is a 
# vector which contains an X coordinate, a Y coordinate, and a heading direction,
# which is the angle at which the robot points relative to the X axis.
# So this robot moves forward, it will move slightly upwards. In fact, now a code, 
# everytime  you call the function robot, and assign it say to a particle, these
# elements p[i] x,y,orientation, which is the same as the heading, are initialized
# at random. So to make a particle set of 1000 particles, what you have to do is 
# program a simple piece of code that assigns 1000 of these to a list. 
# So, set N=1000 for 1000 particles, p is the initial set of particles, which is
# an empty list, and fill in code after which there are 1000 particles assigned to
# this vector over here.
# so it is a thousand vectors for a thousand particles, each vector as x y orientation
# initialized at random.
#
# Now we want to create particles,
# p[i] = robot(). In this assignment, write
# code that will assign 1000 such particles
# to a list.
#
# Your program should print out the length
# of your list (don't cheat by making an
# arbitrary list of 1000 elements!)
#
# Don't modify the code below. Please enter
# your code at the bottom.
from math import *
import random
landmarks  = [[20.0, 20.0], [80.0, 80.0], [20.0, 80.0], [80.0, 20.0]]
world_size = 100.0
class robot:
    def __init__(self):
        self.x = random.random() * world_size
        self.y = random.random() * world_size
        self.orientation = random.random() * 2.0 * pi
        self.forward_noise = 0.0;
        self.turn_noise    = 0.0;
        self.sense_noise   = 0.0;    
    def set(self, new_x, new_y, new_orientation):
        if new_x < 0 or new_x >= world_size:
            raise ValueError('X coordinate out of bound')
        if new_y < 0 or new_y >= world_size:
            raise ValueError('Y coordinate out of bound')
        if new_orientation < 0 or new_orientation >= 2 * pi:
            raise ValueError('Orientation must be in [0..2pi]')
        self.x = float(new_x)
        self.y = float(new_y)
        self.orientation = float(new_orientation)    
    def set_noise(self, new_f_noise, new_t_noise, new_s_noise):
        # makes it possible to change the noise parameters
        # this is often useful in particle filters
        self.forward_noise = float(new_f_noise);
        self.turn_noise    = float(new_t_noise);
        self.sense_noise   = float(new_s_noise); 
    def sense(self):
        Z = []
        for i in range(len(landmarks)):
            dist = sqrt((self.x - landmarks[i][0]) ** 2 + (self.y - landmarks[i][1]) ** 2)
            dist += random.gauss(0.0, self.sense_noise)
            Z.append(dist)
        return Z    
    def move(self, turn, forward):
        if forward < 0:
            raise ValueError('Robot cant move backwards')                 
        # turn, and add randomness to the turning command
        orientation = self.orientation + float(turn) + random.gauss(0.0, self.turn_noise)
        orientation %= 2 * pi        
        # move, and add randomness to the motion command
        dist = float(forward) + random.gauss(0.0, self.forward_noise)
        x = self.x + (cos(orientation) * dist)
        y = self.y + (sin(orientation) * dist)
        x %= world_size    # cyclic truncate
        y %= world_size        
        # set particle, THIS IS NEW
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
    #end robot class
# myrobot = robot()
# myrobot.set_noise(5.0, 0.1, 5.0)
# myrobot.set(30.0, 50.0, pi/2)
# myrobot = myrobot.move(-pi/2, 15.0)
# print myrobot.sense()
# myrobot = myrobot.move(-pi/2, 10.0)
# print myrobot.sense()
####   DON'T MODIFY ANYTHING ABOVE HERE! ENTER CODE BELOW ####
N = 1000
p = []
for i in range(N):
    x = robot()
    p.append(x)
print(len(p))
print(p)

#14 Robot Particles
# Now we want to simulate robot
# motion with our particles.
# Each particle should turn by 0.1
# and then move by 5. 
#
# Don't modify the code below. Please enter
# your code at the bottom.
from math import *
import random
landmarks  = [[20.0, 20.0], [80.0, 80.0], [20.0, 80.0], [80.0, 20.0]]
world_size = 100.0
class robot:
    def __init__(self):
        self.x = random.random() * world_size
        self.y = random.random() * world_size
        self.orientation = random.random() * 2.0 * pi
        self.forward_noise = 0.0;
        self.turn_noise    = 0.0;
        self.sense_noise   = 0.0;    
    def set(self, new_x, new_y, new_orientation):
        if new_x < 0 or new_x >= world_size:
            raise ValueError('X coordinate out of bound')
        if new_y < 0 or new_y >= world_size:
            raise ValueError('Y coordinate out of bound')
        if new_orientation < 0 or new_orientation >= 2 * pi:
            raise ValueError('Orientation must be in [0..2pi]')
        self.x = float(new_x)
        self.y = float(new_y)
        self.orientation = float(new_orientation)   
    def set_noise(self, new_f_noise, new_t_noise, new_s_noise):
        # makes it possible to change the noise parameters
        # this is often useful in particle filters
        self.forward_noise = float(new_f_noise);
        self.turn_noise    = float(new_t_noise);
        self.sense_noise   = float(new_s_noise);    
    def sense(self):
        Z = []
        for i in range(len(landmarks)):
            dist = sqrt((self.x - landmarks[i][0]) ** 2 + (self.y - landmarks[i][1]) ** 2)
            dist += random.gauss(0.0, self.sense_noise)
            Z.append(dist)
        return Z    
    def move(self, turn, forward):
        if forward < 0:
            raise ValueError('Robot cant move backwards')                
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
# myrobot.set_noise(5.0, 0.1, 5.0)
# myrobot.set(30.0, 50.0, pi/2)
# myrobot = myrobot.move(-pi/2, 15.0)
# print myrobot.sense()
# myrobot = myrobot.move(-pi/2, 10.0)
# print myrobot.sense()
####   DON'T MODIFY ANYTHING ABOVE HERE! ENTER CODE BELOW ####
# create 100 particles, 
# rotate each particle by 0.1 in heading
N = 1000
p = []
for i in range(N):
    x = robot()
    p.append(x)
p2 = [] #a temporary particle set
for i in range(N):
    #append to list P2 the result of our motion of 0.1 and 5.0
    #applied to the ith particle chosen from the original particle set
    p2.append(p[i].move(0.1, 5.0))
p=p2
print(p) #PLEASE LEAVE THIS HERE FOR GRADING PURPOSES

#15 importance weight 
# Let me explain how the second part works. Suppose an actual robot sits over here,
# and it measures these exact distances to the landmarks over here. Obviously 
# there are some measurement noise that we just model as an added gaussian 
# with 0 mean. Meaning there would be certain chance of being too short or too long, 
# and the probability is governed by a gaussian.
# So, this process gives us a measurement vector of 4 values of those 4 distances
# to the landmarks L1 to L4. Now let's consider a particle that hypothesizes the
# robot coordinates are over here and not over here, and it als hypothesizes a 
# different heading direction. We can then take the measurement vector and apply 
# it to this particle. 
# Obviously this would be a very poor measurement vector. In particular, the measurement
# vector we've expected looks more like this. This just makes this specific location 
# really unlikely. In fact, the closer our particle to the correct position, the 
# more likely will be the set of measurements given that position.
# And now here comes the big trick in particle filters: the mismatch between actual
# measurement and the predicted measurement leads to a so called importance weight
# that tells us how important that specific particle is. The larger the weight the
# more important it is.
# Well, we now have many, many different particles and a specific measurement. Each
# of these particles will have a different weight.Some look very plausible, while 
# others look very implausible, as indicated by the size of the circles over here.
# We now let these particles survive somewhat at random, but the probability of 
# survival will be proportional to their weights.
# If something has a big weight like this guy over here, will survive at a higher
# proportion than someone with a really small weight overe here, which means after
# what's called resampling, which is just a technical term for randomly drawing
# N new particles from these old ones with replacement in proportion to the importance
# weight. 
# After that resampling phase, those guys over here very likely live on, in fact
# many, many times. That's exactly what happens in our movie in the beginning when 
# we looked at localization in the corridor environment. The particles that are 
# very consistent with the sensor measurement survive with a higher probability, 
# and the ones with lower importance weight tend to die out.
# So, we get the fact that the particles cluster around regions of higher posterior 
# probability. This is really cool and all we have to do is we have to implement
# a method for setting importance weights, and that is, of course, related to the
# likelihood of a measurement, as we will find out, and we have to implement a 
# method for resampling that grabs particles in proportion to those weights. So, 
# let's just do this.
# So, let me add back the robot code. We build a robot, and we make the robot move, 
# and now we get a sensor measurement for that specific robot using the sense function.
# So, let's just print this out. 
# These are the range or distances to the 4 landmarks, and by adding a print myrobot 
# statement, you can also figure out where the robot is, it's at 33, 48, 0.5. Obviously
# this is a random output because you randomly initialized the position of the robot.
# What I want you to program now is a way to assign importance weights to each of 
# the particles in here. I want you to make a list of 1000 elements where each element
# on the list contains a number. So this number is proportional to how important
# that particle is.
# Now we want to give weight to our 
# particles. This program will print a
# list of 1000 particle weights.
#
# Don't modify the code below. Please enter
# your code at the bottom.
from math import *
import random
landmarks  = [[20.0, 20.0], [80.0, 80.0], [20.0, 80.0], [80.0, 20.0]]
world_size = 100.0
class robot:
    def __init__(self):
        self.x = random.random() * world_size
        self.y = random.random() * world_size
        self.orientation = random.random() * 2.0 * pi
        self.forward_noise = 0.0;
        self.turn_noise    = 0.0;
        self.sense_noise   = 0.0;    
    def set(self, new_x, new_y, new_orientation):
        if new_x < 0 or new_x >= world_size:
            raise ValueError('X coordinate out of bound')
        if new_y < 0 or new_y >= world_size:
            raise ValueError('Y coordinate out of bound')
        if new_orientation < 0 or new_orientation >= 2 * pi:
            raise ValueError('Orientation must be in [0..2pi]')
        self.x = float(new_x)
        self.y = float(new_y)
        self.orientation = float(new_orientation)    
    def set_noise(self, new_f_noise, new_t_noise, new_s_noise):
        # makes it possible to change the noise parameters
        # this is often useful in particle filters
        self.forward_noise = float(new_f_noise);
        self.turn_noise    = float(new_t_noise);
        self.sense_noise   = float(new_s_noise);    
    def sense(self):
        Z = []
        for i in range(len(landmarks)):
            dist = sqrt((self.x - landmarks[i][0]) ** 2 + (self.y - landmarks[i][1]) ** 2)
            dist += random.gauss(0.0, self.sense_noise)
            Z.append(dist)
        return Z    
    def move(self, turn, forward):
        if forward < 0:
            raise ValueError('Robot cannot move backwards')                 
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
#myrobot = robot()
#myrobot.set_noise(5.0, 0.1, 5.0)
#myrobot.set(30.0, 50.0, pi/2)
#myrobot = myrobot.move(-pi/2, 15.0)
#print myrobot.sense()
#myrobot = myrobot.move(-pi/2, 10.0)
#print myrobot.sense()
####   DON'T MODIFY ANYTHING ABOVE HERE! ENTER CODE BELOW ####
myrobot = robot()
myrobot = myrobot.move(0.1, 5.0)
Z = myrobot.sense()
print(Z)
print(myrobot)
N = 1000 #particle creation
p = []
for i in range(N):
    x = robot()
    x.set_noise(0.05, 0.05, 5.0)
    p.append(x)
p2 = []
for i in range(N):
    p2.append(p[i].move(0.1, 5.0))
p = p2
# This is a list of 1000 measurement probabilities of particles, so that each
# number in this vector reflects the output of the real robot, such that when I
# hit print(w), I get a list of 1000 importance weights.
w = []
for i in range(N):
    # you construct the list W by appending the output of the function
    # measurement_prob() applied to the [i] particle, with the argument of the
    # actual measurement.
    # most of them are insanely unlikely 10^-24, 10^-36. Some of them are more
    # likely, the ones that are closer to the truth like -5. Those are the particles
    # that should survive. The others are ready to die off, because they are so
    # far away from the truth we don't really need them anymore. So, in the final
    # step of the particle filter algorithm, we just have to sample particles from
    # P with a probability that is porportional to its corresponding W value.
    # particles in p that have a large value over here, should be drawn more frequently
    # than the ones with a smaller value here. How hard can that be?
    w.append(p[i].measurement_prob(Z))
print(w) #these are the weights

#16 resampling
# And it turns out it's actually harder than you think, but once you have coded
# it you can use the exact same code forever for any particle filter. But what is
# resampling?
# We are given N particles, each of which has 3 values, and there are N of them, 
# and they also have weights. These are simple floats of continuous values. The 
# sum of all these weights is denoted big W, and let's normalize them for the consideration
# of what to do, and it's called the normalized weights alpha, where alpha1 = w1/W, 
# all the way down to alpha N,and it goes without saying that the sum of all alphas
# is now 1. What resampling does is it puts all these particles and the normalized
# weights into a big bag, and then it draws with replacement N particles by picking
# each particle with probability alpha. So, for example alpha2 might be large, so we're
# going to pick this one, P2. Alpha 3 might also be large so we pick that one. Alpha 4
# might be really small but just by chance you might actually pick it, so we have P4 
# here, and then we might pick alpha 2 again. So you get two versions of p2, perhaps
# even three versions of p2, depending on the probabilities. We have N particles 
# over here. We do this thing N times, which is why I said with replacement we can 
# draw multiple copies of the same particle, and in the end those particles that 
# have a high-normalized weight alphe over here will occur more likely in the new
# set over here. That is called resampling.
# So to make sure yo understand this, suppose we have the following importance weights:
# p1 w1=0.6
# p2 w2=1.2
# p3 w3=2.4
# p4 w4=0.6
# p5 w5=1.2
# If I, in the resampling process, randomly draw a particle in accordance to the
# normalized importance weights, what is the pro of drawing p1? We need to normalize
# the importance weights.

#17 never resampled 1
# It is actually quite likely that p1 will never get resampled in the next set,
# since it is does not have a high probability to begin with.

#18 never resampled 2
# It is also possible that p3 is never resampled again, remember the particles
# are sampled AT RANDOM.

#19 never resampled 3
# The probability of never resampling p3 is 0.0777 approximately, why?
# the probability of not drawing p3 once is 0.6, over the course of 5 independant
# trials the probability of not drawing p3 in any of those trials is 0.6^5 = 0.0777.
# Put differently, there is a 7.7% chance this particle is missing, but the probability
# that this particle is included is 93%. if we had, instead of P3, gone for P1 here, 
# which has a much smaller probability of being drawn, then this 0.07 will be as 
# large as 0.59, which is 0.9^5. This means there is a 60% chance you will lose 
# p1, and 40% chance you will include it. Put differently, particles with smaller
# weight will survive at a much lower rate than ones with larger importance weight,
# which is exactly what we wish to get from the resampling step.

#20 new particle
# In this exercise, try to write a program that
# will resample particles according to their weights.
# Particles with higher weights should be sampled
# more frequently (in proportion to their weight).
# Don't modify anything below. Please scroll to the 
# bottom to enter your code.
from math import *
import random
landmarks  = [[20.0, 20.0], [80.0, 80.0], [20.0, 80.0], [80.0, 20.0]]
world_size = 100.0
class robot:
    def __init__(self):
        self.x = random.random() * world_size
        self.y = random.random() * world_size
        self.orientation = random.random() * 2.0 * pi
        self.forward_noise = 0.0;
        self.turn_noise    = 0.0;
        self.sense_noise   = 0.0;   
    def set(self, new_x, new_y, new_orientation):
        if new_x < 0 or new_x >= world_size:
            raise ValueError('X coordinate out of bound')
        if new_y < 0 or new_y >= world_size:
            raise ValueError('Y coordinate out of bound')
        if new_orientation < 0 or new_orientation >= 2 * pi:
            raise ValueError('Orientation must be in [0..2pi]')
        self.x = float(new_x)
        self.y = float(new_y)
        self.orientation = float(new_orientation)    
    def set_noise(self, new_f_noise, new_t_noise, new_s_noise):
        # makes it possible to change the noise parameters
        # this is often useful in particle filters
        self.forward_noise = float(new_f_noise);
        self.turn_noise    = float(new_t_noise);
        self.sense_noise   = float(new_s_noise);    
    def sense(self):
        Z = []
        for i in range(len(landmarks)):
            dist = sqrt((self.x - landmarks[i][0]) ** 2 + (self.y - landmarks[i][1]) ** 2)
            dist += random.gauss(0.0, self.sense_noise)
            Z.append(dist)
        return Z    
    def move(self, turn, forward):
        if forward < 0:
            raise ValueError('Robot cant move backwards')                 
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
#myrobot = robot()
#myrobot.set_noise(5.0, 0.1, 5.0)
#myrobot.set(30.0, 50.0, pi/2)
#myrobot = myrobot.move(-pi/2, 15.0)
#print myrobot.sense()
#myrobot = myrobot.move(-pi/2, 10.0)
#print myrobot.sense()
myrobot = robot()
myrobot = myrobot.move(0.1, 5.0)
Z = myrobot.sense()
N = 1000
p = []
for i in range(N):
    x = robot()
    x.set_noise(0.05, 0.05, 5.0)
    p.append(x)
p2 = []
for i in range(N):
    p2.append(p[i].move(0.1, 5.0))
p = p2
w = []
for i in range(N):
    w.append(p[i].measurement_prob(Z))
#### DON'T MODIFY ANYTHING ABOVE HERE! ENTER CODE BELOW ####
# You should make sure that p3 contains a list with particles
# resampled according to their weights.
# Also, DO NOT MODIFY p.
p3 = []
# Now it turns out this is not an easy thing to do. An obvious idea might be to 
# complete all this normalized alphas, but you still have to be able to sample
# from those. You make alphas which are the PDF, and betas which are the CDF.
# So, in the spectrum of our alphas you might draw a random variable
# uniformly from the interval 0:1, and then find out the alpha so that all the alphas
# leading up to it, and some are smaller than beta, but if we add the new alphas 
# you would get a value larger than beta, 
# Now this is doable, but it is inefficient. In the best case you get N*log(N) 
# implementation. let me show you what is commonly done, and I take no guarantees
# that it is entirely unbiased, but there is a very simple trick.
# I get the feeling MIT is a really racist place!!!!.

#21 resampling wheel
# The pseudocode in the video should be like this (instead of an if-else block):
# while w[index] < beta:
#     beta = beta - w[index]
#     index = index + 1
# select p[index]
# So here is an idea to make this more efficient, and it turns out emperically it
# also gives better samples. let's represent all our particles and importance
# weight in a big wheel. Each particle occupies a slice that corresponds to its
# importance weight. Particles with bigger weight, like W5, occupy more space.
# Whereas particles with a smaller weight occupy less space. Very initially let's
# guess a particle index uniformly from the set of all indices. I denote this as
# a uniform sample at U from the discrete set of choices of index 1 all the way up
# to N (but in python it goes from 0 to N-1). So, say we pick W6, then the trick is
# then you are going to construct the function Beta. Then, I initialize the 0 and
# to which I add - when I construct these particles - a uniform drawn continuous
# value that sits between 0 and 2 times W max, as in b <- b + u{0...2*w_max}, which
# is the largest of the importance weights in the importance set. W5 is the largest, 
# so we're going to add a random value that might be as large as twice W5. Suppose 
# the value we add brings us to here. So, this is the value we actually drew, measured
# from the beginning of the sixth particle which shows an initialization. I now 
# iterate the following loop: if the importance weight of the present particle doesn't
# suffice to reach all the way to beta. So, if W index isn;t as big as beta, then
# I subtract from beta this very value W index and I increment index by 1. So, what 
# I done? I've moved index to over here, and I removed this part of beta so the
# point over here is still the same as before. We now get to the point where beta
# becomes smaller than W index, which is the case in the next situation. Now index=7.
# Then index is the index of the particle I pick in my resampling process. So, I
# pick the particle index; I now iterate I add another uniform value to beta. Say
# I add this one. This is the value I add, this is the value beta previously had.
# The same iteration now will make index flow up reducing beta by all the slice 
# over here, which is W7,  and then jum over here, and particle 1 is picked. It
# can easily happen that the uniform value is so small that the same particle is
# picked twice, and it's easy to see that each particle is now picked in proportion 
# to the total circumference it spans in this wheel of particles. So, this is essentially
# my implementation for the resampling step. So I want you -if you can- to implement
# that specific resampler in python.
# index = U[1..N]
# beta = 0
# for i=1..N
#   beta <- beta + U{0..2*w_max}
#   if w_max <- beta
#     beta <- beta - w_index
#     index <- index + 1
#   else
#     pick Pindex, i.e. index <- index
#
# In this exercise, you should implement the
# resampler shown in the previous video.
from math import *
import random
landmarks  = [[20.0, 20.0], [80.0, 80.0], [20.0, 80.0], [80.0, 20.0]]
world_size = 100.0
class robot:
    def __init__(self):
        self.x = random.random() * world_size
        self.y = random.random() * world_size
        self.orientation = random.random() * 2.0 * pi
        self.forward_noise = 0.0;
        self.turn_noise    = 0.0;
        self.sense_noise   = 0.0;
    def set(self, new_x, new_y, new_orientation):
        if new_x < 0 or new_x >= world_size:
            raise ValueError('X coordinate out of bound')
        if new_y < 0 or new_y >= world_size:
            raise ValueError('Y coordinate out of bound')
        if new_orientation < 0 or new_orientation >= 2 * pi:
            raise ValueError('Orientation must be in [0..2pi]')
        self.x = float(new_x)
        self.y = float(new_y)
        self.orientation = float(new_orientation)    
    def set_noise(self, new_f_noise, new_t_noise, new_s_noise):
        # makes it possible to change the noise parameters
        # this is often useful in particle filters
        self.forward_noise = float(new_f_noise);
        self.turn_noise    = float(new_t_noise);
        self.sense_noise   = float(new_s_noise);    
    def sense(self):
        Z = []
        for i in range(len(landmarks)):
            dist = sqrt((self.x - landmarks[i][0]) ** 2 + (self.y - landmarks[i][1]) ** 2)
            dist += random.gauss(0.0, self.sense_noise)
            Z.append(dist)
        return Z    
    def move(self, turn, forward):
        if forward < 0:
            raise ValueError('Robot cant move backwards')                
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
#myrobot = robot()
#myrobot.set_noise(5.0, 0.1, 5.0)
#myrobot.set(30.0, 50.0, pi/2)
#myrobot = myrobot.move(-pi/2, 15.0)
#print myrobot.sense()
#myrobot = myrobot.move(-pi/2, 10.0)
#print myrobot.sense()
####   DON'T MODIFY ANYTHING ABOVE HERE! ENTER CODE BELOW ####
print('\n' * 50) #clear screen
myrobot = robot()
myrobot = myrobot.move(0.1, 5.0)
Z = myrobot.sense()
N = 1000
p = []
for i in range(N): #set noise values to the x y and orientation to particles
    x = robot()
    x.set_noise(0.05, 0.05, 5.0)
    p.append(x)
p2 = []
for i in range(N): #move particles 0.1x 5.0y
    p2.append(p[i].move(0.1, 5.0)) 
p = p2
w = []
for i in range(N): #append the measurement probability, how likely a measurement should be
    w.append(p[i].measurement_prob(Z))
# resampling process, not domain specific, but can be reused
p3 = []
index = int(random.random() * N)#choose a random index in the wheel
beta = 0.0
mw = max(w) #max weight in the resampling wheel
for i in range(N): #resample process, using resampling wheel lesson 21
    beta += random.random() * 2.0 * mw #choose new beta based on random fraction of 2*max weight
    while beta > w[index]:      #if new beta crosses old weight into a new weight
        beta -= w[index]        #chop off old w from beta
        index = (index + 1) % N #increment index to index of new w
    #else stay with the original beta and index
    p3.append(p[index])
p = p3
print(p) 
# these are the particles after resampling
# the x and y positions are similar, but the orientation are not similar, because
# ain incorectly positioned particle CAN still give you the correct distances.

#22 orientation 1
# orientation does matter in the second step of particle filtering, because the
# prediction is so different for different orientations.

#23 orientation 2
# In this exercise, write a program that will
# run your previous code twice.
# Please only modify the indicated area below!
from math import *
import random
landmarks  = [[20.0, 20.0], [80.0, 80.0], [20.0, 80.0], [80.0, 20.0]]
world_size = 100.0
class robot:
    def __init__(self):
        self.x = random.random() * world_size
        self.y = random.random() * world_size
        self.orientation = random.random() * 2.0 * pi
        self.forward_noise = 0.0;
        self.turn_noise    = 0.0;
        self.sense_noise   = 0.0;
    def set(self, new_x, new_y, new_orientation):
        if new_x < 0 or new_x >= world_size:
            raise ValueError('X coordinate out of bound')
        if new_y < 0 or new_y >= world_size:
            raise ValueError('Y coordinate out of bound')
        if new_orientation < 0 or new_orientation >= 2 * pi:
            raise ValueError('Orientation must be in [0..2pi]')
        self.x = float(new_x)
        self.y = float(new_y)
        self.orientation = float(new_orientation)    
    def set_noise(self, new_f_noise, new_t_noise, new_s_noise):
        # makes it possible to change the noise parameters
        # this is often useful in particle filters
        self.forward_noise = float(new_f_noise);
        self.turn_noise    = float(new_t_noise);
        self.sense_noise   = float(new_s_noise);    
    def sense(self):
        Z = []
        for i in range(len(landmarks)):
            dist = sqrt((self.x - landmarks[i][0]) ** 2 + (self.y - landmarks[i][1]) ** 2)
            dist += random.gauss(0.0, self.sense_noise)
            Z.append(dist)
        return Z    
    def move(self, turn, forward):
        if forward < 0:
            raise ValueError('Robot cant move backwards')                 
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
#myrobot = robot()
#myrobot.set_noise(5.0, 0.1, 5.0)
#myrobot.set(30.0, 50.0, pi/2)
#myrobot = myrobot.move(-pi/2, 15.0)
#print myrobot.sense()
#myrobot = myrobot.move(-pi/2, 10.0)
#print myrobot.sense()
####   DON'T MODIFY ANYTHING ABOVE HERE! ENTER/MODIFY CODE BELOW ####
print('\n' * 50) #clear screen
N = 1000 #number of particles
T=10      #number of time steps
myrobot = robot()
p = []
for i in range(N):
    r = robot()
    r.set_noise(0.05, 0.05, 5.0)
    p.append(r)
myrobot = myrobot.move(0.1, 5.0)
Z = myrobot.sense()
p2 = []
for i in range(N):
    p2.append(p[i].move(0.1, 5.0))
p = p2
w = []
for i in range(N):
    w.append(p[i].measurement_prob(Z))
p3 = []
index = int(random.random() * N)
beta = 0.0
mw = max(w)
for i in range(N):
    beta += random.random() * 2.0 * mw
    while beta > w[index]:
        beta -= w[index]
        index = (index + 1) % N
    p3.append(p[index])
p = p3
print(p)
# for T=2 nothing happens
# for T=10 the orientations lines up correctly

#24 error
# What I'll do next is give you another programming assignment, instead of printing
# out the particles themselves, print out the overall quality of the solution.
# To do this I've programmed for you an eval function, which takes in a robot position,
# and a particle set, and computes the average error of each particle relative to 
# the robot position in x and y, not the in the orientation.The error is the euclidian
# distance of these differences, and averages all of those. So it sums all of them
# up, and it averages them through the number of particles over here. Now there's
# some funny stuff here. The reason is, the world is cyclic, and it might be that 
# the robot position is at 0.0 or 99.9. It's about the same values, but, according 
# to this calculation over here, they'd be different. So while there's normalization
# here, I make sure the cyclicity of the world doesn't really affect negatively
# the estimated error I've enclosed in the boundary. I'm adding "world_size" over 
# to the computer model operation and then subtract the same thing over here. I want 
# you to take the function, and produce a sequence of performance evaluations.
# Please only modify the indicated area below!
from math import *
import random
landmarks  = [[20.0, 20.0], [80.0, 80.0], [20.0, 80.0], [80.0, 20.0]]
world_size = 100.0
class robot:
    def __init__(self):
        self.x = random.random() * world_size
        self.y = random.random() * world_size
        self.orientation = random.random() * 2.0 * pi
        self.forward_noise = 0.0;
        self.turn_noise    = 0.0;
        self.sense_noise   = 0.0;
    def set(self, new_x, new_y, new_orientation):
        if new_x < 0 or new_x >= world_size:
            raise ValueError('X coordinate out of bound')
        if new_y < 0 or new_y >= world_size:
            raise ValueError('Y coordinate out of bound')
        if new_orientation < 0 or new_orientation >= 2 * pi:
            raise ValueError('Orientation must be in [0..2pi]')
        self.x = float(new_x)
        self.y = float(new_y)
        self.orientation = float(new_orientation)    
    def set_noise(self, new_f_noise, new_t_noise, new_s_noise):
        # makes it possible to change the noise parameters
        # this is often useful in particle filters
        self.forward_noise = float(new_f_noise);
        self.turn_noise    = float(new_t_noise);
        self.sense_noise   = float(new_s_noise);    
    def sense(self):
        Z = []
        for i in range(len(landmarks)):
            dist = sqrt((self.x - landmarks[i][0]) ** 2 + (self.y - landmarks[i][1]) ** 2)
            dist += random.gauss(0.0, self.sense_noise)
            Z.append(dist)
        return Z    
    def move(self, turn, forward):
        if forward < 0:
            raise ValueError('Robot cant move backwards')                 
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
    #end robot class
# calculate a mean square error between position and particle
def eval(r, p):
    sum = 0.0;
    for i in range(len(p)): # calculate mean error
        dx = (p[i].x - r.x + (world_size/2.0)) % world_size - (world_size/2.0)
        dy = (p[i].y - r.y + (world_size/2.0)) % world_size - (world_size/2.0)
        err = sqrt(dx * dx + dy * dy)
        sum += err
    return sum / float(len(p))
#myrobot = robot()
#myrobot.set_noise(5.0, 0.1, 5.0)
#myrobot.set(30.0, 50.0, pi/2)
#myrobot = myrobot.move(-pi/2, 15.0)
#print myrobot.sense()
#myrobot = myrobot.move(-pi/2, 10.0)
#print myrobot.sense()
####   DON'T MODIFY ANYTHING ABOVE HERE! ENTER/MODIFY CODE BELOW ####
print('\n' * 50) #clear screen
myrobot = robot()
myrobot = myrobot.move(0.1, 5.0)
Z = myrobot.sense()
N = 1000
T = 10 #Leave this as 10 for grading purposes.
p = []
for i in range(N):
    r = robot()
    r.set_noise(0.05, 0.05, 5.0)
    p.append(r)
# evaluate particles before particle filter
print(eval(myrobot, p)) #print it for number of steps and number of particles
print('\n' * 10) #clear screen
for t in range(T):
    myrobot = myrobot.move(0.1, 5.0)
    Z = myrobot.sense()
    p2 = []
    for i in range(N):
        p2.append(p[i].move(0.1, 5.0))
    p = p2
    w = []
    for i in range(N):
        w.append(p[i].measurement_prob(Z))
    p3 = []
    index = int(random.random() * N)
    beta = 0.0
    mw = max(w)
    for i in range(N):
        beta += random.random() * 2.0 * mw
        while beta > w[index]:
            beta -= w[index]
            index = (index + 1) % N
        p3.append(p[index])
    p = p3
    #enter code here, make sure that you output 10 print statements.
    print(eval(myrobot, p)) #print it for number of steps

#26 filters
# the measurement update and motion update equations are:
# measurement update:
# P(X|Z) ~ P(Z|X) * P(X)
# motion update:
# P(x') ~ sum {P(X'|X) * P(X)
#
# Where P(X) are the particles, P(Z|X) are the importance weights, ~ is related 
# to resampling and sent to P(X|Z). P(X'|x) is the sample, sum is the sum of the
# samples. 
#
# We have measurement update and motion update. In measurement update we computed
# posterior over state, and it was proportional to - after normalization - of probability
# of the measurement, given the state times "P" of the state itself. 
# In the motion update, if you compute a posterior of the distribution one time 
# step later, and that is the convolution of the transition probability times my
# prior. 
#
# Now thise formulas -those should look familiar. This is exatcly what you implemented.
# You might not know you implemented this; let me explain to you how you implemented
# it. This distribution was a set of particles. A thousand particles, together, 
# represented your prior "X". These were importance weights. And technically speaking,
# the particles with the importance weights, and a representation of distribution.
# But we wanted to get rid of the importance weights. So by resampling, we work
# the importance weights back into the set of particles so the resulting particles -
# the ones over here - would represent the correct posterior.
# 
# This was your set of particles again, and you samples from the sum (P(X) on bottom
# equation by taking a random particle over here and applying the motion model with
# a noise model to generate a random particle "X'". As a result, you get a new particle
# set that is the correct distribution after the robot motion. so you recognize 
# the math, and hopefully you understand how your code implements this math.
# 
# You can prove interesting things, like you can prove convergence if the number
# of particles goes to infinity. it is obviously approximate. particles are not 
# an exact representation. And it was amazingly easy to program. So when you go 
# over your particle code, you realized you implemented a fairly involved piece of
# math that is actually the same for all the filters we talked about so far.
#
# The same math underlies our histogram filter in class 1, is the kalman filter
# we talked in class no. 2. 

#2012
# The difference between the Google car and what you have learned so far, is that the
# Google car follows a bicycle model and the sensor data. Instead of using landmarks like the
# robot, the Google car uses a really elaborate road map. It takes a single snapshot, matches it
# to the map and the better the match, the higher the score. Additional sensors, like GPS and
# inertial sensors, also differentiate the Google car from your robot model.
# Despite these differences, you do have a solid understanding of the gist of how the Google
# car is able to understand where it is and where other cars are. When you build a system, you
# have to dive into more elaborate systems, which is doable.

    
 








