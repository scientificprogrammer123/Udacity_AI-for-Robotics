#1 empty cell
N=0.75**1
print(N)
N=0.75**4
print(N)
N=0.75**10
print(N)

#2 motion quiz
# A5 3B
# C3 1D
# moves 50% horizontally, 50% vertically
# never move diagonally, never move stays in the same cell
# A3 3B
# C3 3D


#3 single particle filter
# particle filter with N=1 particle
# ignores particle motion
# it likely fails


#4 circular motion
#
# A car, not a trash bin
# states are (x, y, theta)
# steering angle is (alpha)
# distance travelled is (d)
# length of car is (L)
#
# turning angle beta = d/L*tan(alpha)
# turning radius R = d/beta
# 
# Robot is turning around a point here at cx and cy.
# cx = x - sin(theta) * R
# cy = y + cos(theta) * R
# |beta| < 0.001
#
# x = cx + sin(theta + beta) * R
# y = cy - cos(theta + beta) * R
# theta = (theta + beta) mod (2*pi)
#
# updated states:
# x' = x + d * cos(theta)
# y' = y + d * sin(theta)
# theta' = (theta * beta) mod (2*pi)
# -----------------
# USER INSTRUCTIONS:
# Landmark coordinates are given in (y, x) form and NOT in the traditional (x, y) format!
# 
# World is NOT cyclic. Robot is allowed to travel "out of bounds"
# Please note that landmark coordinates are given in the form (y, x): not in the 
# traditional (x, y) format. In this assignment, treat the world as an infinite 
# grid: the robot can move so that its x- or y-coordinates are larger in magnitude 
# than the world_size variable. (The world_size variable only affects the robot's 
# initial coordinates.)
#
# -----------------
# USER INSTRUCTIONS
#
# Write a function in the class robot called move()
#
# that takes self and0.75 a motion vector (this
# motion vector contains a steering* angle and a
# distance) as input and returns an instance of the class
# robot with the appropriate x, y, and orientation
# for the given motion.
#
# *steering is defined in the video
# which accompanies this problem.
#
# For now, please do NOT add noise to your move function.
#
# Please do not modify anything except where indicated
# below.
#
# There are test cases which you are free to use at the
# bottom. If you uncomment them for testing, make sure you
# re-comment them before you submit.
from math import *
import random
# --------
# the "world" has 4 landmarks.
# the robot's initial coordinates are somewhere in the square
# represented by the landmarks.
#
# NOTE: Landmark coordinates are given in (y, x) form and NOT
# in the traditional (x, y) format!
landmarks  = [[0.0, 100.0], [0.0, 0.0], [100.0, 0.0], [100.0, 100.0]] # position of 4 landmarks
world_size = 100.0 # world is NOT cyclic. Robot is allowed to travel "out of bounds"
max_steering_angle = pi/4 # You don't need to use this value, but it is good to keep in mind the limitations of a real car.
# ------------------------------------------------
# this is the robot class
class robot:
    # --------
    # init: 
    #	creates robot and initializes location/orientation 
    #
    def __init__(self, length = 10.0):
        self.x = random.random() * world_size # initial x position
        self.y = random.random() * world_size # initial y position
        self.orientation = random.random() * 2.0 * pi # initial orientation
        self.length = length # length of robot
        self.bearing_noise  = 0.0 # initialize bearing noise to zero
        self.steering_noise = 0.0 # initialize steering noise to zero
        self.distance_noise = 0.0 # initialize distance noise to zero    
    def __repr__(self):
        return '[x=%.6s y=%.6s orient=%.6s]' % (str(self.x), str(self.y), str(self.orientation))
    # --------
    # set: 
    #	sets a robot coordinate
    #
    def set(self, new_x, new_y, new_orientation):
        if new_orientation < 0 or new_orientation >= 2 * pi:
            raise (ValueError, 'Orientation must be in [0..2pi]')
        self.x = float(new_x)
        self.y = float(new_y)
        self.orientation = float(new_orientation)
    # --------
    # set_noise: 
    #	sets the noise parameters
    #
    def set_noise(self, new_b_noise, new_s_noise, new_d_noise):
        # makes it possible to change the noise parameters
        # this is often useful in particle filters
        self.bearing_noise  = float(new_b_noise)
        self.steering_noise = float(new_s_noise)
        self.distance_noise = float(new_d_noise)    
    ############# ONLY ADD/MODIFY CODE BELOW HERE ###################
    # --------
    # move:
    #   move along a section of a circular path according to motion
    #    
    def move(self, motion, tolerance = 0.001): # Do not change the name of this function
        steering = motion[0]
        distance = motion[1]
        if abs(steering) > max_steering_angle:
            raise ValueError('Exceeding max steering angle')
        if distance < 0.0:
            raise ValueError('Moving backwards is not valid') 
        #make a new copy    
        res = robot()
        res.length         = self.length
        res.bearing_noise  = self.bearing_noise
        res.steering_noise = self.steering_noise
        res.distance_noise = self.distance_noise   
        #apply noise
        steering2 = random.gauss(steering, self.steering_noise)
        distance2 = random.gauss(distance, self.distance_noise)
        #execute motion
        turn = tan(steering2) * distance2 / res.length
        if abs(turn) < tolerance:
            #approximate by straight line motion
            res.x = self.x + (distance2 * cos(self.orientation))
            res.y = self.y + (distance2 * sin(self.orientation))  
            res.orientation = (self.orientation + turn) % (2.0 * pi) 
        else:
            #apprixmiate bycicle model for motion
            radius = distance2 / turn
            cx = self.x - (sin(self.orientation) * radius)
            cy = self.y + (cos(self.orientation) * radius)
            res.orientation = (self.orientation + turn) % (2.0 * pi)
            res.x = cx + (sin(res.orientation) * radius)
            res.y = cy - (cos(res.orientation) * radius)            
        return res # make sure your move function returns an instance
                      # of the robot class with the correct coordinates.                      
    ############## ONLY ADD/MODIFY CODE ABOVE HERE ####################
## IMPORTANT: You may uncomment the test cases below to test your code.
## But when you submit this code, your test cases MUST be commented
## out. Our testing program provides its own code for testing your
## move function with randomized motion data.
## --------
## TEST CASE:
## 
## 1) The following code should print:
##       Robot:     [x=0.0 y=0.0 orient=0.0]
##       Robot:     [x=10.0 y=0.0 orient=0.0]
##       Robot:     [x=19.861 y=1.4333 orient=0.2886]
##       Robot:     [x=39.034 y=7.1270 orient=0.2886]
##
##
'''
length = 20.
bearing_noise  = 0.0
steering_noise = 0.0
distance_noise = 0.0
##
myrobot = robot(length)
myrobot.set(0.0, 0.0, 0.0)
myrobot.set_noise(bearing_noise, steering_noise, distance_noise)
##
motions = [[0.0, 10.0], [pi / 6.0, 10], [0.0, 20.0]]
##
T = len(motions)
##
print('Robot:    ', myrobot)
for t in range(T):
    myrobot = myrobot.move(motions[t])
    print('Robot:    ', myrobot)
'''
##
##
## IMPORTANT: You may uncomment the test cases below to test your code.
## But when you submit this code, your test cases MUST be commented
## out. Our testing program provides its own code for testing your
## move function with randomized motion data.
## 2) The following code should print: (and it is)
##      Robot:     [x=0.0 y=0.0 orient=0.0]
##      Robot:     [x=9.9828 y=0.5063 orient=0.1013]
##      Robot:     [x=19.863 y=2.0201 orient=0.2027]
##      Robot:     [x=29.539 y=4.5259 orient=0.3040]
##      Robot:     [x=38.913 y=7.9979 orient=0.4054]
##      Robot:     [x=47.887 y=12.400 orient=0.5067]
##      Robot:     [x=56.369 y=17.688 orient=0.6081]
##      Robot:     [x=64.273 y=23.807 orient=0.7094]
##      Robot:     [x=71.517 y=30.695 orient=0.8108]
##      Robot:     [x=78.027 y=38.280 orient=0.9121]
##      Robot:     [x=83.736 y=46.485 orient=1.0135]
##
##
length = 20.
bearing_noise  = 0.0
steering_noise = 0.0
distance_noise = 0.0
##
myrobot = robot(length)
myrobot.set(0.0, 0.0, 0.0)
myrobot.set_noise(bearing_noise, steering_noise, distance_noise)
##
motions = [[0.2, 10.] for row in range(10)]
##
T = len(motions)
##
print('Robot:    ', myrobot)
for t in range(T):
    myrobot = myrobot.move(motions[t])
    print('Robot:    ', myrobot)
## IMPORTANT: You may uncomment the test cases below to test your code.
## But when you submit this code, your test cases MUST be commented
## out. Our testing program provides its own code for testing your
## move function with randomized motion data.


#5 sensing
# this is a measurement model, that is more characteristic of what's often in the
# literature on robotics.
# There is a robot, and a landmark, then the robot can measure bearing or angle
# to this landmark relative to its own local coordinate system. Whereas before we
# measured ranges or distances, now we measure bearings or angles.
# We assume the world has 4 landmarks, all of which are distinguishable. The measurement
# vector is a set of 4 bearings that correspond to those 4 different landmarks.
# When implementing this, use the function arctan, atan2(delta_x, delta_y) and gives
# orientation of vector in global coordinates. 
# We adjust for the fact that it's relative to robot's local coordinates, done by
# subtracting the orientation of the robot. This should give you implementation of
# bearing to a landmark.
# --------------
# USER INSTRUCTIONS
#
# Write a function in the class robot called sense()
# that takes self as input
# and returns a list, Z, of the four bearings* to the 4
# different landmarks. you will have to use the robot's
# x and y position, as well as its orientation, to
# compute this.
#
# *bearing is defined in the video
# which accompanies this problem.
#
# For now, please do NOT add noise to your sense function.
#
# Please do not modify anything except where indicated
# below.
#
# There are test cases provided at the bottom which you are
# free to use. If you uncomment any of these cases for testing
# make sure that you re-comment it before you submit.
from math import *
import random
# --------
# # the "world" has 4 landmarks.
# the robot's initial coordinates are somewhere in the square
# represented by the landmarks.
#
# NOTE: Landmark coordinates are given in (y, x) form and NOT
# in the traditional (x, y) format!
landmarks  = [[0.0, 100.0], [0.0, 0.0], [100.0, 0.0], [100.0, 100.0]] # position of 4 landmarks in (y, x) form.
world_size = 100.0 # world is NOT cyclic. Robot is allowed to travel "out of bounds"
# ------------------------------------------------
# 
# this is the robot class
#
class robot:
    # --------
    # init: 
    #	creates robot and initializes location/orientation
    #
    def __init__(self, length = 10.0):
        self.x = random.random() * world_size # initial x position
        self.y = random.random() * world_size # initial y position
        self.orientation = random.random() * 2.0 * pi # initial orientation
        self.length = length # length of robot
        self.bearing_noise  = 0.0 # initialize bearing noise to zero
        self.steering_noise = 0.0 # initialize steering noise to zero
        self.distance_noise = 0.0 # initialize distance noise to zero
    def __repr__(self):
        return '[x=%.6s y=%.6s orient=%.6s]' % (str(self.x), str(self.y), str(self.orientation))
    # --------
    # set: 
    #	sets a robot coordinate
    #
    def set(self, new_x, new_y, new_orientation):
        if new_orientation < 0 or new_orientation >= 2 * pi:
            raise ValueError('Orientation must be in [0..2pi]')
        self.x = float(new_x)
        self.y = float(new_y)
        self.orientation = float(new_orientation)
    # --------
    # set_noise: 
    #	sets the noise parameters
    #
    def set_noise(self, new_b_noise, new_s_noise, new_d_noise):
        # makes it possible to change the noise parameters
        # this is often useful in particle filters
        self.bearing_noise  = float(new_b_noise)
        self.steering_noise = float(new_s_noise)
        self.distance_noise = float(new_d_noise)
    ############# ONLY ADD/MODIFY CODE BELOW HERE ###################
    # --------
    # sense:
    #   obtains bearings from positions
    #    
    def sense(self, add_noise = 1): #do not change the name of this function
        Z = [] #for 4 bearings
        # ENTER CODE HERE
        # HINT: You will probably need to use the function atan2()
        for i in range(len(landmarks)):
            bearing = atan2(landmarks[i][0] - self.y,
                            landmarks[i][1] - self.x) - self.orientation
            if add_noise:
                bearing += random.gauss(0.0, self.bearing_noise)
            bearing %= 2.0 * pi
            Z.append(bearing)        
        return Z #Leave this line here. Return vector Z of 4 bearings.    
    ############## ONLY ADD/MODIFY CODE ABOVE HERE ####################
## IMPORTANT: You may uncomment the test cases below to test your code.
## But when you submit this code, your test cases MUST be commented
## out. Our testing program provides its own code for testing your
## sense function with randomized initial robot coordinates.    
## --------
## TEST CASES:
##
## 1) The following code should print the list [6.004885648174475, 3.7295952571373605, 1.9295669970654687, 0.8519663271732721]
##
##
'''
length = 20.
bearing_noise  = 0.0
steering_noise = 0.0
distance_noise = 0.0
##
myrobot = robot(length)
myrobot.set(30.0, 20.0, 0.0)
myrobot.set_noise(bearing_noise, steering_noise, distance_noise)
motions = [[-0.2, 10.0] for row in range(10)]
T = len(motions)
##
print('Robot:        ', myrobot)
print('Measurements: ', myrobot.sense())
'''
##
## IMPORTANT: You may uncomment the test cases below to test your code.
## But when you submit this code, your test cases MUST be commented
## out. Our testing program provides its own code for testing your
## sense function with randomized initial robot coordinates.
##
## 2) The following code should print the list [5.376567117456516, 3.101276726419402, 1.3012484663475101, 0.22364779645531352]
##
##
length = 20.
bearing_noise  = 0.0
steering_noise = 0.0
distance_noise = 0.0
##
myrobot = robot(length)
myrobot.set(30.0, 20.0, pi / 5.0)
myrobot.set_noise(bearing_noise, steering_noise, distance_noise)
##
print ('Robot:        ', myrobot)
print ('Measurements: ', myrobot.sense())
#### IMPORTANT: You may uncomment the test cases below to test your code.
## But when you submit this code, your test cases MUST be commented
## out. Our testing program provides its own code for testing your
## sense function with randomized initial robot coordinates.
    
    
#6 final quiz
# # --------------
# USER INSTRUCTIONS
#
# Now you will put everything together.
#
# First make sure that your sense and move functions
# work as expected for the test cases provided at the
# bottom of the previous two programming assignments.
# Once you are satisfied, copy your sense and move
# definitions into the robot class on this page, BUT
# now include noise.
#
# A good way to include noise in the sense step is to
# add Gaussian noise, centered at zero with variance
# of self.bearing_noise to each bearing. You can do this
# with the command random.gauss(0, self.bearing_noise)
#
# In the move step, you should make sure that your
# actual steering angle is chosen from a Gaussian
# distribution of steering angles. This distribution
# should be centered at the intended steering angle
# with variance of self.steering_noise.
#
# Feel free to use the included set_noise function.
#
# Please do not modify anything except where indicated
# below.
from math import *
import random
# --------
# 
# some top level parameters
#
max_steering_angle = pi / 4.0 # You do not need to use this value, but keep in mind the limitations of a real car.
bearing_noise  = 0.1  # Noise parameter: should be included in sense function.
steering_noise = 0.1  # Noise parameter: should be included in move function.
distance_noise = 5.0  # Noise parameter: should be included in move function.
tolerance_xy   = 15.0 # Tolerance for localization in the x and y directions.
tolerance_orientation = 0.25 # Tolerance for orientation.
# --------
# 
# the "world" has 4 landmarks.
# the robot's initial coordinates are somewhere in the square
# represented by the landmarks.
#
# NOTE: Landmark coordinates are given in (y, x) form and NOT
# in the traditional (x, y) format!
landmarks  = [[0.0, 100.0], [0.0, 0.0], [100.0, 0.0], [100.0, 100.0]] # position of 4 landmarks in (y, x) format.
world_size = 100.0 # world is NOT cyclic. Robot is allowed to travel "out of bounds"
# ------------------------------------------------
# 
# this is the robot class
#
class robot:
    # --------
    # init: 
    #    creates robot and initializes location/orientation 
    #
    def __init__(self, length = 20.0):
        self.x = random.random() * world_size # initial x position
        self.y = random.random() * world_size # initial y position
        self.orientation = random.random() * 2.0 * pi # initial orientation
        self.length = length # length of robot
        self.bearing_noise  = 0.0 # initialize bearing noise to zero
        self.steering_noise = 0.0 # initialize steering noise to zero
        self.distance_noise = 0.0 # initialize distance noise to zero
    # --------
    # set: 
    #    sets a robot coordinate
    #
    def set(self, new_x, new_y, new_orientation):
        if new_orientation < 0 or new_orientation >= 2 * pi:
            raise ValueError('Orientation must be in [0..2pi]')
        self.x = float(new_x)
        self.y = float(new_y)
        self.orientation = float(new_orientation)
    # --------
    # set_noise: 
    #    sets the noise parameters
    #
    def set_noise(self, new_b_noise, new_s_noise, new_d_noise):
        # makes it possible to change the noise parameters
        # this is often useful in particle filters
        self.bearing_noise  = float(new_b_noise)
        self.steering_noise = float(new_s_noise)
        self.distance_noise = float(new_d_noise)
    # --------
    # measurement_prob
    #    computes the probability of a measurement
    #  
    # pre notes:
    # So here is the measurement probability function, there is is something 
    # non-trivial here, I compute the predicted measurements, and then I compute
    # a gaussian that measures the distance between the measurement passed into
    # the routine and the measurement passed into the routine. That's all happening
    # down here. Here's my gaussian function with the exponential "update gaussian".
    # Then I return my gaussian error, there should be no suprise here. What's 
    # importance is a little modification to the sense function that we haven't 
    # seen before. I can now give the sense function a parameter, and I give it 
    # the paremeter 0 for forward simulation of the robot.
    # But you don't need it for computing the probability of the measurement. It
    # auguments your sense function to have a flag that if it's set to 0 it switches
    # off the noise modelling and gets you the predicted best possible measurements.
    #    
    # post notes:
    # So to implement the full particle filter, the only thing missing is the
    # measurement_prob function. And that's a little bit more involved because I
    # have to really compare what the exact measurement would be for any ove, overt
    # particle. And what I sense and computh the pribability correspondance between
    # the correct measurement, and what I sensed over here. To do this, I calculate
    # predicted measurements using the sense function. Here comes a little flag 
    # that i defined. If I set it to 0, then the sense function acts noise free.
    # Which is what I want, it could be the measurement model. But even when we
    # left this out, you are going to get a fine answer in my opinion. But that 
    # makes it a little bit more accurate.
    # So that allows me to compute the exact bearings of the landmarks for my parcticle.
    # And then I can compare these correct bearings called predicted measurements.
    # with the ones I received. Now to do this, down here, in the compute errors
    # routine. Where I go through each measurement and in two steps, I calculate 
    # error in bearing. First, it;s the absolute difference between my measurement
    # that I observed, minus the predicted measurement, and there's an i at the
    # end over here. Right there. And this difference might fall outside minus pi 
    # plus pi. So this line over here just brings it back to the smallest possible
    # value in the cyclic space of 0 to 2 pi. So adding pi, adding more load 2 
    # times pi and I subtract pi again. So this gives me a value between minus pi 
    # plus pi. I then pluck this error-bearing into a gaussian. And here is my 
    # gaussian where I squared it, I divide it by my bearing-noise squared, complete
    # the exponential, and use my normalizer to strictly speaking of, don't really
    # need for the implementation, I can safely omit because weights are self-normalized.
    # But i left it in, so it's actually really a gaussian. And I take this gaussian
    # value and multiply it up into my error function. So for each of the measurements
    # I multiply in one gaussian, and the final gaussian is my importance whether I
    # return in this function over here. 
    # So this is not easy to implement, I hope you got it right. 
    #
    def measurement_prob(self, measurements):
        # calculate the correct measurement
        predicted_measurements = self.sense(0) # Our sense function took 0 as an argument to switch off noise.
        # compute errors
        error = 1.0
        for i in range(len(measurements)):
            #print(i)
            #pPROBLEM HERE!!:
            #measurements has 8 measurements
            #predicted_measurements = self.sense(0)
            #sense() has only 4 elements
            #list index out or range error encountered... due to incorrect indent
            error_bearing = abs(measurements[i] - predicted_measurements[i])
            error_bearing = (error_bearing + pi) % (2.0 * pi) - pi # truncate
            # update Gaussian
            error *= (exp(- (error_bearing ** 2) / (self.bearing_noise ** 2) / 2.0) /  
                      sqrt(2.0 * pi * (self.bearing_noise ** 2)))
        return error    
    def __repr__(self): #allows us to print robot attributes.
        return '[x=%.6s y=%.6s orient=%.6s]' % (str(self.x), str(self.y), 
                                                str(self.orientation))    
    #
    # You have to copy over your move function and work in and, as it say in the
    # instructions, the steering noise and the distance noise. Then you also have 
    # to plug in the sense function, and you also ahve to plug in bearing noise
    # and make sure there's a flag that allows you to switch off the bearing noise.
    # It should be an optional flag, otherwise your code won't run.
    #
    ############# ONLY ADD/MODIFY CODE BELOW HERE ###################       
    # --------
    # move:       
    # copy your code from the previous exercise
    # and modify it so that it simulates motion noise
    # according to the noise parameters
    #           self.steering_noise
    #           self.distance_noise
    # --------
    def move(self, motion, tolerance = 0.001): # Do not change the name of this function
        steering = motion[0]
        distance = motion[1]
        res = robot()
        res.length         = self.length
        res.bearing_noise  = self.bearing_noise
        res.distance_noise = self.distance_noise 
        #res.steering_noise = self.steering_noise
        if abs(steering) > max_steering_angle:
            raise ValueError('Exceeding max steering angle')
        if distance < 0.0:
            raise ValueError('Moving backwards is not valid')      
        #apply noise
        steering2 = random.gauss(steering, self.steering_noise)
        distance2 = random.gauss(distance, self.distance_noise)
        #execute motion
        turn = tan(steering2) * distance2 / res.length
        #turn = distance/self.length*tan(steering)
        if abs(turn) < tolerance:
            #approximate by straight line motion
            res.x = self.x + (distance2 * cos(self.orientation))
            res.y = self.y + (distance2 * sin(self.orientation))  
            res.orientation = (self.orientation + turn) % (2.0 * pi) 
        else:
            #apprixmiate bycicle model for motion
            radius = distance2 / turn
            cx = self.x - (sin(self.orientation) * radius)
            cy = self.y + (cos(self.orientation) * radius)
            res.orientation = (self.orientation + turn) % (2.0 * pi)
            res.x = cx + (sin(res.orientation) * radius)
            res.y = cy - (cos(res.orientation) * radius)            
        return res # make sure your move function returns an instance
                      # of the robot class with the correct coordinates.                      
    # sense:    
    # copy your code from the previous exercise
    # and modify it so that it simulates bearing noise
    # according to
    #           self.bearing_noise
    # --------
    # sense:
    #   obtains bearings from positions
    #    
    def sense(self, add_noise = 1): #do not change the name of this function
        Z = [] #for 4 bearings
        # ENTER CODE HERE
        # HINT: You will probably need to use the function atan2()
        for i in range(len(landmarks)):
            bearing = atan2(landmarks[i][0] - self.y,
                            landmarks[i][1] - self.x) - self.orientation
            if add_noise:
                bearing += random.gauss(0.0, self.bearing_noise)
            bearing %= 2.0 * pi
            Z.append(bearing)        
        return Z #Leave this line here. Return vector Z of 4 bearings.
    ############## ONLY ADD/MODIFY CODE ABOVE HERE ####################    
# --------
#
# extract position from a particle set
# 
def get_position(p):
    x = 0.0
    y = 0.0
    orientation = 0.0
    for i in range(len(p)):
        x += p[i].x
        y += p[i].y
        # orientation is tricky because it is cyclic. By normalizing
        # around the first particle we are somewhat more robust to
        # the 0=2pi problem
        orientation += (((p[i].orientation - p[0].orientation + pi) % (2.0 * pi)) 
                        + p[0].orientation - pi)
    return [x / len(p), y / len(p), orientation / len(p)]
# --------
#
# The following code generates the measurements vector
# You can use it to develop your solution.
# 
def generate_ground_truth(motions):
    myrobot = robot()
    myrobot.set_noise(bearing_noise, steering_noise, distance_noise)
    Z = []
    T = len(motions)
    for t in range(T):
        myrobot = myrobot.move(motions[t])
        Z.append(myrobot.sense())
    #print 'Robot:    ', myrobot
    return [myrobot, Z]
# --------
#
# The following code prints the measurements associated
# with generate_ground_truth
#
def print_measurements(Z):
    T = len(Z)
    print ('measurements = [[%.8s, %.8s, %.8s, %.8s],' % \
        (str(Z[0][0]), str(Z[0][1]), str(Z[0][2]), str(Z[0][3])))
    for t in range(1,T-1):
        print ('                [%.8s, %.8s, %.8s, %.8s],' % \
            (str(Z[t][0]), str(Z[t][1]), str(Z[t][2]), str(Z[t][3])))
        print ('                [%.8s, %.8s, %.8s, %.8s]]' % \
            (str(Z[T-1][0]), str(Z[T-1][1]), str(Z[T-1][2]), str(Z[T-1][3])))
# --------
#
# The following code checks to see if your particle filter
# localizes the robot to within the desired tolerances
# of the true position. The tolerances are defined at the top.
#
def check_output(final_robot, estimated_position):
    error_x = abs(final_robot.x - estimated_position[0])
    error_y = abs(final_robot.y - estimated_position[1])
    error_orientation = abs(final_robot.orientation - estimated_position[2])
    error_orientation = (error_orientation + pi) % (2.0 * pi) - pi
    correct = error_x < tolerance_xy and error_y < tolerance_xy \
              and error_orientation < tolerance_orientation
    return correct
# Scrolling futher down in my code, I now implement the particle field as follows.
# And this is exactly the same routine we had before, where we generate our 
# initial particle. Here, I set the noise for these particles, to be bearing-noise,
# steering-noise and distance-noise.
# I don't comment out the measurement generation step, I just take the input.
# And then as I go gurther down, I just run my particle theta. This is exactly 
# the same cde you are familiar with. there is a motion update, there is a 
# measurement update, and there's a resampling step over here. And at the very 
# end I just print the result of get_position.
def particle_filter(motions, measurements, N=500): # I know it's tempting, but don't change N!
    # --------
    #
    # Make particles
    # 
    N = 1000
    p = []
    for i in range(N):
        r = robot()
        r.set_noise(bearing_noise, steering_noise, distance_noise)
        p.append(r)
    # --------
    #
    # Update particles
    #     
    for t in range(len(motions)):    
        # motion update (prediction)
        p2 = []
        for i in range(N):
            p2.append(p[i].move(motions[t]))
        p = p2
        # measurement update
        w = []
        for i in range(N):
            w.append(p[i].measurement_prob(measurements[t]))
        # resampling
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
    # So if I do this for my example, here is the position I get. And I guess for,
    # I forgot to uncomment the robot coordinate over here. Bur if you look at the 
    # values over here, 7.0 is about the same as 8, 49 is about the same as 48, 
    # and 4.31 is about the same as 4.35. so this particle filter, clearly does
    # a pretty good job in estimating the forward position.
    #print(get_position(p))
    return get_position(p)
## IMPORTANT: You may uncomment the test cases below to test your code.
## But when you submit this code, your test cases MUST be commented
## out.
##
## You can test whether your particle filter works using the
## function check_output (see test case 2). We will be using a similar
## function. Note: Even for a well-implemented particle filter this
## function occasionally returns False. This is because a particle
## filter is a randomized algorithm. We will be testing your code
## multiple times. Make sure check_output returns True at least 80%
## of the time.
## --------
## TEST CASES:
## 
##1) Calling the particle_filter function with the following
##    motions and measurements should return a [x,y,orientation]
##    vector near [x=93.476 y=75.186 orient=5.2664], that is, the
##    robot's true location.
##    This was the ground true final position.
##
# 8 motions, at each of these time steps, the robot turns a little bit and moves
# forward.
# It also has 8 measurements, which are the bearings to those 4 different landmarks.
motions = [[2. * pi / 10, 20.] for row in range(8)]
measurements = [[4.746936, 3.859782, 3.045217, 2.045506],
                [3.510067, 2.916300, 2.146394, 1.598332],
                [2.972469, 2.407489, 1.588474, 1.611094],
                [1.906178, 1.193329, 0.619356, 0.807930],
                [1.352825, 0.662233, 0.144927, 0.799090],
                [0.856150, 0.214590, 5.651497, 1.062401],
                [0.194460, 5.660382, 4.761072, 2.471682],
                [5.717342, 4.736780, 3.909599, 2.342536]]

print('\n' * 50)
print('Estimated Car status (x, y, orientation):')
print(particle_filter(motions, measurements)) # this is the final value

##
## 2) You can generate your own test cases by generating
##    measurements using the generate_ground_truth function.
##    It will print the robot's last location when calling it.
##
##
'''
##number_of_iterations = 6
number_of_iterations = 8
##motions = [[2. * pi / 20, 12.] for row in range(number_of_iterations)]
motions = [[2. * pi / 10, 20.] for row in range(number_of_iterations)]
##
# gives you a set of measurements and a robot position that we can split as follows
# using a robot simulation.
x = generate_ground_truth(motions)
final_robot = x[0]
measurements = x[1]
# then you run the particle filter over here, and the function "check_output" down
# here compares the final robot position, and the ground truth, with your particle
# filter position, estimated position, from here and gives us a single flag whether
# it is all correct. 
estimated_position = particle_filter(motions, measurements)
##print_measurements(measurements)
print('Ground truth:    ', final_robot)
print('Particle filter: ', estimated_position)
print('Code check:      ', check_output(final_robot, estimated_position))
'''


