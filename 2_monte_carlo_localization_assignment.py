# The function localize takes the following arguments:
#
# colors:
#        2D list, each entry either 'R' (for red cell) or 'G' (for green cell)
#
# measurements:
#        list of measurements taken by the robot, each entry either 'R' or 'G'
#
# motions:
#        list of actions taken by the robot, each entry of the form [dy,dx],
#        where dx refers to the change in the x-direction (positive meaning
#        movement to the right) and dy refers to the change in the y-direction
#        (positive meaning movement downward)
#        NOTE: the *first* coordinate is change in y; the *second* coordinate is
#              change in x
#
# sensor_right:
#        float between 0 and 1, giving the probability that any given
#        measurement is correct; the probability that the measurement is
#        incorrect is 1-sensor_right
#
# p_move:
#        float between 0 and 1, giving the probability that any given movement
#        command takes place; the probability that the movement command fails
#        (and the robot remains still) is 1-p_move; the robot will NOT overshoot
#        its destination in this exercise
#
# The function should RETURN (not just show or print) a 2D list (of the same
# dimensions as colors) that gives the probabilities that the robot occupies
# each cell in the world.
#
# Compute the probabilities by assuming the robot initially has a uniform
# probability of being in any cell.
#
# Also assume that at each step, the robot:
# 1) first makes a movement,
# 2) then takes a measurement.
#
# Motion:
#  [0,0] - stay
#  [0,1] - right
#  [0,-1] - left
#  [1,0] - down
#  [-1,0] - up

################################################################################
# start of program

# global variables
sensor_right = 0.7
sensor_wrong = 1.0 - sensor_right #book keeping assignment
p_move = 0.8
p_stay = 1.0 - p_move 

# sense routine
def sense(p, colors, measurement): #input is prob distribution, world map, measurement
    #construct and cite new posterior distribution
    #initialuze with zero, set same size as vector p
    aux = [[0.0 for row in range(len(p[0]))] for col in range(len(p))]    
    s = 0.0 
    for i in range(len(p)): #iterate over all elements of grid cell
        for j in range(len(p[i])):
            hit = (measurement == colors[i][j]) #does measurement match color in cell?
            aux[i][j] = p[i][j] * (hit * sensor_right + (1-hit) * sensor_wrong)
            s += aux[i][j] #add up all the values in aux
    for i in range(len(aux)):
        for j in range(len(p[i])):
            aux[i][j] /= s #normalize aux polynomial to have total probability of 1
    return aux #end up at line 37

# move routine
def move(p, motion):
    aux = [[0.0 for row in range(len(p[0]))] for col in range(len(p))]
    for i in range(len(p)):
        for j in range(len(p[i])):
            aux[i][j] = (p_move * p[(i - motion[0]) % len(p)][j - motion[1] % len(p[i])]) + (p_stay * p[i][j])            
    return aux #corresponding posterior distribution
    
# routine to show the posterior
def show(p):
    rows = ['[' + ','.join(map(lambda x: '{0:.5f}'.format(x),r)) + ']' for r in p]
    print('[' + ',\n '.join(rows) + ']')
    #for i in range(len(p)):
    #    print(p[i])
    
#MAIN ROUTINE, localize
def localize(colors, measurements, motions, sensor_right, p_move):
    # check length of vector        
    if (len(measurements) != len(motions)): 
        raise ValueError('error in size of measurement/motion vector')    
    # initialize probability table   
    pinit = 1.0/float(len(colors)) / float(len(colors[0])) #compute initial uniform distribution
    # builds up an array of the size of the colors array
    p = [[pinit for row in range(len(colors[0]))] for col in range(len(colors))]
    # iterate
    for k in range(len(measurements)):
        p = move(p, motions[k])               #predict
        p = sense(p, colors, measurements[k]) #update, new probabilty distribution
    # output final distribution
    return p       

################################################################################
# test cases
#
# For the following test case, your output should be 
# [[0.01105, 0.02464, 0.06799, 0.04472, 0.02465],
#  [0.00715, 0.01017, 0.08696, 0.07988, 0.00935],
#  [0.00739, 0.00894, 0.11272, 0.35350, 0.04065],
#  [0.00910, 0.00715, 0.01434, 0.04313, 0.03642]]
# (within a tolerance of +/- 0.001 for each entry)
colors = [['R','G','G','R','R'],
          ['R','R','G','R','R'],
          ['R','R','G','G','R'],
          ['R','R','R','R','R']]
measurements = ['G','G','G','G','G']
motions = [[0,0],[0,1],[1,0],[1,0],[0,1]] #2 dimension space of moves
p = localize(colors,measurements,motions,sensor_right = 0.7, p_move = 0.8)
show(p) # displays your answer

'''
################################################################################
# Example codes in tutorial, related to 1D
def sense(p, Z):
    q=[]
    for i in range(len(p)):
        hit = (Z == world[i]) 
        q.append(p[i] * (hit * pHit + (1-hit) * pMiss))
    s = sum(q)
    for i in range(len(q)):
        q[i] = q[i] / s
    return q
def move(p, U):
    q = []
    for i in range(len(p)):
        s = pExact * p[(i-U) % len(p)]
        s = s + pOvershoot * p[(i-U-1) % len(p)]
        s = s + pUndershoot * p[(i-U+1) % len(p)]
        q.append(s)
    return q
'''

'''
################################################################################
# test cases from tutorial
# test 1
colors = [['G', 'G', 'G'],
          ['G', 'R', 'G'],
          ['G', 'G', 'G']]
measurements = ['R']
motions = [[0,0]]
sensor_right = 1.0
p_move = 1.0
p = localize(colors,measurements,motions,sensor_right,p_move)
correct_answer = ([[0.0, 0.0, 0.0],
                  [0.0, 1.0, 0.0],
                  [0.0, 0.0, 0.0]])
# test 2
colors = [['G', 'G', 'G'],
          ['G', 'R', 'R'],
          ['G', 'G', 'G']]
measurements = ['R']
motions = [[0,0]]
sensor_right = 1.0
p_move = 1.0
p = localize(colors,measurements,motions,sensor_right,p_move)
correct_answer = ([[0.0, 0.0, 0.0],
                   [0.0, 0.5, 0.5],
                   [0.0, 0.0, 0.0]])
# test 3
colors = [['G', 'G', 'G'],
          ['G', 'R', 'R'],
          ['G', 'G', 'G']]
measurements = ['R']
motions = [[0,0]]
sensor_right = 0.8
p_move = 1.0
p = localize(colors,measurements,motions,sensor_right,p_move)
correct_answer = ([[0.06666666666, 0.06666666666, 0.06666666666],
                   [0.06666666666, 0.26666666666, 0.26666666666],
                   [0.06666666666, 0.06666666666, 0.06666666666]])
# test 4
colors = [['G', 'G', 'G'],
          ['G', 'R', 'R'],
          ['G', 'G', 'G']]
measurements = ['R', 'R']
motions = [[0,0], [0,1]]
sensor_right = 0.8
p_move = 1.0
p = localize(colors,measurements,motions,sensor_right,p_move)
correct_answer = ([[0.03333333333, 0.03333333333, 0.03333333333],
                   [0.13333333333, 0.13333333333, 0.53333333333],
                   [0.03333333333, 0.03333333333, 0.03333333333]])
# test 5
colors = [['G', 'G', 'G'],
          ['G', 'R', 'R'],
          ['G', 'G', 'G']]
measurements = ['R', 'R']
motions = [[0,0], [0,1]]
sensor_right = 1.0
p_move = 1.0
p = localize(colors,measurements,motions,sensor_right,p_move)
correct_answer = ([[0.0, 0.0, 0.0],
                   [0.0, 0.0, 1.0],
                   [0.0, 0.0, 0.0]])
# test 6
colors = [['G', 'G', 'G'],
          ['G', 'R', 'R'],
          ['G', 'G', 'G']]
measurements = ['R', 'R']
motions = [[0,0], [0,1]]
sensor_right = 0.8
p_move = 0.5
p = localize(colors,measurements,motions,sensor_right,p_move)
correct_answer = ([[0.0289855072, 0.0289855072, 0.0289855072],
                   [0.0724637681, 0.2898550724, 0.4637681159],
                   [0.0289855072, 0.0289855072, 0.0289855072]])
# test 7
colors = [['G', 'G', 'G'],
          ['G', 'R', 'R'],
          ['G', 'G', 'G']]
measurements = ['R', 'R']
motions = [[0,0], [0,1]]
sensor_right = 1.0
p_move = 0.5
p = localize(colors,measurements,motions,sensor_right,p_move)
correct_answer = ([[0.0, 0.0, 0.0],
                   [0.0, 0.33333333, 0.66666666],
                   [0.0, 0.0, 0.0]])
'''
                   
'''
################################################################################
# notes from assignment page:
The video shows sensor_wrong and p_stay defined globally and referenced from 
sense() and move() helper functions. This approach will not actually work when 
submitting the assignment. You must pass in all required state into your sense() 
and move() functions from inside your localize() routine: def sense(p, colors, 
measurements, sensor_wrong): ... def move(p, motion, p_stay): ... def 
localize(...): ... p = move(p, motion, p_stay) p = sense(p, colors, measurement, 
sensor_wrong) [You should NOT modify the function signature of localize()]                   
'''

