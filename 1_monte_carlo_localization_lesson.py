#6 uniform distribution
p=[0.2, 0.2, 0.2, 0.2, 0.2]
print(p)

#7 generalized uniform distribution
p=[]
n=5
for i in range(n):
    p.append(1/n)
print(p)

#11 pHit and pMiss
# not elegent but does the job
pHit=0.6
pMiss=0.2
p[0]=p[0]*pMiss
p[1]=p[1]*pHit
p[2]=p[2]*pHit
p[3]=p[3]*pMiss
p[4]=p[4]*pMiss
print(p)

#12 print out sum of probabilities
print(sum(p))

#13 sense function
# sense function is the measurement update, which takes as input the initial 
# distribution p, measurement z, and other global variables, and outputs a normalized 
# distribution Q, which reflects the non-normalized product of imput probability
# i.e.0.2 and so on, and the corresponding pHit(0.6) and or pMiss(0.2) in accordance
# to whether these colours over here agree or disagree.
# The reason for the localizer is because later on as we build our localizer we will 
# this to every single measurement over and over again.
p=[0.2, 0.2, 0.2, 0.2, 0.2]
world=['green', 'red', 'red', 'green', 'green']
Z = 'red' #or green
pHit = 0.6
pMiss = 0.2
def sense(p, Z):
    q=[]
    for i in range(len(p)):
        hit = (Z == world[i])
        q.append(p[i] * (hit * pHit + (1-hit) * pMiss))
    s = sum(q)
    for i in range(len(p)):
        q[i] = q[i]/s
    return q
print(sense(p,Z))

#16 multiple measurements
# This is a way to test the code, we grab the kth measurement element and apply
# it into the current belief, then recursively update that belief into itself.
# we should get back the uniform distribution
# Modify the code so that it updates the probability twice
# and gives the posterior distribution after both 
# measurements are incorporated. Make sure that your code 
# allows for any sequence of measurement of any length.
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
for k in range(len(measurements)):
    p = sense(p, measurements[k])
print(p)

#19 move function
# shift data one cell to the right
# Program a function that returns a new distribution 
# q, shifted to the right by U units. If U=0, q should 
# be the same as p.
p=[0, 1, 0, 0, 0]
world=['green', 'red', 'red', 'green', 'green']
measurements = ['red', 'green']
pHit = 0.6
pMiss = 0.2
#match the measurement with the belief, measurement update function
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
        q.append(p[(i-U)%len(p)]) #minus sign, 1 place to the left
    return q
print(move(p, 1))

#23 inexact move function
# Modify the move function to accommodate the added 
# probabilities of overshooting and undershooting 
# the intended destination.
p=[0, 1, 0, 0, 0]
world=['green', 'red', 'red', 'green', 'green'] #specifies colour of cell
measurements = ['red', 'green']
pHit = 0.6 #probability of hitting the correct colour
pMiss = 0.2 #probability of missing the correct colour
pExact = 0.8
pOvershoot = 0.1
pUndershoot = 0.1
def sense(p, Z):
    q=[]
    for i in range(len(p)):
        hit = (Z == world[i])
        q.append(p[i] * (hit * pHit + (1-hit) * pMiss))
    s = sum(q)
    for i in range(len(q)):
        q[i] = q[i] / s
    return q
#circle shift
def move(p, U): #grid cells the robot is moving to the right
    q = []
    for i in range(len(p)):
        #auxillary variable
        s = pExact * p[(i-U) % len(p)] 
        s = s + pOvershoot  * p[(i-U-1) % len(p)] #one step further
        s = s + pUndershoot * p[(i-U+1) % len(p)] #one step behind
        q.append(s)
    return q   
print(move(p,1))

#24 limit distribution
# [1 0 0 0 0] spreads out to [0.2 0.2 0.2 0.2 0.2] after some time
# everytime you move you lose information
# without update, no information
# moving many times without update
# p(x4) = 0.8*x2 + 0.1*x1 + 0.1*x3 balanc equation in the limit

#25 move twice quiz
# Write code that makes the robot move twice and then prints 
# out the resulting distribution, starting with the initial 
# distribution p = [0, 1, 0, 0, 0]
p=[0, 1, 0, 0, 0]
world=['green', 'red', 'red', 'green', 'green']
measurements = ['red', 'green']
pHit = 0.6
pMiss = 0.2
pExact = 0.8
pOvershoot = 0.1
pUndershoot = 0.1
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
for i in range(1000):
    p=move(p,1)
print(p)

#notes:
# localization is sense/move conbination
# everytime it moves robot loses information
# everytime it moves robot regains information
# entropy

#27 sense and move
# Given the list motions=[1,1] which means the robot 
# moves right and then right again, compute the posterior 
# distribution if the robot first senses red, then moves 
# right one, then senses green, then moves right again, 
# starting with a uniform prior distribution.
p=[0.2, 0.2, 0.2, 0.2, 0.2]
world=['green', 'red', 'red', 'green', 'green']
measurements = ['red', 'green']
motions = [1,1]
pHit = 0.6
pMiss = 0.2
pExact = 0.8
pOvershoot = 0.1
pUndershoot = 0.1
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
for k in range(len(measurements)):
    p = sense(p,measurements[k])
    p = move(p,motions[k])
print(p) #results show that robot most likely ended up in the 5th cell        

#28 move twice
# Modify the previous code so that the robot senses red twice.
p=[0.2, 0.2, 0.2, 0.2, 0.2]
world=['green', 'red', 'red', 'green', 'green']
measurements = ['red', 'red']
motions = [1,1]
pHit = 0.6
pMiss = 0.2
pExact = 0.8
pOvershoot = 0.1
pUndershoot = 0.1
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
for k in range(len(measurements)):
    p = sense(p, measurements[k])
    p = move(p, motions[k])    
print(p)
# most likely in 4th cell, it's the cell after the last observation         
# this is the essense of google's self driving car code
# It is crucial that the car knows where it is.
# While the road is not painted red and green, the road has lane markers.
# instead of read and green cells over here, we plug in the colour of the pavement
# versus the colour of the lane markers, it isn't just one observation per time 
# step, it is an entire field of observations. an entire camara image.
# As long as you can correspond a car image in your model, with a camera image in
# your model, then the piece of code is not much more difficult than what I have here.