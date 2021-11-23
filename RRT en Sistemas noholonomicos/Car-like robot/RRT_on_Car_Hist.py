import pygame
import time
import numpy as np
import random


# Colors
white = (255,255,255)
black = (0,0,0)
red = (255,0,0)


## Initialize pygame
pygame.init()
pygame.display.set_caption('RRT on Car')

Length = 800
Width = 600
screen = pygame.display.set_mode((Length,Width))
screen.fill(white)

n = 100000
delta_t = 15.0
Car = pygame.image.load('car.png')
Car_pos = np.array([100.0, 100.0])
Car_angle = 0.0
Car_diam = 70.0
Car_radius = int(Car_diam/2)

def draw_Car(state):
    angle_ = -np.rad2deg(state[2])
    state_new = np.array([int(round(state[0]-48)), int(round(state[1]-48))])

    orig_rect = Car.get_rect()
    rot_image = pygame.transform.rotate(Car, angle_)
    rot_rect = orig_rect.copy()
    rot_rect.center = rot_image.get_rect().center
    rot_image = rot_image.subsurface(rot_rect).copy()

    screen.blit(rot_image, state_new)


# Euclidean distance function
def dist_euclidean(x,y):
    return np.sqrt((y[0]-x[0])**2 + (y[1]-x[1])**2)

# Metric to measure distances between states
def dist(x,xprime):
    alpha = min(np.abs(x[2]-xprime[2]), 2*np.pi-np.abs(x[2]-xprime[2]))

    return np.sqrt((x[0]-xprime[0])**2 + (x[1]-xprime[1])**2 + alpha**2)

# Define a random control
def U_control():
    if np.random.random() >= 0.5:
        v = 1.
    else:
        v = -1.
    
    epsilon = np.deg2rad(5.)
    PI_4 = np.pi/4 - epsilon
    phi = np.random.uniform(-PI_4, PI_4)

    #u = v * np.array([np.cos(phi), np.sin(phi)])
    u = np.array([v*np.cos(phi), v*np.tan(phi)/Car_radius])

    return u


def coll_check(p):
    if len(p) == 3:
        center = np.array([p[0]+16*np.cos(p[2]), p[1]+16*np.sin(p[2])])
    else:
        center = p

    samples = 24
    angle = 0.0
    p_ = np.array([int(round(center[0])), int(round(center[1]))])
    for i in range(0,samples):
        p1 = p_ + np.asarray((int(Car_radius*np.cos(np.deg2rad(angle))), int(Car_radius*np.sin(np.deg2rad(angle)))))

        if 0<=p1[0]<Length and 0<=p1[1]<Width:
            if screen.get_at(p_) == white and screen.get_at(p1) == white:
                valid = True
            else:
                valid = False
                break
        else:
            valid = False
            break
        if valid == False:
            break

        angle += 360/samples
    
    return valid


def RRT(state_init, x_goal):
    goal_reached = False
    T = []

    node = []
    node.append(state_init)
    node.append(-1)

    T.append(node)

    for i in range(0,n):
        x = random.randint(0,Length - 1)
        y = random.randint(0,Width - 1)
        theta = random.uniform(0.0, 2*np.pi)
        state_rand = np.array([x, y, theta])

        # Find closest node in the tree
        d_node = float('inf')
        for k in range(len(T)):
            if dist(state_rand, T[k][0]) < d_node:
                d_node = dist(state_rand, T[k][0])
                state_near = T[k][0]
                parent = k

        # Apply all controls to state_near and select the one with lowest distance to state_near
        d_node = float('inf')
        samples = 100
        for control in range(samples):
            u = U_control()

            x_k = state_near[0]
            y_k = state_near[1]
            theta_k = state_near[2]

            x_k1 = x_k + u[0] * np.cos(theta_k) * delta_t
            y_k1 = y_k + u[0] * np.sin(theta_k) * delta_t
            theta_k1 = theta_k + u[1] * delta_t

            distance = dist(state_rand, np.array([x_k1,y_k1,theta_k1]))
            if distance < d_node:
                d_node = distance
                state_new = np.array([x_k1,y_k1,theta_k1])

        # Check if state_new is in free space
        if coll_check(state_new):
            node = []
            node.append(state_new)
            node.append(parent)

            T.append(node)

        # Check if goal is reached
        if dist_euclidean(state_new,x_goal) < Car_radius:
            goal_reached = True
            break
    
    return T, goal_reached


# game loop
running = True
phase = 0
valid = valid2 = False
while running:
    
    for event in pygame.event.get():
        if event.type == pygame.QUIT:
            running = False
        if pygame.mouse.get_pressed()[0] and phase == 0:
            mouse_pos = pygame.mouse.get_pos()
            pygame.draw.circle(screen, black, mouse_pos, 10)
            pygame.image.save(screen,'obstacles.png')
            obs = pygame.image.load('obstacles.png')

        if pygame.mouse.get_pressed()[0] and phase == 1 and valid == True:
            Car_pos = np.asarray(pygame.mouse.get_pos())
            Car_pos = Car_pos.astype('float64')
            print('x_init', Car_pos)
            phase = 2
        if pygame.mouse.get_pressed()[0] and phase == 2 and valid2 == True:
            pygame.draw.circle(screen, red, goal, Car_radius)
            print('x_goal', goal)
            phase = 3

        if pygame.key.get_pressed()[273] == 1 and phase == 1:
            Car_angle = -np.pi/2.0
        if pygame.key.get_pressed()[274] == 1 and phase == 1:
            Car_angle = -3.0*np.pi/2.0
        if pygame.key.get_pressed()[275] == 1 and phase == 1:
            Car_angle = 0.0
        if pygame.key.get_pressed()[276] == 1 and phase == 1:
            Car_angle = np.pi
        if pygame.key.get_pressed()[13] == 1:
            phase = 1
            #pygame.time.delay(100)

    try:
        screen.blit(obs, (0,0))
    except:
        screen.fill(white)


    if phase == 1:  # Place ship
        p = np.asarray(pygame.mouse.get_pos())
        state_init = np.array([p[0], p[1], Car_angle])
        valid = coll_check(state_init)
            
        if valid:
            draw_Car(state_init)

    elif phase == 2:    # Place goal
        goal = np.asarray(pygame.mouse.get_pos())
        valid2 = coll_check(goal)
            
        if valid2:
            pygame.draw.circle(screen, red, goal, Car_radius)
        
        draw_Car(state_init)

    elif phase == 3:
        print('Computing path...')
        T, reached = RRT(state_init, goal)

        if reached:
            print('Goal reached')
            # Get trajectory
            traj =[]
            traj.append(T[-1][0])
            parent = T[-1][1]
            
            while parent != -1:
                traj.append(T[parent][0])
                parent = T[parent][1]

            traj.reverse()
        else:
            print('Goal NOT reached')
        
        phase = 4
    elif phase == 4:
        if reached:
            for k in range(len(traj)):
                try:
                    screen.blit(obs, (0,0))
                except:
                    screen.fill(white)
                pygame.draw.circle(screen, red, goal, Car_radius)
                
                state = traj[k]
                
                draw_Car(state)
                pygame.display.update()
                pygame.time.delay(100)

                pygame.image.save(screen,'obstacles.png')
                obs = pygame.image.load('obstacles.png')
        else:
            state = state_init
        
        phase = 5
    elif phase == 5:
        pygame.draw.circle(screen, red, goal, Car_radius)
        draw_Car(state)
        


    
    pygame.display.update()
    pygame.time.delay(10)