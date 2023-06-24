from os import path
from matplotlib.patches import Rectangle
import matplotlib.pyplot as plt
from numpy import pi, sqrt
import numpy as np
import math
import pygame

RED = (255, 0, 0)
GREEN = (0, 255, 0)
BLUE = (0, 255, 0)
YELLOW = (255, 255, 0)
WHITE = (255, 255, 255)
BLACK = (0, 0, 0)
PURPLE = (128, 0, 128)
ORANGE = (255, 165 ,0)
GREY = (128, 128, 128)
TURQUOISE = (64, 224, 208)

car1_posx = 30
car1_posy = 10
car2_posx = 110
car2_posy = 10
obstacle_posx = 70
obstacle_posy = 90
obstacle_width = 50
obstacle_height = 50
car_width = 20
car_height = 20 
del_carx = 10
del_cary = 180
agent_theta = 0
car1_1 = pygame.Rect(120, 600, 120, 80)
car2_1 = pygame.Rect(520,600,120,80)
surface = pygame.display.set_mode((800,800))
block= pygame.Rect(320, 240, 200, 200)
surface.fill('WHITE')
wheelbase = 28
steering_angle = 30
vel = 1


agent_start = [del_carx + 10, del_cary + car_height/2, 0]
agent_goal = [car1_posx+car_width+30, car_height/2 + 10, 0]
padding = 5 # to be applied on the obstacles


wheelRadius = 1
wheelDist = 10
vel_L = [1,0,-1]
vel_R = [1,0,-1]

park_point = False

#Parameters for collision check
agent = [[del_carx,del_cary,1],
                [del_carx+car_width,del_cary,1],
                [del_carx+car_width,del_cary+car_height,1],
                [del_carx,del_cary+car_height,1]]
obstacle = [[obstacle_posx-padding,obstacle_posy-padding],
            [obstacle_posx+obstacle_width+padding,obstacle_posy-padding],
            [obstacle_posx+obstacle_width+padding,obstacle_posy+obstacle_height+padding],
            [obstacle_posx-padding,obstacle_posy+obstacle_height+padding]]
car1 = [[car1_posx-padding,car1_posy-padding],
        [car1_posx+car_width+padding,car1_posy-padding],
        [car1_posx+car_width+padding,car1_posy+car_height+padding],
        [car1_posx-padding,car1_posy+car_height+padding]]
car2 = [[car2_posx-padding,car2_posy-padding],[car2_posx+car_width+padding,car2_posy-padding],[car2_posx+car_width+padding,car2_posy+car_height+padding],[car2_posx-padding,car2_posy+car_height+padding]]


agent_boundary = [[-10,10,10,-10],[-10,-10,10,10],[1,1,1,1]] 

def world(x,y,theta):
    fig = plt.figure("Delivery Robot")
    ax = fig.add_subplot(111)

    obstacle_mid = Rectangle((obstacle_posx, obstacle_posy),obstacle_width,obstacle_height,color ='black')

    car1 = Rectangle((car1_posx, car1_posy),car_width, car_height,color ='red')
    car2 = Rectangle((car2_posx, car2_posy),car_width, car_height,color ='red')

    agent = Rectangle((del_carx, del_cary),car_width, car_height,color ='green')

    ax.add_patch(obstacle_mid)
    ax.add_patch(car1)
    ax.add_patch(car2)
    ax.add_patch(agent)
    ax.add_patch( Rectangle((car1_posx+car_width+15, 5),car_width+10, car_height+10,fc ='none',ec ='g',lw = 2) ) #goal
    plt.plot(x,y,"sk")
    boundary = get_boundary(x,y,theta) # getting the vertices of the robot

    X = []
    Y = []
    for x,y in boundary:
        X.append(x)
        Y.append(y)

    plt.plot(X,Y)
    plt.xlim([0, 200])
    plt.ylim([-20, 200])

    return

def get_neighbours(x,y,theta):
    neighbour = []
    for vr in vel_R:
        for vl in vel_L:
            if vl == 0 and vr == 0 : 
                break
            vel = (vl+vr)/2
            x_dot = vel*math.cos(theta*(pi/180))
            y_dot = vel*math.sin(theta*(pi/180))
            theta_dot = (wheelRadius/(2*wheelDist))*(vr-vl)*(180/pi)

            if(valid_point(x+x_dot,y+y_dot,theta+theta_dot)): 
                neighbour.append([round(x+x_dot,2),round(y+y_dot,2),(round(theta+theta_dot,2))%360,vl,vr])
    return neighbour

def straight_available(x,y):
    boundary_line = [[x,y],[agent_goal[0],agent_goal[1]],[agent_goal[0]+1,agent_goal[1]],[x+1,y]]
    if collision_check(boundary_line,obstacle):
        return False
    if collision_check(boundary_line,car1):
        return False
    return True


def get_boundary(x,y,theta):
    tx = x 
    ty = y 
    th = theta-agent_start[2]
    homogeneous_matrix = [[math.cos(th*(pi/180)),-math.sin(th*(pi/180)),tx],[math.sin(th*(pi/180)),math.cos(th*(pi/180)),ty]]
    mat_mul = np.dot(homogeneous_matrix,agent_boundary)
    new_boundary = [[mat_mul[0][0],mat_mul[1][0]],[mat_mul[0][1],mat_mul[1][1]],[mat_mul[0][2],mat_mul[1][2]],[mat_mul[0][3],mat_mul[1][3]]]
    return new_boundary

# to check if two pollygons intersect to check for collision
# Separating Axis Theorem 
def collision_check(a, b):
    polygons = [a, b]

    for polygon in polygons:
        for i, p1 in enumerate(polygon):
            p2 = polygon[(i + 1) % len(polygon)]
            normal = (p2[1] - p1[1], p1[0] - p2[0])
            minA, maxA = None, None
            for p in a:
                projected = normal[0] * p[0] + normal[1] * p[1]
                if minA is None or projected < minA:
                    minA = projected
                if maxA is None or projected > maxA:
                    maxA = projected
            minB, maxB = None, None
            for p in b:
                projected = normal[0] * p[0] + normal[1] * p[1]
                if minB is None or projected < minB:
                    minB = projected
                if maxB is None or projected > maxB:
                    maxB = projected
            if maxA < minB or maxB < minA:
                return False

    return True

def valid_point(x, y, theta):
     # Get the boundary of the car at the given position and angle
    boundary = get_boundary(x, y, theta)
    bounds = (1, car_height, 200 - car_width, 200 - car_height / 2.0)
    
    if any(coord < bound for coord, bound in zip((x, y), bounds)) or collision_check(boundary, obstacle) or any(collision_check(boundary, car) for car in (car1, car2)):
        return False
    
    return True

#shortest path from the 'Priority Queue'
def priority(queue): 
    min = math.inf
    index = 0
    for check in range(len(queue)):
        _,value,_,_ = queue[check]
        if value<min:
            min = value
            index = check 
    return index

    return True

def gx(x1,y1,x2,y2):
    distance = sqrt((pow(x1-x2,2)+pow(y1-y2,2)))
    return distance


def hx(x,y,theta):
    theta_ = 0
    theta = (theta+360)%360 
    distance = sqrt((pow(agent_goal[0]-x,2)+pow(agent_goal[1]-y,2))) 
    distance += sqrt(((pow((agent_goal[0]+car_width)-(x+car_width*math.cos(theta*(pi/180))),2)+pow((agent_goal[1]+car_height)-(y+car_width*math.sin(theta*(pi/180))),2)))) 
    if straight_available(x,y) and not(x>agent_goal[0]-5 and y>agent_goal[1]-5 and x <agent_goal[0]+5 and y <agent_goal[1]+5): 
        theta_ = abs((360 + (math.atan2(y-agent_goal[1],x-agent_goal[0]))*(180/pi))%360 - theta+180) 
    hurestic = distance+theta_
    return hurestic

def check_visited(current,visited):
    for x,y,th in visited:
        if current[0]== x and current[1]== y and current[2]==th :
            return True
    return False

def A_star():
    open_set = []
    visited = []
    start = agent_start
    tcost = 0
    gcost = 0
    path = [start]
    open_set.append((start,tcost,gcost,path))
    while len(open_set)>0:
        index = priority(open_set)
        (shortest,_,gvalue,path) = open_set[index] 
        open_set.pop(index)
        if not (check_visited([round(shortest[0]),round(shortest[1]),round(shortest[2])],visited)): # Check if node already visited
            visited.append([round(shortest[0]),round(shortest[1]),round(shortest[2])])
            if round(shortest[0]) <= agent_goal[0]+5 and round(shortest[0]) >= agent_goal[0]-5 and round(shortest[1]) <= agent_goal[1]+5 and round(shortest[1]) >= agent_goal[1]-5 and shortest[2] <= agent_goal[2]+15 and shortest[2] >= agent_goal[2]-15: #goal condition
                return path
            neighbours= get_neighbours(shortest[0],shortest[1],shortest[2]) 
            for neighbour in neighbours:
                temp_gcost = gvalue+(0.1*gx(shortest[0],shortest[1],neighbour[0],neighbour[1]))
                temp_tcost = temp_gcost+(0.9*hx(neighbour[0],neighbour[1],neighbour[2]))
                open_set.append((neighbour,temp_tcost,temp_gcost,path+ [neighbour]))
    print("not working")      
    return path


path = A_star()
print("reached")

def blit_rotate(surface, image, position, origin_position, angle):
    rotated_image = pygame.transform.rotate(image, angle)
    rotated_rect = rotated_image.get_rect(center=position)
    offset = rotated_rect.topleft - pygame.math.Vector2(position)
    surface.blit(rotated_image, rotated_rect)


for i in range(25,len(path)):
    x,y,theta=path[i][:3]
    y=200-y
    x=x*4
    y=y*4
    surface.fill((255,255,255))
    pygame.draw.rect(surface,'RED',car1_1)
    pygame.draw.rect(surface,'RED',car2_1)
    pygame.draw.rect(surface,'GREEN',block)
    
    car_sur=pygame.image.load("C:/Users/upasa/OneDrive/Desktop/VALET/truck.png")
    car_sur=pygame.transform.scale(car_sur,(80,80))
    blit_rotate(surface,car_sur,(x,y-80),(0,40),theta)
    pygame.display.flip()
    pygame.time.delay(50)

plt.figure("Path")
plt.xlim([0, 200])
plt.ylim([-20, 200])
for points in path:
    plt.scatter(points[0],points[1],color = 'BLUE')  

plt.show()

    