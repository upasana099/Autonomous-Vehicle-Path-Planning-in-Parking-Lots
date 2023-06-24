from os import path
import matplotlib as mpl
import pygame
from matplotlib.patches import Rectangle
import matplotlib.pyplot as plt
from numpy import pi, sqrt
import numpy as np
import math



car1_posx = 30
car1_posy = 10
agent_posx = 49
agent_posy = 180
obstacle_posx = 70
obstacle_posy = 90
obstacle_width = 50
obstacle_height = 50
agent_trailer_posx = agent_posx-23
agent_trailer_posy = agent_posy
agent_theta = 0
agent_trailer_theta = 0
car_width = 22
car_height = 17.5 
police_carx = 0
police_cary = 180
padding = 2
car1_1 = pygame.Rect(120, 600, 120, 80)
surface = pygame.display.set_mode((800,800))
block= pygame.Rect(280, 240, 200, 200)
surface.fill('WHITE')

# start and end for motion planning
police_start = [police_carx + 1+5, police_cary + car_height/2,0]
police_goal = [car1_posx+car_width+15+1+5,10 + car_height/2,0]

#parameters for motion planning
agent_start = [agent_posx, agent_posy + car_height/2,0]
agent_trailer_start = [agent_trailer_posx,agent_trailer_posy + car_height/2,0]
agent_goal = [car1_posx+car_width+15+1+5+car_width,10 + car_height/2,180]

wheelbase = 15
hitch_lenght = 25
steering_angle = 30
vel = 1

agent = [[agent_posx,agent_posy,1],[agent_posx + car_width,agent_posy,1],[agent_posx + car_width,agent_posy + car_height,1], [agent_posx, agent_posy + car_height,1]]
agent_boundary = [[-1,21,21,-1],[-7.5,-7.5,7.5,7.5],[1,1,1,1]]
agent_trailer_boundary = [[-1,11,11,-1],[-7.5,-7.5,7.5,7.5],[1,1,1,1]]

# Define an obstacle as a list of four vertices
obstacle = [[obstacle_posx-padding, obstacle_posy-padding],[obstacle_posx + obstacle_width + padding, obstacle_posy-padding],[obstacle_posx + obstacle_width + padding, obstacle_posy + obstacle_height +padding],[obstacle_posx-padding, obstacle_posy+obstacle_height+padding]]
car1 = [[car1_posx - padding, car1_posy-padding],[car1_posx + car_width + padding, car1_posy - padding],[car1_posx + car_width + padding, car1_posy + car_height+padding], [car1_posx - padding, car1_posy+car_height+padding]]

def get_neighbours(x,y,theta,xt,yt,theta_t):
    neighbour = []
    for i in range(-steering_angle,steering_angle+1,5):
        x_dot = vel*math.cos(theta*(pi/180))
        y_dot = vel*math.sin(theta*(pi/180))
        theta_dot = (vel*math.tan(i*(pi/180))/wheelbase)*(180/pi)
        xt_dot = vel*math.cos(theta_t*(pi/180))
        yt_dot = vel*math.sin(theta_t*(pi/180))
        theta_t_dot = (vel*math.sin((theta-theta_t)*(pi/180))/hitch_lenght)*(180/pi)
        if(valid_point(x+x_dot,y+y_dot,theta+theta_dot,xt+xt_dot,yt+yt_dot,theta_t+theta_t_dot)):
            neighbour.append([round(x+x_dot,2),round(y+y_dot,2),(round(theta+theta_dot,2))%360,round(xt+xt_dot,2),round(yt+yt_dot,2),(round(theta_t+theta_t_dot,2)+360)%360,1,i])
        if(valid_point(x-x_dot,y-y_dot,theta-theta_dot,xt-xt_dot,yt-yt_dot,theta_t-theta_t_dot)):
            neighbour.append([round(x-x_dot,2),round(y-y_dot,2),(round(theta-theta_dot,2)+360)%360,round(xt-xt_dot,2),round(yt-yt_dot,2),(round(theta_t-theta_t_dot,2)+360)%360,-1,i])
    return neighbour

def straight_available(x,y,xt,yt):
    boundary_line = [[x,y],[agent_goal[0],agent_goal[1]],[agent_goal[0]+1,agent_goal[1]],[x+1,y]]
    boundary_line_t = [[xt,yt],[agent_goal[0]+50,agent_goal[1]],[agent_goal[0]+1+50,agent_goal[1]],[xt+1,yt]]
    if collision_check(boundary_line,obstacle) and collision_check(boundary_line_t,obstacle):
        return False
    if collision_check(boundary_line,car1) and collision_check(boundary_line_t,car1):
        return False
    return True


def get_boundary(x,y,theta,car_type="car"):
    tx = x 
    ty = y 
    th = theta-agent_start[2]
    homogeneous_matrix = [[math.cos(th*(pi/180)),-math.sin(th*(pi/180)),tx],[math.sin(th*(pi/180)),math.cos(th*(pi/180)),ty]]
    if car_type == "car":
        mat_mul = np.dot(homogeneous_matrix,agent_boundary)
    else:
        mat_mul = np.dot(homogeneous_matrix,agent_trailer_boundary)
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

def valid_point(x,y,theta,xt,yt,theta_t): 
    boundary = get_boundary(x,y,theta)
    boundary_t = get_boundary(xt,yt,theta_t,"trailer")
    if x < 1 or y < car_height/2.0 or x > 200-car_width or y > 200-car_height/2.0:
        return False 
    if collision_check(boundary,obstacle):
        return False
    if collision_check(boundary,car1) or collision_check(boundary_t,car1):
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

def cost_function(x1,y1,x2,y2,theta1,theta2,vel): 
    cost = 0
    reverse_penalty = 0
    turn_penalty = 0
    distance = sqrt((pow(x1-x2,2)+pow(y1-y2,2)))
    cost = distance+reverse_penalty+turn_penalty
    return cost

def hurestic_function(x,y,theta,xt,yt,theta_t,vel,turn): 
    theta_ = 0
    theta = (theta+360)%360
    theta_t = (theta_t+360)%360
    if theta_t == 0:
        theta_t=360
    reverse_penalty = 0 
    turn_penalty = 0
    obstacle_penalty = 0 
    distance = sqrt((pow(agent_goal[0]-x,2)+pow(agent_goal[1]-y,2))) 
    distance += sqrt(((pow((agent_goal[0]-car_width)-(x+car_width*math.cos(theta*(pi/180))),2)+pow((agent_goal[1]+car_height)-(y+car_width*math.sin(theta*(pi/180))),2))))

    if straight_available(x,y,xt,yt) and not(x>agent_goal[0]-5 and y>agent_goal[1]-5 and x <agent_goal[0]+5 and y <agent_goal[1]+15):
        theta_ = abs((360 + (math.atan2(y-agent_goal[1],x-agent_goal[0]))*(180/pi))%360 - theta+180) 
    else:
        theta_ = 180
    if collision_check([[x-15,y],[x+200*math.cos(theta*(pi/180))-15,y+200*math.sin(theta*(pi/180))],[15+x+200*math.cos(theta*(pi/180)),y+200*math.sin(theta*(pi/180))],[x+15,y]],obstacle):
        obstacle_penalty +=10
    if collision_check([[x-15,y],[x+200*math.cos(theta*(pi/180))-15,y+200*math.sin(theta*(pi/180))],[15+x+200*math.cos(theta*(pi/180)),y+200*math.sin(theta*(pi/180))],[x+15,y]],obstacle):
        obstacle_penalty +=10
    if vel < 0:
        reverse_penalty = 1
    if abs(theta-theta_t)>15 and not(x>agent_goal[0]-5 and y>agent_goal[1]-5 and x <agent_goal[0]+5 and y <agent_goal[1]+15):
        turn_penalty = 5
    hurestic = distance+theta_+reverse_penalty+turn_penalty+obstacle_penalty
    return hurestic

def check_visited(current,visited):
    for x,y,th,xt,yt,th_t in visited:
        if current[0]== x and current[1]== y and current[2]==th and current[3]== xt and current[4]== yt and current[5]==th_t:
            return True
    return False

def A_star():
    open_set = []
    visited = []
    start = [agent_start[0],agent_start[1],agent_start[2],agent_trailer_start[0],agent_trailer_start[1],agent_trailer_start[2]]
    tcost = 0
    gcost = 0
    path = [start]
    open_set.append((start,tcost,gcost,path))
    itr =0
    while len(open_set)>0:
        itr+=1
        index = priority(open_set)
        (shortest,_,gvalue,path) = open_set[index]
        
        open_set.pop(index)
        if not (check_visited([round(shortest[0]),round(shortest[1]),round(shortest[2]),round(shortest[3]),round(shortest[4]),round(shortest[5])],visited)):

            visited.append([round(shortest[0]),round(shortest[1]),round(shortest[2]),round(shortest[3]),round(shortest[4]),round(shortest[5])])
            if round(shortest[0]) <= agent_goal[0]+5 and round(shortest[0]) >= agent_goal[0]-5 and round(shortest[1]) <= agent_goal[1]+5 and round(shortest[1]) >= agent_goal[1]-5 and shortest[2] <= agent_goal[2]+5 and shortest[2] >= agent_goal[2]-5:
                return path
            neighbours= get_neighbours(shortest[0],shortest[1],shortest[2],shortest[3],shortest[4],shortest[5])
            for neighbour in neighbours:
                vel = neighbour[6]
                turn = neighbour[7]
                temp_gcost = gvalue+(0.1*cost_function(shortest[0],shortest[1],neighbour[0],neighbour[1],shortest[2],neighbour[2],vel))
                temp_tcost = temp_gcost+(0.9*hurestic_function(neighbour[0],neighbour[1],neighbour[2],neighbour[3],neighbour[4],neighbour[5],vel,turn))
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

for i in range(50,len(path)):
    x,y,theta=path[i][:3]
    y=200-y
    xt,yt,thetat=path[i][3:6]
    yt=200-yt
    x=x*4
    y=y*4
    xt=xt*4
    yt=yt*4
    #pygame.draw.circle(surface,(255,255,255),(x,y),3)
    surface.fill('WHITE')
    pygame.draw.rect(surface,'RED',car1_1)
    pygame.draw.rect(surface,'GREEN',block)
    

    car_surface=pygame.image.load("C:/Users/upasa/OneDrive/Desktop/VALET/truck.png")
    trailer_surface=pygame.image.load("C:/Users/upasa/OneDrive/Desktop/VALET/trailer.png")
    trailer_surface=pygame.transform.scale(trailer_surface,(80,70))
    car_surface=pygame.transform.scale(car_surface,(40,70))
    pygame.time.delay(50)
    blit_rotate(surface,car_surface,(x,y-70),(0,40),theta)
    blit_rotate(surface,trailer_surface,(xt,yt-70),(0,35),thetat)
    pygame.display.flip()
    
plt.figure()
plt.xlim([0, 200])
plt.ylim([-20, 200])
for points in path:
    plt.scatter(points[0],points[1],color = 'black') 
plt.show()