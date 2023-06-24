# Autonomous Vehicle Path Planning in Parking Lots

Autonomous vehicles are becoming increasingly common on roads worldwide, and as they become more prevalent, they must be capable of performing complex maneuvers in tight and cluttered spaces, such as parking lots. The ability to navigate such environments requires advanced path planning algorithms that take into account the kinematics of the vehicle as well as potential collisions with other objects in the environment.

The challenge of parking becomes even more complex when the available space is compact, and multiple vehicles need to be parked in close proximity to each other. This requires a path planning algorithm that can take into account the kinematics of the vehicles, such as their turning radius and maximum steering angle, and can plan paths that enable the vehicles to park without colliding with each other.

## Environment

The project is built using Python's pygame package, which provides a graphical environment for the simulation. The parking lot is represented as a two-dimensional grid, where each cell represents a portion of the parking area. The vehicles start from the northwest corner of the grid and need to reach a small spot at the southern boundary, surrounded by other parked vehicles and an obstacle in the middle area.
![environment](https://github.com/upasana099/Autonomous-Vehicle-Path-Planning-in-Parking-Lots/assets/89516193/da0070a8-2d24-453c-8c21-6de7764f5381)


## Vehicle Types

The project considers three vehicle types for parking and each vehicle type has different kinematics and steering mechanisms, which need to be taken into account during the path planning process.


1. Delivery Robot with Differential Drive Kinematics

![differential drive kinematics](https://github.com/upasana099/Autonomous-Vehicle-Path-Planning-in-Parking-Lots/assets/89516193/3f372327-8b7c-47b3-9a28-d1b4772ad096)


   
2. Car with Ackerman Steering Kinematics

![ackerman kinematics](https://github.com/upasana099/Autonomous-Vehicle-Path-Planning-in-Parking-Lots/assets/89516193/0ecb2fc0-c54c-4011-8949-119c83b67267)

   
3. Car pulling Trailer Kinematics

![ackerman kinematics](https://github.com/upasana099/Autonomous-Vehicle-Path-Planning-in-Parking-Lots/assets/89516193/9822fcbd-0dbc-41dd-9835-e411b4e9fa64)


Each vehicle type has different kinematics and steering mechanisms, which need to be taken into account during the path planning process.

## Algorithms Used

### A* Algorithm

The A* algorithm is employed to plan a safe and efficient route for the vehicles. The map is represented as a grid, where each cell represents a portion of the road. The algorithm calculates the cost of moving from one cell to another, considering factors such as road conditions, traffic, and obstacles. A heuristic function based on the Euclidean distance is used to estimate the cost of reaching the destination from each cell. The algorithm gradually explores neighboring cells with the lowest cost, moving towards the destination.

## Results

The project provides path plots for the following scenarios:
1. Delivery Robot with Differential Drive
   
3. Car with Ackerman Steering
4. Car pulling Trailer

The path plots demonstrate the planned paths that enable the vehicles to park without colliding with each other.

1. Delivery Robot with Differential Drive

![delivery robot](https://github.com/upasana099/Autonomous-Vehicle-Path-Planning-in-Parking-Lots/assets/89516193/66de9e47-4fc6-4dbd-be70-037d42ee7e40)

   
2. Car with Ackerman Steering

![police car](https://github.com/upasana099/Autonomous-Vehicle-Path-Planning-in-Parking-Lots/assets/89516193/e1671446-6ef1-42ad-a23b-bf9e77b5de28)

   
3. Car pulling Trailer

![WhatsApp Image 2023-06-23 at 20 03 04](https://github.com/upasana099/Autonomous-Vehicle-Path-Planning-in-Parking-Lots/assets/89516193/f23f17ad-5ef7-466e-b354-3e15bea2df8c)


   

The path plots demonstrate the planned paths that enable the vehicles to park without colliding with each other.


