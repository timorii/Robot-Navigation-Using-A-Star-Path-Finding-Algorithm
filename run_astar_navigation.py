#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import Twist 
from nav_msgs.msg import Odometry
import message_filters 
from sensor_msgs.msg import Imu,LaserScan
import time


import heapq
import matplotlib.pyplot as plt
import numpy as np




class astar_Navigation:
    def __init__(self):
        rospy.init_node("turtlebot3")
        rospy.Subscriber("/odom",Odometry,self.odomcallback)
        self.odom = Odometry()
        rospy.Subscriber("/imu",Imu,self.imucallback)
        self.imu = Imu()         
        rospy.Subscriber("/scan",LaserScan,self.scancallback)
        self.scan = LaserScan()
        self.pub = rospy.Publisher('/cmd_vel',Twist,queue_size=1)
        self.cmd_vel = Twist()
        self.pos_offset = 0.1
        self.yaw_offset = 0.05
        self.vel = 0.2
        self.turn = 0.2
       
        
    def odomcallback(self,odom):
        self.odom = odom
        
    def imucallback(self,imu):
        self.imu = imu

    def scancallback(self,scan):
        self.scan = scan
     
     
    def h_cost(self, a, b):
        return abs(a[0] - b[0]) + abs(a[1] - b[1])
    
    def a_star_search(self, grid, start, end):
        neighbors = [(0, 1), (1, 0), (0, -1), (-1, 0)]
        close_set = set()
        came_from = {}
        gscore = {start: 0}
        fscore = {start: self.h_cost(start, end)}
        open_set = []
        heapq.heappush(open_set, (fscore[start], start))

        while open_set:
            current = heapq.heappop(open_set)[1]

            if current == end:
                data = []
                while current in came_from:
                    data.append(current)
                    current = came_from[current]
                return data

            close_set.add(current)
            for i, j in neighbors:
                neighbor = current[0] + i, current[1] + j            
                current_gscore = gscore[current] + 1

                if 0 <= neighbor[0] < grid.shape[0]:
                    if 0 <= neighbor[1] < grid.shape[1]:            
                        if grid[neighbor[0]][neighbor[1]] == 1:
                            continue
                    else:
                        # grid bounds y walls
                        continue
                else:
                    # grid bounds x walls
                    continue

                if neighbor in close_set and current_gscore >= gscore.get(neighbor, 0):
                    continue

                if  current_gscore < gscore.get(neighbor, 0) or neighbor not in [i[1] for i in open_set]:
                    came_from[neighbor] = current
                    gscore[neighbor] = current_gscore
                    fscore[neighbor] = current_gscore + self.h_cost(neighbor, end)
                    heapq.heappush(open_set, (fscore[neighbor], neighbor))

        return False

    def quarternion_to_yaw(self):
        x = self.odom.pose.pose.orientation.x 
        y = self.odom.pose.pose.orientation.y
        z = self.odom.pose.pose.orientation.z
        w = self.odom.pose.pose.orientation.w 
        
        yaw = np.arctan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))
        
        return yaw
        
    def run(self, path):
        time.sleep(1)
       
        current_waypoint = 0
        current_target = 1
            
       # while np.abs(self.odom.pose.pose.position.x - path[-1][0]) > self.pos_offset or np.abs(self.odom.pose.pose.position.y - path[-1][1]) > self.pos_offset : 
        while  current_target < len(path) :  
            yaw = self.quarternion_to_yaw() 
           
            mode = 0 
            
            
            pos_x = self.odom.pose.pose.position.x
            pos_y = self.odom.pose.pose.position.y
           # print("x: " + str(pos_x) + " , y: " + str(pos_y) + " yaw: " + str(yaw)) 
            if path[current_target][0] > path[current_waypoint][0]: 
                # go postive x direction at 0 rad 
                mode = 1
                if np.abs(yaw-0) > self.yaw_offset: 
                    if yaw > 0: 
                        self.cmd_vel.linear.x = 0 
                        self.cmd_vel.angular.z = -self.turn 
                    if yaw < 0:
                        self.cmd_vel.linear.x = 0 
                        self.cmd_vel.angular.z = self.turn
                #elif pos_y - path[current_target][1] > self.pos_offset : 
                    #self.cmd_vel.linear.x = 0
                    #self.cmd_vel.angular.z = self.turn2()
            
                else: 
                    self.cmd_vel.linear.x = self.vel 
                    self.cmd_vel.angular.z = 0     
                    
                 

        # Check if we need to move in the negative x direction (left)
            if path[current_target][0] < path[current_waypoint][0]:
                # Go negative x direction at pi rad
                mode = 2
                if np.abs(yaw-np.pi) > self.yaw_offset:
                    if yaw > 0:
                        self.cmd_vel.linear.x = 0
                        self.cmd_vel.angular.z = self.turn
                    if yaw < 0:
                        self.cmd_vel.linear.x = 0
                        self.cmd_vel.angular.z = -self.turn
                else:
                    self.cmd_vel.linear.x = self.vel
                    self.cmd_vel.angular.z = 0

            # Check if we need to move in the positive y direction (up)
            if path[current_target][1] > path[current_waypoint][1]:
                # Go positive y direction at pi/2 rad
                mode = 3
                if np.abs(yaw - np.pi / 2) > self.yaw_offset:
                    if yaw > np.pi / 2 or yaw < -np.pi / 2 :
                        self.cmd_vel.linear.x = 0
                        self.cmd_vel.angular.z = -self.turn
                    if yaw < np.pi / 2 and yaw > -np.pi / 2:
                        self.cmd_vel.linear.x = 0
                        self.cmd_vel.angular.z = self.turn
                else:
                    self.cmd_vel.linear.x = self.vel
                    self.cmd_vel.angular.z = 0

            # Check if we need to move in the negative y direction (down)
            if path[current_target][1] < path[current_waypoint][1]:
                # Go negative y direction at -pi/2 rad
                mode = 4
                if np.abs(yaw + np.pi / 2) > self.yaw_offset:
                    if yaw > -np.pi / 2 and yaw < np.pi / 2:
                        self.cmd_vel.linear.x = 0
                        self.cmd_vel.angular.z = -self.turn
                    if yaw < -np.pi / 2 or yaw > np.pi / 2:
                        self.cmd_vel.linear.x = 0
                        self.cmd_vel.angular.z = self.turn
                else:
                    self.cmd_vel.linear.x = self.vel
                    self.cmd_vel.angular.z = 0
             
                                     
                                
            
            
            #print(self.odom.pose.pose.position.x)
            
            if np.abs(pos_x - path[current_target][0]) < self.pos_offset and mode <=  2:
                current_target += 1
                current_waypoint += 1

                print("reach way point: " + str(path[current_target][0]) + " , " + str(path[current_target][1]))

            if mode >= 3 and np.abs(pos_y - path[current_target][1]) < self.pos_offset:
                current_target += 1
                current_waypoint += 1

                print("reach way point: " + str(path[current_target][0]) + " , " + str(path[current_target][1]))
            self.pub.publish(self.cmd_vel)
       
            rospy.sleep(0.1) 
            
        self.cmd_vel.linear.x = 0 
        self.cmd_vel.angular.z = 0 
        self.pub.publish(self.cmd_vel)
    
    print("finish")
    
    
    
if __name__== "__main__":

    nav = astar_Navigation()



    # Define the grid (0 is walkable, 1 is an obstacle)
    grid = np.array([
        [1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1],
        [1,0,0,0,0,0,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,1],
        [1,0,1,1,1,0,1,0,0,0,1,1,1,0,1,1,1,1,1,1,1,1,1,1,1,1,1,0,1],
        [1,0,1,0,1,0,1,0,0,0,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,1],
        [1,0,1,0,1,0,1,0,1,1,1,1,1,1,1,0,1,0,1,1,1,1,1,0,1,0,1,0,1],
        [1,0,1,0,1,0,1,0,1,0,0,0,0,0,1,0,1,0,0,0,0,0,1,0,1,0,1,0,1],
        [1,0,1,0,1,0,1,0,1,0,1,1,1,0,1,0,1,0,1,1,1,0,1,0,1,0,1,0,1],
        [1,0,1,0,1,0,0,0,0,0,1,0,1,0,1,0,1,0,1,0,0,0,1,0,1,0,1,0,1],
        [1,0,1,1,1,1,1,0,1,0,1,0,1,0,1,0,1,0,1,1,1,1,1,0,1,0,1,0,1],
        [1,0,1,0,0,0,0,0,1,0,1,0,1,0,1,0,1,0,0,0,0,0,0,0,0,0,1,0,1],
        [1,0,1,0,1,1,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,1,1,1,1,1,1,0,1],
        [1,0,1,0,1,0,1,0,1,0,1,0,0,0,1,0,1,0,1,0,0,0,1,0,0,0,0,0,1],
        [1,0,1,0,1,0,1,0,1,0,1,1,1,1,1,0,1,0,1,0,1,0,1,0,1,1,1,0,1],
        [1,0,1,0,1,0,0,0,1,0,0,0,0,0,0,0,1,0,1,0,1,0,1,0,0,0,1,0,1],
        [1,0,1,0,1,1,1,1,1,0,1,1,1,1,1,0,1,0,1,0,1,0,1,0,1,1,1,0,1],
        [1,0,1,0,0,0,0,0,0,0,1,0,0,0,0,0,0,0,1,0,1,0,1,0,0,0,0,0,1],
        [1,0,1,0,1,0,1,0,1,0,1,0,1,1,1,1,1,0,1,0,1,1,1,0,1,1,1,0,1],
        [1,0,1,0,1,0,1,0,1,0,1,0,0,0,0,0,1,0,1,0,0,0,0,0,0,0,1,0,1],
        [1,0,1,0,1,1,1,0,1,0,1,0,1,1,1,0,1,0,1,0,1,1,1,1,1,1,1,0,1],
        [1,0,1,0,1,0,0,0,1,0,1,0,1,0,0,0,1,0,0,0,0,0,0,0,0,0,0,0,1],
        [1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,0,1,0,1],
        [0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,1,0,1],
        [1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,0,1,1,1,0,1,0,1,0,1,0,1],
        [1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,1,0,0,0,1,0,1,0,1,0,1,0,1],
        [1,0,1,1,1,1,1,1,1,1,1,1,1,1,1,0,1,0,1,1,1,0,1,1,1,0,1,0,1],
        [1,0,1,0,0,0,0,0,0,0,0,0,0,0,0,0,1,0,1,0,0,0,0,0,0,0,1,0,1],
        [1,0,1,0,1,1,1,1,1,1,1,1,1,1,1,0,1,0,1,1,1,1,1,1,1,0,1,0,1],
        [1,0,1,0,0,0,0,0,1,0,0,0,0,0,1,0,1,0,0,0,0,0,0,0,1,0,1,0,1],
        [1,0,1,0,1,1,1,0,1,0,1,1,1,0,1,0,1,0,1,1,1,1,1,1,1,0,1,0,1],
        [1,0,0,0,1,0,1,0,1,0,0,0,1,0,1,0,0,0,0,0,0,0,0,0,0,0,0,0,1],
        [1,1,1,1,1,0,1,0,1,1,1,1,1,1,1,1,1,1,1,0,1,1,1,1,1,0,0,0,1],
        [1,0,0,0,0,0,1,0,0,0,0,0,0,0,0,0,0,0,1,0,1,0,0,0,1,0,0,0,1],
        [1,0,1,1,1,1,1,0,1,1,1,1,1,1,1,1,1,1,1,0,1,0,1,1,1,0,0,0,1],
        [1,0,1,0,0,0,0,0,1,0,0,0,1,0,0,0,1,0,0,0,1,0,0,0,0,0,0,0,1],
        [1,0,1,0,1,1,1,0,1,0,1,0,1,0,1,0,1,1,1,1,1,1,1,1,1,0,1,0,1],
        [1,0,1,0,0,0,1,0,1,0,1,0,0,0,1,0,1,0,0,0,0,0,0,0,1,0,1,0,1],
        [1,0,1,0,0,0,1,0,1,0,1,0,1,1,1,0,1,0,1,1,1,1,1,0,1,0,1,0,1],
        [1,0,1,0,0,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,0,0,1,0,1,0,1,0,1],
        [1,0,1,1,1,1,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,1,1,0,1,1,1,0,1],
        [1,0,0,0,0,0,0,0,1,0,1,0,1,0,0,0,0,0,1,0,0,0,0,0,0,0,0,0,1],
        [1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1]
    ])

    
    x, y = input("Enter start position separated by spaces: ").split()
    position_start = [float(x),float(y)] 
    x, y = input("Enter goal position separated by spaces: ").split()
    position_end = [float(x),float(y)] 

    start = (int(-position_start[1]*2+20), int(position_start[0]*2+20))
    end = (int(-position_end[1]*2+20), int(position_end[0]*2+20))

    grid_vis = np.copy(grid)
    plt.imshow(grid_vis, cmap='Greys')
    plt.scatter(start[1], start[0], marker = "o", color = "green", label="Start")
    plt.scatter(end[1], end[0], marker = "x", color = "red", label="End")
    plt.legend()
    plt.show()

    path = nav.a_star_search(grid, start, end)
    convert_path = [position_start]
    for idx in range(len(path)-1,0,-1):
        convert_path += [[(path[idx][1]-20)/2, -(path[idx][0]-20)/2]]
    convert_path += [position_end]

    actual_path = [position_start]
    for idx in range(1,len(convert_path)-1):
        if convert_path[idx][0] == convert_path[idx-1][0]:
            # same x 
            if not convert_path[idx+1][0] == convert_path[idx-1][0]:
                # go until x changes
                actual_path += [convert_path[idx]]
        if convert_path[idx][1] == convert_path[idx-1][1]:
            # same y
            if not convert_path[idx+1][1] == convert_path[idx-1][1]:
                # go until y changes
                actual_path += [convert_path[idx]]
    actual_path += [position_end]

    print("Path:", path)
    print("Actual Path:", actual_path)

    # Visualization
    def visualize_path(grid, path):
        grid_vis = np.copy(grid)
        for step in path:
            grid_vis[step] = 0.5

        plt.imshow(grid_vis, cmap='Greys')
        plt.plot([p[1] for p in path], [p[0] for p in path], marker = "o", color = "blue")
        plt.scatter(start[1], start[0], marker = "o", color = "green", label="Start")
        plt.scatter(end[1], end[0], marker = "x", color = "red", label="End")
        plt.legend()
        plt.axis('off')
        plt.show()

    if path:
        visualize_path(grid, path[::-1])
        nav.run(actual_path)
        
        try:
            rospy.spin()
        except KeyboardInterrupt:
            print("Shutting down")  
    else:
        print("No path found!")
        
    
    
    
    
    
    


   
      
    
    
    
    
