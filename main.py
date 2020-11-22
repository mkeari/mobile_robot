import matplotlib.pyplot as plt
import matplotlib
import numpy as np
import math
from random import random

# Initial position and parameters of a robot
p_initial = [13, 13]
theta_initial = 1

R_initial = 0.2
L_initial = 1.5
dT_initial = 0.05

robot_speed = 3
prox_sensor_radius = 0.5

# Light source position
p_light = [17, 16]

# Centre coordinates of the obstacles
obstacle_radius = 1.5
p_obstacles = [[6, 7],
               [5, 16],
               [14, 8],
               [18, 3]]


#Class of mobile robot
class mobile_robot():
    def __init__(self, x_initial, y_initial, theta_initial, 
                base_speed = 1.5, R=0.2, L=1.3, dT=0.1, ps_radius = 0.5):
        self.x = x_initial
        self.y = y_initial
        self.theta = theta_initial

        self.R = R
        self.L = L
        self.dT = dT

        self.base_speed = base_speed
        self.v_rwheel = 0
        self.v_lwheel = 0

        self.prox_sensor_radius = ps_radius
        self.fl_sensor = 0
        self.sl_sensor = 0
        self.fr_sensor = 0
        self.sr_sensor = 0

        self.l_light = 0
        self.r_light = 0

    
    def move(self):
        self.forward_kinematics()
        self.calc_velocities()


    def stop(self):
        self.v_lwheel = 0
        self.v_rwheel = 0

    
    def forward_kinematics(self):
        R = (self.L/2)*(self.v_lwheel + self.v_rwheel) / \
            (self.v_rwheel - self.v_lwheel + 0.00001)
        w = (self.v_rwheel - self.v_lwheel + 0.00001) / self.L
        dt = self.dT

        ICC = [(self.x - R * math.sin(self.theta)),
               (self.y + R * math.cos(self.theta))]

        self.x = (math.cos(w*dt) * (self.x - ICC[0]) -
                           math.sin(w*dt) * (self.y - ICC[1]) + ICC[0])
        self.y = (math.sin(w*dt) * (self.x - ICC[0]) +
                           math.cos(w*dt) * (self.y - ICC[1]) + ICC[1])
        self.theta = self.theta + w*dt

    
    def calc_velocities(self):
        self.check_obstacles()
        self.check_light()
        
        self.v_lwheel = self.base_speed + self.l_light + \
                        5*self.fl_sensor - 5*self.fr_sensor + 8*self.sl_sensor
        
        self.v_rwheel = self.base_speed + self.r_light + \
                        5*self.fr_sensor - 5*self.fl_sensor + 8*self.sr_sensor


    def check_light(self):
        #Defining light sensor positions
        T = transformation_matrix(self.x, self.y, self.theta)
        p_ls_left = np.array([0.4 * self.L, 0.15*self.L, 1]).T
        p_ls_right = np.array([0.4 * self.L, -0.15*self.L, 1]).T
        ls_left = np.matmul(T, p_ls_left)
        ls_right = np.matmul(T, p_ls_right)

        #Calculating distances from each sensor to light source
        dist_left = math.hypot(p_light[0] - ls_left[0], 
                               p_light[1] - ls_left[1])
        dist_right = math.hypot(p_light[0] - ls_right[0], 
                               p_light[1] - ls_right[1])

        #Configuring light sensor values 
        dist_average = (dist_left + dist_right)/2
        self.l_light = 100*(dist_left-dist_right)/dist_average
        self.r_light = 100*(dist_right-dist_left)/dist_average

        #Printing for debug
        #print("Left: " + str(self.l_light) + "   Right: " + str(self.r_light))

    
    def check_obstacles(self):
        #Define positions of proximity sensors
        T = transformation_matrix(self.x, self.y, self.theta)
        p_fl = np.array([0.6 * self.L, 0.30*self.L, 1]).T
        p_fr = np.array([0.6 * self.L, -0.30*self.L, 1]).T
        p_sl = np.array([0.2 * self.L, 0.6*self.L, 1]).T
        p_sr = np.array([0.2 * self.L, -0.6*self.L, 1]).T

        ps_fl = np.matmul(T, p_fl)
        ps_fr = np.matmul(T, p_fr)
        ps_sl = np.matmul(T, p_sl)
        ps_sr = np.matmul(T, p_sr)

        #Find nearest obstacle
        min_dist = 40
        nearest_obstacle = [20, 20]
        for obstacle in p_obstacles:
            dist = math.hypot(self.x - obstacle[0], self.y - obstacle[1])
            if dist <= min_dist:
                nearest_obstacle = obstacle
                min_dist = dist

        #print("Nearest obstacle at " + str(nearest_obstacle))

        #Calculate the closeness of nearest obstacle
        self.fl_sensor = self.check_prox_sensor(ps_fl, nearest_obstacle, 
                                            self.prox_sensor_radius, obstacle_radius)
        self.fr_sensor = self.check_prox_sensor(ps_fr, nearest_obstacle, 
                                            self.prox_sensor_radius, obstacle_radius)
        self.sl_sensor = self.check_prox_sensor(ps_sl, nearest_obstacle, 
                                            self.prox_sensor_radius, obstacle_radius)
        self.sr_sensor = self.check_prox_sensor(ps_sr, nearest_obstacle, 
                                            self.prox_sensor_radius, obstacle_radius)                                            
        
        #Printing sensor values
        #print("FL: " + str(self.fl_sensor) + "   FR: " + str(self.fr_sensor) +
        #    "\nSL: " + str(self.sl_sensor) + "   SR: " + str(self.sr_sensor))

    
    def check_prox_sensor(self, sensor_position, obstacle_position, 
                          sensor_radius, obstacle_radius):
        dist = math.hypot(obstacle_position[0] - sensor_position[0],
                          obstacle_position[1] - sensor_position[1])
        if dist <= (sensor_radius + obstacle_radius):
            return 1/dist
        else:
            return 0


def transformation_matrix(x, y, theta):
    return np.array([
        [np.cos(theta), -np.sin(theta), x],
        [np.sin(theta), np.cos(theta), y],
        [0, 0, 1]
    ])


def calc_light_field(reso):
    minx = 0
    miny = 0
    maxx = 200
    maxy = 200
    xw = int(round((maxx - minx) / reso))
    yw = int(round((maxy - miny) / reso))

    light_position[0] = p_light[0] * 10
    light_position[1] = p_light[1] * 10

    # calc each potential
    pmap = [[0.0 for i in range(yw)] for i in range(xw)]

    for ix in range(xw):
        for iy in range(yw):
            dist = math.sqrt((light_position[0] - ix*xw)*(light_position[0] - ix*xw) +
                             (light_position[1] - iy*yw)*(light_position[1] - iy*yw))
            pmap[ix][iy] = dist

    return pmap


def draw_heatmap(data):
    data = np.array(data).T
    data = data/10
    print(data)
    plt.pcolor(data, cmap=plt.cm.Blues)


def plot_vehicle(robot, fig, ax):  
    #Draw rectangle base of a robot
    p1_i = np.array([0.6 * robot.L, 0.30*robot.L, 1]).T
    p2_i = np.array([-0.6 * robot.L, 0.30*robot.L, 1]).T
    p3_i = np.array([-0.6 * robot.L, -0.30*robot.L, 1]).T
    p4_i = np.array([0.6 * robot.L, -0.30*robot.L, 1]).T

    T = transformation_matrix(robot.x, robot.y, robot.theta)
    p1 = np.matmul(T, p1_i)
    p2 = np.matmul(T, p2_i)
    p3 = np.matmul(T, p3_i)
    p4 = np.matmul(T, p4_i)

    plt.plot([p1[0], p2[0]], [p1[1], p2[1]], 'k-')
    plt.plot([p2[0], p3[0]], [p2[1], p3[1]], 'k-')
    plt.plot([p3[0], p4[0]], [p3[1], p4[1]], 'k-')
    plt.plot([p4[0], p1[0]], [p4[1], p1[1]], 'k-')

    ####

    p1_j = np.array([0.2 * robot.L, 0.6*robot.L, 1]).T
    p2_j = np.array([-0.2 * robot.L, 0.6*robot.L, 1]).T
    p3_j = np.array([-0.2 * robot.L, -0.6*robot.L, 1]).T
    p4_j = np.array([0.2 * robot.L, -0.6*robot.L, 1]).T

    T = transformation_matrix(robot.x, robot.y, robot.theta)
    p1_w = np.matmul(T, p1_j)
    p2_w = np.matmul(T, p2_j)
    p3_w = np.matmul(T, p3_j)
    p4_w = np.matmul(T, p4_j)

    plt.plot([p1_w[0], p2_w[0]], [p1_w[1], p2_w[1]], 'k-')
    plt.plot([p2_w[0], p3_w[0]], [p2_w[1], p3_w[1]], 'k-')
    plt.plot([p3_w[0], p4_w[0]], [p3_w[1], p4_w[1]], 'k-')
    plt.plot([p4_w[0], p1_w[0]], [p4_w[1], p1_w[1]], 'k-')

    #Draw wheels
    #p_rwheel = np.array([robot.x, robot.y - robot.L/2]).T
    #p_lwheel = np.array([robot.x, robot.y + robot.L/2]).T

    
    #Draw light sensors
    p1_i_ls = np.array([0.4 * robot.L, 0.15*robot.L, 1]).T
    p2_i_ls = np.array([0.4 * robot.L, -0.15*robot.L, 1]).T
    p1_ls = np.matmul(T, p1_i_ls)
    p2_ls = np.matmul(T, p2_i_ls)

    sensor1 = plt.Circle((p1_ls[0], p1_ls[1]), robot.L/10, color='y')
    sensor2 = plt.Circle((p2_ls[0], p2_ls[1]), robot.L/10, color='y')
    ax.add_artist(sensor1)
    ax.add_artist(sensor2)


    #Draw proximity sensors
    ps_color_detected = 'r'
    ps_plot_color = ['g', 'g', 'g', 'g']
    sensors = [robot.fl_sensor, robot.fr_sensor, robot.sl_sensor, robot.sr_sensor]
    
    i = 0
    for sensor in sensors:
        if(sensor > 0):
            ps_plot_color[i] = ps_color_detected
        i = i + 1
    
    sensor_fl = plt.Circle((p1[0], p1[1]), robot.L/10, color=ps_plot_color[0])
    sensor_sl = plt.Circle((p4[0], p4[1]), robot.L/10, color=ps_plot_color[1])
    sensor_fr = plt.Circle((p1_w[0], p1_w[1]), robot.L/10, color=ps_plot_color[2])
    sensor_sr = plt.Circle((p4_w[0], p4_w[1]), robot.L/10, color=ps_plot_color[3])
    ax.add_artist(sensor_fl)
    ax.add_artist(sensor_sl)
    ax.add_artist(sensor_fr)
    ax.add_artist(sensor_sr)


    #For stopping simulation with the esc key.
    plt.gcf().canvas.mpl_connect('key_release_event',
                                 lambda event: [exit(0) if event.key == 'escape' else None])

    plt.xlim(0, 20)
    plt.ylim(0, 20)

    plt.pause(robot.dT)


def plot_everything(x_start, y_start, theta_start, robot, obs_radius = 1.5):
    plt.cla()

    #Create the figure
    fig = plt.gcf()
    ax = fig.gca()

    #Drawing the initial position
    plt.arrow(x_start, y_start, np.cos(theta_start),
    np.sin(theta_start), color='r', width=0.1)

    #Drawing the light source
    light = plt.Circle(p_light, 0.3, color='y')
    ax.add_artist(light)
    
    #pmap = calc_light_field(10)
    #draw_heatmap(pmap)

    # Drawing obstacle circles
    for obstacle in p_obstacles:
        circle = plt.Circle((obstacle[0],
                            obstacle[1]), obs_radius, color='dodgerblue')
        ax.add_artist(circle)

    #Drawing the robot
    plot_vehicle(robot, fig, ax)



def main():

    robot1 = mobile_robot(p_initial[0], p_initial[1], theta_initial, robot_speed, 
                          R_initial, L_initial, dT_initial, prox_sensor_radius)
    
    dist = math.hypot(robot1.x - p_light[0], robot1.y - p_light[1])
    while dist > 0.8:
        dist = math.hypot(robot1.x - p_light[0], robot1.y - p_light[1])
        print(str(dist))

        robot1.move()
        plot_everything(p_initial[0], p_initial[1], theta_initial, 
                        robot1, obstacle_radius)    
    robot1.stop()    



if __name__ == '__main__':
    main()
