import matplotlib.pyplot as plt
import matplotlib
import numpy as np
import math
from random import random

# Initial position of a robot
p_initial = [3, 3]
theta_initial = 0

R_initial = 0.2
L_initial = 1.5
dT_initial = 0.05

# Light source position
p_light = [17, 16]

# Centre coordinates of the obstacles
p_obstacles = [[6, 7],
               [5, 16],
               [14, 8],
               [18, 3]]

# simulation parameters
Kp_rho = 9
Kp_alpha = 15
Kp_beta = -3
dt = 0.01

show_animation = True


class mobile_robot():
    def __init__(self, x_initial, y_initial, theta_initial,
                 light_position, R=0.2, L=1.3, dT=0.1):
        self.x = x_initial
        self.y = y_initial
        self.theta = theta_initial

        self.R = R
        self.L = L
        self.dT = dT

        self.v_rwheel = 2
        self.v_lwheel = 2

        self.fl_sensor = 0
        self.sl_sensor = 0
        self.fr_sensor = 0
        self.sr_sensor = 0

        self.l_light = 0
        self.r_light = 0

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

    def calc_velocities(self, light_position):
        #self.check_obstacles()
        ls_left = self.x

    def check_light(self, light_position):
        T = transformation_matrix(self.x, self.y, self.theta)
        p_ls_left = np.array([0.4 * self.L, 0.15*self.L, 1]).T
        p_ls_right = np.array([0.4 * self.L, -0.15*self.L, 1]).T
        ls_left = np.matmul(T, p_ls_left)
        ls_right = np.matmul(T, p_ls_right)

        dist_left = math.hypot(light_position[0] - ls_left[0], 
                               light_position[1] - ls_left[1])
        dist_right = math.hypot(light_position[0] - ls_right[0], 
                               light_position[1] - ls_right[1])
        return dist_left, dist_right

    def check_obstacles(self):
        T = transformation_matrix(self.x, self.y, self.theta)
        p_fl = np.array([0.6 * robot.L, 0.30*robot.L, 1]).T
        p_fr = np.array([0.6 * robot.L, -0.30*robot.L, 1]).T
        p_sl = np.array([0.2 * robot.L, 0.6*robot.L, 1]).T
        p_sr = np.array([0.2 * robot.L, -0.6*robot.L, 1]).T

        ps_fl = np.matmul(T, p_fl)
        ps_fr = np.matmul(T, p_fr)
        ps_sl = np.matmul(T, p_sl)
        ps_sr = np.matmul(T, p_sr)

        

def move_robot(p_initial, theta_initial, light_position):
    x_position = x_initial
    y_position = y_initial
    theta = theta_initial

    v_left, vright = robot.calc_velocities()

    x_diff = light_position - x_position
    y_diff = light_position - y_position

    x_traj, y_traj = [], []

    dist = np.hypot(x_diff, y_diff)

    #while dist > 0.05:


def move_to_pose(x_start, y_start, theta_start, x_goal, y_goal, theta_goal):
    """
    rho is the distance between the robot and the goal position
    alpha is the angle to the goal relative to the heading of the robot
    beta is the angle between the robot's position and the goal position plus the goal angle

    Kp_rho*rho and Kp_alpha*alpha drive the robot along a line towards the goal
    Kp_beta*beta rotates the line so that it is parallel to the goal angle
    """
    x = x_start
    y = y_start
    theta = theta_start

    x_diff = x_goal - x
    y_diff = y_goal - y

    x_traj, y_traj = [], []

    rho = np.hypot(x_diff, y_diff)
    while rho > 0.001:
        x_traj.append(x)
        y_traj.append(y)

        x_diff = x_goal - x
        y_diff = y_goal - y

        # Restrict alpha and beta (angle differences) to the range
        # [-pi, pi] to prevent unstable behavior e.g. difference going
        # from 0 rad to 2*pi rad with slight turn

        rho = np.hypot(x_diff, y_diff)
        alpha = (np.arctan2(y_diff, x_diff)
                 - theta + np.pi) % (2 * np.pi) - np.pi
        beta = (theta_goal - theta - alpha + np.pi) % (2 * np.pi) - np.pi

        v = Kp_rho * rho
        w = Kp_alpha * alpha + Kp_beta * beta

        if alpha > np.pi / 2 or alpha < -np.pi / 2:
            v = -v

        theta = theta + w * dt
        x = x + v * np.cos(theta) * dt
        y = y + v * np.sin(theta) * dt

        if show_animation:  # pragma: no cover
            plt.cla()

            fig = plt.gcf()
            ax = fig.gca()

            plt.arrow(x_start, y_start, np.cos(theta_start),
                      np.sin(theta_start), color='r', width=0.1)

            # Drawing obstacle circles
            for obstacle in p_obstacles:
                circle = plt.Circle((obstacle[0],
                                     obstacle[1]), 1.5, color='dodgerblue')
                ax.add_artist(circle)

            plt.arrow(x_goal, y_goal, np.cos(theta_goal),
                      np.sin(theta_goal), color='g', width=0.1)
            plot_vehicle(x, y, theta, x_traj, y_traj)


def transformation_matrix(x, y, theta):
    return np.array([
        [np.cos(theta), -np.sin(theta), x],
        [np.sin(theta), np.cos(theta), y],
        [0, 0, 1]
    ])


def place_obstacle(x, y, size, color="-b"):  # pragma: no cover
    deg = list(range(0, 360, 5))
    deg.append(0)
    xl = [x + size * math.cos(np.deg2rad(d)) for d in deg]
    yl = [y + size * math.sin(np.deg2rad(d)) for d in deg]
    plt.plot(xl, yl, color)


def calc_light_field(light_position, reso):
    minx = 0
    miny = 0
    maxx = 200
    maxy = 200
    xw = int(round((maxx - minx) / reso))
    yw = int(round((maxy - miny) / reso))

    light_position[0] = light_position[0] * 10
    light_position[1] = light_position[1] * 10

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


def plot_vehicle(robot, fig, ax):  # pragma: no cover
    # Corners of triangular vehicle when pointing to the right (0 radians)
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
    sensor_fl = plt.Circle((p1[0], p1[1]), robot.L/10, color='r')
    sensor_sl = plt.Circle((p4[0], p4[1]), robot.L/10, color='r')
    sensor_fr = plt.Circle((p1_w[0], p1_w[1]), robot.L/10, color='r')
    sensor_sr = plt.Circle((p4_w[0], p4_w[1]), robot.L/10, color='r')
    ax.add_artist(sensor_fl)
    ax.add_artist(sensor_sl)
    ax.add_artist(sensor_fr)
    ax.add_artist(sensor_sr)


    #For stopping simulation with the esc key.
    plt.gcf().canvas.mpl_connect('key_release_event',
                                 lambda event: [exit(0) if event.key == 'escape' else None])

    plt.xlim(0, 20)
    plt.ylim(0, 20)

    plt.pause(dt)

def plot_everything(x_start, y_start, theta_start, robot, light_position):
    plt.cla()

    #Create the figure
    fig = plt.gcf()
    ax = fig.gca()

    #Drawing the initial position
    plt.arrow(x_start, y_start, np.cos(theta_start),
    np.sin(theta_start), color='r', width=0.1)

    #Drawing the light source
    light = plt.Circle(light_position, 0.3, color='y')
    ax.add_artist(light)
    
    #pmap = calc_light_field(light_position, 10)
    #draw_heatmap(pmap)

    # Drawing obstacle circles
    for obstacle in p_obstacles:
        circle = plt.Circle((obstacle[0],
                            obstacle[1]), 1.5, color='dodgerblue')
        ax.add_artist(circle)

    #Drawing the robot
    plot_vehicle(robot, fig, ax)

def main():

    robot1 = mobile_robot(p_initial[0], p_initial[1], theta_initial, p_light, 
                          R_initial, L_initial, dT_initial)
    for i in range(1000):
        robot1.forward_kinematics()
        print(robot1.check_light(p_light))
        # print("X: " + str(robot1.position_x) + 
        #     "\nY: " + str(robot1.position_y) + 
        #     "\nTh: " + str(robot1.theta))
        plot_everything(p_initial[0], p_initial[1], theta_initial, 
                        robot1, p_light)        



if __name__ == '__main__':
    main()
