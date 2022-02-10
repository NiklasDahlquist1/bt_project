#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Pose
from nav_msgs.msg import Odometry
from bt_project.msg import auction_winner

from geometry_msgs.msg import Point

import math
import numpy as np

import matplotlib as mpl
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from matplotlib.lines import Line2D
from matplotlib.patches import Patch


curves_x = [[],[], [],[], []]
curves_y = [[],[], [],[], []]
curves_z = [[],[], [],[], []]
curves_t = [[], [], [], [], []]

objects = []


task_aborted_points = []
task_succeeded_points = []

explore_goal_points = []
explore_cancel_points = []

motor_fail_points = []


def curve_0_CB(data):
    curves_x[0].append(data.pose.pose.position.x)
    curves_y[0].append(data.pose.pose.position.y)
    curves_z[0].append(data.pose.pose.position.z)
    curves_t[0].append(data.header.stamp)
    #print(data.header.stamp)

def curve_1_CB(data):
    curves_x[1].append(data.pose.pose.position.x)
    curves_y[1].append(data.pose.pose.position.y)
    curves_z[1].append(data.pose.pose.position.z)
    curves_t[1].append(data.header.stamp)


def curve_2_CB(data):
    curves_x[2].append(data.pose.pose.position.x)
    curves_y[2].append(data.pose.pose.position.y)
    curves_z[2].append(data.pose.pose.position.z)
    curves_t[2].append(data.header.stamp)

def curve_3_CB(data):
    curves_x[3].append(data.pose.pose.position.x)
    curves_y[3].append(data.pose.pose.position.y)
    curves_z[3].append(data.pose.pose.position.z)
    curves_t[3].append(data.header.stamp)


def curve_4_CB(data):
    curves_x[4].append(data.pose.pose.position.x)
    curves_y[4].append(data.pose.pose.position.y)
    curves_z[4].append(data.pose.pose.position.z)
    curves_t[4].append(data.header.stamp)


def auction_winner_CB(data):
    if (data.task_name == "moveTo"):
        p = data.task_data.split(";")
        objects.append([float(p[0]), float(p[1]), float(p[2])])
        #print(p[0])

def task_aborted_CB(data):
    if (data.task_name == "moveTo"):
        p = data.task_data.split(";")
        task_aborted_points.append([float(p[0]), float(p[1]), float(p[2])])

def task_succeeded_CB(data):
    if (data.task_name == "moveTo"):
        p = data.task_data.split(";")
        task_succeeded_points.append([float(p[0]), float(p[1]), float(p[2])])

def explore_goal_CB(data):
    explore_goal_points.append([data.x, data.y, data.z])

def explore_cancel_CB(data):
    explore_cancel_points.append([data.x, data.y, data.z])

def motor_fail_CB(data):
    motor_fail_points.append([data.x, data.y, data.z])


def callbacks():
    # setup subscribers
    rospy.Subscriber("firefly1/odometry_sensor1/odometry", Odometry, curve_0_CB)
    rospy.Subscriber("firefly2/odometry_sensor1/odometry", Odometry, curve_1_CB)
    rospy.Subscriber("firefly3/odometry_sensor1/odometry", Odometry, curve_2_CB)
    rospy.Subscriber("firefly4/odometry_sensor1/odometry", Odometry, curve_3_CB)
    rospy.Subscriber("firefly5/odometry_sensor1/odometry", Odometry, curve_4_CB)

    rospy.Subscriber("pixy/vicon/demo_crazyflie4/demo_crazyflie4/odom", Odometry, curve_0_CB)
    rospy.Subscriber("pixy/vicon/demo_crazyflie3/demo_crazyflie3/odom", Odometry, curve_1_CB) #TODO, fix topic names for current crazyflies
    rospy.Subscriber("pixy/vicon/demo_crazyflie6/demo_crazyflie6/odom", Odometry, curve_2_CB)
    #rospy.Subscriber("pixy/vicon/demo_crazyflie7/demo_crazyflie7/odom", Odometry, curve_3_CB)
    #rospy.Subscriber("pixy/vicon/demo_crazyflie8/demo_crazyflie8/odom", Odometry, curve_4_CB)




    #rospy.Subscriber("auction_winner", auction_winner, auction_winner_CB)

    rospy.Subscriber("plot/task_aborted", auction_winner, task_aborted_CB)
    rospy.Subscriber("plot/task_succeeded", auction_winner, task_succeeded_CB)
    
    rospy.Subscriber("plot/explore_goal", Point, explore_goal_CB)
    rospy.Subscriber("plot/explore_cancel", Point, explore_cancel_CB)

    rospy.Subscriber("plot/pub_motor_failed_point", Point, motor_fail_CB)
  
  
  
    # collect data

    #rate = rospy.Rate(1.0 / 2)
    #rate.sleep()
    input("Press Enter to continue...") # requires python3... 
    #raw_input("Press Enter to continue...") # python2
 
def plot2():


    """figure.titlesize
    axes.labelsize
    axes.titlesize
    font.size"""

    # plot


    plt.rc('text', usetex=False) # gotta install latex for this
    plt.rcParams.update({
    "font.family": "serif",
    "font.serif": ["Computer Modern Roman"],
    #"ps.usedistiller" : "xpdf",
    "font.size" : 11,
    "axes.titlesize" : 15, #16
})


    edgecolor = [0.4, 0.4, 0.4]
    mpl.rcParams['axes.edgecolor'] = edgecolor



    mpl.rcParams['legend.fontsize'] = 11
    fig, (ax1, ax2, ax3) = plt.subplots(3, sharex=True)#, figsize=[4, 3])
    

    plot_color = [1, 1, 1]
    #ax.w_xaxis.set_pane_color(plot_color)
    #ax.w_yaxis.set_pane_color([c * 1 for c in plot_color])
    #ax.w_zaxis.set_pane_color([c * 1 for c in plot_color])

    #ax.zaxis.pane.set_edgecolor(edgecolor)
    #ax.xaxis.pane.set_edgecolor(edgecolor)
    #ax.yaxis.pane.set_edgecolor(edgecolor)
    
    #ax.grid(False)
    #ax.xaxis.pane.fill = False
    #ax.yaxis.pane.fill = False
    #ax.zaxis.pane.fill = False
    #ax.tick_params(axis='x', colors='red')

    
    #ax.view_init(elev=30, azim=135)
    #ax.view_init(elev=25, azim=135)
    #ax.yaxis.pane.set_edgecolor([0,0,0])
    #ax.zaxis.pane.set_edgecolor([0,0,0])

    
    #ax.set_facecolor('xkcd:salmon')


    linewidth_plot = 1.1
    linewidth_marker = 0.8

    print(curves_y[4])

    # plot all curves
    for i in range(0, len(curves_x)):
        x = np.array(curves_x[i])
        y = np.array(curves_y[i])
        z = np.array(curves_z[i])
        
        t = np.array([tt.to_sec() - curves_t[i][0].to_sec() for tt in curves_t[i]])

        l_w = 1.1
        if(len(t) > 0):
            ax1.plot(t, x, label='UAV ' + str(i + 1), linewidth=l_w)
            ax2.plot(t, y, label='Path ' + str(i), linewidth=l_w)
            ax3.plot(t, z, label='Path ' + str(i), linewidth=l_w)


    ax1.set_title('Lab experiment')
    ax1.set_ylabel('x [m]')
    ax2.set_ylabel('y [m]')
    ax3.set_ylabel('z [m]')


    ax3.set_xlabel('t [s]')


    # custom legends
    legend_elements = [Line2D([0], [0], color='b', lw=linewidth_plot, label='UAV Paths'),
                       Line2D([0], [0], marker='^', color='w', label='Failed tasks', markeredgewidth=linewidth_marker, markeredgecolor="k", markerfacecolor='r', markersize=10),
                       Line2D([0], [0], marker='^', color='w', label='Reached tasks', markeredgewidth=linewidth_marker, markeredgecolor="k", markerfacecolor='g', markersize=10),
                       
                       ]
                       #,
                       #Patch(facecolor='orange', edgecolor='r',
                       #  label='Color Patch')]

    plt.legend( loc='best')


    plt.savefig('plot2.eps') 
    plt.savefig('plot2.png', dpi=600) 
    plt.show()






def plot3():



    """figure.titlesize
    axes.labelsize
    axes.titlesize
    font.size"""

    # plot


    plt.rc('text', usetex=False) # gotta install latex for this
    plt.rcParams.update({
    "font.family": "serif",
    "font.serif": ["Computer Modern Roman"],
    #"ps.usedistiller" : "xpdf",
    "font.size" : 11,
    "axes.titlesize" : 15, #16
    })


    edgecolor = [0.4, 0.4, 0.4]
    mpl.rcParams['axes.edgecolor'] = edgecolor



    mpl.rcParams['legend.fontsize'] = 11
    fig = plt.figure()
    ax = fig.gca(projection='3d')

    plot_color = [1, 1, 1]
    ax.w_xaxis.set_pane_color(plot_color)
    ax.w_yaxis.set_pane_color([c * 1 for c in plot_color])
    ax.w_zaxis.set_pane_color([c * 1 for c in plot_color])

    ax.zaxis.pane.set_edgecolor(edgecolor)
    ax.xaxis.pane.set_edgecolor(edgecolor)
    ax.yaxis.pane.set_edgecolor(edgecolor)
    
    ax.grid(False)
    ax.xaxis.pane.fill = False
    ax.yaxis.pane.fill = False
    ax.zaxis.pane.fill = False
    #ax.tick_params(axis='x', colors='red')

    
    #ax.view_init(elev=30, azim=135)
    ax.view_init(elev=25, azim=135)
    #ax.yaxis.pane.set_edgecolor([0,0,0])
    #ax.zaxis.pane.set_edgecolor([0,0,0])

    
    #ax.set_facecolor('xkcd:salmon')


    linewidth_plot = 1.1
    linewidth_marker = 0.8


    # plot all curves
    for i in range(0, len(curves_x)):
        x = np.array(curves_x[i])
        y = np.array(curves_y[i])
        z = np.array(curves_z[i])

        ax.plot(x, y, z, label='Path ' + str(i), linewidth=0.9)
        




    # plot all objects
    for i in range(0, len(objects)):
        #if len(objects) == 0: #empty is ignored anyway...
        #    break
        x = np.array(objects[i][0])
        y = np.array(objects[i][1])
        z = np.array(objects[i][2])

        if z >= 3:
            ax.scatter(x, y, z, """label='objective ' + str(i)""", c='r', marker='o')
        else:
            ax.scatter(x, y, z, """label='objective ' + str(i)""", c='g', marker='o')
        #ax.legend()



    for i in range(0, len(task_succeeded_points)):
        #if len(objects) == 0: #empty is ignored anyway...
        #    break
        x = np.array(task_succeeded_points[i][0])
        y = np.array(task_succeeded_points[i][1])
        z = np.array(task_succeeded_points[i][2])

        ax.scatter(x, y, z, """label='succeeded ' + str(i)""", c='g', edgecolors='k', linewidths=linewidth_marker, marker='^', s=45)
        #ax.legend()

    for i in range(0, len(task_aborted_points)):
        #if len(objects) == 0: #empty is ignored anyway...
        #    break
        x = np.array(task_aborted_points[i][0])
        y = np.array(task_aborted_points[i][1])
        z = np.array(task_aborted_points[i][2])

        ax.scatter(x, y, z, """label='aborted ' + str(i)""", c='r', edgecolors='k', linewidths=linewidth_marker,  marker='^', s=45)
        #ax.legend()

    for i in range(0, len(explore_goal_points)):
        #if len(objects) == 0: #empty is ignored anyway...
        #    break
        x = np.array(explore_goal_points[i][0])
        y = np.array(explore_goal_points[i][1])
        z = np.array(explore_goal_points[i][2])

        ax.scatter(x, y, z, """label='goal ' + str(i)""", c='b', edgecolors='k', linewidths=linewidth_marker,  marker='^', s=45)
        #ax.legend()
    for i in range(0, len(explore_cancel_points)):
        #if len(objects) == 0: #empty is ignored anyway...
        #    break
        x = np.array(explore_cancel_points[i][0])
        y = np.array(explore_cancel_points[i][1])
        z = np.array(explore_cancel_points[i][2])

        ax.scatter(x, y, z, """label='goal ' + str(i)""", c='y', edgecolors='k', linewidths=linewidth_marker,  marker='^', s=45)
        #ax.legend()

    for i in range(0, len(motor_fail_points)):
        #if len(objects) == 0: #empty is ignored anyway...
        #    break
        x = np.array(motor_fail_points[i][0])
        y = np.array(motor_fail_points[i][1])
        z = np.array(motor_fail_points[i][2])

        ax.scatter(x, y, z, """label='goal ' + str(i)""", c='orange', edgecolors='k', linewidths=linewidth_marker,  marker='^', s=45)
        #ax.legend()




    # custom legends
    legend_elements = [Line2D([0], [0], color='b', lw=linewidth_plot, label='UAV Paths'),
                       Line2D([0], [0], marker='^', color='w', label='Failed tasks', markeredgewidth=linewidth_marker, markeredgecolor="k", markerfacecolor='r', markersize=10),
                       Line2D([0], [0], marker='^', color='w', label='Reached tasks', markeredgewidth=linewidth_marker, markeredgecolor="k", markerfacecolor='g', markersize=10),
                       
                       ]
                       #,
                       #Patch(facecolor='orange', edgecolor='r',
                       #  label='Color Patch')]


    # add points related to explore if they exists
    if(len(explore_goal_points) > 0):
        legend_elements.append(Line2D([0], [0], marker='^', color='w', label='Explore points', markeredgewidth=linewidth_marker, markeredgecolor="k", markerfacecolor='b', markersize=10))
        
    if(len(explore_cancel_points) > 0):
        legend_elements.append(Line2D([0], [0], marker='^', color='w', label='Explore timeout', markeredgewidth=linewidth_marker, markeredgecolor="k", markerfacecolor='y', markersize=10))

    if(len(motor_fail_points) > 0):
        legend_elements.append(Line2D([0], [0], marker='^', color='orange', label='UAV Failure', markeredgewidth=linewidth_marker, markeredgecolor="k", markerfacecolor='y', markersize=10))


    ax.legend(handles=legend_elements, loc='best')


    
    #ax.legend( loc='best')
  

    ax.set_ylabel('Y [m]')
    ax.set_xlabel('X [m]')
    ax.set_zlabel('Z [m]')
    ax.set_title('Lab experiment')
    

    plt.savefig('plot3.eps') 
    plt.savefig('plot3.png', dpi=600) 
    plt.show()



    
    #print(objects[0])


















if __name__ == '__main__':
    try:
        rospy.init_node('plotter', anonymous=True)
        callbacks()
        plot3()
        #plot2()

    except rospy.ROSInterruptException:
        pass





