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

objects = []


task_aborted_points = []
task_succeeded_points = []

explore_goal_points = []
explore_cancel_points = []




def curve_0_CB(data):
    curves_x[0].append(data.pose.pose.position.x)
    curves_y[0].append(data.pose.pose.position.y)
    curves_z[0].append(data.pose.pose.position.z)

def curve_1_CB(data):
    curves_x[1].append(data.pose.pose.position.x)
    curves_y[1].append(data.pose.pose.position.y)
    curves_z[1].append(data.pose.pose.position.z)

def curve_2_CB(data):
    curves_x[2].append(data.pose.pose.position.x)
    curves_y[2].append(data.pose.pose.position.y)
    curves_z[2].append(data.pose.pose.position.z)

def curve_3_CB(data):
    curves_x[3].append(data.pose.pose.position.x)
    curves_y[3].append(data.pose.pose.position.y)
    curves_z[3].append(data.pose.pose.position.z)

def curve_4_CB(data):
    curves_x[4].append(data.pose.pose.position.x)
    curves_y[4].append(data.pose.pose.position.y)
    curves_z[4].append(data.pose.pose.position.z)

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






def logic():

    rospy.init_node('plotter', anonymous=True)


    # setup subscribers
    rospy.Subscriber("pixy/vicon/demo_crazyflie4/demo_crazyflie4/odom", Odometry, curve_0_CB)
    rospy.Subscriber("firefly2/odometry_sensor1/odometry", Odometry, curve_1_CB)
    rospy.Subscriber("firefly3/odometry_sensor1/odometry", Odometry, curve_2_CB)
    rospy.Subscriber("firefly4/odometry_sensor1/odometry", Odometry, curve_3_CB)
    rospy.Subscriber("firefly5/odometry_sensor1/odometry", Odometry, curve_4_CB)

    #rospy.Subscriber("auction_winner", auction_winner, auction_winner_CB)

    rospy.Subscriber("plot/task_aborted", auction_winner, task_aborted_CB)
    rospy.Subscriber("plot/task_succeeded", auction_winner, task_succeeded_CB)
    
    rospy.Subscriber("plot/explore_goal", Point, explore_goal_CB)
    rospy.Subscriber("plot/explore_cancel", Point, explore_cancel_CB)



    # collect data

    #rate = rospy.Rate(1.0 / 2)
    #rate.sleep()
    input("Press Enter to continue...") # requires python3... 
    #raw_input("Press Enter to continue...") # python2
 



    #while not rospy.is_shutdown():
    #    rate.sleep()





    # plot


    mpl.rcParams['legend.fontsize'] = 10
    fig = plt.figure()
    ax = fig.gca(projection='3d')


    # plot all curves
    for i in range(0, len(curves_x)):
        x = np.array(curves_x[i])
        y = np.array(curves_y[i])
        z = np.array(curves_z[i])

        ax.plot(x, y, z, label='Path ' + str(i))
        


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

        ax.scatter(x, y, z, """label='succeeded ' + str(i)""", c='g', edgecolors='k', linewidths=1, marker='^', s=45)
        #ax.legend()

    for i in range(0, len(task_aborted_points)):
        #if len(objects) == 0: #empty is ignored anyway...
        #    break
        x = np.array(task_aborted_points[i][0])
        y = np.array(task_aborted_points[i][1])
        z = np.array(task_aborted_points[i][2])

        ax.scatter(x, y, z, """label='aborted ' + str(i)""", c='r', edgecolors='k', linewidths=1,  marker='^', s=45)
        #ax.legend()

    for i in range(0, len(explore_goal_points)):
        #if len(objects) == 0: #empty is ignored anyway...
        #    break
        x = np.array(explore_goal_points[i][0])
        y = np.array(explore_goal_points[i][1])
        z = np.array(explore_goal_points[i][2])

        ax.scatter(x, y, z, """label='goal ' + str(i)""", c='b', edgecolors='k', linewidths=1,  marker='^', s=45)
        #ax.legend()
    for i in range(0, len(explore_cancel_points)):
        #if len(objects) == 0: #empty is ignored anyway...
        #    break
        x = np.array(explore_cancel_points[i][0])
        y = np.array(explore_cancel_points[i][1])
        z = np.array(explore_cancel_points[i][2])

        ax.scatter(x, y, z, """label='goal ' + str(i)""", c='y', edgecolors='k', linewidths=1,  marker='^', s=45)
        #ax.legend()




    # custom legends
    legend_elements = [Line2D([0], [0], color='b', lw=2, label='UAV Paths'),
                       Line2D([0], [0], marker='^', color='w', label='Failed tasks', markerfacecolor='r', markersize=10),
                       Line2D([0], [0], marker='^', color='w', label='Reached tasks', markerfacecolor='g', markersize=10),
                       Line2D([0], [0], marker='^', color='w', label='Explore points', markerfacecolor='b', markersize=10),
                       Line2D([0], [0], marker='^', color='w', label='Explore timeout', markerfacecolor='y', markersize=10)]#,
                       #Patch(facecolor='orange', edgecolor='r',
                       #  label='Color Patch')]

    ax.legend(handles=legend_elements, loc='best')

    #plt.rc('text', usetex=True) # gotta install latex for this
    #ax.legend( loc='best')
    

    ax.set_ylabel('Y [m]')
    ax.set_xlabel('X [m]')
    ax.set_zlabel('Z [m]')
    ax.set_title('Simulation')
    

    plt.savefig('plot.eps') 
    plt.savefig('plot.png', dpi=600) 
    plt.show()



    
    #print(objects[0])


















if __name__ == '__main__':
    try:
        logic()
    except rospy.ROSInterruptException:
        pass





