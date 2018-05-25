#!/usr/bin/env python
# Author: Joseph Sahyoun
# This node publishes a sample reference trajectorr for the MPC node and
# receives the optimal state trajectory from the MPC node and plots them 
# both for comparison. 
import rospy
import numpy as np
import matplotlib.pyplot as plt
from barc.msg import barc_state
global counter
counter=0

# Plot the reference vs. optimal state trajectory
def plot_callback(data):
    global xref,yref, counter

    # Plots figure only once 
    if counter > 0 and counter <2:
        line1, = plt.plot(data.x,data.y, label= "optimal state trajectory", linestyle = '--', color='blue')
        line2, = plt.plot(xref,yref, label = "reference trajectory", linestyle = '--',color='red')
        first_legend = plt.legend(handles=[line1],loc=1)
        ax = plt.gca().add_artist(first_legend)
        second_legend= plt.legend(handles=[line2],loc=2)
        plt.xlabel("X")
        plt.ylabel("Y")
        plt.show()
    counter+=1
    

# Create Dummy Image Processing node
def ImgProcess():
    global xref,yref,psiref

    # Initialize node
    rospy.init_node('ImgProcess', anonymous=True)

    # Set Publications/Subscriber
    state = rospy.Publisher('reference_trajectory',barc_state, queue_size=1)
    rospy.Subscriber('optimal_state_trajectory',barc_state,plot_callback)

    # Establishing Rate so that it pulblishes every 5 seconds
    loop_rate   = 0.3
    dt          = 1.0 / loop_rate
    rate        = rospy.Rate(loop_rate)

    # Publish dummy data
    while not rospy.is_shutdown():
        xref=np.array([0.0, 0.14, 0.28, 0.38, 0.48, 0.56, 0.64, 0.7])
        yref=np.array([0.0, 0.14, 0.28, 0.45, .73,  0.913, 1.095, 1.21])
        psiref=np.array([np.pi/4, np.pi/4, np.pi/3, np.pi/3, 1.1*np.pi/3, 1.1*np.pi/3, np.pi/2, np.pi/2])
        state.publish(barc_state(xref,yref,psiref))
        rate.sleep()
    
if __name__ == '__main__':
    try:
        ImgProcess()
    except rospy.ROSInterruptException:
        pass